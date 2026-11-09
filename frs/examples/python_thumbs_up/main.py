"""独立点赞 Atom：读取反馈、调用规划插件、申请资源、执行并上报结果。"""

import ctypes
import json
import math
import os
from pathlib import Path
import threading
import time
from typing import Mapping

import frs
from frs.aurora_types import AuroraResource, JointControlType, LeaseState, MotorMode

TOPIC = "/motion_ctrl/aurora/control_group_state"
PERIOD = 1.0 / 30.0
HERE = Path(__file__).resolve().parent
ARM_GOAL = [-0.35, -0.25, 0.0, -1.25, 0.0, 0.0, 0.0]
HAND_GOAL = [1.70, 1.70, 1.70, 1.70, 0.15, 0.80]


# FDH-6R 右手的接口顺序与范围直接定义在 Atom 中；单位是 rad。
# channels 的顺序就是反馈/命令数组的顺序，不是 URDF 中的文本顺序。
HAND = {
    "hand_type": "FDH-6R", "group": "right_end_effector", "mapping_mode": "sim", "unit": "rad",
    "channels": [
        {"index": 0, "name": "index",       "lower": 0.17, "upper": 1.78},  # 食指
        {"index": 1, "name": "middle",      "lower": 0.17, "upper": 1.78},  # 中指
        {"index": 2, "name": "ring",        "lower": 0.17, "upper": 1.78},  # 无名指
        {"index": 3, "name": "pinky",       "lower": 0.17, "upper": 1.78},  # 小指
        {"index": 4, "name": "thumb_pitch", "lower": 0.12, "upper": 1.28},  # 拇指俯仰
        {"index": 5, "name": "thumb_yaw",   "lower": 0.0,  "upper": 1.68},  # 拇指偏航
    ],
    "tolerance_rad": 0.03,
}


def checked(values, profile):
    # 每次使用输入、插件目标或输出帧，都检查数量、有限值和当前接口限位。
    if not isinstance(values, list) or len(values) != len(profile["channels"]):
        raise RuntimeError("hand position dimension mismatch")
    for value, channel in zip(values, profile["channels"]):
        if (type(value) not in (int, float) or not math.isfinite(value) or
                not channel["lower"] - 1e-6 <= value <= channel["upper"] + 1e-6):
            raise RuntimeError("hand position outside limits or non-finite: " + channel["name"])
    return list(values)


def plan_arm(directory, message):
    # ctypes 只调用本工程的 JSON/C 适配层，不能直接调用 FBA 的 C++ 虚函数。
    library = ctypes.CDLL(str(directory / "build/libpose_planner.so"))
    library.thumbs_up_plan.argtypes = [ctypes.c_char_p, ctypes.c_void_p, ctypes.c_size_t]
    library.thumbs_up_plan.restype = ctypes.c_int
    request = dict(message, target=ARM_GOAL,
                   plugin=str(directory / "plugins/libfba_movetools.so"),
                   planning=str(directory / "planning/gr3v2_2_4"),
                   robot_config="/opt/fftai/fourier_config_center")
    output = ctypes.create_string_buffer(2 * 1024 * 1024)
    code = library.thumbs_up_plan(json.dumps(request, allow_nan=False).encode(), output, len(output))
    if code == 2:
        raise RuntimeError("planner input/output buffer error")
    result = json.loads(output.value)
    if code != 0 or not result["ok"]:
        raise RuntimeError(result["error"])
    return result  # dt=1/30、positions、模型限位；适配层已检查每帧。


def read_hand(context, profile):
    # get 读取 FRS 共享缓存，公共 IDL codec 解码完整 CDR；不在 Atom 创建 DDS Reader。
    sample = context.topic.get(TOPIC)
    if not sample.available or not sample.fresh or not 0 <= sample.frs.age_ns <= 100_000_000:
        raise RuntimeError("hand feedback is unavailable or older than 100 ms")
    # joint_state 是实测；仿真的手部 joint_cmd 可能没有条目，不能用它替代实测。
    groups = [g for g in sample.msg["joint_state"] if g["group_name"] == profile["group"]]
    if len(groups) != 1:
        raise RuntimeError("right hand feedback is missing or duplicated")
    group = groups[0]
    measured = checked(group["joint_position"], profile)
    for field in ("joint_velocity", "joint_effort"):
        values = group.get(field, [])
        if values and (len(values) != len(measured) or
                       any(type(v) not in (int, float) or not math.isfinite(v) for v in values)):
            raise RuntimeError("invalid hand " + field)
    return sample, measured


def read_feedback(context, profile):
    sample, hand = read_hand(context, profile)
    # 规划需要整机实测和命令姿态；执行与到位检查只取本例控制的右臂、右手。
    def arm(field):
        groups = [g for g in sample.msg[field] if g["group_name"] == "right_manipulator"]
        if len(groups) != 1:
            raise RuntimeError("right arm feedback is missing or duplicated")
        values = groups[0]["joint_position"]
        if len(values) != 7 or any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
            raise RuntimeError("invalid right arm feedback")
        for field in ("joint_velocity", "joint_effort"):
            extra = groups[0].get(field, [])
            if extra and (len(extra) != 7 or any(type(v) not in (int, float) or not math.isfinite(v) for v in extra)):
                raise RuntimeError("invalid right arm " + field)
        return list(values)
    return sample, {"right_manipulator": arm("joint_state"), "right_end_effector": hand}, arm("joint_cmd")


def position_command(positions):
    # 一条命令同时装入右臂轨迹帧和固定手部目标，与 FBA move_pose 同步发布。
    # JSON 代理要求完整 header；零值对应 C++ MessageHeader 的默认构造。
    # 发布时 SDK 填写发送时间、序号和 trace，业务代码不伪造源时间。
    header = {key: "0" for key in ("send_time_ns", "received_time_ns", "sensor_time_ns",
                                   "start_time_ns", "publish_time_ns", "sequence")}
    header.update(frame_id="", trace_id=[0] * 16)
    return {"header": header, "joint_cmd": [
        {"group_name": group, "control_type": JointControlType.JOINT, "motor_mode": MotorMode.POSITION,
         "position": values, "velocity": [0.0] * len(values), "effort": [0.0] * len(values)}
        for group, values in positions.items()], "cartesian_cmd": [], "ttl_ms": 250,
        "admission_deadline_ns": str(time.time_ns() + 250_000_000)}


def difference(a, b):
    if len(a) != len(b):
        raise RuntimeError("position dimension changed")
    return max(abs(x-y) for x, y in zip(a, b))


def bounded_arm(values, path):
    if len(values) != 7 or any(not math.isfinite(v) or not low-1e-6 <= v <= high+1e-6
                              for v, low, high in zip(values, path["lower"], path["upper"])):
        raise RuntimeError("right arm position outside model limits")



class ThumbsUp:
    def __init__(self, context, directory=HERE):
        self.context = context
        self.profile = HAND
        self.directory = Path(directory).resolve()
        self.goal = checked(HAND_GOAL, self.profile)
        self.cancel = threading.Event()
        self.completed = threading.Event()
        self.completed.set()
        self.lock = threading.Lock()
        self.blocked = False
        self.client = None

    def execute(self, command: str, input_value: Mapping[str, object]):
        # Runtime 已创建执行线程，并关联原 request_id。run 不需要任何业务参数。
        if not isinstance(input_value, Mapping) or dict(input_value) not in ({}, {"argv": []}):
            raise ValueError("run/stop accept no arguments")
        if command == "stop":
            with self.lock:
                if self.completed.is_set():
                    raise RuntimeError("no active thumbs-up task")
                self.cancel.set()
            # stop 有自己的请求 ID，等原任务完成保持/释放后才报告 stop 成功。
            if not self.completed.wait(5.0) or self.blocked:
                raise RuntimeError("pose cleanup was not confirmed")
            return {"status": "finished", "message": "pose held; resources released"}
        if command != "run":
            raise ValueError("unsupported command")
        with self.lock:
            if self.blocked or not self.completed.is_set():
                raise RuntimeError("Atom is busy or requires restart")
            self.completed.clear()
            self.cancel.clear()
        client = None
        lease_attempted = published = cleanup_failed = False
        failure = None
        payload = b""
        try:
            # 1. 本例使用 FDH-6R 的 FHM 手部接口；只在仿真模式执行。
            if os.environ.get("FRS_RUNTIME_MODE") != self.profile["mapping_mode"]:
                raise RuntimeError("this hand profile requires simulation")
            self.context.update({"state": "running", "message": "reading_feedback", "progress": 0.0})
            sample, initial, initial_command = read_feedback(self.context, self.profile)
            if difference(initial["right_manipulator"], initial_command) > 0.15:
                raise RuntimeError("initial arm measured/command positions disagree")
            # 2. FRS 的 Aurora 代理沿用 Atom 身份，不创建第二个真实 SDK/DDS 客户端。
            client = self.client = frs.aurora()
            if not client.wait_for_endpoints(500):
                raise RuntimeError("Aurora endpoints are not ready")
            # 全局 FSM 在运行任务前由开发者准备，Atom 申请右臂和右手控制资源。
            # PdStandHanging 不转发上肢/手部 DDS，发布受理并不代表会执行。
            operation, state = client.get_aurora_state()
            if not operation.success() or state["current_state"]["name"] != "UpperBodyUserCmd":
                raise RuntimeError("prepare UpperBodyUserCmd in simulation before running this Atom")
            # 3. 与 FBA move_pose 的右臂分支一样：先用整机快照规划，再申请资源。
            self.context.update({"state": "running", "message": "planning", "progress": 0.0})
            path = plan_arm(self.directory, {key: sample.msg[key] for key in ("joint_state", "joint_cmd")})
            if self.cancel.is_set():
                return {"status": "cancelled", "message": "cancelled before lease"}
            lease_attempted = True
            lease = client.register_lease([AuroraResource.RIGHT_MANIPULATOR, AuroraResource.RIGHT_END_EFFECTOR],
                                         {"duration_ms": 8000, "priority": None, "timeout_ms": 500})
            if not lease.success() or not lease.usable():
                raise RuntimeError("arm/hand lease rejected: " + str(lease))
            # 规划可能花时间；拿到资源后再次读状态，发现起点改变就拒绝播放旧轨迹。
            _, live, command = read_feedback(self.context, self.profile)
            if difference(command, initial_command) > 0.15 or difference(live["right_manipulator"], initial_command) > 0.15:
                raise RuntimeError("arm start changed while planning")
            if difference(live["right_end_effector"], initial["right_end_effector"]) > 0.15:
                raise RuntimeError("hand start changed while planning")
            # 4. 30 Hz 消费插件轨迹；手部每帧发送相同目标，速度由手部控制器管理。
            begin = time.monotonic()
            stable_since = last_sequence = None
            previous = initial_command
            index = 0
            timeout = (len(path["positions"]) - 1) * PERIOD + 2.0
            while time.monotonic() - begin < timeout:
                due = begin + index * PERIOD
                if self.cancel.wait(max(0, due - time.monotonic())):
                    break
                if time.monotonic() > due + 2 * PERIOD:
                    raise TimeoutError("pose publication missed its deadline")
                sample, measured, _ = read_feedback(self.context, self.profile)
                bounded_arm(measured["right_manipulator"], path)
                arm_frame = path["positions"][min(index, len(path["positions"])-1)]
                if difference(arm_frame, previous) > 0.15:
                    raise RuntimeError("arm frame step exceeds publication budget")
                frame = {"right_manipulator": arm_frame, "right_end_effector": self.goal}
                if self.cancel.is_set():
                    break
                # 响应丢失时也可能已送达，先记录尝试，失败仍要保持和释放。
                published = True
                result = client.publish_control_group_command(position_command(frame))
                if not result.success():
                    raise RuntimeError("pose publication rejected: " + str(result))
                previous = arm_frame
                # 5. 与 FBA 一致：手部实测误差≤0.03 rad，稳定100 ms；右臂检查轨迹已发布。
                arm_error = difference(measured["right_manipulator"], ARM_GOAL)
                error = difference(measured["right_end_effector"], self.goal)
                trajectory_done = index + 1 >= len(path["positions"])
                fresh_frame = sample.frs.sequence != last_sequence
                if fresh_frame:
                    last_sequence = sample.frs.sequence
                    stable_since = (stable_since or time.monotonic()) if error <= 0.03 else None
                settled = stable_since is not None and time.monotonic() - stable_since >= 0.1
                if not settled and time.monotonic() - begin >= 2.0:
                    raise TimeoutError("thumbs-up hand feedback did not settle within 2 s")
                payload = json.dumps({"group": self.profile["group"], "target": self.goal,
                                      "measured": measured["right_end_effector"], "arm_target": ARM_GOAL,
                                      "arm_measured": measured["right_manipulator"], "arm_error_rad": arm_error, "max_error_rad": error,
                                      "sequence": str(sample.frs.sequence)}).encode("utf-8")
                self.context.update({"state": "running", "message": "settling" if trajectory_done else "moving",
                                     "progress": min(0.95, (index+1)/len(path["positions"])),
                                     "content_type": "application/json", "data": payload})
                if trajectory_done and settled and fresh_frame:
                    break
                index += 1
            else:
                raise TimeoutError("thumbs-up feedback did not settle after trajectory")
        except Exception as error:
            failure = error
        finally:
            # 6. 成功、取消、异常共用收尾：停止目标出帧，再保持当前实测位置。
            if published:
                try:
                    _, measured, _ = read_feedback(self.context, self.profile)
                    bounded_arm(measured["right_manipulator"], path)
                    held = client.publish_control_group_command(position_command(measured))
                    if not held.success():
                        raise RuntimeError("measured hold rejected")
                except Exception as error:
                    cleanup_failed = True
                    failure = RuntimeError(str(failure or "task ended") + "; hold: " + str(error))
            # 申请响应不确定也尝试释放；UNREGISTERED 才是已确认的释放结果。
            if lease_attempted:
                try:
                    released = client.release_lease()
                    if not released.success() or released.get("state") != LeaseState.UNREGISTERED:
                        raise RuntimeError("arm/hand release was not confirmed")
                except Exception as error:
                    cleanup_failed = True
                    failure = RuntimeError(str(failure or "task ended") + "; release: " + str(error))
            self.blocked = cleanup_failed
            self.completed.set()
        if failure is not None:
            raise failure  # Runtime 将异常提交为原任务的 failed 终态。
        return {"status": "cancelled" if self.cancel.is_set() else "finished",
                "message": "pose held; resources released" if self.cancel.is_set() else "trajectory published; hand targets reached; resources released",
                "content_type": "application/json", "data": payload}

    def close(self):
        # 关闭进程前先用业务 stop；Runtime 等 execute 结束后才调用 close。
        self.cancel.set()
        if not self.completed.wait(5.0):
            raise TimeoutError("pose task has not stopped")
        if self.client is not None:
            self.client.close()


def create(context):
    # FRS Runtime 加载本文件后传入 context；topic.get 读取，update 上报。
    # 返回的对象接收后续 execute("run", {}) / execute("stop", {}) 调用。
    return ThumbsUp(context)
