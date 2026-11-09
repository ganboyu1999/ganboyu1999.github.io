import argparse
import base64
import json
import math
import os
from pathlib import Path
import sys
import time

# FRS_PREFIX 指向本次连接所用的 SDK。系统 sitecustomize 可能优先插入另一套
# 预装包；在 import 前显式选择，避免同时安装多个版本时混用协议。
if os.environ.get("FRS_PREFIX"):
    sdk_path = Path(os.environ["FRS_PREFIX"]) / "lib/python3/dist-packages"
    if not (sdk_path / "frs/__init__.py").is_file():
        raise RuntimeError("FRS_PREFIX does not contain the Python SDK")
    sys.path.insert(0, str(sdk_path))

import frs


def submit(robot, atom_id, command, parameters):
    # 1. 先分配 ID，再用同一个 ID 调用；后续进度和终态都按这个 ID 查询。
    request_id = robot.atom.allocate_request(atom_id)
    print(command + " request_id=" + request_id, flush=True)
    receipt = robot.atom.call(frs.make_atom_call_request(request_id, atom_id, command,
                                                json.dumps(parameters, ensure_ascii=False)))
    # result_code=0 只表示受理，不表示执行完成；超时也不能盲目重复提交。
    if receipt.result_code() != 0:
        raise RuntimeError(receipt.message())
    return request_id


def run(robot, atom_id, command, parameters, cancel_after=None, timeout=30.0):
    if cancel_after is not None and (not math.isfinite(cancel_after) or cancel_after < 0):
        raise ValueError("cancel_after must be finite and nonnegative")
    if cancel_after is not None and command != "run":
        raise ValueError("cancellation example requires run")
    request_id = submit(robot, atom_id, command, parameters)
    begin = time.monotonic()
    stop_id = None
    previous = None
    while time.monotonic() - begin < timeout:
        # 2. include_history 保留阶段报告；data 是当前最新报告。
        result = robot.atom.task_result(request_id, include_history=True)
        state = result.get("data", {})
        change = (state.get("state"), state.get("progress"), state.get("message"))
        if change != previous:
            print(json.dumps({"request_id": request_id, "state": change[0],
                              "progress": change[1], "message": change[2]}, ensure_ascii=False), flush=True)
            previous = change
        if cancel_after is not None and stop_id is None and state.get("message") in {"moving", "settling"} and time.monotonic() - begin >= cancel_after:
            # 3. 看到实际出帧后发送另一项 stop 请求，不覆盖原任务 ID。
            stop_id = submit(robot, atom_id, "stop", {})
        if state.get("state") in {"idle", "failed", "stopped"}:
            # 4. 原任务到终态后仍要等待 stop 结果，确认业务和资源收尾已经完成。
            if stop_id:
                stopped = robot.atom.task_result(stop_id)
                stop_state = stopped.get("data", {})
                if stop_state.get("state") not in {"idle", "failed", "stopped"}:
                    time.sleep(.02)
                    continue
                if stop_state.get("result_code") != 0:
                    raise RuntimeError("stop failed: " + stop_state.get("message", ""))
            if state.get("state") == "failed" or state.get("state") != "stopped" and state.get("result_code") != 0:
                raise RuntimeError(state.get("message", "task failed"))
            # 5. 查询协议中的 bytes 是 base64；本例约定 content_type 为 JSON。
            encoded = state.get("data", {}).get("value", "")
            return {"request_id": request_id, "stop_request_id": stop_id,
                    "task": result, "decoded_data": json.loads(base64.b64decode(encoded)) if encoded else None}
        if result.get("availability") in {"expired", "not_found"}:
            raise RuntimeError("task report is " + result["availability"])
        time.sleep(.02)
    # 查询超时留下原 ID，让开发者继续查结果，不能据此再次执行同一业务。
    raise TimeoutError("query the original request before resubmitting: " + request_id)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("atom_id", choices=("cpp_thumbs_up", "python_thumbs_up"))
    parser.add_argument("--cancel-after", type=float)
    args = parser.parse_args()
    with frs.connect(socket_only=True) as robot:
        print(json.dumps(run(robot, args.atom_id, "run", {}, args.cancel_after),
                         ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
