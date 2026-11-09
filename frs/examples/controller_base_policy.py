import json
import time
import uuid

import frs


def run_stand(robot, duration: float = 0.2, timeout: float = 10.0):
    if robot.system.runtime_mode() != "sim":
        raise RuntimeError("run this example in simulation")
    api = robot.controllers
    request_id = "tutorial-stand-" + uuid.uuid4().hex
    print("request_id=" + request_id, flush=True)
    started = False
    try:
        api.start("base_policy")
        started = True
        api.switch("base_policy")
        receipt = api.run("base_policy", "stand", {"duration_s": duration}, request_id=request_id)
        if receipt.get("fault"):
            raise RuntimeError(receipt.get("fault_reason", "Controller rejected the request"))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            result = api.result(request_id)
            status = result.get("status", {})
            if status.get("state") == "idle" and status.get("can_switch"):
                if result.get("result_code") != 0:
                    raise RuntimeError(result.get("message", "Controller task failed"))
                return result
            if status.get("fault") or status.get("state") == "failed":
                raise RuntimeError(status.get("fault_reason", "Controller failed"))
            time.sleep(0.05)
        raise TimeoutError("Controller did not complete: " + request_id)
    finally:
        if started:
            api.stop("base_policy")


def main():
    with frs.connect(socket_only=True) as robot:
        print(json.dumps(run_stand(robot), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
