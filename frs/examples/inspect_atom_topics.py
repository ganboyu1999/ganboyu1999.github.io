import argparse
import json
import os
from pathlib import Path
import sys

# 与调用脚本一样，明确选择已确认的安装前缀，防止 Python 混用旧 SDK。
if os.environ.get("FRS_PREFIX"):
    sdk_path = Path(os.environ["FRS_PREFIX"]) / "lib/python3/dist-packages"
    if not (sdk_path / "frs/__init__.py").is_file():
        raise RuntimeError("FRS_PREFIX does not contain the Python SDK")
    sys.path.insert(0, str(sdk_path))

import frs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("topic", nargs="?", default="/motion_ctrl/aurora/control_group_state")
    args = parser.parse_args()
    with frs.connect(socket_only=True) as robot:
        # 1. 只连本机 FRS Socket，先查目录；读取名和类型从目录取得，不猜物理 Topic。
        matches = [item for item in robot.topic.catalog() if item.name == args.topic]
        if len(matches) != 1:
            raise RuntimeError("topic is not in the FRS read catalog: " + args.topic)
        item = matches[0]
        print("name=" + item.name + " type=" + item.message_type)
        # 2. 普通客户端 wait 等待样本；Atom context.topic 没有这个 wait 方法。
        sample = robot.topic.wait(item.name, timeout=1.0)
        # 3. 打印完整解码消息和元数据，供开发者确定实际字段、序号和年龄。
        print(json.dumps({"available": sample.available, "quality": sample.frs.quality,
                          "age_ns": sample.frs.age_ns, "sequence": sample.frs.sequence,
                          "msg": sample.msg}, ensure_ascii=False, indent=2))
        # 4. 查询缓存统计；with 退出时关闭本次连接及其读取引用。
        print(json.dumps(robot.topic.health(item.name), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
