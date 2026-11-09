#!/bin/sh
set -eu
# 路径相对模块目录；exec 保留前台进程和宿主管道，不能在后台启动。
cd "$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
exec ./build/cpp_thumbs_up
