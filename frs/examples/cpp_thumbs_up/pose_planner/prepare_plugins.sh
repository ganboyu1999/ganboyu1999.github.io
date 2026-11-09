#!/bin/sh
set -eu
# 在 Atom 工程根目录执行。链接整套配套依赖，不复制或修改系统目录。
FBA_PREFIX="${FBA_PREFIX:-/opt/fftai/fourier_base_atom}"
test -f "$FBA_PREFIX/plugins/libfba_movetools.so"
test -f "$FBA_PREFIX/plugins/movetools/artifact.json"
test -f "$FBA_PREFIX/config/planning/gr3v2_2_4/robot_config.yaml"
python3 - "$FBA_PREFIX/plugins/libfba_movetools.so" <<'PY'
import ctypes
import sys
plugin = ctypes.CDLL(sys.argv[1])
plugin.fba_movetools_version.restype = ctypes.c_uint
if plugin.fba_movetools_version() != 3:
    raise SystemExit("FBA MoveTools plugin requires ABI 3")
print("FBA MoveTools ABI 3: ready")
PY
mkdir plugins
ln -s "$FBA_PREFIX/plugins/libfba_movetools.so" plugins/libfba_movetools.so
ln -s "$FBA_PREFIX/plugins/movetools" plugins/movetools
ln -s "$FBA_PREFIX/config/planning" planning
