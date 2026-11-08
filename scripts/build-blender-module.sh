#!/usr/bin/env bash
# Builds the _mesheal Python module against Blender's bundled Python and copies it into
# blender/mesheal_tools/. Dev build for macOS; set BLENDER_PYTHON to override the interpreter.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${ROOT}/build_python"
DEST="${ROOT}/blender/mesheal_tools"

PY="${BLENDER_PYTHON:-}"
if [[ -z "${PY}" ]]; then
    # Newest Blender version directory, then its python3.* binary.
    for cand in $(ls -d /Applications/Blender.app/Contents/Resources/*/python/bin/python3.* 2>/dev/null | sort -rV); do
        if [[ -x "${cand}" && "${cand}" != *-config ]]; then
            PY="${cand}"
            break
        fi
    done
fi
if [[ -z "${PY}" || ! -x "${PY}" ]]; then
    echo "error: Blender's Python not found. Set BLENDER_PYTHON=/path/to/python3.x" >&2
    exit 1
fi
echo "Using Python: ${PY}"

# Blender's bundled Python ships pyconfig.h but not Python.h. Borrow the headers of another
# CPython with the same major.minor (the ABI is the same); override with PYTHON_INCLUDE_DIR.
PYVER="$("${PY}" -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
INC="${PYTHON_INCLUDE_DIR:-}"
if [[ -z "${INC}" ]]; then
    own="$("${PY}" -c 'import sysconfig; print(sysconfig.get_paths()["include"])')"
    if [[ -f "${own}/Python.h" ]]; then
        INC="${own}"
    else
        for cand in "${HOME}"/.local/share/uv/python/cpython-${PYVER}*/include/python${PYVER} \
                    /opt/homebrew/opt/python@${PYVER}/Frameworks/Python.framework/Versions/${PYVER}/include/python${PYVER}; do
            if [[ -f "${cand}/Python.h" ]]; then INC="${cand}"; break; fi
        done
    fi
fi
if [[ -z "${INC}" ]]; then
    echo "error: no Python.h for Python ${PYVER}. Set PYTHON_INCLUDE_DIR (e.g. from 'uv python install ${PYVER}')." >&2
    exit 1
fi
echo "Using Python headers: ${INC}"

cmake -S "${ROOT}" -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DMESHEAL_BUILD_GUI=OFF \
    -DMESHEAL_BUILD_PYTHON=ON \
    -DMESHEAL_USE_TBB=OFF \
    -DMESHEAL_BUILD_TESTS=ON \
    -DPython_EXECUTABLE="${PY}" \
    -DPython_INCLUDE_DIR="${INC}"

cmake --build "${BUILD_DIR}" -j 2 --target _mesheal

cp -f "${BUILD_DIR}"/src/python/_mesheal*.so "${DEST}/"
echo "Copied module to ${DEST}:"
ls -1 "${DEST}"/_mesheal*.so
