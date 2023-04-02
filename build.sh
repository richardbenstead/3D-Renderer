#!/bin/bash
set -euxo pipefail

BUILD_PATH=build
mkdir -p ${BUILD_PATH}
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo -B ${BUILD_PATH}
make -C ${BUILD_PATH}
