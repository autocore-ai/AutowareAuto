#!/usr/bin/env bash

usage_exit() {
  echo "Usage: ${0} [-b -t -u]" 1>&2
  exit 1
}

COVERAGE_FLAGS="-fprofile-arcs -ftest-coverage -DCOVERAGE_RUN=1"
SKIP_BUILD=0
SKIP_TEST=0
FLAG_U=0
AA_PATH=`dirname $(dirname $(dirname $(realpath $0)))`

while getopts btuh OPT
do
  case ${OPT} in
    b)  SKIP_BUILD=1
      ;;
    t)  SKIP_TEST=1
      ;;
    u)  FLAG_U=1
      ;;
    h)  usage_exit
      ;;
    \?) usage_exit
      ;;
  esac
done

shift $((OPTIND - 1))

if [ ${FLAG_U} -eq 1 ]; then
  apt-get update && apt-get -y --no-install-recommends install git clang-tidy
fi

set -ex

if [ ${SKIP_BUILD} -eq 0 ]; then
  colcon build \
    --ament-cmake-args \
      -DCMAKE_CXX_FLAGS="${COVERAGE_FLAGS}" \
      -DCMAKE_C_FLAGS="${COVERAGE_FLAGS}" \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo
  lcov --config-file .lcovrc --base-directory ${PWD} --capture --directory build -o lcov.base --initial > log/latest_lcov_stdout.logs
fi

if [ ${SKIP_TEST} -eq 0 ]; then
  colcon test \
    --return-code-on-test-failure \
    --packages-skip $(grep -h -r -o -P '(?<=\<name\>).*(?=\<\/name\>)' $(find src/external -name package.xml) | sort)

  mv log/latest_lcov_stdout.logs log/latest/lcov_stdout.logs  # 'latest' will be the latest test job
  lcov --config-file .lcovrc --base-directory ${PWD} --capture --directory build -o lcov.test >> log/latest/lcov_stdout.logs
  lcov --config-file .lcovrc -a lcov.base -a lcov.test -o lcov.total >> log/latest/lcov_stdout.logs
  lcov --config-file .lcovrc -r lcov.total \
    "${AA_PATH}/build/*" \
    "${AA_PATH}/install/*" \
    "${AA_PATH}/src/*/test/*" \
    "${AA_PATH}/src/external/*" \
    "*/CMakeCCompilerId.c" \
    "*/CMakeCXXCompilerId.cpp" \
    "*_msgs/*" \
    -o lcov.total.filtered >> log/latest/lcov_stdout.logs
  genhtml --config-file .lcovrc -p ${PWD} --legend --demangle-cpp lcov.total.filtered -o coverage >> log/latest/lcov_stdout.logs
  # The last four lines of the logs should have the coverage numbers
  tail -n 4 log/latest/lcov_stdout.logs
fi
