#!/usr/bin/env bash

usage_exit() {
	echo "Usage: ${0} [-u]" 1>&2
	exit 1
}

COVERAGE_FLAGS="-fprofile-arcs -ftest-coverage -DCOVERAGE_RUN=1"
FLAG_U=0

while getopts uh OPT
do
	case ${OPT} in
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

colcon build \
	--ament-cmake-args \
	  -DCMAKE_CXX_FLAGS="${COVERAGE_FLAGS}" \
	  -DCMAKE_C_FLAGS="${COVERAGE_FLAGS}"
lcov --config-file .lcovrc --base-directory ${PWD} --capture --directory build -o lcov.base --initial > log/latest_lcov_stdout.logs
colcon test \
	--return-code-on-test-failure

mv log/latest_lcov_stdout.logs log/latest/lcov_stdout.logs  # 'latest' will be the latest test job
lcov --config-file .lcovrc --base-directory ${PWD} --capture --directory build -o lcov.test >> log/latest/lcov_stdout.logs
lcov --config-file .lcovrc -a lcov.base -a lcov.test -o lcov.total >> log/latest/lcov_stdout.logs
lcov --config-file .lcovrc -r lcov.total \
	"*/AutowareAuto/install/*" "*/CMakeCCompilerId.c" "*/CMakeCXXCompilerId.cpp" "*_msgs/*" \
	"*/AutowareAuto/build/mpc_planner/*" "*/AutowareAuto/build/mpc_controller/*" \
	"*/AutowareAuto/src/external/*" \
	"*/AutowareAuto/build/recordreplay_planner_actions/*" \
	"*/AutowareAuto/build/*/rclcpp_components/*" \
	"*/AutowareAuto/src/*/test/*" \
	-o lcov.total.filtered >> log/latest/lcov_stdout.logs
genhtml --config-file .lcovrc -p ${PWD} --legend --demangle-cpp lcov.total.filtered -o coverage >> log/latest/lcov_stdout.logs
# The last four lines of the logs should have the coverage numbers
tail -n 4 log/latest/lcov_stdout.logs
