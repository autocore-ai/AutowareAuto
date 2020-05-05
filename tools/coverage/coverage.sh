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
	git submodule update --init --recursive
fi

colcon build \
	--cmake-args \
       	  -DCMAKE_BUILD_TYPE=Debug \
	--ament-cmake-args \
	  -DCMAKE_CXX_FLAGS="${COVERAGE_FLAGS}" \
	  -DCMAKE_C_FLAGS="${COVERAGE_FLAGS}"
lcov --config-file .lcovrc --base-directory ${PWD} --capture --directory build -o lcov.base --initial
colcon test \
	--return-code-on-test-failure

lcov --config-file .lcovrc --base-directory ${PWD} --capture --directory build -o lcov.test
lcov --config-file .lcovrc -a lcov.base -a lcov.test -o lcov.total
lcov --config-file .lcovrc -r lcov.total \
	"*/AutowareAuto/install/*" "*/CMakeCCompilerId.c" "*/CMakeCXXCompilerId.cpp" "*_msgs/*" \
	"*/AutowareAuto/build/mpc_planner/*" "*/AutowareAuto/build/mpc_controller/*" \
	"*/AutowareAuto/src/external/*" \
	"*/AutowareAuto/build/recordreplay_planner_actions/*" \
	-o lcov.total.filtered
genhtml --config-file .lcovrc -p ${PWD} --legend --demangle-cpp lcov.total.filtered -o coverage
