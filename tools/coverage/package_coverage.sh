#!/usr/bin/env bash

usage_exit() {
	echo "Usage: ${0} PACKAGE_NAME" 1>&2
	exit 1
}

COVERAGE_FLAGS="-fprofile-arcs -ftest-coverage -DCOVERAGE_RUN=1"
AA_PATH=`dirname $(dirname $(dirname $(realpath $0)))`

if [ $# -eq 0 ]
	then
		usage_exit
fi

declare -a dirs=("build" "install" "log" "lcov")
for i in "${dirs[@]}"
do
	if [ -d "$i" ]; then
		read -p "'$i' directory exists. Delete it? " -n 1 -r
		echo
		if [[ $REPLY =~ [Yy]$ ]]
		then
			rm -rf $i
		fi
	fi
done

if [ ! -d lcov ]; then
	mkdir lcov
fi

# Build with correct flags
colcon build \
	--packages-up-to $1 \
	--ament-cmake-args \
	  -DCMAKE_CXX_FLAGS="${COVERAGE_FLAGS}" \
	  -DCMAKE_C_FLAGS="${COVERAGE_FLAGS}" || { echo "Build failed." ; exit 1; }

# Get a zero-coverage baseline
lcov \
	--config-file .lcovrc \
	--base-directory "$(pwd)" \
	--capture \
	--directory "$(pwd)/build/$1" \
	-o "$(pwd)/lcov/lcov.base" \
	--initial || { echo "Zero baseline coverage failed."; exit 1; }

# Run unit and integration tests
colcon test \
  --packages-select $1 \
	--return-code-on-test-failure || { echo "Unit/integration testing failed."; exit 1; }

# Get coverage
lcov \
	--config-file .lcovrc \
	--base-directory "$(pwd)" \
	--capture \
	--directory "$(pwd)/build/$1" \
	--output-file "$(pwd)/lcov/lcov.run" || { echo "Coverage generation failed."; exit 1; }

# Combine zero-coverage with coverage information.
lcov \
	--config-file .lcovrc \
  -a "$(pwd)/lcov/lcov.base" \
  -a "$(pwd)/lcov/lcov.run" \
  -o "$(pwd)/lcov/lcov.total" || { echo "Coverage combination failed."; exit 1; }

# Filter test, build, and install files and generate html
lcov --config-file .lcovrc -r "$(pwd)/lcov/lcov.total" \
	"${AA_PATH}/build/*" \
	"${AA_PATH}/install/*" \
	"${AA_PATH}/src/external/*" \
        "${AA_PATH}/src/*/test/*" \
        "*/CMakeCCompilerId.c" \
        "*/CMakeCXXCompilerId.cpp" \
        "*_msgs/*" \
	-o "$(pwd)/lcov/lcov.total.filtered" || { echo "Filtering failed."; exit 1; }

genhtml \
	--config-file .lcovrc \
	-p "$(pwd)" \
	--legend \
	--demangle-cpp \
	"$(pwd)/lcov/lcov.total.filtered" \
	-o "$(pwd)/lcov/" || { echo "HTML generation failed."; exit 1; }

echo "Check lcov/index.html to see the coverage report."
