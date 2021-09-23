#!/usr/bin/env bash

# calculate test coverage for packages that are modified compared to the 'master' branch

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

# get all modified packages (relative to the 'master' branch)
all_pkgs=`colcon list`
all_modified_files=`git diff --name-only origin/master`
all_modified_pkgs=()
while IFS= read -r file; do
	while IFS= read -r pkg; do
		pkg_path=$(echo ${pkg} | awk '{print $2}')
		match=`echo $file | grep $pkg_path/`  # add / to guarantee an exact subpath match
		if [ ! -z ${match} ]; then
			pkg_name=$(echo ${pkg} | awk '{print $1}')
      echo ${pkg}
      echo ${match}
      echo ${pkg_name}
			all_modified_pkgs+=( $pkg_name )
		fi
	done < <(printf '%s\n' "$all_pkgs")
done < <(printf '%s\n' "$all_modified_files")

if [ ${#all_modified_pkgs[@]} -eq 0 ]; then
    echo "No modified packages. Skipping coverage."
    exit 0
fi
PKGS=${all_modified_pkgs[@]}
echo "Modified packages: ${PKGS[@]}"

OPTS=("--config-file" ".lcovrc" "--base-directory" "${PWD}" "--no-external" "--capture")
for pkg in ${PKGS[@]}; do
  OPTS+=( "--directory" "build/$pkg" )
done

set -ex

if [ ${SKIP_BUILD} -eq 0 ]; then
  colcon build \
	--packages-up-to ${PKGS[@]} \
    --ament-cmake-args \
      -DCMAKE_CXX_FLAGS="${COVERAGE_FLAGS}" \
      -DCMAKE_C_FLAGS="${COVERAGE_FLAGS}" \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo
  lcov ${OPTS[@]} -o lcov.base --initial > log/latest_lcov_stdout.logs
fi

if [ ${SKIP_TEST} -eq 0 ]; then
  colcon test \
	--packages-select ${PKGS[@]} \
    --return-code-on-test-failure \
    --packages-skip $(grep -h -r -o -P '(?<=\<name\>).*(?=\<\/name\>)' $(find src/external -name package.xml) | sort)

  mv log/latest_lcov_stdout.logs log/latest/lcov_stdout.logs  # 'latest' will be the latest test job
  lcov ${OPTS[@]} --capture -o lcov.test >> log/latest/lcov_stdout.logs
  if [ ! -s lcov.test ]; then
    echo "No coverage result. Modified packages may not contain any test."
    exit 0
  fi
  lcov -a lcov.base -a lcov.test -o lcov.total >> log/latest/lcov_stdout.logs
  lcov -r lcov.total \
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
