#!/bin/bash

# ------------------------------- Usage ---------------------------------
# Script to build the codebase
#
# Usage: ./build.sh [-w <workers>] [-j <jobs>] [package1] [package2] ...
#
# Flags:
#   -w Number of cores
#   -j Number of jobs, increase this if you have RAM at your disposal
#
# Optimal settings depend on the number of cores, the amount of memory
# and the I/O-system of your machine.
#
# ----------------------------- examples --------------------------------
# Simply build all packages
#   Example: ./build.sh
#
# Build specific packages
#   Example: ./build.sh package1 package2
#
# Overwrite number of workers and jobs
#   Example: ./build.sh -w 4 -j 8
#
# Or a combination of both
#   Example: ./build.sh -w 4 -j 8 package1 package2
# -----------------------------------------------------------------------

# default workers and jobs
WORKERS=3
JOBS=2

# Overwrite default values with command line arguments (if any)
while getopts ":w:j:" Option; do
	case $Option in
	w)
		echo "Workers specified in command, set to $OPTARG"
		WORKERS=$OPTARG
		;;
	j)
		echo "Jobs specified in command, set to $OPTARG"
		JOBS=$OPTARG
		;;
	esac
done

shift $(($OPTIND - 1))
echo "date: $(date)"
echo "workers: ${WORKERS}"
echo "jobs: ${JOBS}"
if [ $# -eq 0 ]; then
	echo "packages: nothing provided, building all packages"
else
	echo "packages: $@"
fi

BUILD_TYPE=RelWithDebInfo

# export CC=clang
# export CXX=clang++

MAKEFLAGS=-j${JOBS} colcon build \
	--cmake-force-configure \
	--merge-install \
	--symlink-install \
	--install-base install \
	--build-base build \
	--parallel-workers ${WORKERS} \
	--cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DBUILD_TESTING=ON" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
	-Wall -Wextra -Wpedantic \
	--packages-up-to "$@"

EXIT_CODE=$?

# If the build was successful, run the source command to source the workspace
if [ $EXIT_CODE -eq 0 ]; then
    . install/setup.sh
fi

echo "date: $(date)"

# Exit the script with the same exit code as colcon build
exit $EXIT_CODE