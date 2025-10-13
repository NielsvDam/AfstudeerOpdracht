#!/bin/bash

set -e

if [ -f install/setup.bash ]; then source install/setup.bash; fi

colcon test-result --delete-yes

export GTEST_COLOR=1 # otherwise colored output is not shown for gtest, this might be handy for debugging

if [ $# -eq 0 ]; then
  # No arguments provided, test all packages
  colcon test --merge-install --return-code-on-test-failure --event-handlers console_direct+ store_result+ console_stderr-
else
  # Test specific packages
  colcon test "$@" --merge-install --return-code-on-test-failure --event-handlers console_direct+ store_result+ console_stderr-
fi

EXIT_CODE=$?

# Exit the script with the same exit code as colcon build
exit $EXIT_CODE