#!/bin/bash

# Script to run linters on the codebase
# Usage: ./lint.sh 
#

EXCLUDE_PATHS=("src/mitsubishi_driver/"
    "src/another_path"
)

overall_status_code=0

# Clear lint files
lint_clear() {
    find lint/ -type f ! -name '.gitignore' -delete
}

# Filter source files based on exclude paths
filter_source_files() {
    local source_files=("$@")
    local filtered_source_files=()
    for file in "${source_files[@]}"; do
        exclude=false
        for exclude_path in "${EXCLUDE_PATHS[@]}"; do
            if [[ $file == $exclude_path* ]]; then
                exclude=true
                break
            fi
        done
        if [ "$exclude" = false ]; then
            filtered_source_files+=("$file")
        fi
    done
    echo "${filtered_source_files[@]}"
}

# Run static code checker cppcheck on ROS packages
run_cppcheck() {
    echo -e "\e[1m--------------------------------------------------------\e[0m"
    echo -e "\e[1m------------------- Running cppcheck -------------------\e[0m"
    echo -e "\e[1m--------------------------------------------------------\e[0m"

    # Create an array of source files
    source_files=($(find src/ -name "*.cpp" -o -name "*.hpp" | grep -v 'test'))

    # Filter the source files
    source_files=($(filter_source_files "${source_files[@]}"))

    status_code=0
    if [ ${#source_files[@]} -eq 0 ]; then
        echo -e "\e[1mNo files found for cppcheck\e[0m"
    else 
        # Print the source files
        echo "The following files will be passed to the cppcheck executable: "
        for file in "${source_files[@]}"; do
            echo "$file"
        done

        start_time=$(date +%s)
        # Run cppcheck
        cppcheck --enable=all --std=c++17 --inline-suppr --quiet --suppressions-list=.cppcheck_suppressions.txt "${source_files[@]}"

        # set status code
        status_code=$?


        # print closing message
        end_time=$(date +%s)
        elapsed_time=$((end_time - start_time))
        echo -e "\e[1mCppcheck exited with status code: $status_code!\e[0m elapsed time: $(date -u -d @$elapsed_time +'%H:%M:%S')"
    fi

    # set overall status code
    if [ $status_code -ne 0 ]; then
        overall_status_code=1
    fi
}

# Lint files with clang_tidy
clang_tidy() {
    echo -e "\e[1m----------------------------------------------------------\e[0m"
    echo -e "\e[1m---------------------- clang_tidy ------------------------\e[0m"
    echo -e "\e[1m----------------------------------------------------------\e[0m"

    # Create an array of source files
    source_files=($(find src/ -name "*.cpp" -o -name "*.hpp" -o -name "*.inc" | grep -v 'test'))

    # Filter the source files
    source_files=($(filter_source_files "${source_files[@]}"))

    status_code=0
    if [ ${#source_files[@]} -eq 0 ]; then
        echo -e "\e[1mNo files found for clang_tidy\e[0m"
    else 

        # Print the source files
        echo "The following files will be passed to the clang-tidy executable: "
        for file in "${source_files[@]}"; do
            echo "$file"
        done

        echo -e "\e[1;33mPlease be patient, clangd-tidy takes a while to complete, this is normal.\e[0m"

        start_time=$(date +%s)
        # Run clangd-tidy
        clangd-tidy -p build/ "${source_files[@]}"
        # set status code
        status_code=$?

        # print closing message
        end_time=$(date +%s)
        elapsed_time=$((end_time - start_time))
        echo -e "\e[1mClang-tidy exited with status code: $status_code!\e[0m elapsed time: $(date -u -d @$elapsed_time +'%H:%M:%S')"
    fi

    # set overall status code
    if [ $status_code -ne 0 ]; then
        overall_status_code=1
    fi
}

run_cppcheck
clang_tidy

exit $overall_status_code
