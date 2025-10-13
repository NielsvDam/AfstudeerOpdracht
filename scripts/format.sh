#!/bin/bash
# Script to format code with Clang Format and CMake Format
# Usage: ./format.sh [file|all]
# If the argument is a path to a file, only that file will be formatted.
# If the argument is 'all', all files will be formatted, excluding specified paths.

arg1="$1"

EXCLUDE_PATHS=(
	"build/"
	"install/"
)

CPP_EXTENSIONS=(
	"cpp"
	"hpp"
	"c"
	"h"
	"cc"
	"hh"
	"imp"
	"inc"
)

# Functie om te controleren of een pad is uitgesloten
is_excluded() {
	local file="$1"
	for exclude in "${EXCLUDE_PATHS[@]}"; do
		if [[ $file == *$exclude* ]]; then
			echo "Excluded: $file"
			return 0
		fi
	done
	return 1
}

# Format all code with Clang Format, excluding specified paths
format_with_clang_format() {
	if [ "$arg1" == "all" ]; then
		for ext in "${CPP_EXTENSIONS[@]}"; do
			for file in $(find src/ -type f -name "*.$ext"); do
				if ! is_excluded "$file"; then
					echo "Formatting $file"
					clang-format -i "$file"
				fi
			done
		done
	else
		echo "Formatting $arg1"
		clang-format -i $arg1
	fi
	echo -e "\e[1mClang format done!\e[0m"
}

# Format CMakeLists.txt with CMake Format
format_with_cmake_format() {
	if [ "$arg1" == "all" ]; then
		for file in $(find . -type f -name 'CMakeLists.txt'); do
			if ! is_excluded "$file"; then
				echo "Formatting $file"
				cmake-format $file --config-file .cmake-format -i
			else
				echo "Excluded $file"
			fi
		done
	else
		echo "formatted $arg1"
		cmake-format $arg1 --config-file .cmake-format -i
	fi
	echo -e "\e[1mCMake format done!\e[0m"
}

clear # Clear the terminal
if [ "$arg1" == "all" ]; then
	format_with_clang_format
	format_with_cmake_format
elif [ -f "$arg1" ]; then
	fileName=$(basename "$arg1")
	fileExtension="${arg1##*.}"
	fileExtension="${fileExtension,,}" # convert to lowercase
	case "$fileExtension" in
	cpp | hpp | c | h | cc | hh | imp | inc)
		format_with_clang_format
		;;
	txt)
		if [ "$fileName" == "CMakeLists.txt" ]; then
			format_with_cmake_format
		else
			echo -e "\e[31mUnsupported txt file. Only CMakeLists.txt is supported.\e[0m"
		fi
		;;
	*)
		echo -e "\e[31mUnsupported file type. Only cpp, hpp, c, h, cc, hh, imp, inc, and CMakeLists.txt files are supported.\e[0m"
		;;
	esac
else
	echo -e "\e[31mNo file provided or file does not exist.\e[0m"
fi
