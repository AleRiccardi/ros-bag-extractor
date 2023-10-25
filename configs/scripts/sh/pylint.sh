#!/bin/bash

##
# @brief Pylint script for local development.
# @author Alessandro Riccardi
# @version 0.1.0

source $RBE_PATH/configs/scripts/sh/functions.sh

path_rel=$1

if [[ $path_rel == "" ]]; then
    path_abs=$RBE_PATH
else
    path_abs="$PWD/$path_rel"
fi

files=$(find $path_abs -type f -name "*.py")
files_count=$(find $path_abs -type f -name "*.py" -printf x | wc -c)

files=$(find $path_abs -type f -name "*.py")

print_title "Pylint tool"
echo -e " - path: $path_abs"
echo -e " - num files: $files_count\n"

pylint $files 

echo -e ""
