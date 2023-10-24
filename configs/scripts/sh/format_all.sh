#!/bin/bash

source $MLP_PATH/configs/scripts/sh/functions.sh

path_rel=$1

if [[ $path_rel == "" ]]; then
    path_abs=$MLP_PATH
else
    path_abs="$PWD/$path_rel"
fi

files=$(find $path_abs -type f -name "*.py")
files_count=$(find $path_abs -type f -name "*.py" -printf x | wc -c)

print_title "Black | Isort | Autoflake | formatter tool"
echo -e " - path: $path_abs"
echo -e " - num files: $files_count"

echo -e "\n - Autoflake"
autoflake --in-place --verbose --remove-unused-variables --remove-all-unused-imports --ignore-init-module-imports  $files
echo -e " -> Done"

echo -e "\n - Black"
black $files
echo -e " -> Done"

echo -e "\n - Isort"
isort $files
echo -e " -> Done"
