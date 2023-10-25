#!/bin/bash

##
# @file functions.bash
# @brief A set of usefull functions.
# @author Alessandro Riccardi
# @version 0.0.0
# @date 2022-07-27


# Colors
RED='\e[0;31m'
GREEN='\e[0;32m'
YELLOW='\e[0;33m'
BLUE='\e[0;34m'
PURPLE='\e[0;35m'
CYAN='\e[0;36m'

NC='\e[0m'

function print_info() {
    echo -e "${PURPLE}${1}${NC}"
}

function print_green() {
    echo -e "${GREEN}${1}${NC}"
}

function print_ok() {
    echo -e "${GREEN}OK${NC} - $1"
}

function print_warning() {
    echo -e "${YELLOW}WARNING${NC} - $1"
}

function print_error() {
    echo -e "${RED}ERROR${NC} - $1"
}

function print_enter() {
    read -p "Enter to continue ..."
    # Erase previous line
    echo -e "\e[1A\e[K\e[1A\e[1A"
}

function fill_line() {
    local l=
    builtin printf -vl "%${2:-${COLUMNS:-$(tput cols 2>&- || echo 80)}}s" && echo -e "${l// /${1:-=}}"
}

function make_capital() {
    text=$(echo "${1}" | tr 'a-z' 'A-Z')
    echo $text
}

function print_intro() {
    text=$(make_capital "${1}")
    echo
    fill_line "="
    print_info " ${text}"
    fill_line "="
}

function print_title() {
    text=$(make_capital "${1}")
    echo
    print_info " ${text}"
    fill_line "-"
    echo
}

function print_success() {
    text=$(make_capital "${1}")
    echo
    print_green " ${text}"
    fill_line "-"
    echo
}


function get_value() {
    param="${1}="
    file_path=${2}

    value_raw=$(awk -F $param '{print $2}' ${file_path})
    value_raw=$(echo ${value_raw} | tr -d '\n' | tr -d '"')
    value=$(eval "echo $value_raw")

    echo $value
}

# Get the project directory path
function get_project_path() {
    file_path=$(realpath "$0")
    sh_path=$(dirname "$file_path")
    script_path=$(dirname "$sh_path")
    config_path=$(dirname "$script_path")
    project_path=$(dirname "$config_path")
    echo $project_path
}

function copy_file() {
    file_path_default=${1}
    file_path=${2}

    cp $file_path_default $file_path

    if [[ $? == 0 ]]; then
        print_ok "Created: $file_path"
    else
        print_error "Error copying the file $file_path_default to $file_path"
        exit 1
    fi
}

# Add backslash for each slash
function path_correction() {
    echo "${1//\//\\\/}"
}

function replace_pattern() {
    param=${1}
    value=${2}
    file_path=${3}

    value=$(path_correction $value)
    sed -i "s/$param/$value/g" $file_path
}

function replace() {
    param=${1}
    value=${2}
    file_path=${3}

    value=$(path_correction $value)
    from="$param=\"(.*?)\""
    to="$param=\"$value\""

    sed -i -E "s/$from/$to/g" $file_path
}

function type_n_replace() {
    param=${1}
    value=${2}
    message=" - ${3}"
    file_path=${4}

    read -p "$message [$value]: "

    value_tmp=${REPLY}
    if [[ $value_tmp != "" ]]; then
        value=$value_tmp
    fi

    replace $param $value $file_path
}


function read_n_replace() {
    param_from=${1}
    file_path_from=${2}
    param_to=${3}
    file_path_to=${4}

    value=$(get_value $param_from $file_path_from)
    replace_pattern $param_to $value $file_path_to
}

function confirm_or_replace() {
    param=${1}
    file_path=${2}
    message=${3}

    value=$(get_value $param $file_path)

    if [[ $message != "" ]]; then
        read -p " - $message [$value]: "
    fi

    value_tmp=${REPLY}
    if [[ $value_tmp != "" ]]; then
        value=$value_tmp
    fi

    replace $param $value $file_path
}

function ask_yes_no() {
    message=${1}

    answer=false
    while true; do
        read -p "$message [y/n]: " yn
        case $yn in
            [Yy]* )
                answer=true
                break
                ;;
            [Nn]* )
                answer=false
                break
                ;;
            * );;
        esac
    done

    echo $answer
}

function ask_continue() {
    message=${1}

    while true; do
        read -p "Continue? [y]: " yn
        case $yn in
            [Yy]* ) break ;;
            * ) ;;
        esac
    done
}

function service_is_running() {
    service_uri=${1}

    if [[ $(ss --tcp --numeric --all | grep $service_uri) == "" ]]; then
        echo false
    else
        echo true
    fi
}

function wait_for_service() {
    service_uri=${1}

    while [[ $(service_is_running $service_uri) == false ]]; do
        sleep 2
        wait
    done
}

function find_conda_env(){
    conda env list | grep "${1}"
}

function poetry_path() {
    path_env=$(poetry env list --full-path)
    result=$?

    if [[ $result != 0 ]]; then
        echo "Aborting."
        return $result
    fi


    if [[ $path_env =~ "Activate" ]]; then
        path_env=$(echo $path_env | grep Activated | cut -d' ' -f1 )
        result=$?

        if [[ $result != 0 ]]; then
            echo "Aborting."
            return $result
        fi
    fi
    echo $path_env
}
