#!/bin/bash


source ./configs/scripts/sh/functions.sh

project_path=$(get_project_path)

# ==============================================================================
# Vision installation
# ==============================================================================

tput clear
print_intro " RBE Package installation"

# ==============================================================================
# Installing poetry dependencies
# ==============================================================================

print_title "Installing poetry dependencies"

poetry install

env_path=$(poetry_path)

# ==============================================================================
# Creating custom files
# ==============================================================================

print_title "Creating custom files"

# ------------------------------------------------------------------------------
# Setup local file
# ------------------------------------------------------------------------------
setup_file="$project_path/configs/devel/setup.sh"
setup_custom_file="$project_path/configs/devel/templates/setup_template.sh"

copy_file $setup_custom_file $setup_file


# ==============================================================================
# Parameters setup file
# ==============================================================================
print_title "Parameters configuration"

replace "RBE_PATH" $project_path $setup_file
replace "RBE_ENV_PATH" $env_path $setup_file

configure=$(ask_yes_no "Configure the parameters?")

if [[ $configure == true ]]; then
    # General
    echo -e "\nGeneral:"
    confirm_or_replace "RBE_PATH" $setup_file "Project path"
    confirm_or_replace "RBE_DATA_PATH" $setup_file "Data local path"
    confirm_or_replace "RBE_ENV_PATH" $setup_file "Poetry env path"
fi


export PYTHONPATH="${PYTHONPATH}:${RBE_PATH}"

print_ok "done"

# ==============================================================================
# Installation completed
# ==============================================================================

print_success "Installation completed"

echo -e "Paste these lines into your environment file (e.g. ~/.bashrc):\n"
echo -e "${CYAN}source $setup_file${NC}"
echo -e "${CYAN}export PYTHONPATH=\"\${PYTHONPATH}:\${RBE_PATH}/src/\"${NC}"
echo -e "${CYAN}export PYTHONPATH=\"\${PYTHONPATH}:\${RBE_ENV_PATH}/lib/python3.10/site-packages/\"${NC}\n"
