#!/bin/bash


source ./configs/scripts/sh/functions.sh

project_path=$(get_project_path)

# ==============================================================================
# Vision installation
# ==============================================================================

tput clear
print_intro "Package installation"


# ------------------------------------------------------------------------------
# Select the installation type
# ------------------------------------------------------------------------------
echo -e "\nAvailable installation types:\n"

echo -e "Remote: if you want to install this package on the company's servers"
echo -e "Local: if you wont to install it on your local pc"

echo -e "\nSelect the installation type: "
PS3=": "

install_choices=("Remote" "Local" "Quit")
install_type=""
select fav in "${install_choices[@]}"; do
    case $fav in
        "Remote")
            install_type=$fav
            break
            ;;
        "Local")
            install_type=$fav
            break
            ;;
        "Quit")
            echo "User requested exit"
            exit
            ;;
        *) echo "invalid option $REPLY";;
    esac
done

# ==============================================================================
# Creating custom files
# ==============================================================================

print_title "Creating custom files"

# ------------------------------------------------------------------------------
# Setup local file
# ------------------------------------------------------------------------------
setup_file="$project_path/configs/devel/setup.sh"
if [[ $install_type == "Remote" ]]; then
    setup_custom_file="$project_path/configs/devel/templates/setup_template_remote.sh"
elif [[ $install_type == "Local" ]]; then
    setup_custom_file="$project_path/configs/devel/templates/setup_template_local.sh"
fi

copy_file $setup_custom_file $setup_file


# ==============================================================================
# Parameters setup file
# ==============================================================================
print_title "Parameters configuration"

replace "MLP_PATH" $project_path $setup_file

configure=$(ask_yes_no "Configure the parameters?")

if [[ $configure == true ]]; then
    # General
    echo -e "\nGeneral:"
    confirm_or_replace "MLP_PATH" $setup_file "Project path"
    confirm_or_replace "MLP_DATA_PATH" $setup_file "Data local path"
    echo -e "\nDocker:"
    confirm_or_replace "MLP_PI_DOCKER_HOST" $setup_file "Host"
    confirm_or_replace "MLP_PI_DOCKER_PORT" $setup_file "Port"
    echo -e "\nMLFlow:"
    confirm_or_replace "MLP_MLFLOW_TRACKING_URI" $setup_file "Tracking URI"
    confirm_or_replace "MLP_MLFLOW_S3_ENDPOINT_URL" $setup_file "MinIO endpoint URL"
    confirm_or_replace "MLP_MLFLOW_S3_BUCKET_NAME" $setup_file "MinIO bucket name"
    echo -e "\nS3:"
    confirm_or_replace "MLP_PI_LOCAL_S3_HOST" $setup_file "Host"
    confirm_or_replace "MLP_PI_LOCAL_S3_PORT" $setup_file "Port"
    confirm_or_replace "AWS_ACCESS_KEY_ID" $setup_file "Access key id"
    confirm_or_replace "AWS_SECRET_ACCESS_KEY" $setup_file "Secret access key"
fi


export PYTHONPATH="${PYTHONPATH}:${MLP_PATH}"

print_ok "done"

# ==============================================================================
# Installation completed
# ==============================================================================

print_title "${GREEN}$install_type installation completed${NC}"

echo -e "\nPaste these lines into your environment file (e.g. ~/.bashrc):\n"
echo -e "${CYAN}source $setup_file${NC}"
echo -e "${CYAN}export PYTHONPATH=\"\${PYTHONPATH}:\${MLP_PATH}/src/\"${NC}\n"
