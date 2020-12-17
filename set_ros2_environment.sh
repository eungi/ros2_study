#!/bin/bash
# Authors: Eungi Cho

LOCALE_PATH=$PWD
source "$LOCALE_PATH"/bash/common.sh

################################################################################
# Parameters: version, path and options
################################################################################
TARGET_OS=("LinuxMint-19.3" "Ubuntu-18.04")
TARGET_ROS2_VERSION="dashing"

LOCALE_OS_NAME=`sudo sh -c 'sudo lsb_release -irdc | grep "Distributor ID:" | cut -d":" -f2'`
LOCALE_OS_NAME=$(echo ${LOCALE_OS_NAME} | cut -d $'\t' -f2)
LOCALE_OS_VERSION=`sudo sh -c 'sudo lsb_release -irdc | grep Release: | cut -d":" -f2'`
LOCALE_OS_VERSION=$(echo ${LOCALE_OS_VERSION} | cut -d $'\t' -f2)

if [ ! `os_check ${LOCALE_OS_NAME} ${LOCALE_OS_VERSION} ${TARGET_OS[@]}` == "true" ]; then
    cecho "[Error] $LOCALE_OS_NAME $LOCALE_OS_VERSION is not supported."
    exit 0
fi

################################################################################
# Note and default settings
################################################################################
cecho ""
cecho "[Note] PLEASE CHECK YOUR TARGET OS, ROS2 VERSION:"
cecho ">>> Target OS version = $LOCALE_OS_NAME-$LOCALE_OS_VERSION"
cecho ">>> Target ROS2 version = $TARGET_ROS2_VERSION"
cecho ""
cecho "[Note] If the target version and options are different,"
cecho "[Note] please modify the script and re-execute."
cecho ""
cecho "[Note] PRESS [ENTER] TO CONTINUE THE INSTALLATION"
cecho "[Note] IF YOU WANT TO CANCEL, PRESS [CTRL] + [C]"
read -r

cecho "[Note] Update package lists and upgrade them"
sudo apt update -y && sudo apt upgrade -y

cecho "[Note] Install basic tools"
sudo apt install -y \
    cmake \
    curl \
    g++ \
    htop \
    iftop \
    terminator \
    vim \
    wget

################################################################################
# Install ROS2
################################################################################
source "$LOCALE_PATH"/bash/ros2_install.sh

set_locale
add_ros2_apt_repository
install_ros2_packages $TARGET_ROS2_VERSION

# check result of build
if [[ "$?" -ne 0 ]] ; then
    cecho "[Failed installing ROS2!]"; exit
else
    cecho "[Finished installing ROS2!]"
fi
