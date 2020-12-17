#!/bin/bash
# Authors: Eungi Cho

LOCALE_PATH=$PWD
source "$LOCALE_PATH"/bash/common.sh

cecho ""
cecho "[Install ROS2]"

function set_locale()
{
    cecho "[Note] Set locale environment (en_US.UTF-8)"
    sudo apt install locales -y
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
}

function add_ros2_apt_repository()
{
    cecho ""
    cecho "[Note] Install curl gnupg2, lsb-release to setup sources"
    sudo apt update && sudo apt install curl gnupg2 lsb-release -y

    cecho ""
    cecho "[Note] Add ROS2 key"
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

    cecho ""
    cecho "[Note] Add the ROS2 repository"
    sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu bionic main" > /etc/apt/sources.list.d/ros2-latest.list'

    cecho ""
    cecho "[Note] Update the package lists and upgrade them"
    sudo apt update -y && sudo apt upgrade -y
}

function install_ros2_packages()
{
    ROS_2_VERSION=$1

    sudo apt update -y

    cecho ""
    cecho "[Note] Install ROS2"
    sudo apt install ros-$ROS_2_VERSION-desktop -y

    cecho "[Note] Install argcomplete to command line autocompletion"
    sudo apt install -y \
        python3-argcomplete \
        python3-colcon-common-extensions \
        ros-dashing-launch-testing \
        ros-dashing-launch-testing-ament-cmake

    cecho "[Note] Create local development environment"
    mkdir -p ~/ros2_ws/src
    cp -r "$LOCALE_PATH"/test_codes/* ~/ros2_ws/src
    source ~/.bashrc
    cd ~/ros2_ws
    colcon build --symlink-install
    source ~/.bashrc

    sh -c "echo \"\" >> ~/.bashrc"
    sh -c "echo \"### Set the ROS2 environment\" >> ~/.bashrc"
    sh -c "echo \"source /opt/ros/$ROS_2_VERSION/setup.bash\" >> ~/.bashrc"
    sh -c "echo \"source ~/ros2_ws/install/local_setup.bash\" >> ~/.bashrc"

    sh -c "echo \"\" >> ~/.bashrc"
    sh -c "echo \"### Set the alias commands for test\" >> ~/.bashrc"
    sh -c "echo \"alias testpub='ros2 run demo_nodes_cpp talker'\" >> ~/.bashrc"
    sh -c "echo \"alias testsub='ros2 run demo_nodes_cpp listener'\" >> ~/.bashrc"
    sh -c "echo \"alias testpubimg='ros2 run image_tools cam2image'\" >> ~/.bashrc"
    sh -c "echo \"alias testsubimg='ros2 run image_tools showimage'\" >> ~/.bashrc"

    sh -c "echo \"\" >> ~/.bashrc"
    sh -c "echo \"### Set the alias commands for easy build\" >> ~/.bashrc"
    sh -c "echo \"alias sr='source ~/.bashrc'\" >> ~/.bashrc"
    sh -c "echo \"alias dw='cd ~/ros2_ws && rm -rf build/ install/ log/'\" >> ~/.bashrc"
    sh -c "echo \"alias cw='cd ~/ros2_ws'\" >> ~/.bashrc"
    sh -c "echo \"alias cs='cd ~/ros2_ws/src'\" >> ~/.bashrc"
    sh -c "echo \"alias cb='cd ~/ros2_ws && colcon build --symlink-install'\" >> ~/.bashrc"
    sh -c "echo \"alias cbs='colcon build --symlink-install'\" >> ~/.bashrc"
    sh -c "echo \"alias cbp='colcon build --symlink-install --packages-select'\" >> ~/.bashrc"
    sh -c "echo \"alias cbu='colcon build --symlink-install --packages-up-to'\" >> ~/.bashrc"

    source ~/.bashrc
}
