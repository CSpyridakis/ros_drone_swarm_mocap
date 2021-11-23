#!/bin/bash

# TODO: edit them
WIFI_SSID=""
WIFI_PASS=""

ros_version=`echo $ROS_DISTRO`

__INST (){
    sudo apt-get -y install ros-${ros_version}-$@
}

download_imu_package(){
    (cd ~/catkin_ws/src/ && git clone https://github.com/CSpyridakis/ros-adafruit-10dof-imu.git)
}

install_ros_packages(){
    __INST usb-cam
    __INST image-proc
    __INST web-video-server
    __INST nmea-navsat-driver
    __INST robot-localization
    __INST image-transport
    __INST tf2
    __INST tf2_ros
}

basic_setup(){
    # Disable gui on boot
    sudo systemctl set-default multi-user
    gnome-session-quit

    # Disable suspend
    sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target

    # Connect to wifi
    if [ ! -e ${WIFI_SSID} ] && [ ! -e ${WIFI_PASS} ] ; then
        echo "Connect to [${WIFI_SSID}] with password [${WIFI_PASS}]"
        nmcli device wifi connect ${WIFI_SSID} password ${WIFI_PASS} hidden yes
    fi
}

setup_uart(){
    # 1. Add ‘enable_uart=1’ to /boot/config.txt file
    sudo bash -c ’echo "enable_uart=1" >> /boot/config.txt’
    
    # 2. Remove ‘console=serial0,115200’ from /boot/firmware/cmdline.txt
    sudo sed -e "s/console=serial0,115200//g" -i /boot/firmware/cmdline.txt
    
    # 3. Disable serial console service
    sudo systemctl stop serial-getty@ttyS0.service
    sudo systemctl disable serial-getty@ttyS0.service

    # 4. Give privileges to user
    sudo adduser $USER tty
    sudo adduser $USER dialout
    sudo chmod g+r /dev/ttyS0
}

setup_i2c(){
    # 1. Install needed library
    sudo apt-get install -y libi2c-dev i2c-tools
    
    # 2. Give privileges to user
    sudo adduser $USER i2c
    sudo chmod g+r /dev/i2c-1
}

doAll(){
    download_imu_package
    setup_i2c
    setup_uart
    install_ros_packages
}

help_menu(){
    echo "Usage: $0 [Option] [Option] ..."
    echo "Setup your Ubuntu environment in order to run this ROS package"
    echo 
    echo "Options:"
    echo "  -h,  --help                 show this help page"
    echo "  -b,  --basic                basic setup (connect to wifi, disable autosuspend, etc.)"
    echo "       --ssid                 wifi's SSID"
    echo "       --pass                 wifi's password"
    echo "  -i,  --imu-package          download needed IMU ROS package from GitHub"
    echo "  -p,  --ros-packages         install all ROS requirement packages from APT"
    echo "  -U,  --setup-uart           setup UART to read messages from there"
    echo "  -I,  --setup-i2c            download needed lib and setup I2C to read messages from there"
    echo "  -A,  --all                  do all above actions"
}

#MAIN
while :
do
    case "$1" in
        -i | --imu-package)     download_imu_package    ; shift ;;
        -p | --ros-packages)    install_ros_packages    ; shift ;;

        -b | --basic)           basic_setup             ; shift ;;
             --ssid)            WIFI_SSID=$2    ; shift ; shift ;;
             --pass)            WIFI_PASS=$2    ; shift ; shift ;;

        -U | --setup-uart)      setup_uart              ; shift ;;  
        -I | --setup-i2c)       setup_i2c               ; shift ;;

        -A | --all)             doAll                   ; exit 0 ;;

        -h | --help)            help_menu  ; exit 0 ;;
        
        --*)
            echo "Unknown option: $1" >&2
            helpMenu
            exit 1
            ;;
        -*)
            echo "Unknown option: $1" >&2
            helpMenu
            exit 1 
            ;;
        *) 
            break
    esac
done