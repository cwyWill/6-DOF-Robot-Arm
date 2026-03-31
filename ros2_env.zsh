# source environments for ROS2
source /opt/ros/kilted/setup.zsh

# set domain ID, arbitrarily chose 37
export ROS_DOMAIN_ID=43

# add colcon_cd command to the environment. colcon_cd opens the directory where colcon is installed.
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/kilted/
# source colcon autocompletion files
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh

# X11 forward display
export DISPLAY="localhost:11.0"


# setup the executable in this workspace
if [ -f ./install/setup ]; then
    source install/setup.zsh
fi
