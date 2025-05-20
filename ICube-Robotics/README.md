Required setup : Ubuntu 22.04 LTS

Install ros2 packages. The current development is based of ros2 humble. Installation steps are described here.

Source your ros2 environment:

``source /opt/ros/humble/setup.bash``

NOTE: The ros2 environment needs to be sources in every used terminal. If only one distribution of ros2 is used, it can be added to the ~/.bashrc file.
Install colcon and its extensions :

``sudo apt install python3-colcon-common-extensions``

Create a new ros2 workspace:

Dont forget the ``-p`` to create the directory in the second code of line here.

``mkdir -p ~/ros2_ws/src``

Pull relevant packages, install dependencies, compile, and source the workspace by using:

``cd ~/ros2_ws
git clone https://github.com/ICube-Robotics/iiwa_ros2.git src/iiwa_ros2
vcs import src < src/iiwa_ros2.repos
rosdep install --ignore-src --from-paths . -y -r
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
source install/setup.bash``


Use this line to clean build any missing or uninstalled dependencies to fix errors

``colcon build --symlink-install``

to update openGL:

``add-apt-repository ppa:kisak/kisak-mesa``

When launching using this command:

``ros2 launch iiwa_bringup iiwa.launch.py``

Screenshot:

![image](/Attachments/image1.png)
