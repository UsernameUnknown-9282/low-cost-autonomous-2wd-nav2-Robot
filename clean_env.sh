#!/bin/bash
set -e

# Remove old duplicates
sed -i '/TURTLEBOT3_MODEL/d' ~/.bashrc
sed -i '/GAZEBO_MODEL_PATH/d' ~/.bashrc
sed -i '/LIBGL_ALWAYS_SOFTWARE/d' ~/.bashrc
sed -i '/OGRE_RTT_MODE/d' ~/.bashrc
sed -i '/source \/opt\/ros\/humble\/setup.bash/d' ~/.bashrc

# Append clean exports
cat << 'EOF' >> ~/.bashrc
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/opt/ros/humble/share/turtlebot3_gazebo/models
export OGRE_RTT_MODE=Copy
EOF

echo "Environment successfully updated."
