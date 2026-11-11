#!/bin/bash
set -e
source /opt/ros/melodic/setup.bash
source /opt/heron_ws/install/setup.bash
exec "$@"
