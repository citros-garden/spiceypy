#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /workspaces/spiceypy/install/setup.bash 

exec "$@"
