#!/bin/bash

# Exit immediately if any command exits with a nonzero exit value
set -e

# Download planner
PANDA_DOWNLOAD_URL="https://www.uni-ulm.de/fileadmin/website_uni_ulm/iui.inst.090/panda/PANDA.jar"

# Install paths
# CHANGE THIS DIRECTORY
INSTALL_DIR_ROOT="/opt/ropod/task-planner/bin"
INSTALL_DIR_NAME="panda-planner"
INSTALL_DIR=$INSTALL_DIR_ROOT/$INSTALL_DIR_NAME

# create directory (if not already exists)
mkdir -p $INSTALL_DIR

# Download Planner (replace existing file)
echo "Installing PANDA planner"
cd $INSTALL_DIR
wget -O PANDA.jar $PANDA_DOWNLOAD_URL