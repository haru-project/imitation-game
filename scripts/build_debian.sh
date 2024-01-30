#!/usr/bin/env bash
#
# Re-build a debian file
# Currently builds for ubuntu xenial, kinetic ros distro
# Strawberry - dialogueberry
# Copyright 2017-2019 Honda Research Institute Japan. All rights reserved.
#
NEED_ROOT=0
PY_PROJECT="$(dirname "$0")/../"
pushd ${PY_PROJECT}

# Include the common set of functions
DIR="${BASH_SOURCE%/*}"
if [[ ! -d "$DIR" ]]; then DIR="$PWD"; fi
. "$DIR/common.sh"

new_command "Cleaning the build folders"
sudo rm -rf ${PY_PROJECT}/debian ${PY_PROJECT}/obj-*
ret_ok "Cleaning the build folders ... done"

new_command "Generating the build files via Bloom"
bloom-generate rosdebian --os-name ubuntu --os-version "$(lsb_release -c --short)" --ros-distro "$(rosversion -d)"
if [ $? -ne 0 ]; then
    ret_critical "Failed to generate the build files. Exiting"
fi
ret_ok "Generating the build files via Bloom ... done"

new_command "Copying the preinst and postinst from the root folder"
set +e
cp ${PY_PROJECT}*inst ${PY_PROJECT}/debian/
cp ${PY_PROJECT}*trm ${PY_PROJECT}/debian/
set -e
ret_ok "Copying the preinst and postinst from the root folder ... done"

new_command "Generating a debian file"
fakeroot ./debian/rules binary > error.log
if [ $? -ne 0 ]; then
    ret_critical "Failed to build the debian file. Check the error.log file. Exiting."
else
    rm -rf ./error.log
fi

ret_ok "Generating a debian file ... done"

new_command "Cleaning the build folders"
rm -rf ${PY_PROJECT}/debian ${PY_PROJECT}/obj-*
ret_ok "Cleaning the build folders ... done"

popd
