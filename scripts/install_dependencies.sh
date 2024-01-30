#!/usr/bin/env bash
wget -O - http://robotics.upo.es/repos/focal_apt/debian/conf/harurepo.gpg.key | sudo apt-key add -
sudo wget -nc -P /etc/apt/sources.list.d/ http://robotics.upo.es/repos/focal_apt/debian/conf/harufocal.list
PY_PROJECT="$(dirname "$0")/../../../"
pushd ${PY_PROJECT}
sudo apt-get update 
rosdep update --include-eol-distros
rosdep install --ignore-src --from-paths ./src --rosdistro "$(rosversion -d)" -y -r --os=ubuntu:"$(lsb_release -c --short)"
popd