#!/usr/bin/env bash
##
## Copyright (C) 2020 Hanson Robotics - All Rights Reserved
##
## For development please contact <dev@hansonrobotics.com>
##
##

package() {
    local reponame=r2_behavior

    mkdir -p $BASEDIR/src
    rsync -r --delete \
        --exclude ".git" \
        --exclude "package" \
        $BASEDIR/../ $BASEDIR/src/$reponame

    get_version $1
    source_ros
    catkin_make_isolated --directory $BASEDIR --install --install-space $BASEDIR/install -DCMAKE_BUILD_TYPE=Release

    local name=head-r2-behavior
    local desc="ROS Behavior"
    local url="https://api.github.com/repos/hansonrobotics/$reponame/releases"

    fpm -C "${BASEDIR}" -s dir -t deb -n "${name}" -v "${version#v}" --vendor "${VENDOR}" \
        --url "${url}" --description "${desc}" ${ms} --force \
        --deb-no-default-config-files \
        -p $BASEDIR/${name}_VERSION_ARCH.deb \
        install/share=${HR_ROS_PREFIX}/ \
        install/lib=${HR_ROS_PREFIX}/

    cleanup_ros_package_build $BASEDIR
}

if [[ $(readlink -f ${BASH_SOURCE[0]}) == $(readlink -f $0) ]]; then
    BASEDIR=$(dirname $(readlink -f ${BASH_SOURCE[0]}))
    source $BASEDIR/common.sh
    set -e

    package $1
fi
