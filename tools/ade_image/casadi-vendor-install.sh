#! /bin/bash

set -e

ARCH=""

if [ $(uname -m) == 'x86_64' ]; then
  ARCH="amd64"
else
  ARCH="arm64"
fi

curl -Ls -o /tmp/ros-dashing-casadi-vendor.deb https://gitlab.com/autowarefoundation/autoware.auto/casadi_vendor/-/jobs/artifacts/master/raw/ros-dashing-casadi-vendor_3.5.1-0bionic_$ARCH.deb?job=build_debian_$ARCH

apt-get update
apt-get install -y coinor-libipopt-dev gfortran liblapack-dev pkg-config git
rm -rf /var/lib/apt/lists/*

dpkg -i /tmp/ros-dashing-casadi-vendor.deb

rm /tmp/ros-dashing-casadi-vendor.deb
