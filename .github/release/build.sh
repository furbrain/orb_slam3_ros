#!/bin/bash
# Build an orb_slam3 release in a clean debian:bookworm arm64 container:
#   docker run --rm -v $PWD:/src:ro -v /some/dir:/out -e VERSION=v1 -e COMMIT=$(git rev-parse HEAD) \
#     debian:bookworm bash /src/.github/release/build.sh
# The tarball holds one folder, orb_slam3-<version>, which is a colcon prefix for /opt/orb_slam3/current to link to.
set -euo pipefail
umask 022

: "${VERSION:?}" "${COMMIT:?}"
VERSION=$(printf %s "$VERSION" | tr -c 'A-Za-z0-9._+-' '-') # tags can have slashes etc in them
JOBS=${JOBS:-$(nproc)}
export DEBIAN_FRONTEND=noninteractive

# cv_bridge is needed to build, but isn't shipped: devices have it in ros2_ws. Keep this the same as theirs
VISION_OPENCV_COMMIT=27de9ecf9862e6fba509b7e49e3c2511c7d11627

# the same ROS as install.md puts on the devices
ROS_DEB_URL=https://github.com/Ar-Ray-code/rpi-bullseye-ros2/releases/download/ros2-0.3.2/ros-jazzy-desktop-0.3.2_20240525_arm64.deb
ROS_DEB_SHA256=ddf1a5d9fea69f9dc2cd1a905eb8bb423b9f8404dd3b289c1d61e3f268600d32

apt-get update -qq
apt-get install -qq --no-install-recommends ca-certificates curl >/dev/null
curl -fsSL --retry 5 -o /tmp/ros.deb "$ROS_DEB_URL"
echo "$ROS_DEB_SHA256  /tmp/ros.deb" | sha256sum -c -
apt-get install -qq --no-install-recommends /tmp/ros.deb \
  git g++ binutils file qemu-user python3-empy python3-numpy python3-yaml python3-dev \
  libboost-dev libboost-python-dev libboost-system-dev libboost-serialization-dev \
  libboost-filesystem-dev libboost-iostreams-dev libssl-dev zlib1g-dev pybind11-dev >/dev/null
pip install -q --break-system-packages --root-user-action=ignore \
  colcon-core==0.20.0 colcon-common-extensions==0.3.0 catkin-pkg==0.5.2

work=/work
mkdir -p $work/src $work/deps
cp -a /src $work/src/orb_slam3_ros # the build writes into its source tree
git init -q $work/deps/vision_opencv
git -C $work/deps/vision_opencv fetch -q --depth 1 https://github.com/ros-perception/vision_opencv.git $VISION_OPENCV_COMMIT
git -C $work/deps/vision_opencv checkout -q FETCH_HEAD

export MAKEFLAGS=-j$JOBS
cd $work
set +u # ROS's setup files use unset variables
source /opt/ros/jazzy/setup.bash
colcon build --base-paths deps/vision_opencv/cv_bridge --build-base build-deps --install-base install-deps \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source $work/install-deps/setup.bash

# Debug is what's been used on the devices. It is still -O3 (see CMakeLists.txt), but Release would add -DNDEBUG,
# which removes the asserts. -g0 as the debug info is 90% of the size and needs lots of memory to build.
# No -march=native: the GitHub runners have newer CPUs than the Pi.
colcon build --base-paths src/orb_slam3_ros --build-base build --install-base install \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_C_FLAGS_DEBUG=-g0 -DCMAKE_CXX_FLAGS_DEBUG=-g0 -DBUILD_TESTING=OFF

name=orb_slam3-$VERSION
dest=$work/dist/$name
mkdir -p $work/dist
cp -a install/orb_slam3 "$dest"
find "$dest" -type f \( -name '*.so' -o -path '*/lib/orb_slam3/*' \) -exec strip --strip-debug {} +
echo "$VERSION $COMMIT" > "$dest/VERSION"

# check that it works with the build gone, loaded the way TacticalFrog's ros_env.sh does
rm -rf build install src
export COLCON_CURRENT_PREFIX=$dest
source "$dest/share/orb_slam3/package.bash"
unset COLCON_CURRENT_PREFIX
while read -r f; do
  if file -b "$f" | grep -q ELF && ldd "$f" | grep 'not found'; then
    echo "$f has missing libraries" >&2
    exit 1
  fi
done < <(find "$dest/lib" -type f)
test -s "$dest/share/orb_slam3/vocab/ORBvoc.txt.bin"
# cortex-a76 is the CM5's CPU
qemu-aarch64 -cpu cortex-a76 /usr/bin/python3 -c '
import orb_slam3_py
from orb_slam3.msg import Atlas, State
from rclpy.type_support import check_for_type_support as check
check(Atlas); check(State)
print("orb_slam3 release ok")'

cd /out
tarball=$name-$(uname -m).tar.gz
tar -C $work/dist --owner=0 --group=0 -czf "$tarball" "$name"
sha256sum "$tarball" > "$tarball.sha256"
