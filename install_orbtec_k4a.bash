#!/bin/bash

# Check root access
if [ "$EUID" -ne 0 ]; then
  echo "❌ This script must be run as root. Try using: sudo $0"
  exit 1
fi

# Check OS version
. /etc/os-release
if [[ "$NAME" == "Ubuntu" && "$VERSION_ID" == "22.04" ]]; then
    echo "✅ This system is running Ubuntu 22.04."
else
    echo "❌ This script is for Ubuntu 22.04 only. Detected: $NAME $VERSION_ID"
    exit 1
fi

# Check if nvidia driver is installed
if command -v nvidia-smi &> /dev/null; then
    echo "✅ NVIDIA driver is installed:"
    nvidia-smi
else
    echo "❌ NVIDIA driver is NOT installed. Please install it first"
    exit 1
fi

# Install dependencies
apt update

apt install -y wget git build-essential cmake ninja-build libeigen3-dev wayland-protocols libwayland-dev libxkbcommon-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libgl1-mesa-dev

wget https://archive.ubuntu.com/ubuntu/pool/universe/libs/libsoundio/libsoundio-dev_1.1.0-1_amd64.deb
wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4/libk4a1.4_1.4.1_amd64.deb
wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4-dev/libk4a1.4-dev_1.4.1_amd64.deb
wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/k/k4a-tools/k4a-tools_1.4.1_amd64.deb
wget https://packages.microsoft.com/ubuntu/20.04/prod/pool/main/libk/libk4abt1.1/libk4abt1.1_1.1.2_amd64.deb
wget https://packages.microsoft.com/ubuntu/20.04/prod/pool/main/libk/libk4abt1.1-dev/libk4abt1.1-dev_1.1.2_amd64.deb

dpkg -i libsoundio-dev_1.1.0-1_amd64.deb
dpkg -i libk4a1.4_1.4.1_amd64.deb
dpkg -i libk4a1.4-dev_1.4.1_amd64.deb
dpkg -i k4a-tools_1.4.1_amd64.deb
dpkg -i libk4abt1.1_1.1.2_amd64.deb
dpkg -i libk4abt1.1-dev_1.1.2_amd64.deb

if locate k4abtConfig.cmake >/dev/null 2>&1; then
  echo "✅ K4A Body Tracking library is successfully installed"
else
  echo "❌ K4A Body Tracking library installation failed"
  exit 1
fi

# Install Orbbec Femto Bolt driver
wget https://github.com/orbbec/OrbbecSDK-K4A-Wrapper/releases/download/v1.10.3/OrbbecSDK_K4A_Wrapper_v1.10.3_linux_202408091809.zip
unzip OrbbecSDK_K4A_Wrapper_v1.10.3_linux_202408091809
cd OrbbecSDK_K4A_Wrapper_v1.10.3_linux_202408091809

cp -r bin/* /usr/bin/
cp -r lib/* /usr/lib/lib/x86_64-linux-gnu/
cp -r include/* /usr/include/

# Build Kinect Teleop
cd ..
mkdir build
cd build
cmake .. -GNinja
ninja

