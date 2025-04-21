#!/bin/bash
# install_base.sh - Setup script for Gazebo/SPlisHSPlasH integration (excluding ROS) on Ubuntu 20.04

set -e

# Create log file
LOGFILE="install_base.log"
echo "Installation started at $(date)" > $LOGFILE

# 1. Configure Gazebo repo
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable focal main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# 2. Update system and install all required dependencies
sudo apt update && sudo apt install -y \
  acpi-support \
  aptitude \
  backport-iwlwifi-dkms \
  base-passwd \
  bcmwl-kernel-source \
  build-essential \
  cmake \
  cmake-qt-gui \
  curl \
  dash \
  debhelper \
  diffutils \
  dkms \
  e2fsprogs \
  findutils \
  firefox-locale-en \
  firefox-locale-es \
  fonts-indic \
  freeglut3-dev \
  g++ \
  g++-11 \
  gcc-11 \
  gir1.2-goa-1.0 \
  git \
  gnome-getting-started-docs-es \
  gnome-user-docs-es \
  gnupg2 \
  graphviz \
  grep \
  grub-common \
  grub-efi-amd64-signed \
  grub-gfxpayload-lists \
  grub-pc \
  grub-pc-bin \
  grub2-common \
  gzip \
  hostname \
  hunspell-en-au \
  hunspell-en-ca \
  hunspell-en-gb \
  hunspell-en-za \
  hunspell-es \
  hyphen-en-ca \
  hyphen-en-gb \
  hyphen-en-us \
  hyphen-es \
  init \
  language-pack-en \
  language-pack-en-base \
  language-pack-es \
  language-pack-es-base \
  language-pack-gnome-en \
  language-pack-gnome-en-base \
  language-pack-gnome-es \
  language-pack-gnome-es-base \
  libassimp-dev \
  libavcodec-dev \
  libavformat-dev \
  libavutil-dev \
  libboost-all-dev \
  libboost-filesystem-dev \
  libboost-program-options-dev \
  libboost-system-dev \
  libboost-test-dev \
  libboost-thread-dev \
  libbullet-dev \
  libccd-dev \
  libcurl4-openssl-dev \
  libdart6-dev \
  libdart6-utils-urdf-dev \
  libdcmtk-dev \
  libdebconfclient0 \
  libdrm-dev \
  libeigen3-dev \
  libfcl-dev \
  libfmt-dev \
  libfprint-2-tod1 \
  libfreeimage-dev \
  libfwupdplugin1 \
  libgdal-dev \
  libgl1-mesa-dev \
  libglu1-mesa-dev \
  libglvnd-dev \
  libgpg-error0 \
  libgts-dev \
  libhdf5-dev \
  libignition-common3-dev \
  libignition-fuel-tools4-dev \
  libignition-math6-dev \
  libignition-msgs5-dev \
  libignition-transport8-dev \
  libllvm9 \
  liboctomap-dev \
  libogre-1.9-dev \
  libopenal-dev \
  libopenscenegraph-dev \
  libpam-modules \
  libprotobuf-dev \
  libqt5core5a \
  libqt5widgets5 \
  libqwt-qt5-dev \
  libreoffice-help-common \
  libreoffice-help-en-gb \
  libreoffice-help-en-us \
  libreoffice-help-es \
  libreoffice-l10n-en-gb \
  libreoffice-l10n-en-za \
  libreoffice-l10n-es \
  libsdformat6-dev \
  libsimbody-dev \
  libswscale-dev \
  libtar-dev \
  libtbb-dev \
  libtcmalloc-minimal4 \
  libtinyxml-dev \
  libtinyxml2-dev \
  liburdfdom-dev \
  liburdfdom-headers-dev \
  libusb-1.0-0-dev \
  libxcursor-dev \
  libxi-dev \
  libxinerama-dev \
  libxmlb1 \
  libxmu-dev \
  libxrandr-dev \
  linux-generic-hwe-20.04 \
  mainline \
  mokutil \
  mythes-en-au \
  mythes-en-us \
  mythes-es \
  ncurses-base \
  ncurses-bin \
  orca \
  os-prober \
  pkg-config \
  protobuf-compiler \
  python3-catkin-tools \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-rosinstall \
  python3-rosinstall-generator \
  python3-vcstool \
  python3-wstool \
  qtbase5-dev \
  quilt \
  rdfind \
  ronn \
  ros-noetic-ros-base \
  ruby \
  ruby-dev \
  shim-signed \
  software-properties-common \
  synaptic \
  thunderbird-locale-en \
  thunderbird-locale-en-gb \
  thunderbird-locale-en-us \
  thunderbird-locale-es \
  thunderbird-locale-es-ar \
  thunderbird-locale-es-es \
  ubuntu-desktop \
  ubuntu-desktop-minimal \
  ubuntu-minimal \
  ubuntu-standard \
  ubuntu-wallpapers \
  uuid-dev \
  wget \
  wspanish \
  xsltprocsudo apt update && sudo apt install -y \
  acpi-support170

  aptitude170

  backport-iwlwifi-dkms170

  base-passwd170

  bcmwl-kernel-source170

  build-essential170

  cmake170

  cmake-qt-gui170

  curl170

  dash170

  debhelper170

  diffutils170

  dkms170

  e2fsprogs170

  findutils170

  firefox-locale-en170

  firefox-locale-es170

  fonts-indic170

  freeglut3-dev170

  g++170

  g++-11170

  gcc-11170

  gir1.2-goa-1.0170

  git170

  gnome-getting-started-docs-es170

  gnome-user-docs-es170

  gnupg2170

  graphviz170

  grep170

  grub-common170

  grub-efi-amd64-signed170

  grub-gfxpayload-lists170

  grub-pc170

  grub-pc-bin170

  grub2-common170

  gzip170

  hostname170

  hunspell-en-au170

  hunspell-en-ca170

  hunspell-en-gb170

  hunspell-en-za170

  hunspell-es170

  hyphen-en-ca170

  hyphen-en-gb170

  hyphen-en-us170

  hyphen-es170

  init170

  language-pack-en170

  language-pack-en-base170

  language-pack-es170

  language-pack-es-base170

  language-pack-gnome-en170

  language-pack-gnome-en-base170

  language-pack-gnome-es170

  language-pack-gnome-es-base170

  libassimp-dev170

  libavcodec-dev170

  libavformat-dev170

  libavutil-dev170

  libboost-all-dev170

  libboost-filesystem-dev170

  libboost-program-options-dev170

  libboost-system-dev170

  libboost-test-dev170

  libboost-thread-dev170

  libbullet-dev170

  libccd-dev170

  libcurl4-openssl-dev170

  libdart6-dev170

  libdart6-utils-urdf-dev170

  libdcmtk-dev170

  libdebconfclient0170

  libdrm-dev170

  libeigen3-dev170

  libfcl-dev170

  libfmt-dev170

  libfprint-2-tod1170

  libfreeimage-dev170

  libfwupdplugin1170

  libgdal-dev170

  libgl1-mesa-dev170

  libglu1-mesa-dev170

  libglvnd-dev170

  libgpg-error0170

  libgts-dev170

  libhdf5-dev170

  libignition-common3-dev170

  libignition-fuel-tools4-dev170

  libignition-math6-dev170

  libignition-msgs5-dev170

  libignition-transport8-dev170

  libllvm9170

  liboctomap-dev170

  libogre-1.9-dev170

  libopenal-dev170

  libopenscenegraph-dev170

  libpam-modules170

  libprotobuf-dev170

  libqt5core5a170

  libqt5widgets5170

  libqwt-qt5-dev170

  libreoffice-help-common170

  libreoffice-help-en-gb170

  libreoffice-help-en-us170

  libreoffice-help-es170

  libreoffice-l10n-en-gb170

  libreoffice-l10n-en-za170

  libreoffice-l10n-es170

  libsdformat6-dev170

  libsimbody-dev170

  libswscale-dev170

  libtar-dev170

  libtbb-dev170

  libtcmalloc-minimal4170

  libtinyxml-dev170

  libtinyxml2-dev170

  liburdfdom-dev170

  liburdfdom-headers-dev170

  libusb-1.0-0-dev170

  libxcursor-dev170

  libxi-dev170

  libxinerama-dev170

  libxmlb1170

  libxmu-dev170

  libxrandr-dev170

  linux-generic-hwe-20.04170

  mainline170

  mokutil170

  mythes-en-au170

  mythes-en-us170

  mythes-es170

  ncurses-base170

  ncurses-bin170

  orca170

  os-prober170

  pkg-config170

  protobuf-compiler170

  python3-catkin-tools170

  python3-colcon-common-extensions170

  python3-pip170

  python3-rosdep170

  python3-rosinstall170

  python3-rosinstall-generator170

  python3-vcstool170

  python3-wstool170

  qtbase5-dev170

  quilt170

  rdfind170

  ronn170

  ros-noetic-ros-base170

  ruby170

  ruby-dev170

  shim-signed170

  software-properties-common170

  synaptic170

  thunderbird-locale-en170

  thunderbird-locale-en-gb170

  thunderbird-locale-en-us170

  thunderbird-locale-es170

  thunderbird-locale-es-ar170

  thunderbird-locale-es-es170

  ubuntu-desktop170

  ubuntu-desktop-minimal170

  ubuntu-minimal170

  ubuntu-standard170

  ubuntu-wallpapers170

  uuid-dev170

  wget170

  wspanish170

  xsltproc170
sudo apt update && sudo apt install -y \
  acpi-support170

  aptitude170

  backport-iwlwifi-dkms170

  base-passwd170

  bcmwl-kernel-source170

  build-essential170

  cmake170

  cmake-qt-gui170

  curl170

  dash170

  debhelper170

  diffutils170

  dkms170

  e2fsprogs170

  findutils170

  firefox-locale-en170

  firefox-locale-es170

  fonts-indic170

  freeglut3-dev170

  g++170

  g++-11170

  gcc-11170

  gir1.2-goa-1.0170

  git170

  gnome-getting-started-docs-es170

  gnome-user-docs-es170

  gnupg2170

  graphviz170

  grep170

  grub-common170

  grub-efi-amd64-signed170

  grub-gfxpayload-lists170

  grub-pc170

  grub-pc-bin170

  grub2-common170

  gzip170

  hostname170

  hunspell-en-au170

  hunspell-en-ca170

  hunspell-en-gb170

  hunspell-en-za170

  hunspell-es170

  hyphen-en-ca170

  hyphen-en-gb170

  hyphen-en-us170

  hyphen-es170

  init170

  language-pack-en170

  language-pack-en-base170

  language-pack-es170

  language-pack-es-base170

  language-pack-gnome-en170

  language-pack-gnome-en-base170

  language-pack-gnome-es170

  language-pack-gnome-es-base170

  libassimp-dev170

  libavcodec-dev170

  libavformat-dev170

  libavutil-dev170

  libboost-all-dev170

  libboost-filesystem-dev170

  libboost-program-options-dev170

  libboost-system-dev170

  libboost-test-dev170

  libboost-thread-dev170

  libbullet-dev170

  libccd-dev170

  libcurl4-openssl-dev170

  libdart6-dev170

  libdart6-utils-urdf-dev170

  libdcmtk-dev170

  libdebconfclient0170

  libdrm-dev170

  libeigen3-dev170

  libfcl-dev170

  libfmt-dev170

  libfprint-2-tod1170

  libfreeimage-dev170

  libfwupdplugin1170

  libgdal-dev170

  libgl1-mesa-dev170

  libglu1-mesa-dev170

  libglvnd-dev170

  libgpg-error0170

  libgts-dev170

  libhdf5-dev170

  libignition-common3-dev170

  libignition-fuel-tools4-dev170

  libignition-math6-dev170

  libignition-msgs5-dev170

  libignition-transport8-dev170

  libllvm9170

  liboctomap-dev170

  libogre-1.9-dev170

  libopenal-dev170

  libopenscenegraph-dev170

  libpam-modules170

  libprotobuf-dev170

  libqt5core5a170

  libqt5widgets5170

  libqwt-qt5-dev170

  libreoffice-help-common170

  libreoffice-help-en-gb170

  libreoffice-help-en-us170

  libreoffice-help-es170

  libreoffice-l10n-en-gb170

  libreoffice-l10n-en-za170

  libreoffice-l10n-es170

  libsdformat6-dev170

  libsimbody-dev170

  libswscale-dev170

  libtar-dev170

  libtbb-dev170

  libtcmalloc-minimal4170

  libtinyxml-dev170

  libtinyxml2-dev170

  liburdfdom-dev170

  liburdfdom-headers-dev170

  libusb-1.0-0-dev170

  libxcursor-dev170

  libxi-dev170

  libxinerama-dev170

  libxmlb1170

  libxmu-dev170

  libxrandr-dev170

  linux-generic-hwe-20.04170

  mainline170

  mokutil170

  mythes-en-au170

  mythes-en-us170

  mythes-es170

  ncurses-base170

  ncurses-bin170

  orca170

  os-prober170

  pkg-config170

  protobuf-compiler170

  python3-catkin-tools170

  python3-colcon-common-extensions170

  python3-pip170

  python3-rosdep170

  python3-rosinstall170

  python3-rosinstall-generator170

  python3-vcstool170

  python3-wstool170

  qtbase5-dev170

  quilt170

  rdfind170

  ronn170

  ros-noetic-ros-base170

  ruby170

  ruby-dev170

  shim-signed170

  software-properties-common170

  synaptic170

  thunderbird-locale-en170

  thunderbird-locale-en-gb170

  thunderbird-locale-en-us170

  thunderbird-locale-es170

  thunderbird-locale-es-ar170

  thunderbird-locale-es-es170

  ubuntu-desktop170

  ubuntu-desktop-minimal170

  ubuntu-minimal170

  ubuntu-standard170

  ubuntu-wallpapers170

  uuid-dev170

  wget170

  wspanish170

  xsltproc170
echo "[1/8] Installing system dependencies..." | tee -a $LOGFILE
  build-essential cmake pkg-config libboost-all-dev libprotobuf-dev protobuf-compiler \
  libqt5core5a qtbase5-dev libqt5widgets5 libtinyxml-dev libogre-1.9-dev \
  libfreeimage-dev libtbb-dev libtar-dev libgts-dev libavcodec-dev libavformat-dev \
  libavutil-dev libswscale-dev libdcmtk-dev libbullet-dev libsimbody-dev \
  libignition-math6-dev libignition-common3-dev libignition-fuel-tools4-dev \
  libeigen3-dev libcurl4-openssl-dev libtcmalloc-minimal4 freeglut3-dev libxmu-dev \
  libxi-dev libdrm-dev libglu1-mesa-dev uuid-dev software-properties-common \
  wget libassimp-dev libccd-dev libfcl-dev liboctomap-dev libtinyxml2-dev \
  liburdfdom-dev liburdfdom-headers-dev libboost-system-dev libboost-filesystem-dev \
  libboost-program-options-dev libboost-test-dev libboost-thread-dev \
  libopenscenegraph-dev libgl1-mesa-dev libfmt-dev ruby ruby-dev \
  libignition-transport8-dev libqwt-qt5-dev libopenal-dev libusb-1.0-0-dev \
  libhdf5-dev libignition-msgs5-dev libgdal-dev graphviz xsltproc ronn \
  libxrandr-dev libxinerama-dev libxcursor-dev | tee -a $LOGFILE

wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | sudo apt-key add -
sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ focal main'
sudo apt update && sudo apt install -y cmake cmake-qt-gui

# 4. Install Eigen 3.4.0
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
tar -xzf eigen-3.4.0.tar.gz
cd eigen-3.4.0 && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
sudo make install && sudo ldconfig
cd ~

# 5. GCC/G++ 11 toolchain setup
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt update && sudo apt install -y gcc-11 g++-11
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 90
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 100
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 90
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 100

# 6. Build and install FCL
cd ~
git clone https://github.com/flexible-collision-library/fcl.git
cd fcl && git checkout v0.7.0
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install

# 7. Build and install Assimp
sudo apt remove --purge libassimp-dev assimp-utils -y
git clone https://github.com/assimp/assimp.git
cd assimp && git checkout v5.2.5
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install
export CMAKE_PREFIX_PATH="/usr/local:$CMAKE_PREFIX_PATH"
source ~/.bashrc

# 8. Build and install DART
cd ~
git clone https://github.com/dartsim/dart.git
cd dart && mkdir build && cd build
rm -rf CMakeCache.txt CMakeFiles/
cmake .. \
  -DCMAKE_PREFIX_PATH="/usr/local" \
  -DCMAKE_MODULE_PATH="/usr/local/lib/cmake/assimp" \
  -DASSIMP_INCLUDE_DIR="$(pkg-config --cflags-only-I assimp | sed 's/-I//')" \
  -DASSIMP_LIBRARY="$(pkg-config --libs-only-L assimp | sed 's/-L//')/libassimp.so"
make -j4
sudo make install && sudo ldconfig

# Additional build steps are handled by external scripts:
# - install_gazebo.sh
# - install_splishsplash.sh
# - install_ruby.sh
# - install_sdformat.sh
# - install_ignition_math.sh

# Backup installed packages
BACKUP_FILE="apt-packages.txt"
echo "Backing up manually installed APT packages to $BACKUP_FILE..." | tee -a $LOGFILE
apt-mark showmanual > "$BACKUP_FILE"

# Final message
echo "Base installation completed. Check $LOGFILE for full output." | tee -a $LOGFILE

# ===== update.sh script begins here =====
cat << 'EOF' > update.sh
#!/bin/bash
# update.sh - Update the integration repo and system dependencies

set -e

echo "[1/4] Pulling latest changes from Git..."
git pull

echo "[2/4] Updating APT packages..."
sudo apt update && sudo apt upgrade -y

echo "[3/4] Updating manual package list..."
apt-mark showmanual > apt-packages.txt

echo "[4/4] Update complete. You may want to rebuild some sources manually if needed."
EOF

chmod +x update.sh

