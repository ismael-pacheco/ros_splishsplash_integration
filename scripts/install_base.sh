#!/bin/bash
# install_base.sh - Setup script for Gazebo/SPlisHSPlasH integration (excluding ROS) on Ubuntu 20.04

set -e

# Create log file
LOGFILE="install_base.log"
echo "Installation started at $(date)" > $LOGFILE
timedatectl set-local-rtc 1
sudo apt remove --purge cmake cmake-qt-gui cmake-data -y
sudo rm -rf /usr/local/bin/cmake* /usr/local/share/cmake-*
sudo apt update
sudo apt install -y wget build-essential libssl-dev
cd ~ 
if [ ! -f "cmake-4.0.2.tar.gz" ]; then
  wget https://cmake.org/files/v4.0/cmake-4.0.2.tar.gz
  tar -xzf cmake-4.0.2.tar.gz
else
  tar -xzf cmake-4.0.2.tar.gz
fi



cd cmake-4.0.2
./bootstrap --prefix=/usr/local
make -j$(nproc)
sudo make install
cmake --version
# 1. Configure Gazebo repo
sudo add-apt-repository ppa:dartsim/ppa
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable focal main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# 2. Update system and install all required dependencies
sudo apt update && sudo apt install -y \
  acpi-support   aptitude   astyle   autoconf   automake   backport-iwlwifi-dkms   base-passwd   bc   bcmwl-kernel-source   binutils-dev   bison   build-essential   ccache   clang   clang-format   clang-tidy   cmake   cmake-data   cppcheck   cuda-repo-ubuntu2004-12-9-local   cuda-toolkit-12-9   curl   dash   debhelper   diffutils   dkms   dmidecode   e2fsprogs   ffmpeg   file   findutils   firefox-locale-en   firefox-locale-es   flex   fonts-freefont-ttf   fonts-indic   freeglut3-dev   g++   g++-11   g++-multilib   gawk   gcc   gcc-11   gcc-arm-none-eabi   gcc-multilib   gcovr   gdb   gdb-multiarch   genromfs   geographiclib-tools   gettext   gir1.2-goa-1.0   git   gnome-getting-started-docs-es   gnome-user-docs-es   gnupg   gnupg2   gperf   gpg   graphviz   grep   grub-common   grub-efi-amd64-signed   grub-gfxpayload-lists   grub-pc   grub-pc-bin   grub2-common   gstreamer1.0-gl   gstreamer1.0-libav   gstreamer1.0-plugins-bad   gstreamer1.0-plugins-base   gstreamer1.0-plugins-good   gstreamer1.0-plugins-ugly   gstreamer1.0-tools   gzip   hostname   hunspell-en-au   hunspell-en-ca   hunspell-en-gb   hunspell-en-za   hunspell-es   hyphen-en-ca   hyphen-en-gb   hyphen-en-us   hyphen-es   init   kconfig-frontends   language-pack-en   language-pack-en-base   language-pack-es   language-pack-es-base   language-pack-gnome-en   language-pack-gnome-en-base   language-pack-gnome-es   language-pack-gnome-es-base   lcov   libassimp-dev   libavcodec-dev   libavdevice-dev   libavformat-dev   libavutil-dev   libboost-all-dev   libboost-filesystem-dev   libboost-program-options-dev   libboost-system-dev   libboost-test-dev   libboost-thread-dev   libbullet-dev   libc6-i386   libccd-dev   libcsfml-audio2.5   libcsfml-dev   libcsfml-graphics2.5   libcsfml-network2.5   libcsfml-system2.5   libcsfml-window2.5   libcurl4-openssl-dev   libdcmtk-dev   libdebconfclient0   libdrm-dev   libeigen3-dev   libelf-dev   libexpat1-dev   libfcl-dev   libffi-dev   libfmt-dev   libfprint-2-tod1   libfreeimage-dev   libfreetype6-dev   libftdi-dev   libfuse2   libfwupdplugin1   libgdal-dev   libgdbm-dev   libgeographic-dev   libgl1-mesa-dev   libgl1-mesa-glx   libglu1-mesa-dev   libglvnd-dev   libgmp-dev   libgpg-error0   libgraphviz-dev   libgstreamer-plugins-base1.0-dev   libgstreamer1.0-dev   libgts-dev   libhdf5-dev   libignition-cmake2-dev   libignition-common3-dev   libignition-common3-graphics-dev   libignition-fuel-tools4-dev   libignition-math6-dev   libignition-msgs-dev   libignition-msgs5-dev   libignition-transport-dev   libignition-transport8-dev   libimage-exiftool-perl   libisl-dev   libjpeg8-dev   libllvm9   libmpc-dev   libmpfr-dev   libncurses-dev   libncurses5   libncurses5-dev   libncurses6   libncursesw5-dev   libncursesw6   libnewlib-arm-none-eabi   liboctomap-dev   libogre-1.9-dev   libopenal-dev   libopencv-dev   libopenscenegraph-dev   libpam-modules   libpng16-16   libportmidi-dev   libprotobuf-dev   libpython2.7-stdlib   libpython3-stdlib   libqt5core5a   libqt5gui5   libqt5opengl5-dev   libqt5widgets5   libqwt-qt5-dev   libreadline-dev   libreoffice-help-common   libreoffice-help-en-gb   libreoffice-help-en-us   libreoffice-help-es   libreoffice-l10n-en-gb   libreoffice-l10n-en-za   libreoffice-l10n-es   libsdformat9-dev   libsdl-image1.2-dev   libsdl-mixer1.2-dev   libsdl-ttf2.0-dev   libsdl1.2-dev   libsfml-audio2.5   libsfml-dev   libsfml-graphics2.5   libsfml-network2.5   libsfml-system2.5   libsfml-window2.5   libsimbody-dev   libsqlite3-dev   libssl-dev   libstdc++-arm-none-eabi-newlib   libswscale-dev   libtar-dev   libtbb-dev   libtcmalloc-minimal4   libtinyxml-dev   libtinyxml2-dev   libtool   libtool-bin   libunwind-dev   liburdfdom-dev   liburdfdom-headers-dev   libusb-1.0-0-dev   libxcb-cursor-dev   libxcb-xinerama0   libxcursor-dev   libxi-dev   libxinerama-dev   libxkbcommon-x11-0   libxml2-dev   libxml2-utils   libxmlb1   libxmu-dev   libxrandr-dev   libxslt1-dev   libyaml-dev   linux-generic-hwe-20.04   linux-headers-5.15.0-139-generic   lld   lldb   locate   make   mokutil   mythes-en-au   mythes-en-us   mythes-es   ncurses-base   ncurses-bin   net-tools   ninja-build   nvidia-cuda-toolkit   nvidia-open   openocd   orca   os-prober   paraview   pkg-config   pkg-config-arm-linux-gnueabihf   ppp   protobuf-compiler   python-is-python3   python-wxgtk3.0   python3   python3-catkin-lint   python3-catkin-pkg   python3-catkin-tools   python3-colcon-common-extensions   python3-colour   python3-dev   python3-empy   python3-future   python3-jinja2   python3-matplotlib   python3-numpy   python3-opencv   python3-pexpect   python3-pip   python3-psutil   python3-pygments   python3-pyparsing   python3-rosdep   python3-rosinstall   python3-rosinstall-generator   python3-scipy   python3-serial   python3-setuptools   python3-toml   python3-vcstool   python3-wheel   python3-wstool   python3-wxgtk4.0   python3-yaml   qtbase5-dev   qtcreator   quilt   rbenv   rdfind   ronn   ros-noetic-cmake-modules   ros-noetic-geographic-msgs   ros-noetic-mavros   ros-noetic-mavros-extras   ros-noetic-ros-base   ros-noetic-ros-control   ros-noetic-ros-controllers   ros-noetic-tf2-geometry-msgs   ros-noetic-xacro   rsync   ruby   ruby-dev   screen   shellcheck   shim-signed   simplescreenrecorder   software-properties-common   sqlite3   synaptic   terminator   texinfo   thunderbird-locale-en   thunderbird-locale-en-gb   thunderbird-locale-en-us   thunderbird-locale-es   thunderbird-locale-es-ar   thunderbird-locale-es-es   u-boot-tools   ubuntu-desktop   ubuntu-desktop-minimal   ubuntu-minimal   ubuntu-standard   ubuntu-wallpapers   unzip   util-linux   uuid-dev   valgrind   vim-common   wget   wspanish   xfonts-base   xsltproc   xterm   zip   zlib1g-devsudo apt update && sudo apt install -y \
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
  xsltproc
\

echo "[1/8] Installing system dependencies..." | tee -a $LOGFILE
  sudo apt update && sudo apt install -y 
  libignition-transport8-dev\
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
  libopenscenegraph-dev libgl1-mesa-dev libfmt-dev\
  libignition-transport8-dev libqwt-qt5-dev libopenal-dev libusb-1.0-0-dev \
  libhdf5-dev libignition-msgs5-dev libgdal-dev graphviz xsltproc ronn \
  libxrandr-dev libxinerama-dev libxcursor-dev | tee -a $LOGFILE

  sudo ldconfig
# 4. Install Eigen 3.4.0
cd ~
if [ ! -f "eigen-3.4.0.tar.gz" ]; then
  wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
  tar -xzf eigen-3.4.0.tar.gz
else
  tar -xzf eigen-3.4.0.tar.gz
fi

cd eigen-3.4.0 
if [ ! -d "build" ]; then
  mkdir build && cd build
else
  cd build
fi


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

if [ ! -d "fcl" ]; then
  git clone https://github.com/flexible-collision-library/fcl.git
else
  cd fcl
  git checkout master
  git pull
  cd ~
fi


cd fcl && git checkout 0.7.0
if [ ! -d "build" ]; then
  mkdir build && cd build
else
  cd build
fi
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install

# 7. Build and install Assimp
sudo apt remove --purge libassimp-dev assimp-utils -y

if [ ! -d "assimp" ]; then
  git clone https://github.com/assimp/assimp.git
else
  cd assimp && git checkout v5.2.5
  git checkout master
  git pull

fi

if [ ! -d "build" ]; then
  mkdir build && cd build
else
  cd build
fi
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install
export CMAKE_PREFIX_PATH="/usr/local:$CMAKE_PREFIX_PATH"
source ~/.bashrc


cd ~

if [ ! -d "dart" ]; then
  git clone https://github.com/dartsim/dart.git
else
  cd dart
  git checkout main
  git pull

fi

if [ ! -d "build" ]; then
  mkdir build && cd build
else
  cd build
  rm -rf CMakeCache.txt CMakeFiles/
fi

cmake .. \
  -DCMAKE_PREFIX_PATH="/usr/local" \
  -DCMAKE_MODULE_PATH="/usr/local/lib/cmake/assimp" \
  -DASSIMP_INCLUDE_DIR="$(pkg-config --cflags-only-I assimp | sed 's/-I//')" \
  -DASSIMP_LIBRARY="$(pkg-config --libs-only-L assimp | sed 's/-L//')/libassimp.so"
make -j4
sudo make install && sudo ldconfig


# Actualizar caché de bibliotecas
sudo ldconfig

# Verificar versión instalada
grep "set(PACKAGE_VERSION" /usr/local/share/dart/cmake/DARTConfigVersion.cmake

sudo rm -rf /usr/local/bin/cmake* /usr/local/share/cmake-*  # Elimina instalación manual
sudo apt remove --purge cmake cmake-qt-gui cmake-data -y    # Elimina versión de apt
sudo apt autoremove -y

cd ~

if [ ! -f "cmake-3.16.0.tar.gz" ]; then
cd ~
else
  rm ~/cmake-3.16.0.tar.gz
fi

if [ ! -f "cmake-3.16.0-Linux-x86_64.sh" ]; then
  wget https://cmake.org/files/v3.16/cmake-3.16.0-Linux-x86_64.sh
  chmod +x cmake-3.16.0-Linux-x86_64.sh
  sudo ./cmake-3.16.0-Linux-x86_64.sh --skip-license --prefix=/usr/local
else
  chmod +x cmake-3.16.0-Linux-x86_64.sh
  sudo ./cmake-3.16.0-Linux-x86_64.sh --skip-license --prefix=/usr/local
fi






echo 'export PATH=/usr/local/bin:$PATH' >> ~/.bashrc
source ~/.bashrc

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

