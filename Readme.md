# Dependencies

## Installeer realtime kernel (vereist)

```shell
sudo apt install linux-realtime

sudo sed -i 's/^GRUB_DEFAULT=[^ ]*/GRUB_DEFAULT=saved/' /etc/default/grub
sudo sed -i 's/^GRUB_SAVEDEFAULT=[^ ]*/GRUB_SAVEDEFAULT=true/' /etc/default/grub
sudo sed -i 's/^GRUB_TIMEOUT=[^ ]*/GRUB_TIMEOUT=10/' /etc/default/grub
sudo sed -i 's/^GRUB_HIDDEN_TIMEOUT=[^ ]*/GRUB_HIDDEN_TIMEOUT=0/' /etc/default/grub
sudo sed -i 's/^GRUB_HIDDEN_TIMEOUT_QUIET=[^ ]*/GRUB_HIDDEN_TIMEOUT_QUIET=false/' /etc/default/grub
sudo sed -i 's/^GRUB_TIMEOUT_STYLE=[^ ]*/GRUB_TIMEOUT_STYLE=menu/' /etc/default/grub
sudo sed -i 's/^GRUB_FORCE_HIDDEN_MENU=[^ ]*/GRUB_FORCE_HIDDEN_MENU=false/' /etc/default/grub

sudo update-grub
```

Herstart de computer:
1.	Een opstartmenu komt tevoorschijn
2.	Ga naar advanced options
3.	Selecteer kernel 5.15.0-xxxx-realtime

Valideer na opstarten of de realtime kernel wordt gebruikt.

```shell
uname -a
```

Maak opstarten met de kernel onaanpasbaar.

```shell
sudo sed -i 's/^GRUB_TIMEOUT=[^ ]*/GRUB_TIMEOUT=0/' /etc/default/grub
```

## Installeer ros2 humble

(https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

_Set locale._

```shell
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

_Ensure that the Ubuntu Universe repository is enabled._

```shell
sudo apt install software-properties-common
sudo add-apt-repository universe
```

_add the ROS 2 GPG key with apt._

```shell
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

_Add the repository to the sources list._

```shell
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

_Install: ROS, RViz, demos, tutorials._

```shell
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
```

_Add sourcing to shell startup script._

```shell
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## Installeer cpp-member-accessor 

(gebruikt in unit tests)

```shell
mkdir -p ~/sources
cd ~/sources
git clone https://github.com/hliberacki/cpp-member-accessor.git
cd cpp-member-accessor
git checkout e72109f1400a9b70cdaaae1bf0e9192900dcc332
mkdir build
cd build
cmake ..
make
sudo make install
```

## Installeer linters

```shell
sudo apt install -y python3-pip clangd cppcheck
python3 -m pip install clangd-tidy
```

## Installeer librealsense/realsense-viewer 

(https://dev.intelrealsense.com/docs/compiling-librealsense-for-linux-ubuntu-guide) 

_Dependencies:_

```shell
sudo apt-get dist-upgrade
sudo apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev
sudo apt-get install git wget cmake build-essential
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at
```

_Librealsense_
```shell
mkdir -p ~/sources
cd ~/sources
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
./scripts/setup_udev_rules.sh
./scripts/patch-realsense-ubuntu-lts-hwe.sh
mkdir build && cd build
cmake ../ -DBUILD_EXAMPLES=true
make && sudo make install
```

## Install MoveIt2

```shell
sudo apt install -y \
    ros-humble-hardware-interface \
    ros-humble-generate-parameter-library \
    ros-humble-controller-interface \
    ros-humble-ros2-control \
    ros-humble-moveit  \
    ros-humble-moveit-servo
```

## Install realsense-ros

```sh
sudo apt install -y \
    ros-humble-realsense2-*
```
# Installatie

## Codebase builden

``./scripts/build.sh``

## Robot drivers starten

``./scripts/launch_drivers.sh``

## Robot drivers starten (gesimuleerd)

``./scripts/launch_drivers_fake_robot.sh``

## Applicatie starten

``./scripts/launch_app.sh``

## Applicatie starten (gesimuleerde camera)

``./scripts/launch_app_fake_detector.sh``

## Unit-Tests draaien

``./scripts/test.sh``

## Linters draaien

``./scripts/lint.sh``

## Hoe format ik broncode?

C++ en cmake bestanden (CMakeLists.txt) kunnen automatisch worden geformat. De configuratie van de formatters valt te vinden in de root onder .clang-format en .cmake-format

``./scripts/format.sh <pad-naar-document>``

en voor alle bestanden in 1 keer

``./scripts/format.sh all``
