# ROS2 Workshop

![ROS-2_logo](https://github.com/user-attachments/assets/734acefc-bf0a-456d-a5cb-c2e577d20cf5)

## Basic Overview & Installation

---

### ROS 1 Distributions

- **Box Turtle**
- **C Turtle**
- **Diamondback**
- **Electric Emys**
- **Fuerte Turtle**
- **Groovy Galapagos**
- **Hydro Medusa**
- **Indigo Igloo**
- **Jade Turtle**
- **Kinetic Kame**
- **Lunar Loggerhead**
- **Melodic Morenia**
- **Noetic Ninjemys**

### ROS 2 Distributions

- **Ardent Apalone**
- **Bouncy Bolson**
- **Crystal Clemmys**
- **Dashing Diademata**
- **Eloquent Elusor**
- **Foxy Fitzroy**
- **Galactic Geochelone**
- **Humble Hawksbill**
- **Iron Irwini**
- **Jazzy Jalisco**

---
![compatible-robots-ros-en](https://github.com/user-attachments/assets/cda26829-5a03-496b-922d-2ce9b1d6437d)

### Prerequisites

Before installing ROS 2, make sure you have Git installed. You can install Git using the following command:

```bash
sudo apt-get update
sudo apt-get install git
```
### Official Documentation

   https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
   
#### ROS 1

- **Ubuntu 14.04** - ROS Indigo
- **Ubuntu 16.04** - ROS Kinetic
- **Ubuntu 18.04** - ROS Melodic
- **Ubuntu 20.04** - ROS Noetic

#### ROS 2

- **Ubuntu 18.04** - ROS Eloquent Elusor
- **Ubuntu 20.04** - ROS Galactic Geochelone
- **Ubuntu 22.04** - ROS Humble Hawksbill

### If your getting Sudoers error , see below :
 First Switch to root user
 ```bash
su
```
if getting authentication failure error , Check User Privileges: Make sure your user is in the sudo group. You can verify this by running:
```bash
groups <your-username>
```
If your username is listed under sudo, you have the correct privileges. If not, you'll need to add your user to the sudo group

use  'whoami' command to check user
```bash
whoami
```
then , 
```bash
apt install sudo

usermod -aG sudo <your_username>

exit
```
After that restart your virtual machine
```bash
reboot
```

#### ROS 2 Installation Script

You can use the following script to automatically detect your Ubuntu release and install a compatible ROS 2 distro:

```bash
sudo git clone https://github.com/linorobot/ros2me

cd ros2me

./install

```

#### To check the current Distro installed

```bash
printenv
```
```bash
echo $ROS_DISTRO
```
### Add sourcing to your shell startup script

```bash
gedit .bashrc

source /opt/ros/humble/setup.bash
```
OR
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
### By default, ROS 2 communication is not limited to localhost. You can set the environment variable with the following command:
```bash
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
```

## Creating a Workspace
```bash
mkdir -p ~/ws_name/src
```

### Initialize the Workspace 
```bash
colcon build
```
#### The colcon tool is used for building ROS 2 packages. If colcon is not installed, you can install it using:
```bash
sudo apt-get install python3-colcon-common-extensions
```
## Source the Workspace
```bash
source install/setup.bash
```

 ## Creating a Package

### https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html

 ```bash
ros2 pkg create my_package --build-type ament_cmake
```
