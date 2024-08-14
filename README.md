# ROS Workshop

## Basic Overview & Installation

### Day 1

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

### Prerequisites

Before installing ROS 2, make sure you have Git installed. You can install Git using the following command:

```bash
sudo apt-get update
sudo apt-get install git
```
### Official Documentation

   https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
   
### [Installation](https://github.com/AnandhakrishnanT/ros2direct)

#### ROS 2 Installation Script

You can use the following script to automatically detect your Ubuntu release and install a compatible ROS 2 distro:

```bash
sudo git clone https://github.com/linorobot/ros2me

cd ros2me

./install

```

#### ROS 1

- **Ubuntu 14.04** - ROS Indigo
- **Ubuntu 16.04** - ROS Kinetic
- **Ubuntu 18.04** - ROS Melodic
- **Ubuntu 20.04** - ROS Noetic

#### ROS 2

- **Ubuntu 18.04** - ROS Eloquent Elusor
- **Ubuntu 20.04** - ROS Galactic Geochelone
- **Ubuntu 22.04** - ROS Humble Hawksbill

#### To check the current Distro installed

```bash
printenv
```
```bash
echo $ROS_DISTRO
```
### Sourcing the setup file

```bash
source /opt/ros/humble/setup.bash
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

## Day 2
 ### Creating a Package
 ```bash
ros2 pkg create my_package --build-type ament_cmake
```
### https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html
