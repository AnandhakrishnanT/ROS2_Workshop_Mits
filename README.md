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
mkdir -p ~/<ws_name>/src
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

 # Creating a Package

## https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html

## Using cpp

 ```bash
cd <your_workspace>/src

ros2 pkg create --build-type ament_cmake --license Apache-2.0 <package_name>

```
## Creating a Node (Simple Publisher and Subscriber)

![ROSpublisher](https://github.com/user-attachments/assets/0ba9a1b9-f4fd-4e2f-aa1d-8ef9064497d5)

#### Then navigate to src folder inside your created package and lets create our First NODE

```bash
cd <package_name>/src

gedit publisher.cpp

```
### publisher.cpp

```bash
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```
### subscriber.cpp
```bash
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std::shared_ptr<const std_msgs::msg::String> msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```
### Update the CmakeLists.txt file
```bash
cmake_minimum_required(VERSION 3.5)
project(mypackage)
 
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
 
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
 
# Declare a C++ executable for talker
add_executable(talker src/publisher.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
 
# Declare a C++ executable for listener
add_executable(listener src/subscriber.cpp)
ament_target_dependencies(listener rclcpp std_msgs)
 
# Install the executables
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME}
)
 
# Export the dependencies
ament_package()
```

#### Then come to workspace and build / source the workspace using :
```bash
cd <your_workspace>

colcon build

source install/setup.bash

```

### To Run The Talker NODE ;
```bash

ros2 run <your_package_name> talker

```
### To Run The Subscriber NODE ;
```bash

ros2 run <your_package_name> listener

```

# TURTLESIM
