# Snake Robot Project
## 1. Installation
To install the Snake Robot project, follow these steps:
### 1.1. Cloning the Repository
Clone the repository into any desired folder using the following command:
```bash
git clone https://github.com/djetshu/snake_robot.git
```
### 1.2. Completing Submodule Cloning
After cloning the main repository, complete the cloning process for the submodules, which are the Arduino Libraries used in this project. Run the following commands:
```bash
git submodule init
git submodule update
```
### 1.3. Copying Arduino Libraries
Copy and paste the Arduino libraries located in your_path\snake_robot\libraries to the Arduino libraries directory, typically found at C:\Documents\Arduino\libraries.

## 2. Brief code explanation
### 2.1. motor_encoder_interrupt_pid
The [motor_encoder_interrupt_pid](scripts/motor_encoder_interrupt_pid/motor_encoder_interrupt_pid.ino]) script controls two position motors for orientation purposes through the Serial Terminal. It incorporates feedback from encoders and utilizes a PID controller to manage position. The pins utilized are aligned with the PCB's.

## 3. Testing
[![Video](http://img.youtube.com/vi/pFnQ5ds0D3Y/0.jpg)](http://www.youtube.com/watch?v=pFnQ5ds0D3Y)

Click the image above to view a test video demonstrating the Snake Robot in action.
