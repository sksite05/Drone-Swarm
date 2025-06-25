# DRONE SWARM

## Installations for Drone Simulation
First, enable Windows Subsystem for Linux (WSL) on your PC using any YouTube video, if not doing dual boot or not using a virtual machine. Then perform the following installations:

1. **Ubuntu 20.04** (from Microsoft Store)
2. *(Use GPT for installations by specifying the versions, use CMD)*
3. **Gazebo11 Classic**
4. **ROS1 Noetic**
5. **Ardupilot, SITL**
6. **MAVProxy**
7. Connect Mission Planner and Ardupilot SITL via UDP.
8. Copy the GitHub repository of models from this page:  
   [Swarming with Ardupilot](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/swarming_ardupilot.md)
9. From now onward, we’ll follow this video (3-video series: 20, 21, 22):  
   [Video Tutorial](https://youtu.be/r15Tc6e2K7Y?si=BZBRU9V97PnklaUC)
10. Check the `iq_sim` folder in the Ubuntu folder. It should contain the models as shown in the video.
11. Use ChatGPT if you encounter any errors.
12. Use this video to configure drone model files:  
    [Model Configuration](https://youtu.be/r15Tc6e2K7Y?start=160&end=520)
13. Check models using the following command:  
    ```bash
    cd /home/wizard/catkin_ws/src/iq_sim/models
    ```
14. Use the GitHub repository to install the Sublime Text editor used in the video for editing model files. Repository:  
    [Sublime Text Installation](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/gnc_tutorial.md)  
    Copy the first code snippet in the terminal.
15. After installing, use these commands to launch Sublime:
    ```bash
    cd catkin_ws/src/
    subl .
    ```

16. After successfully editing the drone models file and the world file as shown in the video, proceed to the next steps.

---

## Starting SITL

1. First, navigate to the ArduCopter directory:
    ```bash
    cd ~/ardupilot/ArduCopter
    ```

2. Open the Gazebo model:
    ```bash
    roslaunch iq_sim runway.launch
    ```
   This opens two configured Iris drones (in my case, as I’ve added only two in the world file) on a runway. Use the video tutorial to add more drones.

3. In separate terminals, start SITL for each drone:
    - **Drone 1 SITL**:
      ```bash
      sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console -I0 --out=udp:127.0.0.1:14550
      ```
    - **Drone 2 SITL**:
      ```bash
      sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console -I1 --out=udp:127.0.0.1:14560
      ```
    - **Drone 3 SITL**:
      ```bash
      sim_vehicle.py -v ArduCopter -f gazebo-iris --map --console -I2 --out=udp:127.0.0.1:14570
      ```

---

## Common Terminal Commands
Use the following commands for drone operations:
```bash
mode guided
arm throttle
takeoff 5
mode land

```
# MULTIPLE DRONE SWARM

## Step-by-Step Procedure

1. **Install MAVROS**
Run the following command to install MAVROS:
```bash
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
```
2. **Follow Video Instructions**
Watch Video 21 only up to 4:16. No need to do the QGC part for swarming. After that, move on to the next step:
[Watch Video 21](https://youtu.be/UWsya46ZG4M?si=FxfZhbX2qkvJiC84)

3. *Error Point*
Ensure to launch the ROS Gazebo model using Point 16 from the previous "DRONE SWARM" page before starting SITL.

4. **Continue with Video 22**
Use Video 22 for additional guidance:
[Watch Video 22](https://youtu.be/kcCL0w4NbIc?si=_V7Vl--2XGDBHoHx)

5. **Reference GitHub Repository**
Refer to the following GitHub repository for multi-MAVROS drones:
[Multi-MAVROS Drones](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/multi_mavros_drones.md)

6. **Learn IQ_GNC Functions**
In this tutorial, you'll learn to use the iq_gnc functions for controlling multiple drones on the same ROS network.

7. **Review GNC Tutorial**
Go through the gnc_tutorial.md file in the repository mentioned above and copy the repo before following the video.

8. **Modify MAVROS Configuration**
Adjust the apm.launch file based on your setup:
*For simulation, use UDP:*
Use `<arg name="fcu_url" default="/dev/ttyACM0:57600" />` for simulation.
*For hardware, change to TCP if required.

9. **Individual SITL Launch**
The multi-SITL script from the video did not work for me. Instead, I launched SITL individually in separate terminals. You may try the script to see if it works for you.


## Final Steps for Square Swarm
 
1. Launch the multi-drone simulation:
   ```bash
   roslaunch iq_sim multi_drone.launch```
2. Start SITL for each drone:

*Drone 1 SITL:
   ```bash
   sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I0 --sysid=1 --out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14555
*Drone 2 SITL:
   ```bash
   sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I1 --sysid=1 --out=udp:127.0.0.1:14561 --out=udp:127.0.0.1:14565
```
*Drone 3 SITL:
   ```bash
   sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I2 --sysid=1 --out=udp:127.0.0.1:14571 --out=udp:127.0.0.1:14575
```
3. Launch the APM models:
   ```bash
   roslaunch iq_sim multi_apm.launch
4. Execute the square formation swarm:
   ```bash
   roslaunch iq_gnc multi_square.launch
1. **Install MAVROS**
Run the following command to install MAVROS:
```bash
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
```
2. **Follow Video Instructions**
Watch Video 21 only up to 4:16. No need to do the QGC part for swarming. After that, move on to the next step:
[Watch Video 21](https://youtu.be/UWsya46ZG4M?si=FxfZhbX2qkvJiC84)

3. *Error Point*
Ensure to launch the ROS Gazebo model using Point 16 from the previous "DRONE SWARM" page before starting SITL.

4. **Continue with Video 22**
Use Video 22 for additional guidance:
[Watch Video 22](https://youtu.be/kcCL0w4NbIc?si=_V7Vl--2XGDBHoHx)

5. **Reference GitHub Repository**
Refer to the following GitHub repository for multi-MAVROS drones:
[Multi-MAVROS Drones](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/multi_mavros_drones.md)

6. **Learn IQ_GNC Functions**
In this tutorial, you'll learn to use the iq_gnc functions for controlling multiple drones on the same ROS network.

7. **Review GNC Tutorial**
Go through the gnc_tutorial.md file in the repository mentioned above and copy the repo before following the video.

8. **Modify MAVROS Configuration**
Adjust the apm.launch file based on your setup:
*For simulation, use UDP:*
Use `<arg name="fcu_url" default="/dev/ttyACM0:57600" />` for simulation.
*For hardware, change to TCP if required.

9. **Individual SITL Launch**
The multi-SITL script from the video did not work for me. Instead, I launched SITL individually in separate terminals. You may try the script to see if it works for you.


## Final Steps for Square Swarm
 
1. Launch the multi-drone simulation:
   ```bash
   roslaunch iq_sim multi_drone.launch
 
2. Start SITL for each drone:
  Drone 1 SITL:
   ```bash
   sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I0 --sysid=1 --out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14555
   ```
  Drone 2 SITL:
   ```bash
   sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I1 --sysid=1 --out=udp:127.0.0.1:14561 --out=udp:127.0.0.1:14565
```
  Drone 3 SITL:
   ```bash
   sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I2 --sysid=1 --out=udp:127.0.0.1:14571 --out=udp:127.0.0.1:14575
```
3. Launch the APM models:
   ```bash
   roslaunch iq_sim multi_apm.launch
4. Execute the square formation swarm:
   ```bash
   roslaunch iq_gnc multi_square.launch

