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
