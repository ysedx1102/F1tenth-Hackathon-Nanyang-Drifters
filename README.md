# F1tenth-Hackathon-Nanyang-Drifters

# Quick Start:
1. Follow the installation steps of the NTU Autonomous Racing GitHub. Here are the steps if you wish to open again after your first time installation and change the map.
2. Open WSL
3. Copy paste below code step-by-step in your wsl terminal(Assume you are window user):
Setting up Docker Containers:
```
cd ~/F1Tenth_Workshop_2526/install_windows/
sudo docker network create f1tenth_net
sudo docker build -t f1tenth_gym_ros .
```
Run this script to start the Docker container:
```
sudo ./run_docker_container_win.sh
```
Running the simulator:
```
source /opt/ros/foxy/setup.bash
cd sim_ws
source ./install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
After the simulator has been launched, press `Ctrl+C` to stop it.

4. Copy & paste the code below to open the sim.yaml, you should see A GNU text nano editor.
```
nano src/f1tenth_gym_ros/config/sim.yaml
```
Find these two lines of code by scrolling down.
```
    map_path: '/sim_ws/src/f1tenth_gym_ros/maps/Spielberg_map'
    map_img_ext: '.png'
```
Change the `Spielberg_map` to `Nuerburgring_map`

and then press `Ctrl+X` to exit. Press "Y" to save your changes.

5. Now, before launching the simulator again, rebuild the workplace:
```
colcon build
source install/local_setup.bash
```
6. Finally after copy & paste the following code, your map has been changed and you're ready to race!
```
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
