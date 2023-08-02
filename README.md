# Development instructions

These instructions are meant to help run the demo in simulation or an a real
robot.

## I- Turning the robot on

1. Turn the robot on, click on the red button in the lower left part of the
   tablet, click on "ON", verify that the two first steps on the screen turn
   green. Click on "START". The remaining states should turn green.
2. Load the file ros.urp on the tablet.
ros.urp uses the URCap External Control. See https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap
3. Verify that the robot is in the home position. The robot can automatically
   move to the position by entering "Home" and hold the button
   "Move robot to New Position".
   Sometime,the robot will go the protective state to prevent collision.
   In that case, rotate the wrist to a safe state and try again.
4. In the Installation tab, check the Safety => Robot Limits section to ensure
   the robot is in the least restricted conifiguration.

to point the ROS_MASTER_URI, HPP_HOST, ROS_IP to the right configuration.
6. The computer connected to robot should have the Ethernet configuration as following
    IPv4 addr: 192.168.56.1
    Netmask  : 255.255.255.0
    Gateway  : 192.168.56.1

7. This needs to be done only once: In the docker add --privileged parameter to enable running realsense camera in docker image
For example:
docker run -it --rm --name ur3 --cap-add NET_ADMIN -v "/dev:/dev" --privileged --net=host gitlab.laas.fr:4567/rob4fam/docker/ur10-pointing:5

Copy the files https://github.com/IntelRealSense/librealsense/blob/master/config/99-realsense-libusb.rules to /etc/udev/rules.d/ before connecting the camera


## II- 3D models (this needs to be done only once)

For Onshape, an account is needed before downloading stl file

Camera mount: https://cad.onshape.com/documents/e21ee279b60811091f599ced/w/b0798cf871b5308cc07c899f/e/5201c11ca0cbe439c3f7114c

Drill Tip mount: https://cad.onshape.com/documents/c03aa227492bc9ee7d6bcfaf/w/85e5617ae64ca6b7c1ea8b43/e/44cc71f7db254f57f77def39

Wires clamps: https://www.thingiverse.com/thing:3832407

## III- Steps to run the demo

Open several tabs in a terminal, go to directory
`/root/catkin_ws/src/agimus-demos/ur3/pointing/launch` and type the following
instructions

0. Follow the instructions in `README-localization.md` to launch the camera node and the localizer node.

### III-1) Launch the robot driver and controller

1. in terminal 1

In simulation:
```bash
roslaunch ./simulation.launch
```

On the robot instead:
```bash
roslaunch ur_robot_driver ur3_bringup.launch robot_ip:=192.168.56.3 robot_description_file:=$DEVEL_HPP_DIR/install/share/agimus_demos/launch/ur3_pointing_load_ur3.launch
```

If this fails, with the error message `Could not get fresh data package from robot`, turn off and on the robot by clicking on the green button in the lower left of the tablet, then on `OFF`, then `ON` then `START` and try to run the robot bringup again.

After ROS is launched successfully, press the "play" icon in the tablet
and select "Play from beginning Robot Program". In the terminal you should
read
```bash
[ INFO] [1634638700.530389390]: Robot connected to reverse interface. Ready to receive control commands.
```

2. After press "play", open new terminal (terminal 2) at `/agimus-demos/ur3/pointing/launch` and run:
```bash
python ../stop-controllers.py
```

3. in terminal 2:
```bash
roslaunch ./demo.launch simulation:=true
```
or on the real robot:
```bash
roslaunch ./demo.launch
```
On the robot, after the file is run, the robot should have a lock sound of the joint.

### III-2) Launch HPP

4. In terminal 3
```bash
hppcorbaserver
```

5. In terminal 4
```bash
gepetto-gui
```

6. In terminal 5
```bash
python -i demo_vision.py
```
7. Move the sensor in front of the part, in python terminal:
```
ps.setInitialConfig(q_init)
ps.addGoalConfig(q_calib)
id = ps.solve()
pg.demo_execute(id)
pg.planTo("calib")
```
TODO:
in gepetto, look for the path : (file:///home/react/devel/install/share/doc/agimus-doc/doxygen-html/tutorial_gepetto-gui.html)


### III-3) Launch the vision module

Follow the instructions in ```README-localization.md``` to launch the INRIA REACT vision node.


### III-4) Run the demo

1. Choose a nut to grasp (handles 92,93,94). In the script terminal (terminal 5), run:
```python
i = 94
id, q = pg.planPointingPathForHole(i, q_init, 50)
pg.demo_execute(id)
```
2. Choose a screw to release the nut on (handles 68 to 91). In the script terminal (terminal 5), run:
```python
i = 75
q_init = ri.getCurrentConfig(q_init)
id, q = pg.planPointingPathForHole(i, q_init, 50)
pg.demo_execute(id)
```

