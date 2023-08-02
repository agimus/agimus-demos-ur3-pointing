# Calibration

This file explains how to plan, execute motions that move the camera in front
of a chessboard, and how to collect calibration data. 
Before starting, place a chessboard of 10 by 7 squares of size 2.62cm
on a horizontal plane in front of the robot.

## Generating and executing motions

Start the demo with the following differences:

  - no need to start react_inria node,
  - roslaunch demo_calib.launch

Move the robot to the "calib" configuration.

In the terminal where demo_calib.py has been run, copy paste the
following lines. 

```python
from calibration import Calibration, generateDataForFigaroh, checkData

calibration.generateConfigurationsAndPaths(q_init, nbConfigs = 50)
```

If no path appears in the "Path Player" of gepetto-gui after
refreshing, re-execute the last line of the later script after
increasing "calibration.nbConfigs".

in a new terminal, cd into ur10/pointing and run
```python
python -i play_path.py
```

In the same terminal, run
```python
playAllPaths(0)
```
The robot should execute the paths and collect the required data at each end
of path. After the last path has been executed,

```bash
mkdir measurements
```
and run
```python
cc.save('./measurements')
```

The procedure can be run several times if not enough data has been
acquired the first time. For that, it is recommended

  - to go back to the "calib" configuration,
  - to save the data in another directory to avoid overwriting the previous data.

## Hand eye calibration

1. Copy your previous measurements into visp folder:

```bash
cp ./measurements/* /root/devel/src/visp/build-rel/tutorial/calibration

```
2. Generate the cPo poses with your chessboard parameters (at visp/build-rel/tutorial/calibration):
```bash
./tutorial-chessboard-pose --square_size 0.0262 --input image-%d.png --intrinsic camera.xml --output pose_cPo_%d.yaml
```
3. Generate the eMc TF:
```bash
./tutorial-hand-eye-calibration --data-path . --fPe pose_fPe_%d.yaml --cPo pose_cPo_%d.yaml --output eMc.yaml
```
3. Copy the eMc.txt file. Do not use eMc.yaml as it is not using the same convention of agimus:

### Updating the model

The calibration procedure computes the pose of the optical frame
"camera_color_optical_frame" in the end-effector link, here "ref_camera_link".
The problem is that this transformation is hard-coded in the camera model.
We therefore need to compute a new transform between "ur3_d435_mount_link"
and "ref_camera_link" in such a way that the pose of the
"camera_color_optical_frame" in "ur3_d435_mount_link" is correct. For that,
we denote by

  - m ur3_d435_mount_link,
  - c camera_color_optical_frame,
  - e ref_camera_link (end effector).

The desired value of mMc is given by:

  mMc_desired = mMe * eMc_measured

where mMe is the pose of "ref_camera_link" in "ur3_d435_mount_link".
eMc_measured is the result of hand eye calibration. As explained above,
eMc is provided by the camera urdf model that we do not want to modify. We
therefore need to modify mMe into mMe_new to get the same mMc_desired:

  mMc_desired = mMe_new * eMc_provided

Therefore

  mMe_new = mMe * eMc_measured * eMc_provided.inverse()

1. In file calibration.py at agimus-demos/ur3/pointing/scripts/calibration.py, update eMc_measured with eMc.txt values
2. Run the script to compute mMe_new + xyz and rpy camera position
```bash
python -i calibration.py
```
```python
xyz
rpy
```
3. Update the model of the camera at agimus-demos/ur3/pointing/config/calibrated-params.yaml
