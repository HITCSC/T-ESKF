
# T-ESKF: Transformed Error-State Kalman Filter for Consistent Visual-Inertial Navigation
We propose a novel approach to address inconsistency caused by observability mismatch in VINS. The key idea is to apply a linear time-varying transformation to the error-state. This transformation is designed to make the transformed Jacobians independent of states, thereby preserving the correct observability of the transformed system against variations in linearization points. Compared to Lie group-based methods, the proposed method is more straightforward and flexible, as it does not require redefining state variables on manifolds. We also provide a  [supplemental material](./teskf_doc/Supplementary_Material.pdf) for T-ESKF.

The code is developed on [OpenVINS](https://docs.openvins.com/index.html). During the development of the T-ESKF, utmost care is taken to preserve the original architecture of OpenVINS. This allows (standard) ESKF and FEJ-ESKF to remain in the codebase. Moreover, RI-EKF is also integrated into this codebase.

<div align=center><img src="./teskf_doc/exp08.gif" width="960"></div>

## Build 
We have tested the codebase in Ubuntu **20.04** with **ROS Noetic**. 
Please additionally refer to the [OpenVINS installation guide](https://docs.openvins.com/gs-installing.html), as all dependencies are shared with it.
```
# Assuming you have a valid installation of ros-$(Distro)-desktop-full.
# Dependencies
sudo apt-get install libeigen3-dev libboost-all-dev libceres-dev

# Build T-ESKF
mkdir -p ~/catkin_ws/src  
cd ~/catkin_ws/src 
git clone https://github.com/HITCSC-Robotics/T-ESKF 
cd ~/catkin_ws
catkin build 
source devel/setup.bash
```
## Run 
We provide examples to run ESKF, FEJ-ESKF, RI-ESKF, and T-ESKF with EuRoC dataset.  

There are two ways to run the code:
- subscribe the ROS topics 
    ```
    roslaunch ov_msckf subscribe.launch config:=euroc_mav                               # ESKF 
    roslaunch ov_msckf subscribe.launch config:=euroc_mav use_fej:=true                 # FEJ-ESKF 
    roslaunch ov_msckf subscribe.launch config:=euroc_mav use_tekf:=true tekf_method:=2 # RI-EKF 
    roslaunch ov_msckf subscribe.launch config:=euroc_mav use_tekf:=true                # T-ESKF 
    # play the bag
    rosbag play V1_01_easy.bag 
    ```

- read the rosbag directly (You need to configure the path to datasets correctly in `serial.launch`)
    ```
    roslaunch ov_msckf serial.launch config:=euroc_mav dataset:="V1_01_easy"                                # ESKF 
    roslaunch ov_msckf serial.launch config:=euroc_mav dataset:="V1_01_easy" use_fej:=true                  # FEJ-ESKF 
    roslaunch ov_msckf serial.launch config:=euroc_mav dataset:="V1_01_easy" use_tekf:=true tekf_method:=2  # RI-EKF 
    roslaunch ov_msckf serial.launch config:=euroc_mav dataset:="V1_01_easy" use_tekf:=true                 # T-ESKF 
    ```
