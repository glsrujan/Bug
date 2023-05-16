# Bug0 Algorithm
## Programming Assignment - CSCI5551
`1st May 2022`

`Srujan Gowdru Lingaraju`

`ID 5733332`
### Environment Setup
1. ROS - Noetic Ninjemys
2. Python3
3. Ubuntu 20.04.4 LTS (Focal Fossa)

## Python Packages used.
1) Scipy                         1.3.3
    ```bash
    sudo apt-get install python3-scipy
    ```
2) Numpy                         1.22.3
    ```bash
    sudo apt-get install python3-numpy
    ```   
## Code Structure
Class `MyBug` has 
1. Two subscriber callbacks for `/base_pose_ground_truth` and `/base_scan` ROS Topics for Robot Pose and Sensor data.
2. A Publisher for `/cmd_vel` Topic

- `bug_controller` function has the controller code to make decissions and updates the self.bug_msg for linear and angular velocities.

- Heading and Bearing are calculated using the Robot Pose and Destination location.

- Initially The Robot orients itself towards the target location and makes calculated decission based on the Laser Scan data.

- There is a field of view of **109** degrees, which is constantly monitored so that the robot doesn't crash into obstacles while turning.

- There is a gap of **0.6** that is maintained w.r.t robot and the obstacle.
If the bearing angle is above **135**, Robot orients itself and moves towards the Target.

## Instructions
1. Run ROS core
    ```bash
    roscore
    ```
2. In a new terminal Window run `stage_ros`
    ```bash
    rosrun stage_ros stageros bug-test.world
    ```
3. Extract the stage_controller Package to a catkin workspace's Source directory
4. In a new terminal Window navigate to catkin Workspace and run catkin make using the below command
    ```bash
    catkin_make
    ```
5. Source the catkin workspace
    
    example: 
    ```bash
    source devel/setup.bash
    ```
6. Navigate to ROS Package directory

    example:
    ```bash
    roscd stage_control
    ```
7.  Navigate to source directory and make `bug0_ver_2.py` as an executable script
    ```bash
    chmod +x bug0_ver_2.py
    ```
8. Run the `bug0_ver_2.py` script using the below commands
    example:
    ```bash
    rosrun stage_control bug0_ver_2.py 
    ```

### Follow the On screen Instructions and provide X and Y co-ordinates of the Destination.
> Note: Input only **int** or **float** values
