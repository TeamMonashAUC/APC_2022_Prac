# APC_2022_Prac

This is the repository that contains workspace and packages used for the Autonomous Programming Challenge in the year 2022. The information stated below will be based on what workspaces and packages are provided.

## autoware.ai
This is initially the workspace that is copied from the official Autoware AI Github repository. This contains the majority of the code that performs computation for object detection, localization, prediction etc. The version currently in this repository is modified to fit for Airsim simulator which initially is not able to.

## catkin_ws
This workspace contains some packages or files that initially is not within the autoware.ai repository. As it is built differently than autoware.ai workspace( catkin_ws built using command catkin_make while autoware.ai is built using colcon build command), some launch files are able to launch here but not in autoware.ai.

## Special instructions for using and modifying autoware.ai
As autoware.ai can be built via different commands, one with or without CUDA support. (If you have an Nvidia GPU, you should have CUDA support available. If you have AMD or Intel GPU used, CUDA support is not available. )


### If you Git Clone for the first time, read this section:
You should have ROS melodic available if you are using Ubuntu 18.04.
 
Install Qt (this version below is for Ubuntu 18.04):
Follow this link: https://lucidar.me/en/dev-c-cpp/how-to-install-qt-creator-on-ubuntu-18-04/ 

If you have Nvidia GPU, install CUDA ( this version below is for Ubuntu 18.04):
#### 1) Follow this link (For installation instructions for CUDA 10.0): https://docs.nvidia.com/cuda/archive/10.0/cuda-installation-guide-linux/index.html 

#### 2) NOTE: To enable CUDA support on Melodic, Eigen is required to be updated.
##### WARNING: This might break your system, or the compilation of other programs
    $ cd && wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz #Download Eigen
    $ mkdir eigen && tar --strip-components=1 -xzvf eigen-3.4.0.tar.gz -C eigen #Decompress
    $ cd eigen && mkdir build && cd build && cmake .. && make && make install #Build and install
    $ cd && rm -rf 3.3.7.tar.gz && rm -rf eigen #Remove downloaded and temporary files
    
    

### If you git clone for the first time or just git pull the latest changes, read this every single time:

     
   ##### 1. Install dependencies using rosdep in autoware.ai:
       $ cd ~/APC_2022_Prac/AutonomousAirsimNeighborhood/autoware.ai
       $ rosdep update
       $ rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

   ##### 2. Compile the workspace in autoware.ai

    a. With CUDA support
       $ AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

    b. Without CUDA Support
       $ colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   
   ##### 3. Compile the workspace in catkin_ws
   
       $ cd ~/APC_2022_Prac/AutonomousAirsimNeighborhood/catkin_ws
       $ catkin_make


### GIT PUSHING EVERY SINGLE TIME:
#### Protocols for Git Pushing changes to the Github repository when working as a team
(Read all the steps in advanced, do not follow the first step without reading all of the other steps) These steps are used to avoid merging conflicts in github.

Steps:
1) Prliminary step: always have 2 copies of the Github repository that wanted to be modified. One is used for Git Pulling whenever someone does any changes on the repository you are modifying or will modify which we call it the main repo. Another one is used to do trial and error modification to see the code works as expected which we call it trial repo. 

2) Assume currently you have a main repo that has not been modified yet and a trial repo that you finish modifying, git pull the latest github repo to the main repo even though your main repo is the latest version just in case of any unexpected conflict issues. 

3) Transfer the changes from your successful trial repo into the main repo. Then, use your main repo to run the changes you made to see if it's successful or not while checking if someone is going to make changes to the Github repo in the group. Negotiate with them about who or when shall push changes if required.

4) If not successful, continue to modify and to check in the group if anyone informs the group that they are going to git    push new changes. If yes, negotiate with them about who or when shall push changes while continuing if no. If      successful, process to step 5.

5) Assume currently you have a correctly modified latest main repo and a trial repo that you finish modifying, inform        everyone in the group that you are going to Git Push. Inform them in the format of:

   a) Changes in what repository (if your team is working with multiple repo simultaneously , if one repo you can skip         this)
   
   b) Changes in what workspace (for example , catkin_ws or autoware.ai)
   
   c) Changes in what package in src. (for example, ndt_mapping or autoware/core_perception/lidar_localizer)
   
   d) Date and time you are going to git push. (if you are going to make changes very soon, please put the time ahead at       least 10 minutes, for example currently is 10:00 am and I want to push soon I will pull 10:10 am, the reason why is       that maybe someone is stuck at step 3 or step 4 )
   
6) If no one says that they are doing anything with Github repo, you shall push changes on the specified time in 5 d).      If someone is doing changes, negotiate and discuss with them.
   
7) Replace your old trial repo with a new trial repo which is the copy of the main repo that you git pushed into the        latest Github.



