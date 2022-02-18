# APC_2022_Prac

This is the repository that contains workspace and packages used for the Autonomous Programming Challenge in the year 2022. The information stated below will be based on what workspaces and packages are provided.

## autoware.ai
This is initially the workspace that is copied from the official Autoware AI Github repository. This contains the majority of the code that performs computation for object detection, localization, prediction etc. The version currently in this repository is modified to fit for Airsim simulator which initially is not able to.

## catkin_ws
This workspace contains some packages or files that initially is not within the autoware.ai repository. As it is built differently than autoware.ai workspace( catkin_ws built using command catkin_make while autoware.ai is built using colcon build command), some launch files are able to launch here but not in autoware.ai.

## Special instructions for modifying autoware.ai
As autoware.ai can be built via different commands, one with or without CUDA support. (If you have an Nvidia GPU, you should have CUDA support available. If you have AMD or Intel GPU used, CUDA support is not available. )


### Cloning for the first time:
You should have ROS melodic available if you are using Ubuntu 18.04.
 
Install Qt (this version below is for Ubuntu 18.04):
Follow this link: https://lucidar.me/en/dev-c-cpp/how-to-install-qt-creator-on-ubuntu-18-04/ 

If you have Nvidia GPU, install CUDA ( this version below is for Ubuntu 18.04):
 1) Follow this link (For installation instructions for CUDA 10.0): https://docs.nvidia.com/cuda/archive /10.0/cuda-installation-guide-linux/index.html 

   2) NOTE: To enable CUDA support on Melodic, Eigen is required to be updated.
#### WARNING: This might break your system, or the compilation of other programs
     $ cd && wget http://bitbucket.org/eigen/eigen/get/3.3.7.tar.gz #Download Eigen
     $ mkdir eigen && tar --strip-components=1 -xzvf 3.3.7.tar.gz -C eigen #Decompress
     $ cd eigen && mkdir build && cd build && cmake .. && make && make install #Build and install
     $ cd && rm -rf 3.3.7.tar.gz && rm -rf eigen #Remove downloaded and temporary files

### Git cloning for the first time and Git pulling every single time:

Markup : 1. Install dependencies using rosdep:
$ rosdep update
$ rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

   2. Compile the workspace

    1. With CUDA support
       $ AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

    2. Without CUDA Support
      $ colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release


### GIT PUSHING EVERY SINGLE TIME:
1) ALWAYS remove the log files, install files and build files in autoware.ai before Git Pushing. You can either delete it or temporarily move to other directories. THIS IS IMPORTANT because different members may have or do not have CUDA support on their laptop or pc which they compile differently using different command (check the section just above state different compiling command)

2) After Git Pushing, you can recompile again to retrieve back your files mentioned above if you deleted it using compiling command in the above section or you can just move back the files from the other directory you put in before you git push.

3) Extra: you no need to remove the additional files built in catkin_ws as we all compile workspace the same way using catkin_make. (Assumption: this assume that you are using catkin_make command to compile workspace. If you are not, discuss with the current person in-charge.)

### EXTRA NOTICE: 
If you find out that there is log files, install files and build files in the Github repository, you can recorrect it by pulling it , delete them then pushing back again.
