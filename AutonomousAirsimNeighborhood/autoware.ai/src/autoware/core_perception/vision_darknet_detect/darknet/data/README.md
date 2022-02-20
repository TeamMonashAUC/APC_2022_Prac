# YOLO Algorithm Pre-Trained Weights
The yolov3.weights is a pre-trained weights file for the yolo v3 algorithm to work. As the file is quite large which exceeds the github 100MB limit, you
will have to download the file by yourself by clicking on this link https://pjreddie.com/media/files/yolov3.weights.

After downloading the file, you will have to make a directory called data in 
$HOME/APC_2022_Prac/AutonomousAirsimNeighborhood/autoware.ai/install/vision_darknet_detect/share/vision_darknet_detect/darknet and copy the downloaded file into 
$HOME/APC_2022_Prac/AutonomousAirsimNeighborhood/autoware.ai/install/vision_darknet_detect/share/vision_darknet_detect/darknet/data

##### You can also use the command line interface to do so:
    cd $HOME/Downloads
    wget https://pjreddie.com/media/files/yolov3.weights
    cd $HOME/APC_2022_Prac/AutonomousAirsimNeighborhood/autoware.ai/install/vision_darknet_detect/share/vision_darknet_detect/darknet
    mkdir data
    cd $HOME/APC_2022_Prac/AutonomousAirsimNeighborhood/autoware.ai/install/vision_darknet_detect/share/vision_darknet_detect/darknet/data
    cp $HOME/Downloads/yolov3.weights .
