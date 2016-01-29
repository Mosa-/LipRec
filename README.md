LipRec
======
The **LipRec** module is a visual speech recognition system for [ROS (Robot Operating System)](http://www.ros.org "Robot Operating System"). It provides a [rqt-plugin](http://wiki.ros.org/rqt "rqt-plugin"). With **LipRec** it's possible to read the lips of a human and recognize predefined commands/sentences. Incentive of the system is to support an acoustic speech recognition system in a noisy environment (for example in the industry or in a disaster prevention situation), where roboters are in action.  
Two features (area, aspect ratio) of the lips/mouth are used for the recognition. In an utterane the features creates over a period values which can seen as trajectories. With Dynamic-time-warping, a similarity measurement technique for trajectories, it's possible to perform a recognition.

#### ROS nodes
Following ROS nodes are used in the **LipRec** module:
* [rqt_liprec](https://github.com/Mosa-/LipRec/tree/master/src/rqt_liprec "rqt_liprec")
* [face_detection](https://github.com/Mosa-/face_detection "face_detection")
* [Kinect2 Bridge](https://github.com/code-iai/iai_kinect2/tree/master/kinect2_bridge "Kinect2 Bridge") 

#### Functionality
* Live recognition of commands
* Record trajectories of commands into [MongoDB](https://www.mongodb.org "MongoDB") (necessary for recognition)
* Load an utterance trajectory file for off-line recognition
* Clustering of trajectories to find ambassador for each command
* LipRecRecorder for recording recognition results and current live utterance into a file

#### Usages
