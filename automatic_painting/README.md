##1. Steps to run the code
Please clone the git repository into a workspace with catkin: 

```
$ mkdir catkin_ws
$ cd catkin_ws
$ mkdir src
```
Go to `src/` directory and clone the source from.

```
$ git clone git@github.com:ros-industrial-consortium/galileo.git
$ catking_make install 
$ source devel/setup.bash
```

##2. Data Generation 

Run the preconfigured launch file and please position your kinect/asus sensor in front of a planar surface to segment it.

```
$ roscd automatic_painting
$ roslaunch automatic_painting start.launch
```
If roscd fails, remember to set the ROS_PACKAGE_PATH variable in your terminal.
```
$ export ROS_PACKAGE_PATH=$~/path_to_workspace/src:$ROS_PACKAGE_PATH
```

Once running stand of front of Kinect or Asus sensor and surrender Psi pose to be detected and features vectors will be generated. Then start painting a planar surface doing movements from the left to the right near to surface in order to generate features vectors in a bag file. If you are right or left handed please change the ros param `joint_name` with `XN_SKEL_LEFT_HAND` or `XN_SKEL_RIGHT_HAND` of the `start.launch` file to start to paint a surface:

```
 <param name="joint_name" value="15" />
```

##2.1 Saving bag files

Don't worry about it, because the launch file `start.launch` automatically will record bag file under `bags` directory with name `features_stampdate.bag`.

##3. Trainning Random Forest algorithm 

First of all, convert the bag file to csv file with the next script:
```
$ python read_bagfile.py bags/features_stampdate.bag
```
Type the topic number of "features" to extract the data (0), then this script will generate a file called output-features.csv with the numbers of samples in rows. 

Before you start training the algorithm, you must change the `number of samples` in `launch/training.launch` file that `read_bagfile.py` has generated. 

``` 
<param name="num_samples" value="new_value" />
```

Then call `training.launch` file to train the Random Forest algorithm

```
$ roslaunch automatic_painting training.launch
```

Ros node is going to train the algorithm forest and save data in the `data.xml` file.

##3.1 Feature Vector

Vector is conformed by many varaibles that are extracted from a planar surface like: positions, distances, angles, etc. We defined all variables of a feature vector within a ROS message in `msg/Features.msg`. These are conformed by:

```
class  hand_position  hand_orientation   right_hand   closest_point_to_hand base_link_position  closest_distances_to_hand
 cls  |  x,  y,  z  |  x,  y,  z,  w  |  pitch,  yaw      | x,  y,  z   |      x,  y,  z       | d1,  d2,  d3,  d4,  d5  
```

##4. Testing 

Run `testing.launch` file in other terminal to test a new feature vector with new data that comes from kinect/asus sensor:

```
$ roslaunch automatic_painting testing.launch
```
this node is going to classify the new values of pitch and yaw. Note that `start.launch` file should be running
