##1. Steps to run the code
Please clone the git repository into a workspace with catkin 

```
$ mkdir catkin_ws
$ cd catkin_ws
$ mkdir src
```
Go to `src/` and clone the source from.

```
$ git clone git@github.com:ros-industrial-consortium/galileo.git
$ catking_make install 
$ source devel/setup.bash
```

##2. Generation of data with preconfigured launch file

Run the preconfigured launch file and please position your kinect/asus sensor in front of a planar surface for segmentation.


```
$ roscd automatic_painting
$ roslaunch automatic_painting start.launch
```
If roscd fails, remember to set the ROS_PACKAGE_PATH variable in your terminal.
```
$ export ROS_PACKAGE_PATH=$~/path_to_workspace/src:$ROS_PACKAGE_PATH
```

Once running stand of front of Kinect or Asus sensor and surrender Psi pose to be detected and It will generate features vectors. Then start painting a surface doing movements(with right hand) from left to right on a planar surface to generate the features.

##3. Saving bag files

Don't worried, the launch file will record the bag file under `bags` directory

##3.1 Trainning Random Forest algorithm 

First all, convert the bag file to csv file with next script:
```
$ python read_bagfile.py bags/features_stampdate.bag
```
Type the topic number of "features" to extract the data (0), then this script will generate a file called output-features.csv with the numbers of samples in rows. 

Before training algorithm, you need to change de `number of samples` or rows that `read_bagfile.py` file generated in the `launch/training.launch` file: 

``` 
<param name="num_samples" value="new_value" />

```

Then runnnig to train node:

```
$ roslaunch automatic_painting training.launch
```

Node will train the algorithm forest and will save the data in `data.xml` file.

##4. Testing 
Run launch file in other terminal for testing a feauture vector with new data from kinect sensor:

```
$ roslaunch automatic_painting testing.launch
```
