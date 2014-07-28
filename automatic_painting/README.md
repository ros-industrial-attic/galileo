##1. Steps to run the code
Please clone the git repository into a workspace with catkin
```
$ git clone git@github.com:ros-industrial-consortium/galileo.git
```
```
$ catking_make install 
```
```
$ source devel/setup.bash
```
In another console:
```
$ roslaunch openni_launch openni.launch
```
##2. Generation of data with preconfigured launch file

Run the preconfigured launch file
```
$ roslaunch automatic_painting demo.launch
```
Once running stand of front of Kinect or Asus sensor and surrender Psi pose to generate the trainning data

##3. Saving bag files

Open other console doing:
```
$ rosbag record -o features /features
```
##3.1 Trainning Random Forest algorithm 

First convert the bag to csv file
```
$ python read_bagfile.py features.bag
```
Type a topic number to extract the data, this script will generate a file called output-features.csv. Now, trainning the forest   

```
$ ../../../install/lib/galileo/train_data_node -data output-features.csv
```

