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
In another console run openni to generate Point Cloud information:
```
$ roslaunch openni_launch openni.launch
``` 
##2. Generation of data with preconfigured launch file

Run the preconfigured launch file
```
$ roslaunch automatic_painting demo.launch
```
Once running stand of front of Kinect or Asus sensor and surrender Psi pose to generate the trainning data and move the left hand from left to right upper a planar surface. 

##3. Saving bag files

Open another console and save a bag file:
```
$ rosbag record -o features /features
```
##3.1 Trainning Random Forest algorithm 

First convert the bag to csv file with next script
```
$ python read_bagfile.py features_date.bag
```
Type the topic number of "features" to extract the data, then this script will generate a file called output-features.csv with the numbers of samples in rows. Now, train the algorithm with the data of .csv file as input:    

```
$ ../../../install/lib/automatic_painting/train_data_node -num number_of_samples -data output-features.csv -save trained_data.xml
```

