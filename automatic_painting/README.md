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
##2. Generation of data with preconfigured launch file

Run the preconfigured launch file and please position your kinect/asus sensor in front of a planar surface for segmentation.

```
$ roslaunch automatic_painting demo.launch
```

Once running stand of front of Kinect or Asus sensor and surrender Psi pose to generate the training data and move the left hand from left to right on a planar surface to generate the pitch and yaw angles

##3. Saving bag files

Don't worried, the launch file will record the bag file under `bags` directory

##3.1 Trainning Random Forest algorithm 

First all, convert the bag to csv file with next script:
```
$ python read_bagfile.py bags/features_date.bag
```
Type the topic number of "features" to extract the data, then this script will generate a file called output-features.csv with the numbers of samples in rows. Now, train the algorithm with the data of .csv file as input:    

```
$ train_data_node -num number_of_samples -data output-features.csv -save trained_data.xml
```

