// author: Steve Ataucuri Cruz
#include <ros/ros.h>

#include "Classifier.h"

using namespace std;

int main( int argc, char** argv )
{
    ros::init(argc, argv, "training_node");
    
    string data_file;
    string save_file;
    int num;

    ros::NodeHandle pnh("~");
    pnh.getParam("data_file", data_file);
    pnh.getParam("save_file", save_file);
    pnh.getParam("num_samples", num);

    Classifier classi(data_file, save_file, "", num, TRAINING);
    
    return 0;
}
