// author: Steve Ataucuri Cruz

#include <ros/ros.h>

#include "Classifier.h"

int main( int argc, char** argv )
{
    ros::init(argc, argv, "classifier_features");

    string data_load;

    ros::NodeHandle pnh("~");
    pnh.getParam("load_file", data_load);
    
    
    Classifier classi("", "", data_load, 0, TESTING);

    ros::spin(); 

    return 0;   
}
