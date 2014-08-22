// author: Steve Ataucuri Cruz

#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>

#include <unistd.h>
#include <iostream>
#include <vector>

#include <automatic_painting/Features.h>
#include <boost/shared_ptr.hpp>

/** this is the number of features of the vector */
#define NUMBER_ATTRIBUTES 20  
/** number of classes for testing */
#define NUMBER_OF_CLASSES 600

using namespace cv;
using namespace std;

/** this is a shared pointer to Features message */
typedef const boost::shared_ptr<const automatic_painting::Features> FeaturesPtr;


/** this enum indicates the mode for Classifier class, it can be training or testing mode */
enum Mode{
    TRAINING, TESTING
};

/** 
* \class Classifier
* \brief This class allows to train and testing a Random Forest Algorithm using a .csv data file
*
*/

class Classifier{
private:
    /** Node, subscriber and publisher */
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;

    /** name of file names*/
    string filename_to_load;
    string filename_to_save;
    string data_filename;

    /** Random forest object*/
    CvRTrees forest;
    
    /** Mode for training or testing */
    Mode mode_;

public:
    /**
    * Constructor that set the file names to given value
    * @param data_file file name of .csv data file
    * @param file_save file name to save training data
    * @param file_load file name to load into Random Forest algorihtm to testing
    * @param num_samples number of rows of .csv file
    * @param mode it is used for testing or training phase
    */
    Classifier(string  data_file, string file_save, string file_load,  int num_samples, Mode mode)
    {   
        
        ROS_INFO("Using file name for classifier  %s", data_file.c_str());    
    
        data_filename = ros::package::getPath("automatic_painting") + "/" + data_file;
        filename_to_load = ros::package::getPath("automatic_painting") + "/" + file_load;
        filename_to_save = ros::package::getPath("automatic_painting") + "/" + file_save;
        
        mode_ = mode;
            
        if (mode == TRAINING)
        {   
            const char* cfilename = filename_to_load.c_str();
        
            build_rtrees_classifier(num_samples, data_filename.c_str(),
             filename_to_save.c_str(), filename_to_load.c_str());

        }else
        {
            const char* cfilename = filename_to_load.c_str();
        
            if (load_classifier(cfilename) != -1)
                sub = nh.subscribe<automatic_painting::Features>("features", 1, &Classifier::testing_rtrees_classifier, this);

        }
            
    }

    /** this function reads data from a .csv file data and save it in a Mat object */
    int read_data_from_csv(const char* filename, Mat *data, Mat *classes,
                           int n_samples )
    {
        float tmp;
        FILE* f = fopen( filename, "r" );

        if( !f )
        {
            ROS_ERROR("ERROR: cannot read file %s",  filename);
            return 0;
        }
        
        for(int line = 0; line < n_samples; line++)
            for(int attribute = 0; attribute < (NUMBER_ATTRIBUTES + 1); attribute++)
            {
                if (attribute == 0)
                {
                    fscanf(f, "%f,", &tmp);
                    classes->at<float>(line, 0) = tmp; //we save class label
                }
                else
                {
                    fscanf(f, "%f,", &tmp);
                    data->at<float>(line, attribute) = tmp;
                }
            }     

        fclose(f);
        return 1; // all OK
    }

    /**
    *    This function reads data and responses from the file <filename>
    */
    static bool
    read_num_class_data( const string& filename, int var_count,
                         Mat* _data, Mat* _responses )
    {
        const int M = 1024;
        char buf[M+2];

        Mat el_ptr(1, var_count, CV_32F);
        int i;
        vector<int> responses;

        _data->release();
        _responses->release();

        FILE* f = fopen( filename.c_str(), "rt" );
        if( !f )
        {
            ROS_INFO("Could not read the database %s", filename.c_str());
            return false;
        }

        for(;;)
        {
            char* ptr;
            if( !fgets( buf, M, f ) || !strchr( buf, ',' ) )
                break;
            responses.push_back((int)buf[0]);
            ptr = buf+2;
            for( i = 0; i < var_count; i++ )
            {
                int n = 0;
                sscanf( ptr, "%f%n", &el_ptr.at<float>(i), &n );
                ptr += n + 1;
            }
            if( i < var_count )
                break;
            _data->push_back(el_ptr);
        }

        fclose(f);
        Mat(responses).copyTo(*_responses);

        ROS_INFO("The database %s", filename.c_str());

        return true;
    }

    /** this function loads data previously training from a xml file */
    int load_classifier(const char* filename_to_load)
    {
        if(filename_to_load )
        {
            // load classifier from the specified file
            forest.load( filename_to_load );

            if( forest.get_tree_count() == 0 )
            {
                ROS_INFO("Could not read the classifier %s make sure using 'roscd automatic_painting' ", filename_to_load );
                return -1;
            }

            ROS_INFO("The classifier is %s loaded with pre-trainned data.\n", filename_to_load );
            ROS_INFO("Waiting for feature vector ...");
            return 0;
        }else
            return -1;
    }

    /**
    *    testing_rtrees_classifier : evaluate a new feature vector after being training
    *    and output the class result
    */
    void testing_rtrees_classifier(FeaturesPtr &feature)
    {
        
        Mat test_sample = Mat(1, NUMBER_ATTRIBUTES, CV_32FC1);;

        float result;
        
        get_mat(&test_sample, feature);    
        
        result = forest.predict(test_sample, Mat());
        ROS_INFO("Testing feature vector: class %i -> class result (digit %d)", feature->cls, (int) result);
    }

    /**
    *    this function coverts a feature vector to matrix to be evaluated
    */
    void get_mat(Mat *mat, FeaturesPtr &feature)
    {   
        mat->at<float>(0, 0) = feature->cls;
        mat->at<float>(0, 1) = feature->mainJoint.position.x;
        mat->at<float>(0, 2) = feature->mainJoint.position.y;
        mat->at<float>(0, 3) = feature->mainJoint.position.z;
        mat->at<float>(0, 4) = feature->mainJoint.orientation.x;
        mat->at<float>(0, 5) = feature->mainJoint.orientation.y;
        mat->at<float>(0, 6) = feature->mainJoint.orientation.z;
        
        mat->at<float>(0, 7) = feature->mainJoint.orientation.w;
        mat->at<float>(0, 8) = feature->mainJoint.pitch;
        mat->at<float>(0, 9) = feature->mainJoint.yaw;
        mat->at<float>(0, 10) = feature->closestPoint.x;
        mat->at<float>(0, 11) = feature->closestPoint.y;
        mat->at<float>(0, 12) = feature->closestPoint.z;
        mat->at<float>(0, 13) = feature->basePoint.x;
        mat->at<float>(0, 14) = feature->basePoint.y;
        mat->at<float>(0, 15) = feature->basePoint.z;
        mat->at<float>(0, 16) = feature->distances[0];
        mat->at<float>(0, 17) = feature->distances[1];
        mat->at<float>(0, 18) = feature->distances[2];
        mat->at<float>(0, 19) = feature->distances[3];
        mat->at<float>(0, 20) = feature->distances[4];
    }

    /**
    *    this function train the classifier with data from csv file     
    */
    
    int build_rtrees_classifier(int num_samples, const char* data_filename,
        const char* filename_to_save, const char* filename_to_load )
    {
        Mat training_data = Mat(num_samples, NUMBER_ATTRIBUTES, CV_32FC1);
        Mat training_classifications = Mat(num_samples, 1, CV_32FC1);
        Mat var_type;
        Mat sample_idx;

        CvRTrees forest;
        int nsamples_all = 0, ntrain_samples = 0;

        int ok = read_data_from_csv(data_filename, &training_data, &training_classifications, num_samples);

        if(!ok)
        {
            ROS_INFO("Could not read the database %s", data_filename);
            return -1;
        }

        ROS_INFO("The database %s is loaded ok.", data_filename );

        nsamples_all = training_data.rows;
        ntrain_samples = (int)(nsamples_all*0.8);

        // create classifier by using <data> and <responses>
        ROS_INFO("Training the classifier ...");

        // 1. create type mask
        var_type = Mat(NUMBER_ATTRIBUTES + 1, 1, CV_8U);
        var_type.setTo(Scalar(CV_VAR_NUMERICAL)); 
        var_type.at<uchar>(NUMBER_ATTRIBUTES, 0) = CV_VAR_CATEGORICAL;

        CvRTParams params = CvRTParams(25,5,0,false,15,0,false,4,100,0.01f,CV_TERMCRIT_ITER | CV_TERMCRIT_EPS );

        // 2. train classifier
        forest.train(training_data, CV_ROW_SAMPLE, training_classifications, Mat(), Mat(), var_type, Mat(),
                     params);

        ROS_INFO("Training finished");


        // perform classifier testing and report results

        Mat test_sample;
        int correct_class = 0;
        int wrong_class = 0;
        double result; // value returned from a prediction
        int false_positives [NUMBER_OF_CLASSES] = {0,0,0,0,0,0,0,0,0,0};

        ROS_INFO( "Using testing database: %s\n\n", data_filename);
        ROS_INFO(" Class \t| Training Range Yaw \t| Class Output");        
            
        for (int tsample = 0; tsample < nsamples_all ; tsample++)
        {
            // extract a row from the testing matrix
            test_sample = training_data.row(tsample);

            // run random forest prediction

            result = forest.predict(test_sample, Mat());

            ROS_INFO(" %i \t   [%f]\t\t   %d ", test_sample.at<float>(0,0), test_sample.at<float>(0,9), (int) result);

            // if the prediction and the (true) testing classification are the same
            // (N.B. openCV uses a floating point decision tree implementation!)

            if (fabs(result - training_classifications.at<float>(tsample, 0))
                    >= FLT_EPSILON)
            {
                // if they differ more than floating point error => wrong class
                wrong_class++;
                false_positives[(int) result]++;

            }
            else
                correct_class++;
        }

        // Save Random Trees classifier to xml file 
        if(filename_to_save)
        { 
            forest.save( filename_to_save );
            ROS_INFO("The classifier has saved the data in %s", filename_to_save);
        }

    }
};