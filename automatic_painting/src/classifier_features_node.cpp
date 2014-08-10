#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>

#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <vector>

#include <automatic_painting/Features.h>
#include <boost/shared_ptr.hpp>

#define NUMBER_ATTRIBUTES 20

using namespace cv;
using namespace std;

typedef const boost::shared_ptr<const automatic_painting::Features> FeaturesPtr;

class Classifier{
public:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;

    string filename;
    bool enabledNode;

    CvRTrees forest;
    
    Classifier()
    {
        nh.getParam("enabled_classifier", enabledNode); // from global
        
        ros::NodeHandle pnh("~");
        pnh.getParam("load_file", filename);
 
        ROS_INFO("..... file %s enabled %i", filename.c_str(), enabledNode );
        if(enabledNode!=false)
        {
            ROS_INFO("Using file name for classifier  %s", filename.c_str());    
    
            string filename_to_load = ros::package::getPath("automatic_painting") + "/" + filename;
            const char* cfilename = filename_to_load.c_str();
            load_classifier(cfilename);

            sub = nh.subscribe<automatic_painting::Features>("features", 1, &Classifier::testing_rtrees_classifier, this);
            

        }else
            ROS_INFO("The classifier is desactivate it is on train mode ... please paint on surface!!! ");
    }

    int read_data_from_csv(const char* filename, Mat *data, Mat *classes,
                           int n_samples )
    {
        float tmp;
        FILE* f = fopen( filename, "r" );

        if( !f )
        {
            ROS_ERROR("ERROR: cannot read file %s\n",  filename);
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

    // This function reads data and responses from the file <filename>
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
            cout << "Could not read the database " << filename << endl;
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

        cout << "The database " << filename << " is loaded.\n";

        return true;
    }

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
        }else
            return -1;
    }

    void testing_rtrees_classifier(FeaturesPtr &feature)
    {
        
        Mat test_sample = Mat(1, NUMBER_ATTRIBUTES, CV_32FC1);;

        float result;
        
        get_mat(&test_sample, feature);    
        
        result = forest.predict(test_sample, Mat());
        ROS_INFO("Testing feature vector %i -> class result (digit %d)", feature->cls, (int) result);
    
   
    }

    void get_mat(Mat *mat, FeaturesPtr &feature)
    {   
        mat->at<float>(0, 0) = feature->cls;
        mat->at<float>(0, 1) = feature->rightHand.position.x;
        mat->at<float>(0, 2) = feature->rightHand.position.y;
        mat->at<float>(0, 3) = feature->rightHand.position.z;
        mat->at<float>(0, 4) = feature->rightHand.orientation.x;
        mat->at<float>(0, 5) = feature->rightHand.orientation.y;
        mat->at<float>(0, 6) = feature->rightHand.orientation.z;
        
        mat->at<float>(0, 7) = feature->rightHand.orientation.w;
        mat->at<float>(0, 8) = feature->rightHand.pitch;
        mat->at<float>(0, 9) = feature->rightHand.yaw;
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

/*

    static void test_and_save_classifier(const Ptr<StatModel>& model,
        const Mat& data, const Mat& responses,  int ntrain_samples,
        int rdelta, const string& filename_to_save)
    {
        int i, nsamples_all = data.rows;
        double train_hr = 0, test_hr = 0;

        // compute prediction error on train and test data
        for( i = 0; i < nsamples_all; i++ )
        {
            Mat sample = data.row(i);

            float r = model->predict( sample );
            r = std::abs(r + rdelta - responses.at<int>(i)) <= FLT_EPSILON ? 1.f : 0.f;

            if( i < ntrain_samples )
                train_hr += r;
            else
                test_hr += r;
        }

        test_hr /= nsamples_all - ntrain_samples;
        train_hr = ntrain_samples > 0 ? train_hr/ntrain_samples : 1.;

        printf( "Recognition rate: train = %.1f%%, test = %.1f%%\n",
                train_hr*100., test_hr*100. );

        if( !filename_to_save.empty() )
        {
            model->save( filename_to_save );
        }
    }
    
    static Ptr<TrainData>
    prepare_train_data(const Mat& data, const Mat& responses, int ntrain_samples)
    {
        Mat sample_idx = Mat::zeros( 1, data.rows, CV_8U );
        Mat train_samples = sample_idx.colRange(0, ntrain_samples);
        train_samples.setTo(Scalar::all(1));

        int nvars = data.cols;
        Mat var_type( nvars + 1, 1, CV_8U );
        var_type.setTo(Scalar::all(VAR_ORDERED));
        var_type.at<uchar>(nvars) = VAR_CATEGORICAL;

        return TrainData::create(data, ROW_SAMPLE, responses,
                                 noArray(), sample_idx, noArray(), var_type);
    }
    
    static bool build_rtrees_classifier( const string& data_filename,
        const string& filename_to_save, const string& filename_to_load )
    {
        Mat data;
        Mat responses;
        bool ok = read_num_class_data( data_filename, NUMBER_ATTRIBUTES, &data, &responses );
        if( !ok )
            return ok;

        Ptr<RTrees> model;

        int nsamples_all = data.rows;
        int ntrain_samples = (int)(nsamples_all*0.8);

        // Create or load Random Trees classifier
        if( !filename_to_load.empty() )
        {
            model = load_classifier<RTrees>(filename_to_load);
            if( model.empty() )
                return false;
            ntrain_samples = 0;
        }
        else
        {
            // create classifier by using <data> and <responses>
            cout << "Training the classifier ...\n";
            Ptr<TrainData> tdata = prepare_train_data(data, responses, ntrain_samples);
            model = StatModel::train<RTrees>(tdata, RTrees::Params(10,10,0,false,15,Mat(),true,4,TC(100,0.01f)));
            cout << endl;
        }

        //test_and_save_classifier(model, data, responses, ntrain_samples, 0, filename_to_save);
        cout << "Number of trees: " << model->getRoots().size() << endl;

        // Print variable importance
        Mat var_importance = model->getVarImportance();
        if( !var_importance.empty() )
        {
            double rt_imp_sum = sum( var_importance )[0];
            printf("var#\timportance (in %%):\n");
            int i, n = (int)var_importance.total();
            for( i = 0; i < n; i++ )
                printf( "%-2d\t%-4.1f\n", i, 100.f*var_importance.at<float>(i)/rt_imp_sum);
        }

        return true;
    }


*/        

};




int main( int argc, char** argv )
{
    ros::init(argc, argv, "classifier_features");
    
    Classifier classi;

    ros::spin(); 
    return 0;   
}
