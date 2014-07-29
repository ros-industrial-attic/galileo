#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>
#include <stdio.h>
#include <unistd.h>

#define NUMBER_OF_TRAINING_DATA 124
#define NUMBER_ATTRIBUTES 20
#define NUMBER_OF_TESTING_SAMPLES 124

#define NUMBER_OF_CLASSES 600

using namespace cv;

int read_data_from_csv(const char* filename, Mat *data, Mat *classes,
                       int n_samples )
{
    float tmp;
    FILE* f = fopen( filename, "r" );

    if( !f )
    {
        printf("ERROR: cannot read file %s\n",  filename);
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
                // printf("%f,", data.at<float>(line, attribute));

            }
        }
    

    fclose(f);

    return 1; // all OK
}

static
int build_rtrees_classifier(int num_samples, char* data_filename,
    char* filename_to_save, char* filename_to_load )
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
        printf( "Could not read the database %s\n", data_filename );
        return -1;
    }

    printf( "The database %s is loaded ok.\n", data_filename );

    nsamples_all = training_data.rows;
    ntrain_samples = (int)(nsamples_all*0.8);

    // Create or load Random Trees classifier
    if( filename_to_load )
    {
        // load classifier from the specified file
        forest.load( filename_to_load );
        ntrain_samples = 0;
        if( forest.get_tree_count() == 0 )
        {
            printf( "Could not read the classifier %s\n", filename_to_load );
            return -1;
        }
        printf( "The classifier %s is loaded with pre-trainned data.\n", filename_to_load );
    }else
    {
        // create classifier by using <data> and <responses>
        printf( "Training the classifier ...\n");

        // 1. create type mask
        /*var_type = cvCreateMat( data->cols + 1, 1, CV_8U );
        cvSet( var_type, cvScalarAll(CV_VAR_ORDERED) );
        cvSetReal1D( var_type, data->cols, CV_VAR_CATEGORICAL );*/

        var_type = Mat(NUMBER_ATTRIBUTES + 1, 1, CV_8U);
        var_type.setTo(Scalar(CV_VAR_NUMERICAL)); 
        var_type.at<uchar>(NUMBER_ATTRIBUTES, 0) = CV_VAR_CATEGORICAL;

        // 2. create sample_idx
        /*sample_idx = Mat( 1, nsamples_all, CV_8UC1 );
        {
            CvMat mat;
            cvGetCols( sample_idx, &mat, 0, ntrain_samples );
            cvSet( &mat, cvRealScalar(1) );

            cvGetCols( sample_idx, &mat, ntrain_samples, nsamples_all );
            cvSetZero( &mat );
        }*/

        CvRTParams params = CvRTParams(25,5,0,false,15,0,false,4,100,0.01f,CV_TERMCRIT_ITER | CV_TERMCRIT_EPS );
        //CvRTParams params = CvRTParams(10,10,0,false,15,0,true,4,100,0.01f,CV_TERMCRIT_ITER)

        // 3. train classifier
        forest.train(training_data, CV_ROW_SAMPLE, training_classifications, Mat(), Mat(), var_type, Mat(),
                     params);
        printf( "Training finished\n");

        // Save Random Trees classifier to file if needed
        if(filename_to_save )
        { 
            forest.save( filename_to_save );
            printf("The classifier has save the data in %s\n", filename_to_save);
        }

        // perform classifier testing and report results

        Mat test_sample;
        int correct_class = 0;
        int wrong_class = 0;
        double result; // value returned from a prediction
        int false_positives [NUMBER_OF_CLASSES] = {0,0,0,0,0,0,0,0,0,0};

        printf( "\nUsing testing database: %s\n\n", data_filename);

        for (int tsample = 0; tsample < nsamples_all ; tsample++)
        {
            // extract a row from the testing matrix
            test_sample = training_data.row(tsample);

            // run random forest prediction

            result = forest.predict(test_sample, Mat());

            printf("Testing Sample %i -> class result (digit %d)\n", tsample, (int) result);

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

    }

}


int main( int argc, char** argv )
{
    //ros::init(argc, argv, "galileo");
    //ros::NodeHandle node;
    
    char* filename_to_save = 0;
    char* filename_to_load = 0;
    char default_data_filename[] = "./output-features.csv";
    char* data_filename = default_data_filename;
    int method = 0;
    int num_samples = 0;
    int i;
    
    for( i = 1; i < argc; i++ )
    {
        if( strcmp(argv[i],"-num") == 0 ) // flag "-data filename.csv"
        {
            i++;
            num_samples = atoi(argv[i]);
        }
        else if( strcmp(argv[i],"-data") == 0 ) // flag "-data filename.csv"
        {
            i++;
            data_filename = argv[i];
        }
        else if( strcmp(argv[i],"-save") == 0 ) // flag "-save filename.xml"
        {
            i++;
            filename_to_save = argv[i];
        }
        else if( strcmp(argv[i],"-load") == 0) // flag "-load filename.xml"
        {
            i++;
            filename_to_load = argv[i];
        }
        else
            break;

    }
    
    if (i < 1)
        printf("Please provide .csv file \n");
    else
        build_rtrees_classifier(num_samples, data_filename, filename_to_save, filename_to_load);


    return 0;   
}
