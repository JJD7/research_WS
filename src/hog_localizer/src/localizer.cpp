#include <hog_localizer/localizer.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#include <cv_bridge/cv_bridge.h>
#pragma GCC diagnostic pop

bool vis = true;
int match_method = 0;
int indx = 0;

# define PI           3.14159265358979323846

Localizer::Localizer(ros::NodeHandle& nh)
{
    frameSize_x = 256; //initial value based on map image size. TODO! Need to code this to be dependant in input image size
    frameSize_y = 128; //initial value based on map image size. TODO! Need to code this to be dependant in input image size
    blockSize = 32;     //Hog argument
    blockStride = 16;   //Hog argument
    cellSize = 16;      //Hog argument
    gradientBinSize = 9;//Hog argument


    // **** Onboard Camera Paramters ****
    camRes_x = 1280;
    camRes_y = 720;
    h_FOV = 22.08*PI/180;
    v_FOV = 16.56*PI/180;

    // **** Initialize Map ****
    mapImgDIR = "/home/jd/research_ws/src/hog_localizer/map/keas1.png";
    mapScale = 3.15; //pixels per meter
    loadMap();
    segmentMap(); //Segment map into smaller frames for onboard image registration

    // **** init ROS publisher(s) ****
    pose_estimate = nh_.advertise<geometry_msgs::PoseStamped>("estimated_pose",1);

}

void Localizer::loadMap()
{
    ROS_INFO("Loading Map image");
    mapImg = cv::imread(mapImgDIR.c_str(), CV_LOAD_IMAGE_COLOR);
    mapHog = mapImg.clone();
    mapORB = mapImg.clone();
    if(mapImg.empty() ) // Check for invalid input
    {
        ROS_ERROR("Could not open or find the map image");
        ros::shutdown();
    }

    if(vis)
    {
        //cv::imshow("map", mapImg);
        //cv::waitKey(0);
    }
}

void Localizer::segmentMap()
{
    ROS_INFO("Segmenting Map and Collecting HOG Descriptors of Map Segments");
    //TODO Adjust frame size to match onboard images.

    int w = (int)mapImg.cols/frameSize_x;
    int h = (int)mapImg.rows/frameSize_y;

    numSegments = w*h;

    cv::cvtColor(mapImg, grayMapImg, CV_BGR2GRAY);
    cv::Mat im = grayMapImg.clone();

    int vectIndex = 0;

    for(int j = 0; j < h*frameSize_y; j += frameSize_y) //for all rows
    {
        for(int i=0; i < w*frameSize_x; i += frameSize_x) // for all colums
        {
            frames.push_back(mapFrame()); //Add new mapframe object to the frames vector

            cv::Rect roi = cv::Rect(i,j, frameSize_x, frameSize_y); //Create a rect around segment of overall map image
            frames[vectIndex].imgFrame = cv::Mat(im, roi).clone();
            frames[vectIndex].px = i;   //record image frame x and y location. NOTE: this is the top left corner of image frame in pixels
            frames[vectIndex].py = j;

//            // Set HOG arguments and comput the descriptors
//            frames[vectIndex].hog.winSize = cv::Size(frameSize_x,frameSize_y);
//            frames[vectIndex].hog.blockSize = cv::Size(blockSize,blockSize);
//            frames[vectIndex].hog.blockStride = cv::Size(blockStride,blockStride);
//            frames[vectIndex].hog.cellSize = cv::Size(cellSize,cellSize);
//            frames[vectIndex].hog.nbins = gradientBinSize;
//            frames[vectIndex].hog.winSigma = -1;
//            frames[vectIndex].hog.histogramNormType = 0;
//            frames[vectIndex].hog.nlevels = 64;
//            frames[vectIndex].hog.compute(frames[vectIndex].imgFrame,frames[vectIndex].descriptors);//, cv::Size(0,0), cv::Size(0,0));//, frames[vectIndex].locations);
//            // Get the visualization of the HOG descriptors
//            frames[vectIndex].visFrame = getHogVis(frames[vectIndex].imgFrame, frames[vectIndex].descriptors);
//            frames[vectIndex].visFrame.copyTo(mapHog(roi));
//            //cv::imwrite( "map_HOG.jpg", mapHog );

            //ORB Feature Extraction
            ORB_Features features = getOrbFeatures(frames[vectIndex].imgFrame);
            frames[vectIndex].features = features;
            cv::drawKeypoints(frames[vectIndex].imgFrame, frames[vectIndex].features.keypoints,frames[vectIndex].visFrame);
            frames[vectIndex].visFrame.copyTo(mapORB(roi));
//            cv::imshow("map", frames[vectIndex].visFrame);
//            cv::waitKey(30);
//            usleep(250000);
            vectIndex += 1;
        }
    }
    std::cout << "Number of map segments " << frames.size() << std::endl;
    //cv::imshow("map", mapORB);
    //cv::waitKey(30);
    cv::imwrite("ORB_Gap.png", mapORB);
    MAX_FEATURES = 500;
}


Localizer::ORB_Features Localizer::getOrbFeatures(cv::Mat& im)
{
    // Detect Orb features
    ORB_Features features;
    cv::Ptr<cv::Feature2D> orb = cv::ORB::create(MAX_FEATURES);
    orb->detectAndCompute(im, cv::Mat(), features.keypoints, features.descriptors);

    return features;
}

void Localizer::_ORB_Feature_Matcher(struct ORB_Features onboardFeatures, cv::Mat& im)
{
    std::vector<cv::DMatch> good_match_Holder;
    unsigned int _numGoodMatches = 0;
    int GoodMatchIndex = 0;

    for (std::vector<Localizer::mapFrame>::iterator itt = frames.begin() ; itt != frames.end(); ++itt)
    {
        // Match features
        std::vector<std::vector<cv::DMatch> > matches;
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        matcher.knnMatch((*itt).features.descriptors, onboardFeatures.descriptors, matches, 2); //k=2: Count of best matches found per each query descriptor

        // Filter Good Matches
        std::vector<cv::DMatch> good_matches;
        for(unsigned int k = 0; k < matches.size(); k++)
        {
            if(matches[k][0].distance < 0.7 * matches[k][1].distance && matches[k][0].distance < 400)
            {
                good_matches.push_back(matches[k][0]);
            }
        }

        if(_numGoodMatches < good_matches.size())  //Found new larger set of good match count. I.e. more good matches found in current frame than previous saved frame
        {
            _numGoodMatches = good_matches.size(); //Store number of good matches for debug purposes
            GoodMatchIndex = std::distance( frames.begin(), itt ); //Store index of current best match
            good_match_Holder = good_matches;   //Store good mathces for drawing and visualizing, homography, and registration.
        }
    }


    // Draw top matches
    cv::Mat imMatches;
    cv::drawMatches(frames[GoodMatchIndex].imgFrame, frames[GoodMatchIndex].features.keypoints, im, onboardFeatures.keypoints, good_match_Holder, imMatches);
    cv::imshow("matches.jpg", imMatches);
    cv::waitKey(30);
}

cv::Mat Localizer::getHogImg(cv::Mat& im)
{

    Localizer::onboardImgFrame obImgFrame;
    cv::cvtColor(im, obImgFrame.imgFrame, CV_BGR2GRAY);

    //Rescale onboard image to map px/meter resolution of map image
    //float resizeScale = mapScale / (camRes_x / (2*mavPose.pose.position.z*tan(h_FOV/2)));

    //std::cout << "resize Scale: " << resizeScale << std::endl;

    //cv::Mat scaledImg;
    //cv::resize(obImgFrame.imgFrame, scaledImg, cv::Size(obImgFrame.imgFrame.cols*resizeScale, obImgFrame.imgFrame.rows*resizeScale), 0, 0, CV_INTER_LINEAR);

    //cv::imshow("scaled image", scaledImg);
    //cv::waitKey(30);

    //obImgFrame.imgFrame = im.clone();
    obImgFrame.hog.winSize = cv::Size(frameSize_x,frameSize_y);
    obImgFrame.hog.blockSize = cv::Size(blockSize,blockSize);
    obImgFrame.hog.blockStride = cv::Size(blockStride,blockStride);
    obImgFrame.hog.cellSize = cv::Size(cellSize,cellSize);
    obImgFrame.hog.nbins = 9;
    obImgFrame.hog.winSigma = -1;
    obImgFrame.hog.histogramNormType = 0;
    obImgFrame.hog.nlevels = 64;
    obImgFrame.hog.compute(obImgFrame.imgFrame,obImgFrame.descriptors);
    obImgFrame.visFrame = getHogVis(obImgFrame.imgFrame, obImgFrame.descriptors);

    //std::cout << "descriptor size: " << obImgFrame.descriptors.size() << std::endl;
    return obImgFrame.visFrame;
}

cv::Mat Localizer::getHogVis(cv::Mat& origImg, std::vector<float>& descriptorValues)
{
    // *** Hog visualization Code modified from originial referenced below ***
    // *** Author: Antonius Harijanto on 1/22/13.
    // *** REF: https://github.com/blacksoil/HOGVisualizer/blob/master/testCV/main.cpp
    // *** While much of the original algorithm is the same as the "get_hogdescripto_visu()" in the authors original program...
    // The code has been modified to integrate into this program. Additionally, calculations for descriptor variance has been added
    // for use in the A* path planning algorithm.

    cv::Mat color_origImg;
    cvtColor(origImg, color_origImg, CV_GRAY2RGB);

    cv::Mat visu;
    resize(color_origImg, visu, cv::Size(color_origImg.cols, color_origImg.rows));

    float radRangeForOneBin = M_PI/(float)gradientBinSize; // dividing 180° into 9 bins, how large (in rad) is one bin?

    // prepare data structure: 9 orientation / gradient strenghts for each cell
    int cells_in_x_dir = frameSize_x / cellSize;
    int cells_in_y_dir = frameSize_y / cellSize;
    float*** gradientStrengths = new float**[cells_in_y_dir];
    int** cellUpdateCounter   = new int*[cells_in_y_dir];
    for (int y=0; y<cells_in_y_dir; y++)
    {
        gradientStrengths[y] = new float*[cells_in_x_dir];
        cellUpdateCounter[y] = new int[cells_in_x_dir];
        for (int x=0; x<cells_in_x_dir; x++)
        {
            gradientStrengths[y][x] = new float[gradientBinSize];
            cellUpdateCounter[y][x] = 0;

            for (int bin=0; bin<gradientBinSize; bin++)
                gradientStrengths[y][x][bin] = 0.0;
        }
    }

    // nr of blocks = nr of cells - 1
    // since there is a new block on each cell (overlapping blocks!) but the last one
    int blocks_in_x_dir = cells_in_x_dir - 1;
    int blocks_in_y_dir = cells_in_y_dir - 1;

    // compute gradient strengths per cell
    int descriptorDataIdx = 0;

    for (int blockx=0; blockx<blocks_in_x_dir; blockx++)
    {
        for (int blocky=0; blocky<blocks_in_y_dir; blocky++)
        {
            // 4 cells per block ...
            for (int cellNr=0; cellNr<4; cellNr++)
            {
                // compute corresponding cell nr
                int cellx = blockx;
                int celly = blocky;
                if (cellNr==1) celly++;
                if (cellNr==2) cellx++;
                if (cellNr==3)
                {
                    cellx++;
                    celly++;
                }

                for (int bin=0; bin<gradientBinSize; bin++)
                {
                    float gradientStrength = descriptorValues[ descriptorDataIdx ];
                    descriptorDataIdx++;

                    gradientStrengths[celly][cellx][bin] += gradientStrength;

                } // for (all bins)


                // note: overlapping blocks lead to multiple updates of this sum!
                // we therefore keep track how often a cell was updated,
                // to compute average gradient strengths
                cellUpdateCounter[celly][cellx]++;

            } // for (all cells)


        } // for (all block x pos)
    } // for (all block y pos)


    // compute average gradient strengths
    for (int celly=0; celly<cells_in_y_dir; celly++)
    {
        for (int cellx=0; cellx<cells_in_x_dir; cellx++)
        {

            float NrUpdatesForThisCell = (float)cellUpdateCounter[celly][cellx];

            // compute average gradient strenghts for each gradient bin direction
            for (int bin=0; bin<gradientBinSize; bin++)
            {
                gradientStrengths[celly][cellx][bin] /= NrUpdatesForThisCell;
            }
        }
    }

    // draw cells
    for (int celly=0; celly<cells_in_y_dir; celly++)
    {
        for (int cellx=0; cellx<cells_in_x_dir; cellx++)
        {
            int drawX = cellx * cellSize;
            int drawY = celly * cellSize;

            int mx = drawX + cellSize/2;
            int my = drawY + cellSize/2;

            rectangle(visu, cv::Point(drawX,drawY), cv::Point((drawX+cellSize),(drawY+cellSize)), CV_RGB(100,100,100), 1);

            // draw in each cell all 9 gradient strengths
            for (int bin=0; bin<gradientBinSize; bin++)
            {
                float currentGradStrength = gradientStrengths[celly][cellx][bin];

                // no line to draw?
                if (currentGradStrength==0)
                    continue;

                float currRad = bin * radRangeForOneBin + radRangeForOneBin/2;

                float dirVecX = cos( currRad );
                float dirVecY = sin( currRad );
                float maxVecLen = cellSize/2;
                float scale = 2.5; // just a visualization scale, to see the lines better

                // compute line coordinates
                float x1 = mx - dirVecX * currentGradStrength * maxVecLen * scale;
                float y1 = my - dirVecY * currentGradStrength * maxVecLen * scale;
                float x2 = mx + dirVecX * currentGradStrength * maxVecLen * scale;
                float y2 = my + dirVecY * currentGradStrength * maxVecLen * scale;

                // draw gradient visualization
                line(visu, cv::Point(x1,y1), cv::Point(x2,y2), CV_RGB(0,255,0), 1);

            } // for (all bins)

        } // for (cellx)
    } // for (celly)


    //Compute Gradient Variance for each cell
    float*** cellVariance = new float**[cells_in_y_dir];
    std::ofstream myfile;
    myfile.open ("cellVariance.txt");
    myfile << "y, x, var\n";

    for (int y=0; y<cells_in_y_dir; y++)
    {
        cellVariance[y] = new float*[cells_in_x_dir];

        for (int x=0; x<cells_in_x_dir; x++)
        {
            cellVariance[y][x] = new float[1];

            cellVariance[y][x][0] = 0.0;    //initialize variance for each cell to 0
        }
    }

    for (int celly=0; celly<cells_in_y_dir; celly++)
    {
        for (int cellx=0; cellx<cells_in_x_dir; cellx++)
        {
            //int px = cellx * cellSize + cellSize/2;
            //int py = celly * cellSize + cellSize/2;

            float sum = 0;
            float mean = 0;

            // compute mean
            for (int bin=0; bin<gradientBinSize; bin++)
            {
                sum += gradientStrengths[celly][cellx][bin];
            }

            mean = sum/gradientBinSize;

            // compute variance
            for (int bin=0; bin<gradientBinSize; bin++)
            {
                sum += pow((gradientStrengths[celly][cellx][bin]-mean),2);
            }

            cellVariance[celly][cellx][0] = sum/(gradientBinSize-1);
            myfile << celly << ", ";
            myfile << cellx << ", ";
            myfile << cellVariance[celly][cellx][0];
            myfile << "\n";
        }
    } // computed variance for each cell
    myfile.close();


    //free memory allocated by helper data structures!
    for (int y=0; y<cells_in_y_dir; y++)
    {
        for (int x=0; x<cells_in_x_dir; x++)
        {
            delete[] gradientStrengths[y][x];
        }
        delete[] gradientStrengths[y];
        delete[] cellUpdateCounter[y];
    }
    delete[] gradientStrengths;
    delete[] cellUpdateCounter;

    return visu;

} // getHogVis


void callback(const sensor_msgs::ImageConstPtr& rect_msg, const nav_msgs::Odometry::ConstPtr& odom_msg, Localizer &localizer_ptr)
{
  cv_bridge::CvImagePtr rectIm_ptr;

  localizer_ptr.mavPose.pose = odom_msg->pose.pose;
  localizer_ptr.mavPose.header = odom_msg->header;

  try
  {
      rectIm_ptr = cv_bridge::toCvCopy(rect_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception: %s", e.what());
  }

    //Rescale onboard image to map px/meter resolution of map image
    float resizeScale = 0.5;

    cv::Mat im;
    cv::Mat scaledImg;
    cv::cvtColor(rectIm_ptr->image, im, CV_BGR2GRAY);     //Convert current image to grayscale
    cv::resize(im, scaledImg, cv::Size(im.cols*resizeScale, im.rows*resizeScale), 0, 0, CV_INTER_LINEAR); // rescale current grayscale onboard image
    Localizer::ORB_Features features = localizer_ptr.getOrbFeatures(scaledImg);        //Extract features from current grayscale image
    localizer_ptr._ORB_Feature_Matcher(features, scaledImg);
    //cv::Mat visImg;
    //visImg = rectIm_ptr->image;
    //cv::drawKeypoints(im, features.keypoints,scaledImg);

    //cv::imshow( "Source Image", scaledImg );
    //cv::waitKey(30);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"hog_localizer");
    ros::NodeHandle nh;
    ros::Duration(.1).sleep();

    if(vis)
    {
        /// Create windows
        //cv::namedWindow("Source Image", CV_WINDOW_NORMAL );
        cv::namedWindow( "matches.jpg", CV_WINDOW_NORMAL );
        //cv::namedWindow("map", CV_WINDOW_NORMAL);
        cv::startWindowThread();
    }

    Localizer localizer(nh);

    ROS_INFO("Initializing Subscribers");
    message_filters::Subscriber<sensor_msgs::Image> left_rect_sub(nh, "/left_right/left_rect/image_raw", 1);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/pinocchio/mavros/local_position/odom", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), left_rect_sub, odom_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2, boost::ref(localizer)));

    ros::spin();

    cv::destroyWindow("Source Image");
    cv::destroyWindow("frame");

}
