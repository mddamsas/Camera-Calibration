#include "CameraCalibration.h"

namespace CC
{
    class Settings
    {
    public:     
        // Constructor
        Settings() : goodInput(false){}
        
        // Available input types
        enum InputType { INVALID, CAMERA, IMAGE_LIST };

        // Write serialization for this class
        void write(FileStorage& fs) const                        
        {
            fs << "{"
                        << "BoardSize_Width"  << boardSize.width
                        << "BoardSize_Height" << boardSize.height
                        << "Square_Size"         << squareSize
                        << "Calibrate_NrOfFrameToUse" << nrFrames
                        << "Calibrate_FixAspectRatio" << aspectRatio
                        << "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
                        << "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

                        << "Write_DetectedFeaturePoints" << writePoints
                        << "Write_extrinsicParameters"   << writeExtrinsics
                        << "Write_gridPoints" << writeGrid
                        << "Write_outputFileName"  << outputFileName

                        << "Show_UndistortedImage" << showUndistorsed

                        << "Input_FlipAroundHorizontalAxis" << flipVertical
                        << "Input_Delay" << delay
                        << "Window_Size" << winSize
                        << "Fast_Check" << fastCheck
                        << "Input" << input
                << "}";
        }

        // Read serialization for this class
        void read(const FileNode& node)                          
        {
            node["BoardSize_Width" ] >> boardSize.width;
            node["BoardSize_Height"] >> boardSize.height;
            node["Square_Size"]  >> squareSize;
            node["Calibrate_NrOfFrameToUse"] >> nrFrames;
            node["Calibrate_FixAspectRatio"] >> aspectRatio;
            node["Write_DetectedFeaturePoints"] >> writePoints;
            node["Write_extrinsicParameters"] >> writeExtrinsics;
            node["Write_gridPoints"] >> writeGrid;
            node["Write_outputFileName"] >> outputFileName;
            node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
            node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
            node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
            node["Show_UndistortedImage"] >> showUndistorsed;
            node["Input"] >> input;
            node["Input_Delay"] >> delay;
            node["Window_Size"] >> winSize;
            node["Fast_Check"] >> fastCheck;
            node["Fix_K1"] >> fixK1;
            node["Fix_K2"] >> fixK2;
            node["Fix_K3"] >> fixK3;
            node["Fix_K4"] >> fixK4;
            node["Fix_K5"] >> fixK5;
            // Validate inputs
            validate();
       
        }

        // To Validate the inputs read from the xml file
        void validate()
        {
            goodInput = true;
            if (boardSize.width <= 0 || boardSize.height <= 0)
            {
                cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << endl;
                goodInput = false;
            }
            if (squareSize <= 10e-6)
            {
                cerr << "Invalid square size " << squareSize << endl;
                goodInput = false;
            }
            if (nrFrames <= 0)
            {
                cerr << "Invalid number of frames " << nrFrames << endl;
                goodInput = false;
            }

            // Check for valid input
            if (input.empty())      
                inputType = INVALID;
            else
            {
                if (input[0] >= '0' && input[0] <= '9')
                {
                    stringstream ss(input);
                    ss >> cameraID;
                    inputType = CAMERA;
                }
                else
                {
                    if (isListOfImages(input) && readStringList(input, imageList))
                    {
                        inputType = IMAGE_LIST;
                        nrFrames = (nrFrames < (int)imageList.size()) ? nrFrames : (int)imageList.size();
                    }

                }
                if (inputType == CAMERA)
                    inputCapture.open(cameraID);
                if (inputType != IMAGE_LIST && !inputCapture.isOpened())
                    inputType = INVALID;
            }
            if (inputType == INVALID)
            {
                cerr << " Input does not exist: " << input;
                goodInput = false;
            }

            flag = 0;
            if(calibFixPrincipalPoint) flag |= CALIB_FIX_PRINCIPAL_POINT;
            if(calibZeroTangentDist)   flag |= CALIB_ZERO_TANGENT_DIST;
            if(aspectRatio)            flag |= CALIB_FIX_ASPECT_RATIO;
            if(fixK1)                  flag |= CALIB_FIX_K1;
            if(fixK2)                  flag |= CALIB_FIX_K2;
            if(fixK3)                  flag |= CALIB_FIX_K3;
            if(fixK4)                  flag |= CALIB_FIX_K4;
            if(fixK5)                  flag |= CALIB_FIX_K5;

            atImageList = 0;
        }

        // Capture next view or to load next image
        Mat nextImage()
        {
            Mat result;
            if( inputCapture.isOpened() )
            {
                Mat view0;
                inputCapture >> view0;
                view0.copyTo(result);
            }
            else if( atImageList < imageList.size() )
                result = imread(imageList[atImageList++], IMREAD_COLOR);
            return result;
        }

        // Read string list of images from xml file
        static bool readStringList( const string& filename, vector<string>& l )
        {
            l.clear();
            FileStorage fs(filename, FileStorage::READ);
            if( !fs.isOpened() )
                return false;
            FileNode n = fs.getFirstTopLevelNode();
            if( n.type() != FileNode::SEQ )
                return false;
            FileNodeIterator it = n.begin(), it_end = n.end();
            for( ; it != it_end; ++it )
                l.push_back((string)*it);
            return true;
        }
        
        // Check the file extension to validate input file
        static bool isListOfImages( const string& filename)
        {
            string s(filename);
            // Look for file extension
            if( s.find(".xml") == string::npos && s.find(".yaml") == string::npos && s.find(".yml") == string::npos )
                return false;
            else
                return true;
        } 

    public:
        Size boardSize;              // The size of the board -> Number of items by width and height
        float squareSize;            // The size of a square in your defined unit (point, millimeter,etc).
        int nrFrames;                // The number of frames to use from the input for calibration
        float aspectRatio;           // The aspect ratio
        int delay;                   // In case of a video input
        int winSize;                 // Half of search window for cornerSubPix
        int fastCheck;               // Fast check for pattern detection 
        bool writePoints;            // Write detected feature points
        bool writeExtrinsics;        // Write extrinsic parameters
        bool writeGrid;              // Write refined 3D target grid points
        bool calibZeroTangentDist;   // Assume zero tangential distortion
        bool calibFixPrincipalPoint; // Fix the principal point at the center
        bool flipVertical;           // Flip the captured images around the horizontal axis
        string outputFileName;       // The name of the file where to write
        bool showUndistorsed;        // Show undistorted images after calibration
        string input;                // The input ->
        bool fixK1;                  // fix K1 distortion coefficient
        bool fixK2;                  // fix K2 distortion coefficient
        bool fixK3;                  // fix K3 distortion coefficient
        bool fixK4;                  // fix K4 distortion coefficient
        bool fixK5;                  // fix K5 distortion coefficient
        int cameraID;                // Camera ID for online calibration
        bool goodInput;              // Validate inputs
        int flag;                    // Set flags for calibration
        vector<string> imageList;    // For calibration using list of images(offline) 
        size_t atImageList;          // Index for imageLIst
        VideoCapture inputCapture;   // Capture a video for live calibration
        InputType inputType;         // Define input type
    };

    //  Read function for serialization
    static inline void read(const FileNode& node, Settings& x, const Settings& default_value = Settings())
    {
        if(node.empty())
            x = default_value;
        else
            x.read(node);
    }

    // Write function for serialization
    static inline void write(FileStorage& fs, const std::string&, const Settings& x)
    {
        x.write(fs);
    }

    // Mode of calibration
    enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

    // Function to execute camera calibration using detected patterns
    bool runCalibrationAndSave(Settings& s, Size imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                               vector<vector<Point2f> > imagePoints, float grid_width)
    {
        
        // Calculate board corner positions
        vector<vector<Point3f> > objectPoints(1);
        for( int i = 0; i < s.boardSize.height; ++i )
            for( int j = 0; j < s.boardSize.width; ++j )
                 objectPoints[0].push_back(Point3f(j*s.squareSize, i*s.squareSize, 0));
        objectPoints.resize(imagePoints.size(),objectPoints[0]);

        // Find intrinsic and extrinsic camera parameters
        vector<Mat> rvecs, tvecs;
        cameraMatrix = Mat::eye(3, 3, CV_64F);
        distCoeffs = Mat::zeros(5, 1, CV_64F);
        double rms;
        if( s.flag & CALIB_FIX_ASPECT_RATIO )   // fixed_aspect  
            cameraMatrix.at<double>(0,0) = s.aspectRatio;
        rms = calibrateCameraRO(objectPoints, imagePoints, imageSize,-1,
                                cameraMatrix, distCoeffs, rvecs, tvecs, noArray(),
                                s.flag | CALIB_USE_LU);
        cout << "Re-projection error reported by calibrateCamera: "<< rms << endl;
        bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

        // Calculate Reprojection errors
        vector<float>reprojErrs(objectPoints.size());
        vector<Point2f> imagePoints2;
        size_t totalPoints = 0;
        double totalErr = 0, err, totalAvgErr = 0;
        for( size_t i = 0; i < objectPoints.size(); ++i )
        {
            projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
            err = norm(imagePoints[i], imagePoints2, NORM_L2);
            size_t n = objectPoints[i].size();
            reprojErrs[i] = (float) std::sqrt(err*err/n);
            totalErr        += err*err;
            totalPoints     += n;
        }
        totalAvgErr = std::sqrt(totalErr/totalPoints);

        cout << (ok ? "Camera Calibration succeeded" : "Calibration failed")
             << ". avg re projection error = " << totalAvgErr << endl;

        // Save parameters
        if (ok)
        {
            FileStorage fs( s.outputFileName, FileStorage::WRITE );
            time_t tm;
            time( &tm );    
            struct tm *t2 = localtime( &tm );
            char buf[1024];
            strftime( buf, sizeof(buf), "%c", t2 );    

            fs << "calibration_time" << buf;

            if( !rvecs.empty() || !reprojErrs.empty() )
                fs << "nr_of_frames" << (int)std::max(rvecs.size(), reprojErrs.size());
            fs << "image_width" << imageSize.width;
            fs << "image_height" << imageSize.height;
            fs << "board_width" << s.boardSize.width;
            fs << "board_height" << s.boardSize.height;
            fs << "square_size" << s.squareSize;

            if( s.flag & CALIB_FIX_ASPECT_RATIO )
                fs << "fix_aspect_ratio" << s.aspectRatio;
            
            if (s.flag)
            {
                std::stringstream flagsStringStream;    
                flagsStringStream << "flags:"
                    << (s.flag & CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "")
                    << (s.flag & CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "")
                    << (s.flag & CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "")
                    << (s.flag & CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "")
                    << (s.flag & CALIB_FIX_K1 ? " +fix_k1" : "")
                    << (s.flag & CALIB_FIX_K2 ? " +fix_k2" : "")
                    << (s.flag & CALIB_FIX_K3 ? " +fix_k3" : "")
                    << (s.flag & CALIB_FIX_K4 ? " +fix_k4" : "")
                    << (s.flag & CALIB_FIX_K5 ? " +fix_k5" : "");
            
                fs.writeComment(flagsStringStream.str());
            }

            fs << "flags" << s.flag;
            fs << "camera_matrix" << cameraMatrix;
            fs << "distortion_coefficients" << distCoeffs;
            fs << "avg_reprojection_error" << totalAvgErr;
            if (s.writeExtrinsics && !reprojErrs.empty())
                fs << "per_view_reprojection_errors" << Mat(reprojErrs);

            if(s.writeExtrinsics && !rvecs.empty() && !tvecs.empty() )
            {
                CV_Assert(rvecs[0].type() == tvecs[0].type());
                Mat bigmat((int)rvecs.size(), 6, CV_MAKETYPE(rvecs[0].type(), 1));
                bool needReshapeR = rvecs[0].depth() != 1 ? true : false;
                bool needReshapeT = tvecs[0].depth() != 1 ? true : false;

                for( size_t i = 0; i < rvecs.size(); i++ )
                {
                    Mat r = bigmat(Range(int(i), int(i+1)), Range(0,3));
                    Mat t = bigmat(Range(int(i), int(i+1)), Range(3,6));

                    if(needReshapeR)
                        rvecs[i].reshape(1, 1).copyTo(r);
                    else
                    {
                        CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
                        r = rvecs[i].t();
                    }

                    if(needReshapeT)
                        tvecs[i].reshape(1, 1).copyTo(t);
                    else
                    {
                        CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
                        t = tvecs[i].t();
                    }
                }
                fs.writeComment("a set of 6-tuples (rotation vector + translation vector) for each view");
                fs << "extrinsic_parameters" << bigmat;
                bigmat.release();

            }

            if(s.writePoints && !imagePoints.empty() )
            {
                Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
                for( size_t i = 0; i < imagePoints.size(); i++ )
                {
                    Mat r = imagePtMat.row(int(i)).reshape(2, imagePtMat.cols);
                    Mat imgpti(imagePoints[i]);
                    imgpti.copyTo(r);
                }
                fs << "image_points" << imagePtMat;
                imagePtMat.release();
            }

            if( s.writeGrid && !objectPoints.empty() )
            {

                fs << "grid_points" << objectPoints[0];
            }
        }

        objectPoints.clear();
        imagePoints2.clear();
        reprojErrs.clear();
        rvecs.clear();
        tvecs.clear();

        return ok;
    }
    
    ///////////////////////////////////////   CalibrateCamera   //////////////////////////////////////////////
	/*********************************************************************************************************
	This function calibrate camera using a configuration file

	Input:
	ip_Filename                   -   Input configuration file name

    Output:
    out_camera_data.xml           -    XML file with camera parameters
	**********************************************************************************************************/
    int CalibrateCamera(const std::string& ip_Filename)
    {

        // Read input configuration file
        FileStorage fs(ip_Filename,FileStorage::READ);
        if(!fs.isOpened())
        {
            cout << "Could not open the configuration file: \"" << ip_Filename << "\"" << endl;
            return -1;
        }
        Settings s;                                   
        fs["Settings"] >> s;
        fs.release();                                 

        // Check whether input config file is invalid or not
        if (!s.goodInput)
        {
            cout << "Invalid input detected. Application stopping. " << endl;
            return -1;
        }

        // Declare variables
        float grid_width = s.squareSize * (s.boardSize.width - 1);
        vector<vector<Point2f> > imagePoints;
        vector<Point2f> pointBuf;
        Mat cameraMatrix, distCoeffs,view;
        Size imageSize;
        bool blinkOutput,found;
        int chessBoardFlags;
        int mode  = s.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;
        clock_t prevTimestamp = 0;
        const Scalar YELLOW(0,255,255), GREEN(0,255,0);
        const char ESC_KEY = 27;

        // Get input
        for(;;)
        {
            blinkOutput = false;

            view = s.nextImage(); // get next view

            if( mode == CAPTURING && imagePoints.size() >= (size_t)s.nrFrames)
            {
                if(runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints, grid_width))
                    mode = CALIBRATED;
                else
                    mode = DETECTION;
                
            }

            if(view.empty())          // If there are no more images stop the loop
            {
                // if calibration threshold was not reached yet, calibrate now
                if( mode != CALIBRATED && !imagePoints.empty() )
                    runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints, grid_width);

                break;
            }

            imageSize = view.size();  // Format input image.
            if( s.flipVertical )    
                flip( view, view, 0 );
            
            // Find pattern
            if(s.fastCheck)
                chessBoardFlags = CALIB_CB_FAST_CHECK;
            else
               chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;
            found = findChessboardCorners( view, s.boardSize, pointBuf, chessBoardFlags);   // Find feature points on the input format
            
            // Pattern found
            if (found)                // If done with success,
            {
                // improve the found corners' coordinate accuracy for chessboard
                Mat viewGray;
                cvtColor(view, viewGray, COLOR_BGR2GRAY);
                cornerSubPix( viewGray, pointBuf, Size(s.winSize,s.winSize),Size(-1,-1), 
                    TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.0001 ));
                
                if( mode == CAPTURING &&  // For camera only take new samples after delay time
                    (!s.inputCapture.isOpened() || clock() - prevTimestamp > s.delay*1e-3*CLOCKS_PER_SEC) ) 
                {
                    imagePoints.push_back(pointBuf);
                    prevTimestamp = clock();
                    blinkOutput = s.inputCapture.isOpened();
                }
                // Draw the corners.
                drawChessboardCorners( view, s.boardSize, Mat(pointBuf), found );
                viewGray.release();
            }

            // Output text
            string msg = (mode == CAPTURING) ? "Press 'c' to capture" : mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
            int baseLine = 0;
            Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
            Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);

            if( mode == CAPTURING && !s.inputCapture.isOpened())
                msg = format( "%d/%d", (int)imagePoints.size(), s.nrFrames );

            if( s.inputCapture.isOpened() && (int)imagePoints.size() > 0 && mode != CALIBRATED )
            {
                msg = format( "Press 'c' to capture, %d/%d", (int)imagePoints.size(), s.nrFrames );
                mode  = DETECTION;
            }
            putText( view, msg, textOrigin, 1, 1, mode == CALIBRATED ?  GREEN : YELLOW);

            // Save and Display the detected pattern
            if((s.inputType == Settings::IMAGE_LIST && !view.empty()) || blinkOutput )
            {
                imwrite("Image"+to_string(imagePoints.size())+".jpg",view);
            }

            if( blinkOutput )
                bitwise_not(view, view);

            imshow("Image View", view);

            // Await_input
            char key = (char)waitKey(s.inputCapture.isOpened() ? 50 : s.delay);

            if( key  == ESC_KEY )
                break;

            if( key == 'u' && mode == CALIBRATED )
                s.showUndistorsed = !s.showUndistorsed;

            if( s.inputCapture.isOpened() && key == 'g' )
            {
                mode = CAPTURING;
                imagePoints.clear();
            }

            if( s.inputCapture.isOpened() &&
                (int)imagePoints.size() > 0 && mode != CALIBRATED && key == 'c')
            {
                mode = CAPTURING;
            }  
            view.release();
            pointBuf.clear();

        }

        // Display_and_Save_the_undistorted_images
        if( s.inputType != Settings::INVALID && s.showUndistorsed )
        { 
            Mat view, rview, map1, map2;

            initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
                getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize,CV_16SC2, map1, map2);
        
            for(size_t i = 0; i < (int)imagePoints.size(); i++ )
            {
                
                if( s.inputType == Settings::IMAGE_LIST)
                    view = imread(s.imageList[i], IMREAD_COLOR);
                else
                    view = imread("Image"+to_string(i+1)+".jpg",IMREAD_COLOR);
                if(view.empty())
                    continue;
                remap(view, rview, map1, map2, INTER_LINEAR);
                imshow("Image View", rview);
                imwrite("Image"+to_string(i+1)+".jpg",rview);
                char c = (char)waitKey();
                if( c  == ESC_KEY || c == 'q' || c == 'Q' )
                    break;
            }
        }

        imagePoints.clear();

        return 0;
    }

} //ends namespce CC

