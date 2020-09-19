#ifndef __CAMERA_CALIBRATION_H__
#define __CAMERA_CALIBRATION_H__

#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

namespace CC
{

    ////////////////////////////////////// CalibrateCamera ////////////////////////////////////////////////////////
	//! @fn  int CalibrateCamera(const std::string& ip_Filename)
	/*! @brief This function has an essential input configuration file in XML format. In the configuration file 
    you may choose to use camera as an input or an image list. If you opt for the last one, 
    need tyou will o create a configuration file where you enumerate the images to use.

	@param ip_Filename                   -   Input configuration file name
    @param out_camera_data.xml           -    XML file with camera parameters

	*/

    int CalibrateCamera(const std::string& ip_Filename);   

} //ends namespce CC

#endif //__CAMERA_CALIBRATION_H__