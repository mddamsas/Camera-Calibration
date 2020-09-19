#include "CameraCalibration.h"

int main(int argc,char* argv[])
{
    // Camera calibration
    std::cout<<"Camera Calibration begins...\n";
    CC::CalibrateCamera("input.xml");
    std::cout<<"Camera Calibration ends...\n";

    return 0;
}