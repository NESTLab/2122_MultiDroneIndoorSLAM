// #include "vicon.h"
#include <iostream>
#include <sstream>
#include "vicon_driver.h"
#include "vicon.h"

namespace VD = vicon_driver;

int main(int argc, char *argv[]){
    bool success;
    VD::ViconDriver vd;

    success = vd.init("169.254.130.80:801");
    if(!success)
    {
        std::cout << "Did not connected to Vicon DataStream Server, exiting." 
                     << std::endl;
        return false;
    }

    vd.enableUnlabeledMarkerData(true);
    vd.setSubjectPubCallback(*subjectCallback);
    vd.setUnlabeledMarkersPubCallback(*markerCallback);
    success = vd.start();
    if(success)
    {
        std::cout << "Created ViconDriver thread." << std::endl;
    }else
    {
        std::cout << "Could not create ViconDriver thread, exiting."
                  << std::endl;
        return false;
    }

    // std::cout << "Running for 10 sec, then shutting down." << std::endl;
    // struct timespec ts_sleep;
    // ts_sleep.tv_sec = 10;
    // ts_sleep.tv_nsec = 0;
    // nanosleep(&ts_sleep, NULL);
    char response = std::cin.get();;
    success = false;
    while(!success)
    {
        std::cout << "Attempting to shutdown." << std::endl;
        success = vd.shutdown();
        if(!success)
            std::cout << "Unsucessfull shutdown." << std::endl;
        else
            std::cout << "Successfull shutdown." << std::endl;

    }

    return true;
}

void subjectCallback(const VD::ViconDriver::Subject& subject){

    std::vector<VD::ViconDriver::Marker> markers = subject.markers;
    std::vector<VD::ViconDriver::Marker>::iterator marker;

    std::cout << subject.name << " has " << markers.size() 
              << " markers and is at:" 
              << formatTranslation(subject.translation, true) 
              << std::endl << "with Rotation "
              << formatRotation(subject.rotation, true)
              << std::endl;

    for(marker = markers.begin(); marker != markers.end(); ++marker)
    {
        std::string trans_string = formatTranslation(marker->translation, true); 
        if(marker->occluded)
        {
            std::cout << "\tMarker '" << marker->name << "is occluded at"
                      << trans_string << "." << std::endl;
        }else
        {
            std::cout << "\tMarker '" << marker->name << "' at " 
                      << trans_string << "." << std::endl;
        }
    }
}


void markerCallback(const VD::ViconDriver::Markers& markers_in){
    std::vector<VD::ViconDriver::Marker> markers = markers_in.markers;
    std::vector<VD::ViconDriver::Marker>::iterator marker;

    std::cout << "There are " << markers.size() << " unknown markers." 
              << std::endl;
    
    for(marker = markers.begin(); marker != markers.end(); ++marker)
    {
        std::string trans_string = formatTranslation(marker->translation, true); 
        if(marker->occluded)
        {
            std::cout << "\tMarker '" << marker->name << "is occluded at"
                      << trans_string << "." << std::endl;
        }else
        {
            std::cout << "\tMarker '" << marker->name << "' at " 
                      << trans_string << "." << std::endl;
        }
    }
}
std::string formatTranslation(const double* translation, bool single_line){
    std::stringstream fmt;
    if(single_line)
    {
        fmt << "(" << translation[0]
            << ", "<< translation[1]
            << ", "<< translation[2]
            << ")";
    }else
    {
        fmt << "x:" << translation[0] << std::endl
            << "y: "<< translation[1] << std::endl
            << "z: "<< translation[2] << std::endl;
    }

    return fmt.str();
}

std::string formatRotation(const double* rotation, bool single_line){
    std::stringstream fmt;
    if(single_line)
    {
        fmt << "(" << rotation[0]
            << ", "<< rotation[1]
            << ", "<< rotation[2]
            << ", "<< rotation[3]
            << ")";
    }else
    {
        fmt << "x:" << rotation[0] << std::endl
            << "y: "<< rotation[1] << std::endl
            << "z: "<< rotation[2] << std::endl
            << "w: "<< rotation[3] << std::endl;
    }

    return fmt.str();
}
