//
// Created by lisa on 03.11.17.
//
#pragma once

#include "msl_ptgrey_camera/MSLPtGreyCamera.h"
#include "msl_ptgrey_camera/CNImage.h"
#include <iostream>
#include <string>

class Recorder
{

public:
    Recorder();

    virtual ~Recorder();
    void initialiseParameters();
    int maxImages;
    string path;
    int freq;



private:
    supplementary::SystemConfig* sc;
};

//} /* namespace msl_ptgrey_camera */


