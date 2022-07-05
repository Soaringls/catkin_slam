#pragma once

#include "common_include.h"
#include "camera.h"




extern Camera::Ptr camera_left_;
extern Camera::Ptr camera_right_;

extern int MAX_CNT;
extern int MIN_DIST;

extern int IMAGE_COL;
extern int IMAGE_ROW;

extern int SCALE_S;

void setParameter(std::string config_file);

