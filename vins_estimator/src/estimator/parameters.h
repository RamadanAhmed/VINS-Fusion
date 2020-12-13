/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "../utility/utility.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <map>

using namespace std;

const double FOCAL_LENGTH = 460.0;
const int WINDOW_SIZE = 10;
const int NUM_OF_F = 1000;
//#define UNIT_SPHERE_ERROR

extern __declspec( dllimport ) double INIT_DEPTH;
extern __declspec( dllimport ) double MIN_PARALLAX;
extern __declspec( dllimport ) int ESTIMATE_EXTRINSIC;

extern __declspec( dllimport ) double ACC_N, ACC_W;
extern __declspec( dllimport ) double GYR_N, GYR_W;

extern __declspec( dllimport ) std::vector<Eigen::Matrix3d> RIC;
extern __declspec( dllimport ) std::vector<Eigen::Vector3d> TIC;
extern __declspec( dllimport ) Eigen::Vector3d G;

extern __declspec( dllimport )double BIAS_ACC_THRESHOLD;
extern __declspec( dllimport )double BIAS_GYR_THRESHOLD;
extern __declspec( dllimport )double SOLVER_TIME;
extern __declspec( dllimport )int NUM_ITERATIONS;
extern __declspec( dllimport )std::string EX_CALIB_RESULT_PATH;
extern __declspec( dllimport )std::string VINS_RESULT_PATH;
extern __declspec( dllimport )std::string OUTPUT_FOLDER;
extern __declspec( dllimport )std::string IMU_TOPIC;
extern __declspec( dllimport )double TD;
extern __declspec( dllimport )int ESTIMATE_TD;
extern __declspec( dllimport )int ROLLING_SHUTTER;
extern __declspec( dllimport )int ROW, COL;
extern __declspec( dllimport )int NUM_OF_CAM;
extern __declspec( dllimport )int STEREO;
extern __declspec( dllimport )int USE_IMU;
extern __declspec( dllimport )int MULTIPLE_THREAD;
// pts___declspec( dllimport )gt for debug purpose;
extern __declspec( dllimport )map<int, Eigen::Vector3d> pts_gt;

extern __declspec( dllimport )std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
extern __declspec( dllimport )std::string FISHEYE_MASK;
extern __declspec( dllimport )std::vector<std::string> CAM_NAMES;
extern __declspec( dllimport )int MAX_CNT;
extern __declspec( dllimport )int MIN_DIST;
extern __declspec( dllimport )double F_THRESHOLD;
extern __declspec( dllimport )int SHOW_TRACK;
extern __declspec( dllimport )int FLOW_BACK;

void readParameters(std::string config_file);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};
