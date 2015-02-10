/*
 * poco2pano - Export openMVG point cloud to freepano
 *
 * Copyright (c) 2015 FOXEL SA - http://foxel.ch
 * Please read <http://foxel.ch/license> for more information.
 *
 *
 * Author(s):
 *
 *      Stéphane Flotron <s.flotron@foxel.ch>
 *
 *
 * This file is part of the FOXEL project <http://foxel.ch>.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * Additional Terms:
 *
 *      You are required to preserve legal notices and author attributions in
 *      that material or in the Appropriate Legal Notices displayed by works
 *      containing it.
 *
 *      You are required to attribute the work as explained in the "Usage and
 *      Attribution" section of <http://foxel.ch/license>.
 */

#ifndef PROJECT_HPP_
#define PROJECT_HPP_

#include <iostream>
#include <iomanip>
#include <fstream>
#include <ctype.h>
#include <cmath>
#include <unistd.h>
#include <string.h>
#include <vector>
#include <utility>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <tools.hpp>
#include <gnomonic-all.h>

#define DEBUG 0

using namespace std;
using namespace cv;

/*********************************************************************
*  project point cloud on panorama
*
**********************************************************************/

bool projectPointCloud (
           std::vector < std::pair < vector <double >, vector <double> > > & pointAndPixels,
           const vector< std::pair < vector <double >, vector<unsigned int> > > & pointAndColor,
           const std::vector < std::vector <double> > & rigPose,
           const std::vector < sensorData > & vec_sensorData
);

/*********************************************************************
*  export projected point cloud to json file
*
**********************************************************************/

void  exportToJson ( const char * jsonName,
                      std::vector < std::pair < std::vector <double>, std::vector <double > > > pointAndPixels
);

/*********************************************************************
*  load calibration data related to elphel cameras
*
**********************************************************************/

bool  loadCalibrationData( std::vector < sensorData > & vec_sensorData );

/*********************************************************************
*  load point cloud
*
**********************************************************************/

bool loadPointCloud ( char * fileName ,
                      vector< std::pair < vector <double >, vector<unsigned int> > > & pointAndColor
);

/*********************************************************************
*  load rig pose
*
**********************************************************************/

bool  loadRigPose ( const char * fileName,
                    vector< std::vector<double> > & rigPose );

#endif /* PROJECT_HPP_ */
