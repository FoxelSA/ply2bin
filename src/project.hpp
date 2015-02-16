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

 /*! \file project.hpp
 * \author Stephane Flotron <s.flotron@foxel.ch>
 */
 /*! \mainpage poco2pano
 * \section poco2pano
 *
 * Point cloud exportation to freepano
 *
 * \section Documentation
 *
 * Documentation can be consulted on the [wiki](https://github.com/baritone/poco2pano/wiki).
 *
 * \section Copyright
 *
 * Copyright (c) 2013-2014 FOXEL SA - [http://foxel.ch](http://foxel.ch)<br />
 * This program is part of the FOXEL project <[http://foxel.ch](http://foxel.ch)>.
 *
 * Please read the [COPYRIGHT.md](COPYRIGHT.md) file for more information.
 *
 * \section License
 *
 * This program is licensed under the terms of the
 * [GNU Affero General Public License v3](http://www.gnu.org/licenses/agpl.html)
 * (GNU AGPL), with two additional terms. The content is licensed under the terms
 * of the [Creative Commons Attribution-ShareAlike 4.0 International](http://creativecommons.org/licenses/by-sa/4.0/)
 * (CC BY-SA) license.
 *
 * You must read <[http://foxel.ch/license](http://foxel.ch/license)> for more
 *information about our Licensing terms and our Usage and Attribution guidelines.
 *
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

/*! \brief Project point cloud on a panorama pose
*
* Given a panorama pose (Rotation and translation), project the point cloud
* on the associated submodules and compute the corresponding pixels on the
* stiched EQR panorama.
*
* \param pointAndPixels   List of 3D points and associated EQR pixels
* \param pointAndColor    List of 3D points and associated color
* \param rigPose          4x3 matrix containing Rotation (first 3X3 block) and translation (line 4)
* \param vec_sensorData   Calibration information for each sensor
*
* \return  bool value indicating if the projection was sucessfull or not
*/

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

/*! \brief Export projected point cloud in a json
*
* Given a list of 3D point and corresponding pixels on stiched EQR panorama,
* export theses informations to a json file.
*
* \param jsonName         Name of json file in c-string array
* \param pointAndPixels   List of 3D points and associated EQR pixels
*
* \return  Nothing
*/

void  exportToJson ( const char * jsonName,
                     std::vector < std::pair < std::vector <double>, std::vector <double > > > pointAndPixels
);

/*********************************************************************
*  export point cloud to json file
*
**********************************************************************/

/*! \brief Export point cloud in ply format into json file
*
* Given a list of 3D point, export it to a json file.
*
* \param jsonName         Name of json file in c-string array
* \param pointAndPixels   List of 3D points and associated EQR pixels
*
* \return  Nothing
*/

void  pointCloudToJson ( const char * jsonName,
                         std::vector < std::pair < std::vector <double>, std::vector <unsigned int> > > pointAndColor
);


/*********************************************************************
*  load calibration data related to elphel cameras
*
**********************************************************************/

/*! \brief Load calibration information
*
* Given a mac address and a mount point, load all information needed to do
* the point cloud projection on the panorama.
*
* \param vec_sensorData   Array used to store the sensor calibration information.
* \param sMountPoint      Camera folder mount point
* \param smacAddress      Elphel camera's mac address
*
* \return  bool value telling if the loading was sucessfull or not.
*/

bool  loadCalibrationData(std::vector < sensorData > & vec_sensorData,
                          const std::string & sMountPoint,
                          const std::string & smacAddress);

/*********************************************************************
*  load point cloud
*
**********************************************************************/

/*! \brief Ply reader
*
* Given a point cloud in ply format, read the file and store the corresponding
* 3D points in a vector of pair < position, color >
*
* \param vec_sensorData   Array used to store the sensor calibration information.
*
* \return  bool value telling if the loading was sucessfull or not.
*/

bool loadPointCloud ( char * fileName ,
                      vector< std::pair < vector <double >, vector<unsigned int> > > & pointAndColor
);

/*********************************************************************
*  load rig pose
*
**********************************************************************/

/*! \brief Pose file parser
*
* Given a rig pose file generated by openMVG, read it and store the information
* in an array of double.
*
* \param  fileName   rig pose filename
* \param  rigPose    The loaded rig pose
*
* \return  bool value telling if the loading was sucessfull or not.
*/

bool  loadRigPose ( const char * fileName,
                    vector< std::vector<double> > & rigPose );

#endif /* PROJECT_HPP_ */
