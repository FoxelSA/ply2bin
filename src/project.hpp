/*
 * project.hpp - Header file of openMVG point cloud exportation to freepano
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
 /*! \mainpage ply2bin
 * \section ply2bin
 *
 * Point cloud exportation to freepano
 *
 * \section Documentation
 *
 * Documentation can be consulted on the [wiki](https://github.com/FoxelSA/ply2bin/wiki).
 *
 * \section Copyright
 *
 * Copyright (c) 2015 FOXEL SA - [http://foxel.ch](http://foxel.ch)<br />
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
#include <map>
#include <set>
#include <unistd.h>
#include <string.h>
#include <vector>
#include <algorithm>
#include <utility>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <tools.hpp>
#include <gnomonic-all.h>
#include "file_system.hpp"

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
* \param alignedPose      4x3 matrix containing Rotation (first 3X3 block) and translation (line 4) that aligne point cloud in MN95
* \param scale            scale factor used in alignment transformation
* \param transformation   additional transformation of the point cloud.
* \param sx               shift on the x-coordinate in order to retrieve the true aligned coordinates
* \param sy               shift on the y-coordinate in order to retrieve the true aligned coordinates
* \param sz               shift on the z-coordinate in order to retrieve the true aligned coordinates
* \param vec_sensorData   Calibration information for each sensor
* \param panoPath         The complete path of the EQR panorama
* \param outputDirectory  The output directory where the json and the projected point cloud on EQR will be
*
* \return  bool value indicating if the projection was sucessfull or not
*/

bool projectPointCloud (
           std::vector < std::pair < vector <double >, vector <double> > > & pointAndPixels,
           const vector< std::pair < vector <double >, vector<unsigned int> > > & pointAndColor,
           const std::vector < std::vector <double> > & rigPose,
           const std::vector < std::vector <double> > & alingnedPose,
           const double & scale,
           const std::vector < std::vector <double> > & tranformation,
           const double &sx,
           const double &sy,
           const double &sz,
           const std::vector < sensorData > & vec_sensorData,
           const std::string panoPath,
           const std::string outputDirectory );


#endif /* PROJECT_HPP_ */
