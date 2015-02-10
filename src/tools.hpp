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

 /*! \file tools.hpp
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

 #ifndef TOOLS_HPP_
 #define TOOLS_HPP_

 #include <fastcal-all.h>
 #include <string>
 #include <cmath>
 #include <iostream>

 using namespace std;


/******************************************************************************
 * sensorData
 *****************************************************************************/

 /*! \struct sensorData
 * \brief structure used to store calibration information
 *
 * This structure is designed to store the needed informations coming from
 * the elphel camera calibration
 *
 * \var sensorData::lfWidth
 *  Width of sensor image
 * \var sensorData::lfHeight
 *  Height of sensor image
 * \var sensorData::lfChannels
 *  Number of channels of elphel camera
 * \var sensorData::lfXPosition
 *  X coordinate of left corner of EQR tile in panorama
 * \var sensorData::lfYPosition
 *  Y coordinate of left corner of EQR tile in panorama
 * \var sensorData::lfImageFullWidth
 *  Sitched EQR panorama width
 * \var sensorData::lfImageFullHeight
 *  Sitched EQR panorama height
 * \var sensorData::lfpixelCorrectionWidth
 *  Width of sensor image
 * \var sensorData::lfpixelCorrectionHeight
 *  Height of sensor image
 * \var sensorData::lfFocalLength
 *  Focal length in mm
 * \var sensorData::lfPixelSize
 *  pixel size in mm
 * \var sensorData::lfAzimuth
 *  azimuth angle in elphel coordinate frame (in radian)
 * \var sensorData::lfHeading
 *  heading angle in elphel coordinate frame (in radian)
 * \var sensorData::lfElevation
 *  Elevation angle in elphel coordinate frame (in radian)
 * \var sensorData::lfRoll
 *  roll around z axis (in radian)
 * \var sensorData::lfpx0
 *  x coordinate of principal point of sensor image, in pixels
 * \var sensorData::lfpy0
 *  y coordinate of principal point of sensor image, in pixels
 * \var sensorData::lfRadius
 *  radius of optical center of channel in elphel coordinate frame
 * \var sensorData::lfCheight
 *  height of optical center of channel in elphel coordinate frame
 * \var sensorData::lfEntrance
 *  Entrance pupil forward of channel
 * \var sensorData::R
 *  Rotation matrix rig coordinate frame to sensor coordinate frame
 * \var sensorData::C
 *  Optical center position in rig coordinate frame
 * \var sensorData::P
 *  Projection matrix associated to sensor
 */

struct sensorData
{

      lf_Size_t   lfWidth     = 0;
      lf_Size_t   lfHeight    = 0;
      lf_Size_t   lfChannels  = 0;
      lf_Size_t   lfXPosition = 0;
      lf_Size_t   lfYPosition = 0;
      lf_Size_t   lfImageFullWidth  = 0;
      lf_Size_t   lfImageFullHeight = 0;
      lf_Size_t   lfpixelCorrectionWidth  = 0;
      lf_Size_t   lfpixelCorrectionHeight = 0;

      lf_Real_t   lfFocalLength = 0.0;
      lf_Real_t   lfPixelSize   = 0.0;
      lf_Real_t   lfAzimuth     = 0.0;
      lf_Real_t   lfHeading     = 0.0;
      lf_Real_t   lfElevation   = 0.0;
      lf_Real_t   lfRoll        = 0.0;
      lf_Real_t   lfpx0         = 0.0;
      lf_Real_t   lfpy0         = 0.0;
      lf_Real_t   lfRadius      = 0.0;
      lf_Real_t   lfCheight     = 0.0;
      lf_Real_t   lfEntrance    = 0.0;

      lf_Real_t R[9] = {
          1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0};

      lf_Real_t C[3] = {0,0,0};

      lf_Real_t P[12] = {
         1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0,
         1.0, 0.0, 1.0, 0.0
      };

 };

 /*******************************************************************************
 *  Given focal, px0, py0, R and optical center C, compute projection matrix
 *
 ********************************************************************************
 */

 void computeProjMat (  lf_Real_t* P ,
                        const lf_Real_t focal ,
                        const lf_Real_t px0,
                        const lf_Real_t py0 ,
                        const lf_Real_t* R,
                        const lf_Real_t* C);

 /*******************************************************************************
 *  Given 4 angles, compute Elphel rotation
 *
 ********************************************************************************
 */
 void computeRotationEl ( lf_Real_t* R ,
                          lf_Real_t az ,
                          lf_Real_t head,
                          lf_Real_t ele ,
                          lf_Real_t roll);

/********************************************************************************
*  Given three angles, entrance pupil forward, radius and height, compute optical center position.
*
********************************************************************************
*/

void getOpticalCenter ( lf_Real_t* C ,
      const lf_Real_t& radius,
      const lf_Real_t& height,
      const lf_Real_t& azimuth,
      const lf_Real_t* R,
      const lf_Real_t& entrancePupilForward );

#endif /* TOOLS_HPP_ */
