/*
 * types.hpp
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

 #include <fastcal-all.h>
 #include <string>
 #include <iostream>

 using namespace std;

 #ifndef TOOLS_HPP_
 #define TOOLS_HPP_

/******************************************************************************
 * sensorData
 *****************************************************************************/
struct sensorData
{
    lf_Size_t   lfWidth     = 0;
    lf_Size_t   lfHeight    = 0;
    lf_Size_t   lfChannels  = 0;
    lf_Size_t   lfXPosition = 0;
    lf_Size_t   lfYPosition = 0;

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
 };

 /*******************************************************************************
 *  Given 4 angles, compute Elphel rotation
 *
 ********************************************************************************
 */
 void computeRotationEl ( lf_Real_t* R , lf_Real_t az , lf_Real_t head, lf_Real_t ele , lf_Real_t roll)
{
  //z-axis rotation
  lf_Real_t Rz[3][3] = {
  { cos(roll),-sin(roll), 0.0},
{-sin(roll),-cos(roll), 0.0},
{       0.0,      0.0, 1.0} };

// x-axis rotation
lf_Real_t Rx[3][3] = {
{1.0,      0.0,     0.0},
{0.0, cos(ele),sin(ele)},
{0.0,-sin(ele),cos(ele)} };

// y axis rotation
lf_Real_t Ry[3][3] = {
{ cos(head+az), 0.0, sin(head+az)},
{          0.0,-1.0,          0.0},
{-sin(head+az), 0.0, cos(head+az)} };

// 3) R = R2*R1*R0 transform sensor coordinate to panorama coordinate
lf_Real_t  RxRz[3][3] = {0.0};
lf_Real_t  RT[3][3] = {0.0};

// compute product of rotations
int i=0, j=0;

for(i=0 ; i < 3 ; ++i)
  for(j=0; j < 3 ; ++j)
    RxRz[i][j] = Rx[i][0] * Rz[0][j] + Rx[i][1] * Rz[1][j] + Rx[i][2] * Rz[2][j];

    for(i=0 ; i < 3 ; ++i)
      for(j=0; j < 3 ; ++j)
        RT[i][j] = Ry[i][0] * RxRz[0][j] + Ry[i][1] * RxRz[1][j] + Ry[i][2] * RxRz[2][j];

        // transpose because we need the transformation panorama to sensor coordinate !
        R[0] = RT[0][0];
        R[1] = RT[1][0];
        R[2] = RT[2][0];
        R[3] = RT[0][1];
        R[4] = RT[1][1];
        R[5] = RT[2][1];
        R[6] = RT[0][2];
        R[7] = RT[1][2];
        R[8] = RT[2][2];
      }

      /********************************************************************************
      *  Given three angles, entrance pupil forward, radius and height, compute optical center position.
      *
      *********************************************************************************
      */

      void getOpticalCenter ( lf_Real_t* C ,
      const lf_Real_t& radius,
      const lf_Real_t& height,
      const lf_Real_t& azimuth,
      const lf_Real_t* R,
      const lf_Real_t& entrancePupilForward )
    {
      // compute lense Center from data
      lf_Real_t lensCenter[3] = {0.0, 0.0, 0.0};

      lensCenter[0] = radius * sin(azimuth);
      lensCenter[1] = height ;
      lensCenter[2] = radius * cos(azimuth);

      // C = lensCenter + R.entrancePupilForward, where R is roation sensor to world.
      C[0] =  lensCenter[0] + R[6] * entrancePupilForward;
      C[1] = -lensCenter[1] + R[7] * entrancePupilForward;
      C[2] =  lensCenter[2] + R[8] * entrancePupilForward;

    }

    #endif /* TOOLS_HPP_ */
