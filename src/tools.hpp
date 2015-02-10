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

 #include <fastcal-all.h>
 #include <string>
 #include <cmath>
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
