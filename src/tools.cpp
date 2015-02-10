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

 #include <tools.hpp>

 using namespace std;

 /*******************************************************************************
 *  Given focal, px0, py0, R and optical center C, compute projection matrix
 *
 ********************************************************************************
 */
 void computeProjMat ( lf_Real_t* P ,
              const lf_Real_t focal ,
              const lf_Real_t px0,
              const lf_Real_t py0 ,
              const lf_Real_t* R,
              const lf_Real_t* C)
{
     // compute t ( = -R * C )
     lf_Real_t  t[3] = {0 , 0, 0};

     t[0] = - 1.0 * ( R[0] * C[0] + R[1] * C[1] + R[2] * C[2] );
     t[1] = - 1.0 * ( R[3] * C[0] + R[4] * C[1] + R[5] * C[2] );
     t[2] = - 1.0 * ( R[6] * C[0] + R[7] * C[1] + R[8] * C[2] );

     // intialize camera Matrix
     lf_Real_t  K[3][3] =
     {
          {focal,   0.0, px0},
          {  0.0, focal, py0},
          {  0.0,   0.0, 1.0}
     };

     // initialize temporary rotation matrix
     lf_Real_t  Q[3][3] =
     {
        {  R[0], R[1], R[2]},
        {  R[3], R[4], R[5]},
        {  R[6], R[7], R[8]}
    };

     // compute projection matrix P = (KR | Kt)
     P[0 ] = K[0][0] * Q[0][0] + K[0][1] * Q[1][0] + K[0][2] * Q[2][0];
     P[1 ] = K[0][0] * Q[0][1] + K[0][1] * Q[1][1] + K[0][2] * Q[2][1];
     P[2 ] = K[0][0] * Q[0][2] + K[0][1] * Q[1][2] + K[0][2] * Q[2][2];

     P[4 ] = K[1][0] * Q[0][0] + K[1][1] * Q[1][0] + K[1][2] * Q[2][0];
     P[5 ] = K[1][0] * Q[0][1] + K[1][1] * Q[1][1] + K[1][2] * Q[2][1];
     P[6 ] = K[1][0] * Q[0][2] + K[1][1] * Q[1][2] + K[1][2] * Q[2][2];

     P[8 ] = K[2][0] * Q[0][0] + K[2][1] * Q[1][0] + K[2][2] * Q[2][0];
     P[9 ] = K[2][0] * Q[0][1] + K[2][1] * Q[1][1] + K[2][2] * Q[2][1];
     P[10] = K[2][0] * Q[0][2] + K[2][1] * Q[1][2] + K[2][2] * Q[2][2];

     // compute Kt
     P[3 ] = K[0][0] * t[0] + K[0][1] * t[1] + K[0][2] * t[2];
     P[7 ] = K[1][0] * t[0] + K[1][1] * t[1] + K[1][2] * t[2];
     P[11] = K[2][0] * t[0] + K[2][1] * t[1] + K[2][2] * t[2];

}

 /*******************************************************************************
 *  Given 4 angles, compute Elphel rotation
 *
 ********************************************************************************
 */
 void computeRotationEl ( lf_Real_t* R , lf_Real_t az , lf_Real_t head, lf_Real_t ele , lf_Real_t roll)
{
    //z-axis rotation
    lf_Real_t Rz[3][3] =
    {
        { cos(roll),-sin(roll), 0.0},
        {-sin(roll),-cos(roll), 0.0},
        {       0.0,       0.0, 1.0}
    };

    // x-axis rotation
    lf_Real_t Rx[3][3] =
    {
        {1.0,      0.0,     0.0},
        {0.0, cos(ele),sin(ele)},
        {0.0,-sin(ele),cos(ele)}
    };

    // y axis rotation
    lf_Real_t Ry[3][3] =
    {
        { cos(head+az), 0.0, sin(head+az)},
        {          0.0,-1.0,          0.0},
        {-sin(head+az), 0.0, cos(head+az)}
    };

    // 3) R = R2*R1*R0 transform sensor coordinate to panorama coordinate
    lf_Real_t  RxRz[3][3] = {0.0};
    lf_Real_t  RT[3][3] = {0.0};

    // compute product of rotations (note elphel rotation is R = S_y.R_y.R_x.R_z.S_y )
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
********************************************************************************
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
