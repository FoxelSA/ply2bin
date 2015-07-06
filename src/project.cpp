/*
 * ply2bin - Export openMVG point cloud to freepano
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

#include <project.hpp>

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
           const std::vector < std::vector <double> > & alignedPose,
           const double & scale,
           const std::vector < sensorData > & vec_sensorData )
{
    // extract rig rotation and center
    double  Rrig[3][3] = {
        { rigPose[0][0], rigPose[0][1], rigPose[0][2] },
        { rigPose[1][0], rigPose[1][1], rigPose[1][2] },
        { rigPose[2][0], rigPose[2][1], rigPose[2][2] }
    };

    vector <double> cRig = rigPose[3];

    // extract aligned transformation
    double  Ralign[3][3] = {
        { alignedPose[0][0], alignedPose[0][1], alignedPose[0][2] },
        { alignedPose[1][0], alignedPose[1][1], alignedPose[1][2] },
        { alignedPose[2][0], alignedPose[2][1], alignedPose[2][2] }
    };

    vector <double> cA = alignedPose[3];

    // local correction of rig pose
    double  Rcorr[3][3] = {
        {  1.000, 0 * 0.008, -0 * 0.008 },
        { -0 * 0.008, 1.000,  0.000 },
        {  0 * 0.008, 0.000,  1.000 }
    };

    double tCorr[3] = { 0 * 0.048, 0 * 0.013, -0 * 0.244 } ;

#if DEBUG
    // load image
    std::string panoPath = "/data/structure/footage/00-0E-64-08-1B-6E/master/1429143317/segment/1429143318/stitch_selections/dav_3/stitched/result_1429143335_882930.tif";
    Mat pano_img;
    pano_img = imread(panoPath.c_str(), CV_LOAD_IMAGE_COLOR );
#endif

    // project point cloud into panorama
    for( size_t i = 0 ; i < pointAndColor.size(); ++i )
    {
          // retrieve point information
          vector <double> pos   = pointAndColor[i].first;
          double xrig, yrig, zrig;

          double  xcentered[3];

          xcentered[0] = pos[0];
          xcentered[1] = pos[1];
          xcentered[2] = pos[2];

          // undo rotation and scaling that aligned point cloud
          double x_pc = ( Ralign[0][0] * xcentered[0] + Ralign[1][0] * xcentered[1] + Ralign[2][0] *  xcentered[2] + cA[0]) / scale;
          double y_pc = ( Ralign[0][1] * xcentered[0] + Ralign[1][1] * xcentered[1] + Ralign[2][1] *  xcentered[2] + cA[1]) / scale;
          double z_pc = ( Ralign[0][2] * xcentered[0] + Ralign[1][2] * xcentered[1] + Ralign[2][2] *  xcentered[2] + cA[2]) / scale;

          // undo rotation and scaling that move point cloud
          double x_tmp = Rcorr[0][0] * ( x_pc -tCorr[0])  + Rcorr[1][0] * ( y_pc -tCorr[1]) + Rcorr[2][0] *  ( z_pc -tCorr[2]) ;
          double y_tmp = Rcorr[0][1] * ( x_pc -tCorr[0])  + Rcorr[1][1] * ( y_pc -tCorr[1]) + Rcorr[2][1] *  ( z_pc -tCorr[2]) ;
          double z_tmp = Rcorr[0][2] * ( x_pc -tCorr[0])  + Rcorr[1][2] * ( y_pc -tCorr[1]) + Rcorr[2][2] *  ( z_pc -tCorr[2]) ;

          // convert point cloud into rig referential, i.e. x_rig = R_rig ( x - C_rig )
          xrig = Rrig[0][0] * (x_tmp -cRig[0]) + Rrig[0][1] * (y_tmp - cRig[1]) + Rrig[0][2] * ( z_tmp - cRig[2] ) ;
          yrig = Rrig[1][0] * (x_tmp -cRig[0]) + Rrig[1][1] * (y_tmp - cRig[1]) + Rrig[1][2] * ( z_tmp - cRig[2] ) ;
          zrig = Rrig[2][0] * (x_tmp -cRig[0]) + Rrig[2][1] * (y_tmp - cRig[1]) + Rrig[2][2] * ( z_tmp - cRig[2] ) ;

          const lf_Real_t  X[4] = { xrig, yrig, zrig, 1.0 };

          // count the number of subcam in which point is apparing
          lf_Size_t cpt = 0;
          const lf_Real_t  max_depth = 1.0e10;

          for( size_t j = 0; j < vec_sensorData.size()-2 ; ++j )
          {
              // extract sensor information
              sensorData  sd = vec_sensorData[j];

              // compute depth related to camera j
              lf_Real_t  X_C[3] = { X[0] - sd.C[0],  X[1] - sd.C[1], X[2] -sd.C[2]};
              lf_Real_t depth = sd.R[6] * X_C[0] + sd.R[7] * X_C[1] + sd.R[8] * X_C[2];

              // initialize projected pixels
              lf_Real_t  ug = -1.0;
              lf_Real_t  vg = -1.0;

              //  if depth > 0, point could be seen from camera j. Exclude point too far ( > 30.0 meter from rig)
              if( cpt == 0 && depth > 1.0e-6
                           && abs(X_C[0]) < max_depth
                           && abs(X_C[1]) < max_depth
                           && abs(X_C[2]) < max_depth )
              {
                  double  PX0 = sd.P[0] * X[0] + sd.P[1] * X[1] + sd.P[2 ] * X[2] + sd.P[3 ] * X[3];
                  double  PX1 = sd.P[4] * X[0] + sd.P[5] * X[1] + sd.P[6 ] * X[2] + sd.P[7 ] * X[3];
                  double  PX2 = sd.P[8] * X[0] + sd.P[9] * X[1] + sd.P[10] * X[2] + sd.P[11] * X[3];

                  // update projected pixel value
                  ug = PX0 / PX2 ;
                  vg = PX1 / PX2 ;

                  if ( ug > 0.0 && ug < sd.lfWidth && vg > 0.0 && vg < sd.lfHeight)
                  {
                      // retreive pixel in panorama
                      lf_Real_t  up = 0.0;
                      lf_Real_t  vp = 0.0;

                      // apply inverse gnomonic projection
                      lg_gtt_elphel_point(
                          &up,
                          &vp,
                          ug,
                          vg,
                          sd.lfpx0,
                          sd.lfpy0,
                          sd.lfImageFullWidth,
                          sd.lfImageFullHeight,
                          sd.lfXPosition,
                          sd.lfYPosition,
                          sd.lfRoll,
                          sd.lfAzimuth,
                          sd.lfElevation,
                          sd.lfHeading,
                          sd.lfPixelSize,
                          sd.lfFocalLength
                      );

                      if( up < 0.0 )
                          up += sd.lfImageFullWidth;

                      if( up > 0 && up < sd.lfImageFullWidth && vp > 0 && vp < sd.lfImageFullHeight )
                      {

#if DEBUG
                          // export point on panorama (for debug purpose only)
                          for(int k = -1 ; k < 2 ; ++k)
                              for( int l = -1; l < 2 ; ++l)
                              {
                                  // export point on stiched panorama
                                  Vec3b color = pano_img.at<Vec3b>(Point(up + sd.lfXPosition + k, vp + sd.lfYPosition + l));
                                  color.val[0] =  0;
                                  color.val[1] =  0;
                                  color.val[2] =  255;

                                  // set pixel
                                  pano_img.at<Vec3b>(Point(up + sd.lfXPosition +k , vp + sd.lfYPosition + l)) = color;
                          }
#endif
                          // export projected point
                          std::vector < double > pixels;
                          std::vector < double > point;

                          pixels.push_back(up + sd.lfXPosition);
                          pixels.push_back(vp + sd.lfYPosition);
                          pixels.push_back( sqrt(xrig * xrig + yrig * yrig + zrig * zrig) );

                          point.push_back( pos[0] );
                          point.push_back( pos[1] );
                          point.push_back( pos[2] );
                          point.push_back( i );

                          pointAndPixels.push_back( std::make_pair( point, pixels ) );

                          ++cpt;
                      }

                  }

              }

          }

    }

#if DEBUG
    imwrite( "./pointcloud_on_pano.tif", pano_img);
#endif

    if( pointAndPixels.size() > 0 )
        return true;
    else
        return false;

};
