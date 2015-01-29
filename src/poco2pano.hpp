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

#ifndef POCOTOPANO_HPP_
#define POCOTOPANO_HPP_

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
           const std::vector < sensorData > & vec_sensorData )
{
  // extract rig rotation and center
  double  Rrig[3][3] = {
      { rigPose[0][0], rigPose[0][1], rigPose[0][2] },
      { rigPose[1][0], rigPose[1][1], rigPose[1][2] },
      { rigPose[2][0], rigPose[2][1], rigPose[2][2] }
  };

  vector <double> cRig = rigPose[3];

  std::cout << " loaded rig pose " << endl;

  // load image
  std::string panoPath = "/home/sflotron/foxel/test/rigid_test/result_1404374415_319830.tif";
  Mat pano_img;
  pano_img = imread(panoPath.c_str(), CV_LOAD_IMAGE_COLOR );

  // project point cloud into panorama
  for( size_t i = 0 ; i < pointAndColor.size(); ++i )
  {
    // retreive point information
    vector <double> pos   = pointAndColor[i].first;

    // convert point cloud into rig referential, i.e. x_rig = R_rig ( x - C_rig )
    double  xcentered[3];

    xcentered[0] = pos[0] - cRig[0];
    xcentered[1] = pos[1] - cRig[1];
    xcentered[2] = pos[2] - cRig[2];

    double xrig = Rrig[0][0] * xcentered[0] + Rrig[0][1] * xcentered[1] + Rrig[0][2] *  xcentered[2];
    double yrig = Rrig[1][0] * xcentered[0] + Rrig[1][1] * xcentered[1] + Rrig[1][2] *  xcentered[2];
    double zrig = Rrig[2][0] * xcentered[0] + Rrig[2][1] * xcentered[1] + Rrig[2][2] *  xcentered[2];

    const lf_Real_t  X[4] = { xrig, yrig, zrig, 1.0 };

    // count the number of subcam in which point is apparing
    lf_Size_t cpt = 0;

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

      //  if depth > 0, point could be seen from camera j
      if( cpt == 0 && depth > 1.0e-6  && sqrt(X_C[0] * X_C[0] + X_C[1]*X_C[1] + X_C[2]*X_C[2]) < 100.0 )
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

            if( up > 5 && up < sd.lfImageFullWidth-5 && vp > 5 && vp < sd.lfImageFullHeight-5 )
            {
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

              // export projected point
              std::vector < double > pixels;
              std::vector < double > point;

              pixels.push_back(up + sd.lfXPosition);
              pixels.push_back(vp + sd.lfYPosition);

              point.push_back( xrig );
              point.push_back( yrig );
              point.push_back( zrig );

              pointAndPixels.push_back( std::make_pair( point, pixels ) );

              ++cpt;
            }
          }
        }
      }
    }

    imwrite( "./pointcloud_on_pano.tif", pano_img);

    if( pointAndPixels.size() > 0 )
      return true;
    else
      return false;

};

/*********************************************************************
*  load calibration data related to elphel cameras
*
**********************************************************************/

bool  loadCalibrationData( std::vector < sensorData > & vec_sensorData )
{

    /* Key/value-file descriptor */
    lf_Descriptor_t lfDesc;
    lf_Size_t       lfChannels=0;

    /* Creation and verification of the descriptor */
    std::string sMountPoint = "/data/";
    char *c_data = new char[sMountPoint.length() + 1];
    std::strcpy(c_data, sMountPoint.c_str());

    std::string smacAddress = "00-0E-64-08-1C-D2";
    char *c_mac = new char[smacAddress.length() + 1];
    std::strcpy(c_mac, smacAddress.c_str());

    // check input data validity
    if ( lf_parse( (unsigned char*)c_mac, (unsigned char*)c_data, & lfDesc ) == LF_TRUE ) {

      /* Query number of camera channels */
      lfChannels = lf_query_channels( & lfDesc );
    }
    else
    {
      std::cerr << " Could not read calibration data. " << std::endl;
      return false;
    }

    lfChannels = lf_query_channels( & lfDesc );

    for( lf_Size_t sensor_index = 0 ; sensor_index < lfChannels ; ++sensor_index )
    {
      sensorData  sD;

      // query panorama width and height
      sD.lfImageFullWidth  = lf_query_ImageFullWidth ( sensor_index, & lfDesc );
      sD.lfImageFullHeight = lf_query_ImageFullLength( sensor_index, & lfDesc );

      sD.lfpixelCorrectionWidth  = lf_query_pixelCorrectionWidth (sensor_index, &lfDesc);
      sD.lfpixelCorrectionHeight = lf_query_pixelCorrectionHeight(sensor_index, &lfDesc);

      /* Query position of eqr tile in panorama */
      sD.lfXPosition = lf_query_XPosition ( sensor_index, & lfDesc );
      sD.lfYPosition = lf_query_YPosition ( sensor_index, & lfDesc );

      /* Query number width and height of sensor image */
      sD.lfWidth  = lf_query_pixelCorrectionWidth ( sensor_index, & lfDesc );
      sD.lfHeight = lf_query_pixelCorrectionHeight( sensor_index, & lfDesc );

      /* Query focal length of camera sensor index */
      sD.lfFocalLength = lf_query_focalLength( sensor_index , & lfDesc );
      sD.lfPixelSize   = lf_query_pixelSize  ( sensor_index , & lfDesc );

      /* Query angles used for gnomonic rotation */
      sD.lfAzimuth    = lf_query_azimuth    ( sensor_index , & lfDesc );
      sD.lfHeading    = lf_query_heading    ( sensor_index , & lfDesc );
      sD.lfElevation  = lf_query_elevation  ( sensor_index , & lfDesc );
      sD.lfRoll       = lf_query_roll       ( sensor_index , & lfDesc );

      // compute rotation and store it.
      computeRotationEl ( &sD.R[0] , sD.lfAzimuth , sD.lfHeading, sD.lfElevation, sD.lfRoll );

      /* Query principal point */
      sD.lfpx0 = lf_query_px0 ( sensor_index , & lfDesc );
      sD.lfpy0 = lf_query_py0 ( sensor_index , & lfDesc );

      /* Query information related to entrance pupil center */
      sD.lfRadius   = lf_query_radius               ( sensor_index , & lfDesc );
      sD.lfCheight  = lf_query_height               ( sensor_index , & lfDesc );
      sD.lfEntrance = lf_query_entrancePupilForward ( sensor_index , & lfDesc );

      // compute optical center in camera coordinate and store it
      getOpticalCenter ( &sD.C[0] , sD.lfRadius, sD.lfCheight, sD.lfAzimuth, sD.R, sD.lfEntrance );

      // compute projection matrix
      computeProjMat ( &sD.P[0] , sD.lfFocalLength / sD.lfPixelSize, sD.lfpx0, sD.lfpy0, sD.R, sD.C);

      vec_sensorData.push_back(sD);
    }

    /* Release descriptor */
    lf_release( & lfDesc );

    return true;
  };

  /*********************************************************************
  *  load point cloud
  *
  **********************************************************************/

bool loadPointCloud ( char * fileName ,   vector< std::pair < vector <double >, vector<unsigned int> > > & pointAndColor )
{
  // create file stream
  ifstream data( fileName );

  //check if file exist for reading
  if( data == NULL){
    fprintf(stderr, "couldn't open point cloud file %s \n ", fileName);
    return false;
  }

  // read data files
  double x,y,z;
  unsigned int r, g, b;

  // skip header and go to line (first 10 lines of file)
  for(int k=0; k < 10 ; ++k)
    data.ignore(10000,'\n');

  while (data >> x >> y >> z >> r >> g >> b){
      // store point information in big vector
      vector <double>  position;
      vector <unsigned int> color;

      position.push_back(x); position.push_back(y); position.push_back(z);
      color.push_back(r);    color.push_back(g);    color.push_back(b);

      pointAndColor.push_back(std::make_pair(position, color));
   }

  // close stream
  data.close();

  if( pointAndColor.size () > 0 )
    return true ;
  else
  {
    std::cerr << "Point cloud is empty " << std::endl;
    return false ;
  }

};

/*********************************************************************
*  load rig pose
*
**********************************************************************/

bool  loadRigPose ( const char * fileName, vector< std::vector<double> > & rigPose )
{

  // load pose
  ifstream pose(fileName);

  //check if file exist for reading
  if( pose == NULL){
    fprintf(stderr, "couldn't open pose file %s \n ", fileName);
    return false;
  }

  // read pose information
  double x,y,z;
  while (pose >> x >> y >> z){
    // store point information in big vector
    vector <double>  position;

    position.push_back(x); position.push_back(y); position.push_back(z);
    rigPose.push_back(position);
  }

  //close stream
  pose.close();

  if( rigPose.size() > 0 )
    return true;
  else
  {
    std::cerr << "Pose file is empty \n" << std::endl;
    return false;
  }

}


#endif /* POCOTOPANO_HPP_ */
