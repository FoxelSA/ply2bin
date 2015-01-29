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
    return false ;

};


#endif /* POCOTOPANO_HPP_ */
