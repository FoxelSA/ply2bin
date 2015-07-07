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
           const std::vector < std::vector <double> > & transformation,
           const double &sx,
           const double &sy,
           const double &sz,
           const std::vector < sensorData > & vec_sensorData,
           const std::string panoPath,
           const std::string outputDirectory )
{
    // extract rig rotation and center
    double  Rrig[3][3] = {
        { rigPose[0][0], rigPose[0][1], rigPose[0][2] },
        { rigPose[1][0], rigPose[1][1], rigPose[1][2] },
        { rigPose[2][0], rigPose[2][1], rigPose[2][2] }
    };

    vector <double> cRig = rigPose[3];

    // intialize alignement transformation
    double  Ralign[3][3] = {
        { 1.0, 0.0, 0.0 },
        { 0.0, 1.0, 0.0 },
        { 0.0, 0.0, 1.0 }
    };

    double cA[3] = {0.0, 0.0, 0.0};

    // if alignement tranformation is provided
    if( alignedPose.size() == 4 )
    {
        // update rotation
        for ( size_t i = 0; i < 3 ; ++i )
            for( size_t j = 0 ; j < 3 ; ++j )
                  Ralign[i][j] = alignedPose[i][j];

        // update translation
        cA[0] = alignedPose[3][0];
        cA[1] = alignedPose[3][1];
        cA[2] = alignedPose[3][2];
    }

    // initialize additonnal transformation
    // local correction of rig pose
    double  Rcorr[3][3] = {
        {  1.0, 0.0, 0.0 },
        {  0.0, 1.0, 0.0 },
        {  0.0, 0.0, 1.0 }
    };

    double tCorr[3] = { 0.0, 0.0, 0.0 } ;

    // if additionnal transformation is provided
    if( transformation.size() == 4 )
    {
        // update rotation
        for ( size_t i = 0; i < 3 ; ++i )
            for( size_t j = 0 ; j < 3 ; ++j )
                  Rcorr[i][j] = transformation[i][j];

        // update translation
        cA[0] = transformation[3][0];
        cA[1] = transformation[3][1];
        cA[2] = transformation[3][2];
    }

    // check if we could print the projected point on the provided EQR panorama
    bool  bPrint = false ;

    // load image
    Mat pano_img;

    // if a panorama is given, check existence and integrity
    if( !panoPath.empty() )
    {
        if ( !stlplus::file_exists( panoPath ) )
        {
            std::cerr << "The provided EQR panorama not exists, do noting" << std::endl;
        }
        else
        {
            pano_img = imread(panoPath.c_str(), CV_LOAD_IMAGE_COLOR );

            // Check for invalid input (corrupted image or so on )
            if(!pano_img.data )
            {
                std::cerr <<  "Could not open the EQR panorama, do nothing" << std::endl;
            }
            else
                bPrint = true ;  // print projected point cloud on the EQR panoram
        }
    }

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

          for( size_t j = 0; j < vec_sensorData.size()-2 ; ++j ) // kept only the 24 first channel of the eyesis
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

                          if( bPrint )
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
                          }

                          // export projected point
                          std::vector < double > pixels;
                          std::vector < double > point;

                          // store pixels coordinate in EQR panorama frame
                          pixels.push_back(up + sd.lfXPosition);
                          pixels.push_back(vp + sd.lfYPosition);

                          // store depth
                          pixels.push_back( scale * sqrt(xrig * xrig + yrig * yrig + zrig * zrig) );

                          // store 3D coordinate of point and index of point
                          point.push_back( pos[0] + sx );
                          point.push_back( pos[1] + sy );
                          point.push_back( pos[2] + sz );
                          point.push_back( i );

                          pointAndPixels.push_back( std::make_pair( point, pixels ) );

                          ++cpt;
                      }

                  }

              }

          }

    }

    // export projected point cloud on the eqr image
    if( bPrint )
    {
        std::string output_image_filename=outputDirectory+"/"; // output image filename

        //extract image basename
        std::vector<string>  split_slash;
        split( panoPath, "/", split_slash );

        const std::string image_basename =  split_slash[split_slash.size()-1];

        // remove extension
        std::vector<string>  out_split;
        split( image_basename, ".", out_split );

        // create output panorama name
        output_image_filename+=out_split[0]+"-projected-pc.tif";

        // write image on disk
        imwrite(output_image_filename.c_str(), pano_img);
    }

    if( pointAndPixels.size() > 0 )
        return true;
    else
        return false;

};
