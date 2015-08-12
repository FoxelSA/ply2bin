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
        { -rigPose[1][0], -rigPose[1][1], -rigPose[1][2] },
        { -rigPose[2][0], -rigPose[2][1], -rigPose[2][2] }
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
        tCorr[0] = transformation[3][0];
        tCorr[1] = transformation[3][1];
        tCorr[2] = transformation[3][2];
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

    // initialize set of point to render
    std::set < size_t >  set_point_to_render;
    std::vector < bool >  is_point_rendered ;

    for( int i = 0 ; i < pointAndColor.size(); ++i )
        is_point_rendered.push_back( false ) ;

    pointAndPixels.resize( pointAndColor.size() );

    // project point cloud into panorama
    for( size_t j = 0; j < vec_sensorData.size()-2 ; ++j ) // kept only the 24 first channel of the eyesis
    {
          // extract sensor information
          sensorData  sd = vec_sensorData[j];

          // create Buffer map
          const unsigned int  S = pow(2,32)-1;
          std::vector < std::vector < size_t > >  buffer;
          std::vector < std::vector < size_t > >  point_to_render;
          std::vector < std::vector < unsigned char > >  color ;

          for( int kl=0;  kl < sd.lfWidth; ++kl )
          {
              std::vector < size_t >  temp;
              std::vector < size_t >  temp_point;
              std::vector < unsigned char >  temp_col ;

              for( int  kn=0; kn < sd.lfHeight; ++kn )
              {
                  temp.push_back( S );
                  temp_point.push_back( pointAndColor.size() );
                  temp_col.push_back( 255 );
              }

              point_to_render.push_back( temp_point );
              buffer.push_back( temp );
              color.push_back( temp_col );

          }

          // compute alignement transformation ( i.e  T X_{webGL} = scale * R_x(\pi) * R_rig ( R_corr^T { [R_aligned^T (X_{alignedShifted}) + C_a ] / scale - t_corr } - C_rig ) )
          // in the following computations, R_x(\pi) is integrated into R_rig
          double  RrRcT[3][3] = {0};

          for(int ix=0; ix < 3; ++ix)
             for( int jx=0; jx<3 ; ++jx)
                  RrRcT[ix][jx] = Rrig[ix][0] * Rcorr[jx][0] + Rrig[ix][1] * Rcorr[jx][1] + Rrig[ix][2] * Rcorr[jx][2];

          // compute full rotation matrix (= R_rig * R_corr^T * R_aligned^T )
          double   R[3][3] = {0};

          for(int ix=0; ix < 3; ++ix)
             for( int jx=0; jx<3 ; ++jx)
                  R[ix][jx] = RrRcT[ix][0] * Ralign[jx][0] + RrRcT[ix][1] * Ralign[jx][1] + RrRcT[ix][2] * Ralign[jx][2];

          // compute the 3 additionnal translation to sum
          double   t0[3] = {0};
          double   t1[3] = {0};
          double   t2[3] = {0};
          double   t[3] = {0};
          double   Shift[3] = {0};

          for( int ix=0; ix < 3 ; ++ix )
          {
              t0[ix] = RrRcT[ix][0] * cA[0] + RrRcT[ix][1] * cA[1] + RrRcT[ix][2] * cA[2];
              t1[ix] = -scale * ( RrRcT[ix][0] * tCorr[0] + RrRcT[ix][1] * tCorr[1] + RrRcT[ix][2] * tCorr[2] );
              t2[ix] = -scale * ( Rrig[ix][0] * cRig[0] + Rrig[ix][1] * cRig[1] + Rrig[ix][2] * cRig[2] );

              // sum all translation in order to get the full one
              t[ix] = t0[ix] + t1[ix] + t2[ix];
          }

          for( size_t i = 0 ; i < pointAndColor.size(); ++i )
          {
              // retrieve point information
              vector <double> pos   = pointAndColor[i].first;
              double xrig, yrig, zrig;

              // convert point cloud into rig referential, i.e. x_rig = R_x(\pi)^T * ( R * x_{alignedShifted)  + t ) / scale ;
              xrig =  ( R[0][0] * pos[0] + R[0][1] * pos[1] + R[0][2] * pos[2] + t[0] ) / scale ;
              yrig = -( R[1][0] * pos[0] + R[1][1] * pos[1] + R[1][2] * pos[2] + t[1] ) / scale ;
              zrig = -( R[2][0] * pos[0] + R[2][1] * pos[1] + R[2][2] * pos[2] + t[2] ) / scale ;

              const lf_Real_t  X[4] = { xrig, yrig, zrig, 1.0 };

              // compute depth related to camera j
              lf_Real_t  X_C[3] = { X[0] - sd.C[0],  X[1] - sd.C[1], X[2] -sd.C[2]};
              lf_Real_t depth = sd.R[6] * X_C[0] + sd.R[7] * X_C[1] + sd.R[8] * X_C[2];

              // additionnal check on depth (to render point only in near and far plane)
              const  lf_Real_t  near = 0.2;
              const  lf_Real_t   far = 50.0;
              lf_Real_t     zp = 2.0 * ( depth - near ) / (far - near ) - 1.0;
              lf_Real_t    zp1 = ( far + near ) / (2.0 * (far - near)) - far * near / depth / (far - near) + 0.5;
              const size_t  zbuff = floor( S * zp1 );

              // initialize projected pixels
              lf_Real_t  ug = -1.0;
              lf_Real_t  vg = -1.0;

              //  if depth > 0, point could be seen from camera j. Exclude point too far ( > 30.0 meter from rig)
              if( ( zp >= -1.0 && zp <= 1.0 ) && is_point_rendered[i] == false )
              {
                  double  PX0 = sd.P[0] * X[0] + sd.P[1] * X[1] + sd.P[2 ] * X[2] + sd.P[3 ] * X[3];
                  double  PX1 = sd.P[4] * X[0] + sd.P[5] * X[1] + sd.P[6 ] * X[2] + sd.P[7 ] * X[3];
                  double  PX2 = sd.P[8] * X[0] + sd.P[9] * X[1] + sd.P[10] * X[2] + sd.P[11] * X[3];

                  // update projected pixel value
                  ug = PX0 / PX2 ;
                  vg = PX1 / PX2 ;

                  if ( ug > 0.0 && ug < sd.lfWidth && vg > 0.0 && vg < sd.lfHeight)
                  {
                      is_point_rendered[i] = true ;
                      // create z-buffer image
                      int u0 = floor( ug );
                      int v0 = floor( vg );

                      unsigned char pixel_color = 255 ;

                      int patch_size = std::max(15, (int) floor(far/depth) );

                      for( int p = -patch_size ; p < patch_size+1 ; ++p )
                      {
                          for( int q = -patch_size ; q < patch_size+1 ; ++q )
                          {
                              if(  u0+p >=0 && u0+p < sd.lfWidth && v0 + q >= 0 && v0+q  < sd.lfHeight )
                              {
                                if ( zbuff < buffer[u0+p][v0+q] )
                                {
                                    // update z-buffer and point to render list
                                    buffer[u0+p][v0+q] = zbuff ;
                                    point_to_render[u0+p][v0+q] = i ;
                                    pixel_color = floor( 255 * depth / far );
                                    color[u0+p][v0+q] = pixel_color ;
                                }
                              }
                          }
                      };

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

                          // export projected point
                          std::vector < double > pixels;
                          std::vector < double > point;

                          // store pixels coordinate in EQR panorama frame
                          pixels.push_back(up + sd.lfXPosition);
                          pixels.push_back(vp + sd.lfYPosition);

                          // store depth
                          pixels.push_back( scale * sqrt(xrig * xrig + yrig * yrig + zrig * zrig) );
                          pixels.push_back( pixel_color ) ;

                          // store 3D coordinate of point and index of point
                          point.push_back( pos[0] + sx );
                          point.push_back( pos[1] + sy );
                          point.push_back( pos[2] + sz );
                          point.push_back( i );
                          point.push_back( scale * xrig );
                          point.push_back( scale * yrig );
                          point.push_back( scale * zrig );

                          pointAndPixels[i] = std::make_pair( point, pixels );

                      }

                  } // end if ug, vg \in [0,width]x [0, height]

              } // end if zp \in [-1,1]

          }  // end loop on point

          // update set of points to render
          for( int kl=0;  kl < sd.lfWidth; ++kl )
          {
              for( int  kn=0; kn < sd.lfHeight; ++kn )
              {
                  // if rendered pixel point is not the default value, render it.
                  if( point_to_render[kl][kn] != pointAndColor.size() )
                  {
                     set_point_to_render.insert( point_to_render[kl][kn] );
                     pointAndPixels[point_to_render[kl][kn]].second[3] = color[kl][kn];
                  }
              }
          }

    } // end loop on channel

    // clean pointAndPixels lists
    std::vector<std::pair <std::vector <double>, std::vector<double> > > pointAndPixels_cleaned;

    for (std::set<size_t>::iterator it=set_point_to_render.begin(); it!=set_point_to_render.end(); ++it)
        pointAndPixels_cleaned.push_back( pointAndPixels[*it]);

    pointAndPixels.swap(pointAndPixels_cleaned);

    // export projected point cloud on the eqr image
    if( bPrint )
    {
        sensorData sd = vec_sensorData[0];
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

        for( size_t i = 0 ; i < pointAndPixels.size() ; ++i )
        {
          double  x = pointAndPixels[i].second[0];
          double  y = pointAndPixels[i].second[1];
          unsigned int col= pointAndPixels[i].second[3];

          // export point on panorama (for debug purpose only)
          for(int k = -10 ; k < 11; ++k)
              for( int l = -10; l < 11; ++l)
              {
                  // check that written point is in full EQR image
                  if( x + k >= sd.lfImageFullWidth )
                      x -= sd.lfImageFullWidth ;

                  if ( x + k < 0 )
                      x += sd.lfImageFullWidth ;

                  if( y + l >= sd.lfImageFullHeight )
                      y -= sd.lfImageFullWidth ;

                  if ( y + l < 0 )
                      y += sd.lfImageFullHeight ;

                  // export point on stiched panorama
                  if ( x+k >= 0 && x+k < sd.lfImageFullWidth && y+l >= 0 && y+l < sd.lfImageFullHeight )
                  {
                      Vec3b color = pano_img.at<Vec3b>(Point(x + k, y + l));
                      color.val[0] =  col;
                      color.val[1] =  col;
                      color.val[2] =  col;

                      // set pixel
                      pano_img.at<Vec3b>(Point(x + k , y + l)) = color;
                  }
              }

        }

        // write image on disk
        imwrite(output_image_filename.c_str(), pano_img);
    }

    if( pointAndPixels.size() > 0 )
        return true;
    else
        return false;

};
