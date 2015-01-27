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

#include <stdlib.h>
#include <cmath>
#include <cstring>
#include <gnomonic-all.h>
#include "types.hpp"
#include "poco2pano.hpp"

int main(int argc, char** argv) {

    /* Usage branch */
    if ( ( argc!= 3 ) || argc<=1 || strcmp( argv[1], "help" ) == 0  ) {

        /* Display help */
        printf( "Usage :\n\n" );
        printf( "poco2pano   point_cloud    json  \n\n");
        printf( "point_cloud   = name of the point cloud  \n" );
        printf( "json          = name of the json file \n");

        return 0;

    } else {

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

        if ( lf_parse( (unsigned char*)c_mac, (unsigned char*)c_data, & lfDesc ) == LF_TRUE ) {

          /* Query number of camera channels */
          lfChannels = lf_query_channels( & lfDesc );
        }
        else
        {
          std::cerr << " Could not read calibration data. " << std::endl;
          return EXIT_FAILURE;
        }

        // query panorama width and height
        lf_Size_t  lf_imageFullWidth  = lf_query_ImageFullWidth ( 0, & lfDesc );
        lf_Size_t  lf_imageFullHeight = lf_query_ImageFullLength( 0, & lfDesc );

        // now extract calibration information related to each module
        std::vector < sensorData > vec_sensorData;

        for( lf_Size_t sensor_index = 0 ; sensor_index < lfChannels ; ++sensor_index )
        {
          sensorData  sD;

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

        printf( "calibration data loaded \n");

        // load output and eliminate false correspondences using fundamental matrix condition
        ifstream data(argv[1]);
        vector< std::pair < vector <double >, vector<unsigned int> > > pointAndColor;

        //check if file exist for reading
        if( data == NULL){
            fprintf(stderr, "couldn't open point cloud file %s \n ", argv[4]);
            return -1;
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

        printf( "data loaded \n");

        // project point cloud into panorama
        for( size_t i = 0 ; i < pointAndColor.size(); ++i )
        {
           // retreive point information
           vector <double> pos   = pointAndColor[i].first;
           const lf_Real_t  X[4] = {pos[0], pos[1], pos[2], 1.0 };

           for( size_t j = 0; j < vec_sensorData.size()-2 ; ++j )
           {
                // extract sensor information
                sensorData  sd = vec_sensorData[j];

                // compute depth related to camera j
                lf_Real_t  X_C[3] = { pos[0] - sd.C[0],  pos[1] - sd.C[1], pos[2] - sd.C[2]};
                lf_Real_t depth = sd.R[6] * X_C[0] + sd.R[7] * X_C[1] + sd.R[8] * X_C[2];

                // initialize projected pixels
                lf_Real_t  ug = -1.0;
                lf_Real_t  vg = -1.0;

                //  if depth > 0, point could be seen from camera j
                if( depth > 1.0e-6 )
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
                    }
                }
           }
        }

#if 0
        // create export stream
        std::string  outpath(argv[2]);

        FILE *out;
        out = fopen(outpath.c_str(), "w");

        //create header
        fprintf(out, "ply\n");
        fprintf(out, "format ascii 1.0\n");
        fprintf(out, "element vertex %d\n", (int) vec_inliers.size());
        fprintf(out, "property float x\n");
        fprintf(out, "property float y\n");
        fprintf(out, "property float z\n");
        fprintf(out, "property uchar red\n");
        fprintf(out, "property uchar green\n");
        fprintf(out, "property uchar blue\n");
        fprintf(out, "end_header\n");

       // clean point cloud
       for( i = 0; i < (int) vec_inliers.size() ; ++i)
       {
          // retreive point information
          vector <double> position = pointAndColor[vec_inliers[i]].first;
          vector <unsigned int> color    = pointAndColor[vec_inliers[i]].second;

          // write coordinates in files
          fprintf(out,"%15f %15f %15f ", position[0] , position[1], position[2]);

          // write colors in file
          fprintf(out, "%6d %6d %6d \n", color[0], color[1], color[2] );

       }

       // close stream
       fclose(out);
#endif

       return 0;
    }

}
