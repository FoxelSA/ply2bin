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

using namespace cv;

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
        /// now extract calibration information related to each module ///
        std::vector < sensorData > vec_sensorData;
        bool bLoadedCalibData = loadCalibrationData( vec_sensorData );

        if( !bLoadedCalibData )
        {
          std::cerr << " Could not read calibration data" << std::endl;
          return EXIT_FAILURE;
        }
        else
        {
          std::cout << "Loaded calibration information\n\n" << std::endl;
        }

        /// load point cloud ///
        vector< std::pair < vector <double >, vector<unsigned int> > > pointAndColor;
        bool  bLoadPC = loadPointCloud( argv[1], pointAndColor);

        if( !bLoadPC )
        {
          return  EXIT_FAILURE;
        }
        else
        {
          std::cout << "Loaded point cloud \n\n" << std::endl;
        }

        /// load panorama pose ///
        std::string  posePath = "/home/sflotron/foxel/test/rigid_test/test/SfM_output/rigs/1404374415_319830.txt";
        vector< std::vector<double> > rigPose;

        bool bLoadPose = loadRigPose ( posePath.c_str(), rigPose);

        if( !bLoadPose )
        {
          return EXIT_FAILURE;
        }
        else
        {
          std::cout << "Loaded panorama pose \n\n" << std::endl;
        }

        /// project point cloud on panorama ///
        std::vector < std::pair < std::vector <double>, std::vector <double > > > pointAndPixels;
        bool  bProject = projectPointCloud ( pointAndPixels, pointAndColor, rigPose, vec_sensorData );

        if( !bProject )
        {
            std::cerr << "No point are projected on your panorama \n \n" << std::endl;
            return EXIT_FAILURE;
        }
        else
        {
            std::cout << "Projected point cloud on panorama \n\n" << std::endl;
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
