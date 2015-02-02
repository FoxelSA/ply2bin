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
          std::cout << "Loaded calibration information " << std::endl;
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
          std::cout << "Loaded point cloud " << std::endl;
        }

        /// load panorama pose ///
        std::string  posePath = "/home/sflotron/foxel/test/muref_crown_25pano/test_export/SfM_output/rigs/10.txt";
        vector< std::vector<double> > rigPose;

        bool bLoadPose = loadRigPose ( posePath.c_str(), rigPose);

        if( !bLoadPose )
        {
          return EXIT_FAILURE;
        }
        else
        {
          std::cout << "Loaded panorama pose " << std::endl;
        }

        /// project point cloud on panorama ///
        std::vector < std::pair < std::vector <double>, std::vector <double > > > pointAndPixels;
        bool  bProject = projectPointCloud ( pointAndPixels, pointAndColor, rigPose, vec_sensorData );

        if( !bProject )
        {
            std::cerr << "No point are projected on your panorama " << std::endl;
            return EXIT_FAILURE;
        }
        else
        {
            std::cout << "Projected point cloud on panorama " << std::endl;
        }

        /// export point cloud to json ///
        exportToJson( argv[2] , pointAndPixels );
        return 0;
    }

}
