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

 /*! \file poco2pano.cpp
 * \author Stephane Flotron <s.flotron@foxel.ch>
 */
 /*! \mainpage poco2pano
 * \section poco2pano
 *
 * Point cloud exportation to freepano
 *
 * \section Documentation
 *
 * Documentation can be consulted on the [wiki](https://github.com/baritone/poco2pano/wiki).
 *
 * \section Copyright
 *
 * Copyright (c) 2013-2014 FOXEL SA - [http://foxel.ch](http://foxel.ch)<br />
 * This program is part of the FOXEL project <[http://foxel.ch](http://foxel.ch)>.
 *
 * Please read the [COPYRIGHT.md](COPYRIGHT.md) file for more information.
 *
 * \section License
 *
 * This program is licensed under the terms of the
 * [GNU Affero General Public License v3](http://www.gnu.org/licenses/agpl.html)
 * (GNU AGPL), with two additional terms. The content is licensed under the terms
 * of the [Creative Commons Attribution-ShareAlike 4.0 International](http://creativecommons.org/licenses/by-sa/4.0/)
 * (CC BY-SA) license.
 *
 * You must read <[http://foxel.ch/license](http://foxel.ch/license)> for more
 *information about our Licensing terms and our Usage and Attribution guidelines.
 *
 */

#include <stdlib.h>
#include <cmath>
#include <cstring>
#include <gnomonic-all.h>
#include <project.hpp>

using namespace cv;

/*********************************************************************
*  software main function
*
**********************************************************************/

/*! \brief  Given a point cloud and a rig pose, project the point cloud
*           on the stiched EQR panorama.
*
* Given a panorama pose (Rotation and translation), project the point cloud
* on the associated submodules and compute the corresponding pixels on the
* stiched EQR panorama.
*
* \param point_cloud      complete path and name of the point cloud
* \param json             complete path and name of the json file
*
* \return 0 if the projection and export was sucessfull.
*/

int main(int argc, char** argv) {

    /* Usage branch */
    if ( ( argc!= 7 ) || argc<=1 || strcmp( argv[1], "help" ) == 0  ) {

        /* Display help */
        printf( "Usage :\n\n" );
        printf( "poco2pano   point_cloud   pose_file  aligned_file  scale_file  mount_point   mac_adress \n\n");
        printf( "point_cloud   = name of the point cloud  \n" );
        printf( "pose_file     = complete path of the pose file \n");
        printf( "aligned_file  = complete path of the alignement transformation \n");
        printf( "scale_file    = complete path of the scaling matrix\n");
        printf( "mount_point   = mount point of camera folder \n");
        printf( "mac_address   = camera mac address \n");

        return 0;

    } else {
        // now extract calibration information related to each module
        std::string  sMountPoint = argv[5];
        std::string  smacAddress = argv[6];

        std::vector < sensorData > vec_sensorData;
        bool bLoadedCalibData = loadCalibrationData( vec_sensorData,
                                                     sMountPoint,
                                                     smacAddress );

        if( !bLoadedCalibData )
        {
            std::cerr << " Could not read calibration data" << std::endl;
            return EXIT_FAILURE;
        }
        else
        {
            std::cout << "Loaded calibration information " << std::endl;
        }

        // load point cloud
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

        // load scale factor
        ifstream pose(argv[4]);

        //check if file exist for reading
         if( pose == NULL){
             fprintf(stderr, "couldn't open scale file %s \n ", argv[4] );
             return false;
         }

         // read pose information
         double x,y,z;
         while (pose >> x >> y >> z){
           std::cout << "load scale file" << std::endl;
         }

         //close stream
         pose.close();

         const double scale = (x + y + z) / 3.0;

        // load panorama pose
        std::string  posePath = argv[2];
        vector< std::vector<double> > rigPose;

        bool bLoadPose = loadRigPose ( posePath, rigPose);

        // load alignement transformation
        std::string  alignedPath = argv[3];
        vector< std::vector<double> > alignedPose;

        bool bLoadAligned = loadRigPose ( alignedPath, alignedPose);

        if( !bLoadPose || !bLoadAligned )
        {
            return EXIT_FAILURE;
        }
        else
        {
            std::cout << "Loaded panorama pose " << std::endl;
        }

        // project point cloud on panorama
        std::vector < std::pair < std::vector <double>, std::vector <double > > > pointAndPixels;
        bool  bProject = projectPointCloud ( pointAndPixels, pointAndColor, rigPose, alignedPose, scale, vec_sensorData );

        if( !bProject )
        {
            std::cerr << "No point are projected on your panorama " << std::endl;
            return EXIT_FAILURE;
        }
        else
        {
            std::cout << "Projected point cloud on panorama " << std::endl;
        }

        // export point cloud to json
        exportToJson( argv[2] , vec_sensorData, scale, pointAndPixels );
        return 0;
    }

}
