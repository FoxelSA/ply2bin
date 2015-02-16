/*
 * ply2json- Convert ply point cloud into json file format
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

 /*! \file ply2json.cpp
 * \author Stephane Flotron <s.flotron@foxel.ch>
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

/*! \brief  Convert ply point cloud into json file format
*
* Given a point in ply file format, export it to json file in order
* to read point cloud in freepano
*
* \param point_cloud      complete path and name of the point cloud
* \param json             complete path and name of the json file
*
* \return 0 if the projection and export was sucessfull.
*/

int main(int argc, char** argv) {

    /* Usage branch */
    if ( ( argc!= 3 ) || argc<=1 || strcmp( argv[1], "help" ) == 0  ) {

        /* Display help */
        printf( "Usage :\n\n" );
        printf( "ply2json   point_cloud    json  \n\n");
        printf( "point_cloud   = name of the point cloud  \n" );
        printf( "json          = name of the json file \n");

        return 0;

    } else {
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

        // export point cloud to json 
        pointCloudToJson( argv[2] , pointAndColor );
        return 0;
    }

}
