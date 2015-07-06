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

 /*! \file ply2bin.cpp
 * \author Stephane Flotron <s.flotron@foxel.ch>
 */
 /*! \mainpage ply2bin
 * \section ply2bin
 *
 * Point cloud exportation to freepano
 *
 * \section Documentation
 *
 * Documentation can be consulted on the [wiki](https://github.com/baritone/poco2pano/wiki).
 *
 * \section Copyright
 *
 * Copyright (c) 2015 FOXEL SA - [http://foxel.ch](http://foxel.ch)<br />
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
#include "file_system.hpp"
#include "cmdLine.h"

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

    CmdLine cmd;

    // load inputs
    std::string  sPointCloud="";       // original point cloud in ply format
    std::string  sPoseFile="";         // camera rig pose file
    std::string  sAlignedFile="";      // alignment transformation
    std::string  sScaleFile="";        // scale factor file
    std::string  sMountPoint="";       // mount point
    std::string  sMacAdress="";        // camera mac address
    std::string  sAddTrans="";         // additionnal transformation
    std::string  sOutputDirectory="";  // output directory
    std::string  sPanoPath="";         // complete path of the eqr panorama
    bool         bUseJson=0;           // export point cloud in json or binary format
    double       sx(0.0);              // shift to add to x to get true aligned coordinates
    double       sy(0.0);              // shift to add to y to get true aligned coordinates
    double       sz(0.0);              // shift to add to z to get true aligned coordinates

    cmd.add( make_option('p', sPointCloud, "pointCloud") );
    cmd.add( make_option('f', sPoseFile, "poseFile") );
    cmd.add( make_option('m', sMacAdress, "macAddress") );
    cmd.add( make_option('d', sMountPoint, "mountPoint") );
    cmd.add( make_option('a', sAlignedFile, "align") );
    cmd.add( make_option('s', sScaleFile, "scale") );
    cmd.add( make_option('t', sAddTrans, "addTrans") );
    cmd.add( make_option('o', sOutputDirectory, "output") );
    cmd.add( make_option('i', sPanoPath, "pano") );
    cmd.add( make_option('b', bUseJson, "useJson") );
    cmd.add( make_option('x', sx, "x") );
    cmd.add( make_option('y', sy, "y") );
    cmd.add( make_option('z', sz, "z") );

    try {
          if (argc == 1) throw std::string("Invalid command line parameter.");
          cmd.process(argc, argv);
    } catch(const std::string& s) {
          std::cerr << "Usage: " << argv[0] << '\n'
          << "[-p|--pointCloud]  pose cloud in ply format\n"
          << "[-f|--poseFile]    pose file\n"
          << "[-m|--macAddress]  camera mac adress\n"
          << "[-d|--mountPoint]  mount point \n"
          << "[-a|--align]       alignment transformation\n"
          << "[-s|--scale]       scale factor file to have metric scale\n"
          << "[-t|--addTrans]    additional transformation \n"
          << "[-o|--output]      output directory\n"
          << "[-b|--useJson]     export point cloud in json file or binary file. Default is binary\n"
          << "[-i|--pano]        complete path of the eqr panorama associated to pose file\n"
          << "[-x|--sx]          x-coordinate shift to get true aligned coordinates\n"
          << "[-y|--sy]          y-coordinate shift to get true aligned coordinates\n"
          << "[-z|--sz]          z-coordinate shift to get true aligned coordinates\n"
          << std::endl;

          std::cerr << s << std::endl;
          return EXIT_FAILURE;
    }

    // now extract calibration information related to each module
    std::vector < sensorData > vec_sensorData;
    bool bLoadedCalibData = loadCalibrationData( vec_sensorData,
                                                     sMountPoint.c_str(),
                                                     sMacAdress.c_str() );

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
    bool  bLoadPC = loadPointCloud( sPointCloud.c_str(), pointAndColor);

    if( !bLoadPC )
    {
        return  EXIT_FAILURE;
    }
    else
    {
        std::cout << "Loaded point cloud " << std::endl;
    }

    // load scale factor
    ifstream pose(sScaleFile.c_str());

    //check if file exist for reading
    if( pose == NULL){
        fprintf(stderr, "couldn't open scale file %s \n ", sScaleFile.c_str() );
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
    vector< std::vector<double> > rigPose;
    bool bLoadPose = loadRigPose ( sPoseFile.c_str(), rigPose);

    // load alignement transformation
    vector< std::vector<double> > alignedPose;

    bool bLoadAligned = loadRigPose ( sAlignedFile.c_str(), alignedPose);

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
    exportToJson( sPoseFile.c_str() , vec_sensorData, scale, pointAndPixels );
    return 0;

}
