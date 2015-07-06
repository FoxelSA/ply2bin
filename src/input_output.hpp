/*
 * input_output.hpp - Header file of openMVG point cloud exportation to freepano
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

 /*! \file input_output.hpp
 * \author Stephane Flotron <s.flotron@foxel.ch>
 */

 #ifndef INPUT_OUTPUT_HPP_
 #define INPUT_OUTPUT_HPP_

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
 #include <tools.hpp>
 #include <gnomonic-all.h>

 #define DEBUG 0

 using namespace std;
 using namespace cv;

 /*********************************************************************
 *  export projected point cloud to json file
 *
 **********************************************************************/

 /*! \brief Export projected point cloud in a json
 *
 * Given a list of 3D point and corresponding pixels on stiched EQR panorama,
 * export theses informations to a json file.
 *
 * \param poseName         Name of the pose file. JSON export will have the same
 * \param vec_sensorData   Calibration informations
 * \param scale            Scale factor to obtain metric point cloud
 * \param pointAndPixels   List of 3D points and associated EQR pixels
 *
 * \return  Nothing
 */

 void  exportToJson (  const std::string  poseFile,
                       const std::vector < sensorData > & vec_sensorData,
                       const double  scale,
                       std::vector < std::pair < std::vector <double>, std::vector <double > > > pointAndPixels
 );

 /*********************************************************************
 *  export point cloud to json file
 *
 **********************************************************************/

 /*! \brief Export point cloud in ply format into json file
 *
 * Given a list of 3D point, export it to a json file.
 *
 * \param jsonName         Name of json file in c-string array
 * \param pointAndPixels   List of 3D points and associated EQR pixels
 *
 * \return  Nothing
 */

 void  pointCloudToJson ( const char * jsonName,
                          std::vector < std::pair < std::vector <double>, std::vector <unsigned int> > > pointAndColor
 );


 /*********************************************************************
 *  load calibration data related to elphel cameras
 *
 **********************************************************************/

 /*! \brief Load calibration information
 *
 * Given a mac address and a mount point, load all information needed to do
 * the point cloud projection on the panorama.
 *
 * \param vec_sensorData   Array used to store the sensor calibration information.
 * \param sMountPoint      Camera folder mount point
 * \param smacAddress      Elphel camera's mac address
 *
 * \return  bool value telling if the loading was sucessfull or not.
 */

 bool  loadCalibrationData(std::vector < sensorData > & vec_sensorData,
                           const std::string & sMountPoint,
                           const std::string & smacAddress
);

/*********************************************************************
*  load point cloud
*
**********************************************************************/

/*! \brief Ply reader
*
* Given a point cloud in ply format, read the file and store the corresponding
* 3D points in a vector of pair < position, color >
*
* \param vec_sensorData   Array used to store the sensor calibration information.
*
* \return  bool value telling if the loading was sucessfull or not.
*/

bool loadPointCloud ( const char * fileName ,
                      vector< std::pair < vector <double >, vector<unsigned int> > > & pointAndColor
);

/*********************************************************************
*  load rig pose
*
**********************************************************************/

/*! \brief Pose file parser
*
* Given a rig pose file generated by openMVG, read it and store the information
* in an array of double.
*
* \param  fileName   rig pose filename
* \param  rigPose    The loaded rig pose
*
* \return  bool value telling if the loading was sucessfull or not.
*/

bool  loadRigPose ( const std::string & fileName,
                    vector< std::vector<double> > & rigPose
);


 /********************************************************************************
  *  Given a scale factor file, load scales and return average value
  *
  ********************************************************************************
 */

 /*! \brief Load scale factors from file and return the average value
 *
 * \param  scale The average scale factor
 * \param  sScaleFile Scale factor file
 *
 * \return The average scale factor
 */

 bool loadScaleFactor (
             lf_Real_t &scale,
             const std::string sScaleFile);

#endif // INPUT_OUTPUT_HPP_
