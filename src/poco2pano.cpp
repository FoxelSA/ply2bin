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
        // now extract calibration information related to each module
        std::vector < sensorData > vec_sensorData;

        bool bLoadedCalibData = loadCalibrationData( vec_sensorData);

        if( !bLoadedCalibData )
        {
          std::cerr << " Could not read calibration data" << std::endl;
          return EXIT_FAILURE;
        }
        else
        {
          std::cout << "Loaded calibration information\n\n" << std::endl;
        }

        // load output and eliminate false correspondences using fundamental matrix condition
        vector< std::pair < vector <double >, vector<unsigned int> > > pointAndColor;

        bool  bLoadPC = loadPointCloud( argv[1], pointAndColor);

        if( !bLoadPC )
        {
          std::cerr << "Could not load point cloud " << std::endl;
          return  EXIT_FAILURE;
        }
        else
        {
          std::cout << "Loaded point cloud \n\n" << std::endl;
        }

        // load panorama pose
        std::string  posePath = "/home/sflotron/foxel/test/muref_crown_25pano/10.txt";

        // load output and eliminate false correspondences using fundamental matrix condition
        ifstream pose(posePath.c_str());
        vector< std::vector<double> > rigPose;

        //check if file exist for reading
        if( pose == NULL){
          fprintf(stderr, "couldn't open pose file %s \n ", posePath.c_str());
          return -1;
        }

        // read pose information
        double x,y,z;
        while (pose >> x >> y >> z){
            // store point information in big vector
            vector <double>  position;

            position.push_back(x); position.push_back(y); position.push_back(z);
            rigPose.push_back(position);
        }

        // extract rig rotation and center
        double  Rrig[3][3] = {
            { rigPose[0][0], rigPose[0][1], rigPose[0][2] },
            { rigPose[1][0], rigPose[1][1], rigPose[1][2] },
            { rigPose[2][0], rigPose[2][1], rigPose[2][2] }
        };

        vector <double> cRig = rigPose[3];

        cout << "pose extracted " << endl;

        // load image
        std::string panoPath = "/home/sflotron/result_1404374551_319830.tif";
        Mat pano_img;
        pano_img = imread(panoPath.c_str(), CV_LOAD_IMAGE_COLOR );

        // project point cloud into panorama
        for( size_t i = 0 ; i < pointAndColor.size(); ++i )
        {
           // retreive point information
           vector <double> pos   = pointAndColor[i].first;

           // convert point cloud into rig referential, i.e. x_rig = R_rig ( x - C_rig )
           double  xcentered[3];

           xcentered[0] = pos[0] - cRig[0];
           xcentered[1] = pos[1] - cRig[1];
           xcentered[2] = pos[2] - cRig[2];

           double xrig = Rrig[0][0] * xcentered[0] + Rrig[0][1] * xcentered[1] + Rrig[0][2] *  xcentered[2];
           double yrig = Rrig[1][0] * xcentered[0] + Rrig[1][1] * xcentered[1] + Rrig[1][2] *  xcentered[2];
           double zrig = Rrig[2][0] * xcentered[0] + Rrig[2][1] * xcentered[1] + Rrig[2][2] *  xcentered[2];

           const lf_Real_t  X[4] = { xrig, yrig, zrig, 1.0 };

           for( size_t j = 0; j < vec_sensorData.size()-2 ; ++j )
           {
                // extract sensor information
                sensorData  sd = vec_sensorData[j];

                // compute depth related to camera j
                lf_Real_t  X_C[3] = { X[0] - sd.C[0],  X[1] - sd.C[1], X[2] -sd.C[2]};
                lf_Real_t depth = sd.R[6] * X_C[0] + sd.R[7] * X_C[1] + sd.R[8] * X_C[2];

                // initialize projected pixels
                lf_Real_t  ug = -1.0;
                lf_Real_t  vg = -1.0;

                //  if depth > 0, point could be seen from camera j
                if( depth > 1.0e-6  && sqrt(X_C[0] * X_C[0] + X_C[1]*X_C[1] + X_C[2]*X_C[2]) < 100.0 )
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

                        if( up > 5 && up < sd.lfImageFullWidth-5 && vp > 5 && vp < sd.lfImageFullHeight-5 )
                        {
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
                    }
                }
           }
        }

        imwrite( "./pointcloud_on_pano.tif", pano_img);


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
