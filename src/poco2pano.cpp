/*
 * poco2pano - Export point cloud to panorama
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

        // set some output parameters
         cout.setf(ios::left);
         cout.setf(ios::scientific);

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

        //extract camera position
        int nPoint = (int) pointAndColor.size();
        std::vector< std::vector <double> > camPos;

        for(int j = 0; j < nPoint ; ++j)
        {
          const unsigned int r =  pointAndColor[j].second[0];
          const unsigned int g =  pointAndColor[j].second[1];
          const unsigned int b =  pointAndColor[j].second[2];

          // if a point has camera color code, add it to camera list
          if( r == 0 && g == 255 && b == 0 )
          {
            const double x = pointAndColor[j].first[0];
            const double y = pointAndColor[j].first[1];
            const double z = pointAndColor[j].first[2];

            std::vector<double> temp;
            temp.push_back(x); temp.push_back(y); temp.push_back(z);
            camPos.push_back(temp);
          }
        }

        // compute number of kept point
        std::vector<size_t> vec_inliers;
        int i = 0;
        for( i = 0; i < nPoint ; ++i)
        {
           // retreive point information
           vector <double> position = pointAndColor[i].first;
           vector <unsigned int> color    = pointAndColor[i].second;

           // compute minimal distance to a camera
           double minDist = 1.0e10;

           for(int j=0; j < (int) camPos.size() ; ++j )
           {
               double xd = position[0] - camPos[j][0];
               double yd = position[1] - camPos[j][1];
               double zd = position[2] - camPos[j][2];

               double distance = sqrt(xd * xd + yd * yd + zd * zd);
               if( distance < minDist )
                 minDist = distance;
           }

           // keep only point whose mininal distance to camera is less than 100 meters
           if( minDist < 10000 )
             vec_inliers.push_back(i);
        }

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

       return 0;
    }

}
