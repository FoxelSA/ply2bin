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

#include <project.hpp>

using namespace std;
using namespace cv;


/*********************************************************************
* Split an input string with a delimiter and fill a string vector
*
*********************************************************************
*/
static bool split ( const std::string src, const std::string& delim, std::vector<std::string>& vec_value )
{
    bool bDelimiterExist = false;
    if ( !delim.empty() )
    {
        vec_value.clear();
        std::string::size_type start = 0;
        std::string::size_type end = std::string::npos -1;
        while ( end != std::string::npos )
        {
            end = src.find ( delim, start );
            vec_value.push_back ( src.substr ( start, end - start ) );
            start = end + delim.size();
        }
        if ( vec_value.size() >= 2 )
            bDelimiterExist = true;
    }
    return bDelimiterExist;
}

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
           const std::vector < sensorData > & vec_sensorData )
{
    // extract rig rotation and center
    double  Rrig[3][3] = {
        { rigPose[0][0], rigPose[0][1], rigPose[0][2] },
        { rigPose[1][0], rigPose[1][1], rigPose[1][2] },
        { rigPose[2][0], rigPose[2][1], rigPose[2][2] }
    };

    vector <double> cRig = rigPose[3];

    // extract aligned transformation
    double  Ralign[3][3] = {
        { alignedPose[0][0], alignedPose[0][1], alignedPose[0][2] },
        { alignedPose[1][0], alignedPose[1][1], alignedPose[1][2] },
        { alignedPose[2][0], alignedPose[2][1], alignedPose[2][2] }
    };

    vector <double> cA = alignedPose[3];

    // local correction of rig pose
    double  Rcorr[3][3] = {
        {  1.000, 0 * 0.008, -0 * 0.008 },
        { -0 * 0.008, 1.000,  0.000 },
        {  0 * 0.008, 0.000,  1.000 }
    };

    double tCorr[3] = { 0 * 0.048, 0 * 0.013, -0 * 0.244 } ;

#if DEBUG
    // load image
    std::string panoPath = "/data/structure/footage/00-0E-64-08-1B-6E/master/1429143317/segment/1429143318/stitch_selections/dav_3/stitched/result_1429143335_882930.tif";
    Mat pano_img;
    pano_img = imread(panoPath.c_str(), CV_LOAD_IMAGE_COLOR );
#endif

    // project point cloud into panorama
    for( size_t i = 0 ; i < pointAndColor.size(); ++i )
    {
          // retrieve point information
          vector <double> pos   = pointAndColor[i].first;
          double xrig, yrig, zrig;

          double  xcentered[3];

          xcentered[0] = pos[0];
          xcentered[1] = pos[1];
          xcentered[2] = pos[2];

          // undo rotation and scaling that aligned point cloud
          double x_pc = ( Ralign[0][0] * xcentered[0] + Ralign[1][0] * xcentered[1] + Ralign[2][0] *  xcentered[2] + cA[0]) / scale;
          double y_pc = ( Ralign[0][1] * xcentered[0] + Ralign[1][1] * xcentered[1] + Ralign[2][1] *  xcentered[2] + cA[1]) / scale;
          double z_pc = ( Ralign[0][2] * xcentered[0] + Ralign[1][2] * xcentered[1] + Ralign[2][2] *  xcentered[2] + cA[2]) / scale;

          // undo rotation and scaling that move point cloud
          double x_tmp = Rcorr[0][0] * ( x_pc -tCorr[0])  + Rcorr[1][0] * ( y_pc -tCorr[1]) + Rcorr[2][0] *  ( z_pc -tCorr[2]) ;
          double y_tmp = Rcorr[0][1] * ( x_pc -tCorr[0])  + Rcorr[1][1] * ( y_pc -tCorr[1]) + Rcorr[2][1] *  ( z_pc -tCorr[2]) ;
          double z_tmp = Rcorr[0][2] * ( x_pc -tCorr[0])  + Rcorr[1][2] * ( y_pc -tCorr[1]) + Rcorr[2][2] *  ( z_pc -tCorr[2]) ;

          // convert point cloud into rig referential, i.e. x_rig = R_rig ( x - C_rig )
          xrig = Rrig[0][0] * (x_tmp -cRig[0]) + Rrig[0][1] * (y_tmp - cRig[1]) + Rrig[0][2] * ( z_tmp - cRig[2] ) ;
          yrig = Rrig[1][0] * (x_tmp -cRig[0]) + Rrig[1][1] * (y_tmp - cRig[1]) + Rrig[1][2] * ( z_tmp - cRig[2] ) ;
          zrig = Rrig[2][0] * (x_tmp -cRig[0]) + Rrig[2][1] * (y_tmp - cRig[1]) + Rrig[2][2] * ( z_tmp - cRig[2] ) ;

          const lf_Real_t  X[4] = { xrig, yrig, zrig, 1.0 };

          // count the number of subcam in which point is apparing
          lf_Size_t cpt = 0;
          const lf_Real_t  max_depth = 1.0e10;

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

              //  if depth > 0, point could be seen from camera j. Exclude point too far ( > 30.0 meter from rig)
              if( cpt == 0 && depth > 1.0e-6
                           && abs(X_C[0]) < max_depth
                           && abs(X_C[1]) < max_depth
                           && abs(X_C[2]) < max_depth )
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

                      if( up > 0 && up < sd.lfImageFullWidth && vp > 0 && vp < sd.lfImageFullHeight )
                      {

#if DEBUG
                          // export point on panorama (for debug purpose only)
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
#endif
                          // export projected point
                          std::vector < double > pixels;
                          std::vector < double > point;

                          pixels.push_back(up + sd.lfXPosition);
                          pixels.push_back(vp + sd.lfYPosition);
                          pixels.push_back( sqrt(xrig * xrig + yrig * yrig + zrig * zrig) );

                          point.push_back( pos[0] );
                          point.push_back( pos[1] );
                          point.push_back( pos[2] );
                          point.push_back( i );

                          pointAndPixels.push_back( std::make_pair( point, pixels ) );

                          ++cpt;
                      }

                  }

              }

          }

    }

#if DEBUG
    imwrite( "./pointcloud_on_pano.tif", pano_img);
#endif

    if( pointAndPixels.size() > 0 )
        return true;
    else
        return false;

};

/*********************************************************************
*  export projected point cloud to json file
*
**********************************************************************/

void  exportToJson (  const std::string  poseFile,
                      const std::vector < sensorData > & vec_sensorData,
                      const double  scale,
                      std::vector < std::pair < std::vector <double>, std::vector <double > > > pointAndPixels)
{
    // extract pose basename
    std::string  poseBaseName;
    std::vector < std::string >  splitted_name_slash;
    split( poseFile, "/", splitted_name_slash);
    poseBaseName = splitted_name_slash[ splitted_name_slash.size() -1 ];

    // load rig pose
    std::vector < std::vector <double > > rigPose;
    bool  bLoadedPose  = loadRigPose ( poseFile, rigPose );

    // remove extension and add json extension
    std::vector < std::string >  splitted_name;
    split( poseBaseName, ".", splitted_name);
    std::string  jsonFile = splitted_name[splitted_name.size()-2] + ".json";

    // create export stream
    std::string  outpath( jsonFile.c_str() );

    FILE *out;
    out = fopen(outpath.c_str(), "w");

    // extract panorama width in order to convert coordinate in latitude-longitude
    const size_t ImageFullWidth = vec_sensorData[0].lfImageFullWidth;
    const double radPerPix = LG_PI2 / (double) ImageFullWidth;

    //create header
    fprintf(out, "{\n");
    fprintf(out, "\"nb_points\": %ld,\n", pointAndPixels.size());
    fprintf(out, "\"points_format\":  [\"depth\", \"index\", \"theta\", \"phi\", \"x\", \"y\", \"z\"],\n");
    fprintf(out, "\"points\":  [\n");

    // export points and coordinates
    for( int i = 0; i < (int) pointAndPixels.size() ; ++i)
    {
        std::vector <double>  pt      = pointAndPixels[i].first;
        std::vector <double>  pixels  = pointAndPixels[i].second;

        fprintf(out, "%f,", scale * pixels[2] );
        fprintf(out, "%d,", (int) pt[3] );
        fprintf(out, "%f,", pixels[0] * radPerPix );
        fprintf(out, "%f,", pixels[1] * radPerPix - 0.5 * LG_PI );
        fprintf(out, "%f,", pt[0] + 2501600 );
        fprintf(out, "%f,", pt[1] + 1117500 );
        fprintf(out, "%f",  pt[2] );


        if ( i < (int) pointAndPixels.size()-1 )
             fprintf(out, ",\n");
        else
             fprintf(out, "\n");
    }

    fprintf(out, "]\n");
    fprintf(out, "}\n");

    // close stream
    fclose(out);
}

/*********************************************************************
*  export point cloud to json file
*
**********************************************************************/

void  pointCloudToJson ( const char * jsonName,
    std::vector < std::pair < std::vector <double>, std::vector <unsigned int> > > pointAndColor )
{

  // create export stream
  std::string  outpath( jsonName );

  FILE *out;
  out = fopen(outpath.c_str(), "w");

  //create header
  fprintf(out, "{\n");
  fprintf(out, "\"nb_points\": %ld,\n", pointAndColor.size());
  fprintf(out, "\"points_format\":  [\"x\",\"y\",\"z\"],\n");
  fprintf(out, "\"points\": [\n");

  // export points and coordinates
  for( int i = 0; i < (int) pointAndColor.size() ; ++i)
  {
    std::vector <double>        pt      = pointAndColor[i].first;
    std::vector <unsigned int>  color   = pointAndColor[i].second;

    fprintf(out, "%f,", pt[0] );
    fprintf(out, "%f,", pt[1] );
    fprintf(out, "%f" , pt[2] );

    if ( i < (int) pointAndColor.size()-1 )
        fprintf(out, ",\n");
    else
        fprintf(out, "\n");
  }

  fprintf(out, "    ]\n");
  fprintf(out, "}\n");

  // close stream
  fclose(out);

}

/*********************************************************************
*  load calibration data related to elphel cameras
*
**********************************************************************/

bool  loadCalibrationData( std::vector < sensorData > & vec_sensorData,
                           const std::string & sMountPoint,
                           const std::string & smacAddress)
{

    /* Key/value-file descriptor */
    lf_Descriptor_t lfDesc;
    lf_Size_t       lfChannels=0;

    /* Creation and verification of the descriptor */
    char *c_data = new char[sMountPoint.length() + 1];
    std::strcpy(c_data, sMountPoint.c_str());

    char *c_mac = new char[smacAddress.length() + 1];
    std::strcpy(c_mac, smacAddress.c_str());

    // check input data validity
    if ( lf_parse( (unsigned char*)c_mac, (unsigned char*)c_data, & lfDesc ) == LF_TRUE ) {
        /* Query number of camera channels */
        lfChannels = lf_query_channels( & lfDesc );

        for( lf_Size_t sensor_index = 0 ; sensor_index < lfChannels ; ++sensor_index )
        {
            sensorData  sD;

            // query panorama width and height
            sD.lfImageFullWidth  = lf_query_ImageFullWidth ( sensor_index, & lfDesc );
            sD.lfImageFullHeight = lf_query_ImageFullLength( sensor_index, & lfDesc );

            sD.lfpixelCorrectionWidth  = lf_query_pixelCorrectionWidth (sensor_index, &lfDesc);
            sD.lfpixelCorrectionHeight = lf_query_pixelCorrectionHeight(sensor_index, &lfDesc);

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

        return true;
    }
    else
    {
        std::cerr << " Could not read calibration data. " << std::endl;
        return false;
    }

  };

  /*********************************************************************
  *  load point cloud
  *
  **********************************************************************/

bool loadPointCloud ( char * fileName ,   vector< std::pair < vector <double >, vector<unsigned int> > > & pointAndColor )
{
    // create file stream
    ifstream data( fileName );

    //check if file exist for reading
    if( data == NULL){
        fprintf(stderr, "couldn't open point cloud file %s \n ", fileName);
        return false;
    }

    // read data files
    double x,y,z;
    double nx, ny, nz;
    unsigned int r, g, b;
    unsigned int nb_point = 0;

    // skip header and go to line (first 10 lines of file)
    bool  bReadHeader = false;
    lf_Size_t  headerSize = 0;
    while( !data.eof() )
    {
        std::string line ;
        getline(data,line);

        // while header is not read, compute number of line
        if( line == "end_header")
        {
            ++headerSize;
            bReadHeader = true;
        }

        // read some header informations (the number of 3D points)
        std::vector < string > splitted_line;
        split(line, " ", splitted_line);

        if( splitted_line.size() >= 3 )
        {
            if(splitted_line[0] == "element" && splitted_line[1] =="vertex")
              nb_point = atoi( splitted_line[2].c_str() );
        }

        if( !bReadHeader )
            ++headerSize ;

        // for now, read only ply file with 3d coordinate and color
        if( headerSize != 10 && headerSize != 13  && headerSize != 12 && headerSize !=29 && bReadHeader )
        {
            std::cerr << "Ply file not yet supported ! Header should have 10 lines and we have " << headerSize << " lines " << std::endl;
            return false;
        }

        // if we read header, load point cloud.
        if( bReadHeader && headerSize == 10 && pointAndColor.size() < nb_point )
        {
            data >> x >> y >> z >> r >> g >> b ;

            // store point information in big vector
            vector <double>  position;
            vector <unsigned int> color;

            position.push_back(x); position.push_back(y); position.push_back(z);
            color.push_back(r);    color.push_back(g);    color.push_back(b);

            pointAndColor.push_back(std::make_pair(position, color));
        }

        // if we read header, load point cloud.
        if( bReadHeader && (headerSize == 13 || headerSize == 29) && pointAndColor.size() < nb_point )
        {
          data >> x >> y >> z >> r >> g >> b >> nx >> ny >> nz ;

          // store point information in big vector
          vector <double>  position;
          vector <unsigned int> color;

          position.push_back(x); position.push_back(y); position.push_back(z);
          color.push_back(r);    color.push_back(g);    color.push_back(b);

          pointAndColor.push_back(std::make_pair(position, color));
        }
    }

    // close stream
    data.close();

    if( pointAndColor.size () == nb_point )
        return true ;
    else
    {
        std::cerr << "Something went wrong during the loading of the point cloud" << std::endl;
        return false ;
    }

};

/*********************************************************************
*  load rig pose
*
**********************************************************************/

bool  loadRigPose ( const std::string & fileName, vector< std::vector<double> > & rigPose )
{

   // load pose
   ifstream pose(fileName.c_str());

   //check if file exist for reading
    if( pose == NULL){
        fprintf(stderr, "couldn't open pose file %s \n ", fileName.c_str() );
        return false;
    }

    // read pose information
    double x,y,z;
    while (pose >> x >> y >> z){
        // store point information in big vector
        vector <double>  position;

        position.push_back(x); position.push_back(y); position.push_back(z);
        rigPose.push_back(position);
    }

    //close stream
    pose.close();

    if( rigPose.size() == 4 )
        return true;
    else
    {
        std::cerr << "Pose file is not valid, please check your input file \n" << std::endl;
        return false;
    }

}
