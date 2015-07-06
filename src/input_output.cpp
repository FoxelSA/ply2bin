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

#include <input_output.hpp>

using namespace std;
using namespace cv;

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

bool loadPointCloud ( const char * fileName ,   vector< std::pair < vector <double >, vector<unsigned int> > > & pointAndColor )
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
    unsigned int r, g, b;
    unsigned int nb_point = 0;
    std::vector < std::pair<size_t, std::string> > order;

    // skip header and go to line (first 10 lines of file)
    bool  bReadHeader = false;
    size_t  cprop=0;
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
        {
          if(splitted_line[0] != "comment" || splitted_line[0]!="obj_info")
            ++headerSize ;
        }

        // detect x,y,z coordinate as colors
        if( splitted_line[0]=="property")
        {
            // detect x coordinate flag
            if( (splitted_line[1]=="float" || splitted_line[1] =="double")  && splitted_line[2]=="x" )
            {
               order.push_back( std::make_pair(cprop, splitted_line[2]) );
            }

            // detect y coordinate flag
            if( (splitted_line[1]=="float" || splitted_line[1] =="double") && splitted_line[2]=="y" )
            {
               order.push_back( std::make_pair(cprop, splitted_line[2]) );
            }

            // detect z coordinate flag
            if( (splitted_line[1]=="float" || splitted_line[1] =="double") && splitted_line[2]=="z" )
            {
               order.push_back( std::make_pair(cprop, splitted_line[2]) );
            }

            //detect color
            // detect red color
            if( ( splitted_line[1]=="uchar" || splitted_line[1]=="char" ) && ( splitted_line[2]=="red" || splitted_line[2]=="diffuse_red") )
            {
               order.push_back( std::make_pair(cprop, splitted_line[2]) );
            }

            // detect green color
            if( ( splitted_line[1]=="uchar" || splitted_line[1]=="char" ) && ( splitted_line[2]=="green" || splitted_line[2]=="diffuse_green") )
            {
               order.push_back( std::make_pair(cprop, splitted_line[2]) );
            }

            // detect blue color
            if( ( splitted_line[1]=="uchar" || splitted_line[1]=="char" ) && ( splitted_line[2]=="blue" || splitted_line[2]=="diffuse_blue") )
            {
               order.push_back( std::make_pair(cprop, splitted_line[2]) );
            }

            ++cprop;
        }

        // if we read header, load point cloud.
        if( bReadHeader && pointAndColor.size() < nb_point && ( order.size() == 6 || order.size() == 3) && line != "end_header")
        {
            for( size_t i = 0; i < order.size() ; ++i )
            {
                // extract position of the value associated to variable
                const size_t index = order[i].first;

                // initalized coordinate value
                if ( order[i].second == "x" )
                    x = atof( splitted_line[index].c_str() );

                if ( order[i].second == "y" )
                    y = atof( splitted_line[index].c_str() );

                if ( order[i].second == "z" )
                    z = atof( splitted_line[index].c_str() );

                // if color field found
                if( order.size() == 6 )
                {
                  if ( order[i].second == "red" || order[i].second == "diffuse_red" )
                      r = atoi( splitted_line[index].c_str() );

                  if ( order[i].second == "green" || order[i].second == "diffuse_green" )
                      g = atoi( splitted_line[index].c_str() );

                  if ( order[i].second == "blue" || order[i].second == "diffuse_blue")
                      b = atoi( splitted_line[index].c_str() );
                }
                else
                {
                    // export default color (white)
                    r = 255; g = 255; b=255;
                }
            }

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
        std::cerr << "Loaded " << pointAndColor.size() << " points over " << nb_point << std::endl;
        std::cerr << "Supports only single  x y z in float / double fields " << std::endl;
        std::cerr << "Supports only single (ambient_)red, (ambient_)green, (ambient_)blue char /  uchar fields " << std::endl;
        std::cerr << "Does not supported meshes" << std::endl;
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
            const std::string sScaleFile)
{
    // initialize scale in each direction
    double x(0.0), y(0.0), z(0.0);

    // load scale factor
    ifstream pose(sScaleFile.c_str());

    //check if file exist for reading
    if( pose == NULL){
        fprintf(stderr, "couldn't open scale file %s \n ", sScaleFile.c_str() );
        return false;
    }
    else
    {
        size_t cpt = 0; // number of line of scale file

        while (pose >> x >> y >> z){
            ++cpt;
        }

        // scale file should have only one line
        if( cpt != 1 )
        {
            std::cerr << "Wrong scale file format. We should have one line and we have " << cpt << " lines " << std::endl;
            return false;
        }
    }
    //close stream
    pose.close();

    // compute average scale factor
    scale = (x + y + z) / 3.0;

    // keep only stricly positive scale factors
    if( scale > 0.0 )
      return true ;
    else
      return false ;
};
