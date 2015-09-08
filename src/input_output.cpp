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

// initialize output variable
char file_header[FILE_HEADER_SIZE];

using namespace std;
using namespace cv;

/*********************************************************************
*  export projected point cloud to json file
*
**********************************************************************/

bool  exportToJson (  const std::string  poseFile,
                      const std::string  sOutputDirectory,
                      const std::vector < sensorData > & vec_sensorData,
                      std::vector < std::pair < std::vector <double>, std::vector <double > > > pointAndPixels)
{
    // extract pose basename
    std::string  poseBaseName;
    std::vector < std::string >  splitted_name_slash;
    split( poseFile, "/", splitted_name_slash);
    poseBaseName = splitted_name_slash[ splitted_name_slash.size() -1 ];

    // remove extension and add json extension
    std::vector < std::string >  splitted_name;
    split( poseBaseName, ".", splitted_name);
    std::string  jsonFile = sOutputDirectory + "/" + splitted_name[splitted_name.size()-2] + ".json";

    // load rig pose
    std::vector < std::vector <double > > rigPose;
    loadRigPose ( poseFile, rigPose );

    // create export stream
    ofstream out;
    out.precision( 6 );  // used fixed notation with 6 digit of precision
    out.setf( std::ios::fixed );
    out.open( jsonFile.c_str(), ios::trunc ); // erease previous content

    if( out.is_open() )
    {
        // extract panorama width in order to convert coordinate in latitude-longitude
        const size_t ImageFullWidth = vec_sensorData[0].lfImageFullWidth;
        const double radPerPix = LG_PI2 / (double) ImageFullWidth;

        //create header
        out << "{\n";
        out << "\"nb_points\": " << pointAndPixels.size() << ",\n";
        out << "\"points_format\":  [\"depth\", \"index\", \"theta\", \"phi\", \"x\", \"y\", \"z\"],\n";
        out << "\"points\":  [\n";

        // export points and coordinates
        for( int i = 0; i < (int) pointAndPixels.size() ; ++i)
        {
            std::vector <double>  pt      = pointAndPixels[i].first;
            std::vector <double>  pixels  = pointAndPixels[i].second;

            out << pixels[2]   << ",";                           // depth
            out << (int) pt[3] << ",";                           // index
            out << pixels[0] * radPerPix << ",";                 // azimuth
            out << pixels[1] * radPerPix - 0.5 * LG_PI << ",";   // elevation
            out << pt[0] << ",";                                 // x coordinate of point
            out << pt[1] << ",";                                 // y coordinate of point
            out << pt[2] ;                                       // z coordinate of point


            if ( i < (int) pointAndPixels.size()-1 )
                 out << ",\n";
            else
                 out << "\n";
        }

        out << "]\n";
        out << "}\n";

        // close stream
        out.close();

        // export json file sucessfully
        return 1;
    }
    else
    {
       std::cerr << " could not create output file " << std::endl;
       return 0;
    }
}

/*********************************************************************
*  export projected point cloud to binary file format
*
**********************************************************************/

bool  exportToBin ( const std::string  poseFile,
                    const std::string  sOutputDirectory,
                    const std::vector < sensorData > & vec_sensorData,
                    std::vector < std::pair < std::vector <double>, std::vector <double > > > pointAndPixels )
{
    // extract pose basename
    std::string  poseBaseName;
    std::vector < std::string >  splitted_name_slash;
    split( poseFile, "/", splitted_name_slash);
    poseBaseName = splitted_name_slash[ splitted_name_slash.size() -1 ];

    // remove extension and add json extension
    std::vector < std::string >  splitted_name;
    split( poseBaseName, ".", splitted_name);
    std::string  jsonFile = sOutputDirectory + "/" + splitted_name[splitted_name.size()-2] + ".bin";

    // load rig pose
    std::vector < std::vector <double > > rigPose;
    loadRigPose ( poseFile, rigPose );

    // create export stream
    ofstream out;
    out.open( jsonFile.c_str() ); // erease previous content

    if( out.is_open() )
    {
        // extract panorama width in order to convert coordinate in latitude-longitude
        const size_t ImageFullWidth = vec_sensorData[0].lfImageFullWidth;
        const double radPerPix = LG_PI2 / (double) ImageFullWidth;

        // initialize sectors (following luc convention )
        std::list<uint32_t> sector[360][180];

        // initialize output structure
        const double step = M_PI / 180.;
        double depth, theta, phi;
        float x,y,z;
        int longitude, latitude;

        double *mn95=new double[pointAndPixels.size()*3];
        float  *eucl=new float [pointAndPixels.size()*3];

        // create table for binary exports
        for( size_t i = 0; i < pointAndPixels.size() ; ++i)
        {
            std::vector <double>  pt      = pointAndPixels[i].first;
            std::vector <double>  pixels  = pointAndPixels[i].second;

            //extract depth
            depth =  pixels[2];

            // convert pixels into radian
            theta = atan2 ( pt[5] , pt[4] );
            phi   = atan( pt[6] / sqrt( pt[4]*pt[4] + pt[5]*pt[5] ) );
            double theta_2  = pixels[0] * radPerPix;
            double phi_2    = pixels[1] * radPerPix - 0.5 * LG_PI;

            // convert angles into degrees
            longitude = ( (int) round(theta_2 / step ) + 180 ) % 360 ;
            latitude  = round( phi_2 / step );

            // keep only positive latitude
            if( latitude < 0 )
                latitude += 180 ;

            // scale latitude for freepano
            latitude = (180 - latitude ) % 180 ;

            // initialize table index
            unsigned long  k = 3 * i ;
            sector[longitude][latitude].push_back(k);

            // convert panoramic pixels into cartersian webgl coordinates
            x =  depth * cos ( phi ) * cos ( theta );
            y = -depth * cos ( phi ) * sin ( theta );
            z = -depth * sin ( phi );

            // update table
            eucl[ k ]     = x ;
            eucl[ k + 1 ] = y ;
            eucl[ k + 2 ] = z ;

            // update aligned table
            mn95[ k ]     =  pt[0] ;
            mn95[ k + 1 ] =  pt[1] ;
            mn95[ k + 2 ] =  pt[2] ;
        }

        // now truely export file
        //create header (using Luc's convention)
        strncpy(file_header,FILE_MARKER,2);
        strncat(file_header,FILE_VERSION,FILE_HEADER_SIZE-2);
        out.write((char*)file_header,FILE_HEADER_SIZE);

        // initialize variables for binary export
        const long int data_offset = out.tellp(); // get the current position in the stream
        std::list < uint32_t > array_index ;

        // output positions formatted as list of x,y,z for each [lon][lat] pair
        // and prepare a 360x180 array index formatted as offset,count

        for(  size_t lat=0; lat < 180; ++ lat)
        {
           for( size_t lon=0; lon < 360 ; ++lon )
           {
               // extract list of point associated with the pair (lon, lat)
               std::list < uint32_t >  *_sector = &sector[lon][lat];

               // update array index
               uint32_t  particle_count = _sector->size();

               if ( particle_count )
               {
                 // particles in this sector: store offset and particle count
                 array_index.push_back((out.tellp()-data_offset)/sizeof(x));
                 array_index.push_back(particle_count);
               }
               else
               {
                  // no particles here
                  array_index.push_back(0);
                  array_index.push_back(0);
                  continue;
               }

              // write points positions for sector[lon][lat]
              for (std::list<uint32_t>::iterator it=_sector->begin(); it!=_sector->end(); ++it) {
                  unsigned int index=*it;
                  out.write((char*)&eucl[index],sizeof(*eucl)*3);
              }
           } // end loop for on lat
        } // end loop of on lon

        // check integrity
        const size_t positions_byteCount=out.tellp()-data_offset;         // equals final position minus initial position
        const bool   bFailure =
                            (positions_byteCount / pointAndPixels.size() != sizeof(*eucl) * 3 ); // check that we exported the correct number of bytes

        if ( bFailure) {
          std::cerr << poseFile << ": position exported " << positions_byteCount << " bytes on " << sizeof(*eucl) * 3 * pointAndPixels.size() << std::endl;
          return 0;
        }

        // now export aligned coordinates

        // align start of double array on 8 bytes
        const size_t  positions_filler=positions_byteCount%8;
        if (positions_filler) {
            out.write("fillfill",8-positions_filler);
        }

        // output positions formatted as list of x,y,z for each [lon][lat] pair
        // and prepare a 360x180 array index formatted as offset,count
        for(  size_t lat=0; lat < 180; ++ lat)
        {
           for( size_t lon=0; lon < 360 ; ++lon )
           {
               // extract list of point associated with the pair (lon, lat)
               std::list < uint32_t >  *_sector = &sector[lon][lat];

               // update array index
               uint32_t  particle_count = _sector->size();

               if ( particle_count == 0 )
               {
                  continue;
               }

              // write points positions for sector[lon][lat]
              for (std::list<uint32_t>::iterator it=_sector->begin(); it!=_sector->end(); ++it) {
                  unsigned int index=*it;
                  out.write((char*)&mn95[index],sizeof(*mn95)*3);
              }

           } // end loop for on lat
        } // end loop of on lon

        // check integrity
        const size_t mn95_byteCount = (size_t) out.tellp() - data_offset - positions_byteCount - positions_filler;         // equals final position minus initial position
        const bool   bAlignedFailure =( mn95_byteCount != 2*positions_byteCount ); // check that we exported the correct number of bytes

        if ( bAlignedFailure ) {
          std::cerr << poseFile << ": aligned exported " << mn95_byteCount << " bytes on " << sizeof(*mn95) * 3 * pointAndPixels.size() << std::endl;
          return 0;
        }

        // if alignment was needed before mn95 array, add it twice after (for proper table size computation in js)
        if (positions_filler) {
            out.write("fillfill",8-positions_filler);
            out.write("fillfill",8-positions_filler);
        }

        // output index formatted as:
        // offset, particle_count
        for (std::list<uint32_t>::iterator it=array_index.begin(); it!=array_index.end(); ++it) {
          uint32_t value=*it;
          out.write((char*)&value,sizeof(value));
        }

        // check integrity
        flush(out);
        size_t  index_byteCount=(size_t)out.tellp() - data_offset - positions_byteCount - mn95_byteCount - (positions_filler * 3);
        const bool bFinalFailure = (index_byteCount % sizeof(uint32_t) );

        if (bFinalFailure) {
          std::cerr << poseFile << " index exported " << index_byteCount << " bytes on " << sizeof(uint32_t) * array_index.size() << std::endl;
          return 0;
        }

        // close stream
        out.write((char*)FILE_MARKER,strlen(FILE_MARKER));
        out.close();

        if (mn95_byteCount != 2*positions_byteCount ) {
          std::cerr << "error: aligned bytes count != 2 * euclidian bytes count ! "  << mn95_byteCount << " " << 2 * positions_byteCount << std::endl;
          return 0;
        }

        // export json file sucessfully
        return 1;
    }
    else
    {
       std::cerr << " could not create output file " << std::endl;
       return 0;
    }
}

/*********************************************************************
*  export point cloud to json file
*
**********************************************************************/

void  pointCloudToJson ( const char * jsonName,
    std::vector < std::pair < std::vector <double>,
    std::vector <unsigned int> > > pointAndColor )
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

    fprintf(out, "%f,", pt[0] );   // x coordinate
    fprintf(out, "%f,", pt[1] );   // y coordinate
    fprintf(out, "%f" , pt[2] );   // z coordinate

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
    // read some header informations (the number of 3D points)
    std::string sFileName(fileName);
    std::vector < string > splitted_name;
    split(sFileName, ".", splitted_name);

    if( splitted_name.size() > 1 )
    {
        if( splitted_name[splitted_name.size()-1] != "ply")
        {
           std::cerr << "provided point cloud is not in ply file format " << std::endl;
           return false;
        }
    }
    else
    {
        std::cerr << "Could not detect point cloud extension." << std::endl;
        return false;
    }

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

        // remove empty strings
        std::vector < string >  splitted_without_empty;
        for( size_t i = 0 ; i < splitted_line.size(); ++i )
        {
          if( !splitted_line[i].empty() )
            splitted_without_empty.push_back( splitted_line[i] );
        }

        splitted_line.swap( splitted_without_empty );

        // if tabular is empty, go ahead
        if(splitted_line.empty() )
          continue ;

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
