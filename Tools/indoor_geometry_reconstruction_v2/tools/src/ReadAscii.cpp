//
// Created by NikoohematS on 23-1-2017.
//

#include <stdlib.h>
#include <string.h>
#include "LaserPoints.h"
#include <boost/algorithm/string.hpp>

float clip(float n, float lower, float upper) {
    return std::max(lower, std::min(n, upper));
}

LaserPoints read_ascii(char *ascii_file){

    FILE *ascii;
    printf ("Reading in ascii laser points...\n");
    ascii = Open_Compressed_File(ascii_file, "r");
    if (!ascii) {
        fprintf(stderr, "Error opening input file %s\n", ascii_file);
        exit(0);
    }
//read in ascii laser points, transfer to laser format
    char    line[2048], *comma;
    double  value[21];
    int     index1=0;
    LaserPoint point;
    LaserPoints temp_laser_points, laser_points;

    fgets(line, 2048, ascii);//ignore the first line as this is probably a header

    do {
        if (fgets(line, 1024, ascii)) {
            index1++;
            // Remove the comma's
            while ((comma = strchr(line, ',')) != NULL) * comma = ' ';

            // Read the next line
            sscanf(line, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   value + 1, value + 2, value + 3, value + 4, value + 5,
                   value + 6, value + 7, value + 8, value + 9, value + 10,
                   value + 11, value + 12, value + 13, value + 14, value + 15,
                   value + 16, value + 17, value + 18, value + 19, value + 20);

            // Copy the data to the laser point
            point.X() = value[6] ;
            point.Y() = value[7] ;
            point.Z() = value[8] ;
            point.SetDoubleAttribute(TimeTag, value[2]); // value [1] hard coded

            temp_laser_points.push_back(point);
        }
        //if (index1 == (index1 / 1000) * 1000) {
        //    temp_laser_points.RemoveAlmostDoublePoints(false, 0.01);
            printf(" %d / %d\r", laser_points.size(), index1);
            fflush(stdout);
            laser_points.AddPoints(temp_laser_points);
            temp_laser_points.ErasePoints();
        //}
        //    }
    } while (!feof(ascii));
    printf("\nRead %d points\n", laser_points.size());

    return laser_points;
}

/// read PCD_ascii conversion for supervoxels
bool read_pcd_ascii(const string &pcd_filename, LaserPoints &lp_out, char * lp_outfile)
{
    ifstream fs;
    fs.open (pcd_filename.c_str (), ios::binary);
    if (!fs.is_open () || fs.fail ())
    {
        printf("Could not open file '%s'! Error : %s\n", pcd_filename.c_str (), strerror (errno));
        fs.close ();
        return (false);
    }

    string line;
    vector<string> st;

    /// skip PCD headers
    for (int i=0; i<11; i++){
        getline (fs, line);
    }

    while (!fs.eof ()) {
        getline(fs, line);
        // Ignore empty lines
        if (line == "")
            continue;

        // Tokenize the line
        boost::trim (line);
        boost::split (st, line, boost::is_any_of ("\t\r , "), boost::token_compress_on);

        if(st.size() < 3)
            continue;

/*        /// NOTE: there is no check if there is color or not
        int r, g, b;
        r = atoi (st[3].c_str ());
        g = atoi (st[4].c_str ());
        b = atoi (st[5].c_str ());*/

        LaserPoint p;
        p.X() = float (atof (st[0].c_str ()));
        p.Y() = float (atof (st[1].c_str ()));
        p.Z() = float (atof (st[2].c_str ()));

        if (st[3].c_str ()){
            p.SegmentNumber() = atoi (st[3].c_str ());;
        }

        lp_out.push_back(p);
    }
    fs.close();

    /// debug
    lp_out.Write(lp_outfile, false);

    return(true);
}


/// conversion of customized ascii files e.g. Navvis trajectory
/// NOT complete
/*bool read_custom_ascii (const string &custom_ascii, LaserPoints &lp_out, char * lp_outfile){
    ifstream fs;
    fs.open (custom_ascii.c_str (), ios::binary);
    if (!fs.is_open () || fs.fail ())
    {
        printf("Could not open file '%s'! Error : %s\n", custom_ascii.c_str (), strerror (errno));
        fs.close ();
        return (false);
    }

    string line;
    vector<string> st;

    /// skip header(s)
    for (int i=0; i<1; i++){
        getline (fs, line);
    }

    while (!fs.eof ()) {
        getline(fs, line);
        // Ignore empty lines
        if (line == "")
            continue;

        // Tokenize the line
        boost::trim (line);
        boost::split (st, line, boost::is_any_of ("\t\r , "), boost::token_compress_on);

        if(st.size() < 3)
            continue;

        //// customize your st[i] based on the file
*//*        /// NOTE: there is no check if there is color or not
        int r, g, b;
        r = atoi (st[3].c_str ());
        g = atoi (st[4].c_str ());
        b = atoi (st[5].c_str ());*//*

        LaserPoint p;
        p.X() = float (atof (st[0].c_str ()));
        p.Y() = float (atof (st[1].c_str ()));
        p.Z() = float (atof (st[2].c_str ()));

        if (st[3].c_str ()){
            /// trim the time column of type long double
            long int t_tmp;
            std::string::size_type sz;
            t_tmp = strtod (st[3].c_str (), &sz); ???
            /// trim the int
            long long int ttrim;
            ttrim = t_tmp % 10000000000000;
            //t_tmp = atof(st[3].c_str ());
            double t;
            t = (double) ttrim;
            //t = clip((float) t_tmp,(float) 26073950994432.0,(float) 29235046924288.0);
            //t = t_tmp - 1460720000000000000.0;
            //p.SegmentNumber() = atoi (st[3].c_str ());;
            p.DoubleAttribute (TimeTag) = t;
        }

        lp_out.push_back(p);
    }
    fs.close();

    /// debug
    lp_out.Write(lp_outfile, false);

    /// debug
    /// print points to a ascii file for check
    FILE *output_points;
    output_points = fopen ("E:/Laser_data/publication_data/Navvis/Kadaster/trace_15_merged2_trimT.txt", "w");
    fprintf(output_points, "X, Y, Z, T \n");
    for( auto &p : lp_out){
        fprintf(output_points, "%.3f,%.3f,%.3f,%lf \n ",
                p.X (), p.Y(), p.Z (), p.DoubleAttribute (TimeTag));
    }

    fclose(output_points);

    return(true);

}*/


