//
// Created by NikoohematS on 24-7-2017.
//
#include <iostream>
#include <sstream>
#include <limits.h>
#include "AnnotationManipulator.h"
#include "LaserBlock.h"

/// function to export CLASSLABELS enum, labels and names
void ExportClasLabelNames(char* directory){

    /// Store filepath for unclassified files in a text file
    FILE *list_of_classes;
    char str_root[500];
    strcpy (str_root,directory); // initialize the str_root with root string
    list_of_classes = fopen(strcat(str_root, "/List_of_Classes.class"),"w");
    Annotator annot;

    for (int label=CLUnclassifiedInvalid; label != CLDirectoryPath; label++){
        ///
        string labelname;
        labelname = annot.ClassLabelName(label);
        fprintf(list_of_classes, "LabelName: %20s, Number: %d \n" , labelname.c_str(), label);
    }
    fclose(list_of_classes);
}

/// arguments for Annotator.cpp class is hardcoded inside this function
/// this function is more like a main function for Annotator.cpp class
/// *Note* ascii_extension should contain Dot in the extension name
void ImportAnnotatedAsciiToLaser(string dirname, string ascii_extension) {

    std::clock_t start;
    double duration;
    start = std::clock();

    ascii_extension = ".txt";
    //dirname = "D:/test/stanford_data/AREA1_testset2";
    //dirname = "E:/Laser_data/Stanford_annotated/Area_1";

    /// Store filepath for unclassified files in a text file
    /// convert root_dir string to char
    char *croot_dir = new char[dirname.length() + 1];
    strcpy(croot_dir, dirname.c_str());

    /// read ascii files and convert to laser in a batchfile
    int col_x=1, col_y=2, col_z=3;
    int col_r=4, col_g=5, col_b=6;
    char * rootname = (char*) "subunit_laserpoints_";

    Annotator annotator;

    int building_division_pos, building_name_pos, building_floor;
    building_division_pos =5;
    building_name_pos =4;
    building_floor =1;
    annotator.Ascii2LaserBatchWithParsingPath(rootname, croot_dir, 0, ".txt",
                                              building_division_pos, building_name_pos, building_floor,
                                              0, 1, 0, 10000000, col_x, col_y, col_z, col_r, col_g, col_b);

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    printf("Total processing time: %.3f m \n", duration/60);
}


LaserPoints MergeLabels (LaserPoints lp){

    LaserPoints::iterator point;
    for (point = lp.begin(); point != lp.end(); point++){
        if (point -> HasAttribute(LabelTag) && point -> Attribute(LabelTag)!=0){
            int object_label, division_label;
            object_label    = point -> Attribute(LabelTag);
            division_label  = point -> Attribute(Label2Tag);

            int object_class, division_class, building_floor;
            int obj_number, div_number;
            object_class    = object_label / 1000;       // 00,000      -->  object-class, obj-number
            obj_number      = object_label % 1000;
            division_class  = (division_label / 1000) % 100;  // 00,00,000    --> floor, division-class, div-number
            div_number      = division_label % 1000;
            building_floor  = division_label / 100000;

            /// between 20 -50 are furniture, we change them to clutter
            if ( 20 < object_class &&  object_class < 50) {
                int new_label;
                /// new object label
                new_label = ClassLabel(CLClutter) * 1000 + obj_number;
                point -> SetAttribute(LabelTag, new_label);
            }

            /// Change CLColumn to Wall
            if (object_class == CLBeam) {
                int new_label;
                /// new object label
                new_label = ClassLabel(CLCeiling) * 1000 + obj_number;
                point -> SetAttribute(LabelTag, new_label);
            }

            /// Change CLBeam to Ceiling
            if (object_class == CLColumn) {
                int new_label;
                /// new object label
                new_label = ClassLabel(CLWall) * 1000 + obj_number;
                point -> SetAttribute(LabelTag, new_label);
            }

            /// change OpenSpace to Corridor
            if (division_class == CLCorridor || division_class == CLOpenspace){
                // new division label
                int new_div_label;
                new_div_label = building_floor * 100000 + ClassLabel(CLCorridor) * 1000 + div_number;
                point -> SetAttribute(Label2Tag, new_div_label);

            } else {
                /// Change all division labels to ROOM
                // new division label
                int new_div_label;
                new_div_label = building_floor * 100000 + ClassLabel(CLRoom) * 1000 + div_number;
                point -> SetAttribute(Label2Tag, new_div_label);
            }
        }
    }
    return lp;
}

/// This function read many laser files and apply a function on them e.g. Mergelabels
/// NOTE: if input and output are the same laserfiles would be overwritten
void LaserPointsManipulationBatch(string directory, string ouput_directory){
    /// extract a vector of filespath
    vector<string> filespath;
    Annotator annotator;
    filespath = annotator.RecursiveDir(directory);

    // collect just laser_files in a vector
    vector <string> lasers_filespath;
    vector <string>::iterator laserfile_it;
    for (laserfile_it = filespath.begin(); laserfile_it != filespath.end(); laserfile_it++){
        string extension;
        extension = boost::filesystem::extension(*laserfile_it);
        if (extension == ".laser"){
            lasers_filespath.push_back(*laserfile_it);
        }
    }

    vector<string>::iterator filepath_it;

    for (filepath_it = lasers_filespath.begin(); filepath_it != lasers_filespath.end(); filepath_it++){
        LaserPoints lpoints;
        /// convert filepath string to char
        char *cfilepath = new char[filepath_it -> length() + 1];
        strcpy(cfilepath, filepath_it -> c_str());
        lpoints.Read(cfilepath);

        /// relabel points and merge labels
        LaserPoints modified_points;
        modified_points = MergeLabels(lpoints);

        /// extract filename with extension from the path
        boost::filesystem::path path(*filepath_it);  // ***.laser
        /// write the file in the disk again
        string relabeled_file;
        std::stringstream sstm;
        sstm << ouput_directory << "/" <<  path.filename().string();
        relabeled_file = sstm.str();;
        modified_points.Write(relabeled_file.c_str(), false);
        sstm.clear();
    }
}

void Laser2AsciiBatch(string directory,
                      int store_x, int store_y, int store_z,
                      string out_put_directory,
                      int store_r, int store_g, int store_b,
                      int store_l, int store_l2, int store_fs,
                      int store_p, int store_lpf, int store_pl,
                      int store_pn, int store_sn, int store_lsn,
                      int store_poln, int store_scan, int store_t,
                      int store_a, int store_cnt, int store_tt,
                      int store_xt, int store_yt, int store_zt)
{

    vector<string> filespath;
    Annotator annotator;
    filespath = annotator.RecursiveDir(directory);

    /// convert directory string to char
    char *c_directory = new char[directory.length() + 1];
    strcpy(c_directory, directory.c_str());

    /// convert output directory string to char
    char *c_output_directory = new char[out_put_directory.length() + 1];
    strcpy(c_output_directory, out_put_directory.c_str());

    // collect just laser_files in a vector
    vector <string> lasers_filespath;
    vector <string>::iterator laserfile_it;
    for (laserfile_it = filespath.begin(); laserfile_it != filespath.end(); laserfile_it++){
        string extension;
        extension = boost::filesystem::extension(*laserfile_it);
        if (extension == ".laser"){
            lasers_filespath.push_back(*laserfile_it);
        }
    }

    vector<string>::iterator filepath_it;
    int filecounter=0;
    int total_points=0;

    for (filepath_it = lasers_filespath.begin(); filepath_it != lasers_filespath.end(); filepath_it++) {
        LaserPoints lpoints;
        //cout << "laser file path:" << *filepath_it << endl;

        /* Open the output file */
        filecounter++;
        /// convert filepath string to char
        char *claser_filepath = new char[filepath_it -> length() + 1];
        strcpy(claser_filepath, filepath_it -> c_str());

        string filename, dir; // e.g. Table_1.txt includes extension
        string filename_without_extension;
        filename = boost::filesystem::path(claser_filepath).filename().string();
        dir      = boost::filesystem::path(claser_filepath).parent_path().string();
        auto pos = filename.find_last_of(".");
        if (pos != std::string::npos) {
            filename_without_extension = filename.substr(0, pos); // e.g. Table_1
        }else{
            filename_without_extension = filename;
        }

        //cout << "dir: " << dir << endl;
        //cout << "filename_without_extension: " << filename_without_extension << endl;

        FILE *ascii;
        char *ctmp;
        if (!out_put_directory.empty()){
            strcpy(c_output_directory, out_put_directory.c_str());
            ctmp = strcat(strcat(c_output_directory, "//"), filename_without_extension.c_str());
        } else{
            /// convert directory string to char
            char *c_dir = new char[dir.length() + 1];
            strcpy(c_dir, dir.c_str());
            ctmp = strcat(strcat(c_dir, "//"), filename_without_extension.c_str());
        }

        //cout << "ctm: " << ctmp << endl;
        char *ascii_out;
        ascii_out = strcat(ctmp, ".txt");
        //cout << "ascii_out: " << ascii_out << endl;
        ascii = fopen(ascii_out, "w");

        if (!ascii) {
            fprintf(stderr, "Error opening output file %s\n", claser_filepath);
            exit(0);
        }

        /* write header in the ascii file */
        if (store_x)    fprintf(ascii, "%s",   "X");
        if (store_y)    fprintf(ascii, ", %s", "Y");
        if (store_z)    fprintf(ascii, ", %s", "Z");
        if (store_g) {
            if (store_r)    fprintf(ascii, ", %s", "Red");
            if (store_g)    fprintf(ascii, ", %s", "Green");
            if (store_b)    fprintf(ascii, ", %s", "Blue");
        }
        else if (store_r)   fprintf(ascii, ", %s", "Reflectance");
        if (store_p)        fprintf(ascii, ", %s", "Pulse Count");
        if (store_lpf)      fprintf(ascii, ", %s", "Last Pulse Count");
        if (store_l)        fprintf(ascii, ", %s", "Label");
        if (store_l2)       fprintf(ascii, ", %s", "Label2");
        if (store_pl)       fprintf(ascii, ", %s", "PulseLength");
        if (store_pn)       fprintf(ascii, ", %s", "PlaneNumber");
        if (store_sn)       fprintf(ascii, ", %s", "SegmentNumber");
        if (store_lsn)      fprintf(ascii, ", %s", "LongSegmentNumber");
        if (store_poln)     fprintf(ascii, ", %s", "PolygonNumber");
        if (store_fs)       fprintf(ascii, ", %s", "Filtered");
        if (store_scan)     fprintf(ascii, ", %s", "ScanNumber");
        if (store_t)        fprintf(ascii, ", %s", "TimeTag");
        if (store_a)        fprintf(ascii, ", %s", "AngleTag");
        if (store_cnt)      fprintf(ascii, ", %s", "ComponentNumberTag");
        if (store_tt)       fprintf(ascii, ", %s", "AverageTimeTag");
        if (store_xt)       fprintf(ascii, ", %s", "XCoordinateTag");
        if (store_yt)       fprintf(ascii, ", %s", "YCoordinateTag");
        if (store_zt)       fprintf(ascii, ", %s", "ZCoordinateTag");
        fprintf(ascii, "\n");
        /* end of writing headers */



        /* Set up a laser block */
        LaserBlock             block;
        LaserBlock::iterator   unitptr;
        LaserUnit::iterator    subunitptr;
        LaserSubUnit::const_iterator point;
        int                    icon, fileclass, colour;


        //cout << "claser_filepath2: " << claser_filepath << endl;
        block.Initialise();
        if (!block.Create(claser_filepath, &fileclass)) {
            fprintf(stderr, "Error reading meta data file %s\n", claser_filepath);
            exit(0);
        }

/* Loop over all strips */
        //cout << "block size: " << block.size() << endl; //debug
        for (unitptr=block.begin(); unitptr!=block.end(); unitptr++) {
            //cout << "block size: " << block.size() << endl; //debug

/* Loop over all strip parts */

            for (subunitptr=unitptr->begin();
                 subunitptr!=unitptr->end();
                 subunitptr++) {

/* Read the point data */

                if (!subunitptr->Read()) {
                    fprintf(stderr, "Error reading laser points from file %s\n",
                            subunitptr->PointFile());
                    exit(0);
                }

// Check if we have colour

                colour = (subunitptr->Scanner().PointType() == ColourPoint ||
                          subunitptr->Scanner().PointType() == MultiColourPoint) ||
                         (store_r && store_g && store_b);

/* Write the point data to the ASCII file */

                for (point=subunitptr->begin(); point!=subunitptr->end(); point++) {
                    if (store_x) fprintf(ascii, "%11.3f", point->X());
                    if (store_y) fprintf(ascii, ", %11.3f", point->Y());
                    if (store_z) fprintf(ascii, ", %7.3f", point->Z());
                    if (colour) {
                        if (store_r) fprintf(ascii, ", %3d", point->Red());
                        if (store_g) fprintf(ascii, ", %3d", point->Green());
                        if (store_b) fprintf(ascii, ", %3d", point->Blue());
                    }
                    else if (store_r) fprintf(ascii, ", %3d", point->Reflectance());
                    if (store_p) fprintf(ascii, ", %3d", point->PulseCount());
                    if (store_lpf) {
                        if (point->IsLastPulse()) fprintf(ascii, " 1");
                        else fprintf(ascii, " 0");
                    }
                    if (store_l) fprintf(ascii, ", %6d", point->Label());
                    if (store_l) fprintf(ascii, ", %8d", point->Attribute(Label2Tag));
                    if (store_pl) fprintf(ascii, ", %3d", point->PulseLength());
                    if (store_pn) fprintf(ascii, ", %5d", point->PlaneNumber());
                    if (store_sn) fprintf(ascii, ", %5d", point->SegmentNumber());
                    if (store_lsn) fprintf(ascii, ", %15d", point->LongSegmentNumber());
                    if (store_poln) fprintf(ascii, ", %5d", point->PolygonNumber());
                    if (store_fs) fprintf(ascii, ", %5d", point->Filtered());
                    if (store_scan) fprintf(ascii, ", %5d", point->ScanNumber());
                    if (store_t) fprintf(ascii, ", %.6f", point->DoubleAttribute(TimeTag));
                    if (store_a) fprintf(ascii, ", %.2f", point->FloatAttribute(AngleTag));
                    if (store_cnt) fprintf(ascii, ", %7d", point->Attribute(ComponentNumberTag));
                    if (store_tt) fprintf(ascii, ", %.6f", point->DoubleAttribute(AverageTimeTag));
                    if (store_xt) fprintf(ascii, ", %.3f", point->DoubleAttribute(XCoordinateTag));
                    if (store_yt) fprintf(ascii, ", %.3f", point->DoubleAttribute(YCoordinateTag));
                    if (store_zt) fprintf(ascii, ", %.3f", point->DoubleAttribute(ZCoordinateTag));
                    fprintf(ascii, "\n");
                }

/* Delete the points */
                total_points += subunitptr -> size();
                printf(" %d points was written to file %s\n", subunitptr -> size(), ascii_out);
                subunitptr->ErasePoints();
            }
        }
        fclose(ascii);
    }
    printf("\n %d files were processed.\n", filecounter);
    printf ("Total number of points: %d\n", total_points);
}



