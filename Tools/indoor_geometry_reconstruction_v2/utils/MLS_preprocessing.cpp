//
// Created by NikoohematS on 1-9-2017.
//

#include "utils.h"
#include <cstdlib>
#include <cstring>
#include "LaserPoints.h"
#include "boost/filesystem.hpp"
#include "boost/regex.hpp"
#include <boost/unordered_map.hpp>
#include <KNNFinder.h>
#include "Directory_Processing.h"
#include <boost/algorithm/string.hpp>
#include <iomanip>
#include <Vector3D.h>


bool compare_lp_segmenttag(const LaserPoints &lp1, const LaserPoints &lp2) {
    return lp1.begin()->Attribute(SegmentNumberTag) < lp2.begin()->Attribute(SegmentNumberTag);
}

bool compare_lp_labeltag(const LaserPoints &lp1, const LaserPoints &lp2) {
    return lp1.begin()->Attribute(LabelTag) < lp2.begin()->Attribute(LabelTag);
}

bool compare_lp_size(const LaserPoints &lp1, const LaserPoints &lp2) {
    return lp1.size() > lp2.size();  /// descending
}

/*bool compare_lp_label2tag(const LaserPoints &lp1, const LaserPoints &lp2) {
    return lp1.begin()->Attribute(Label2Tag) < lp2.begin()->Attribute(Label2Tag) ;
}*/

bool compare_p_time (const LaserPoint &p1, const LaserPoint &p2)
{
    return p1.DoubleAttribute(TimeTag)<p2.DoubleAttribute(TimeTag);
}

/// collect laserpoints per similar_tags and return a vector of them,
std::vector<LaserPoints> PartitionLpByTag(LaserPoints const& lp, LaserPointTag int_tag,
        char* output, bool write_output) {

    //printf("Partitioning points by tag... \n");
    /// if you don't have boost you can use std:unordered_multimap for newer compilers e.g. 2014
    typedef boost::unordered::unordered_multimap<int, LaserPoint> UnorderTaggedPoints;
    UnorderTaggedPoints umap;

    for (auto&& p : lp) {
        /// insert laser points and its key (tag number) as a pair to the multiumap
        umap.emplace(p.Attribute(int_tag), p);
    }

    //cout << "Umap size:" << umap.size() << endl; // debug

    std::vector<LaserPoints> result;
    int total_points=0;

    auto umap_begin = umap.begin();
    auto umap_end = umap.end();

    while (umap_begin != umap_end) {

        /// range is the iterator to pair of multimap objects
        auto range = umap.equal_range(umap_begin->first);

        LaserPoints tmp;
        tmp.reserve(std::distance(range.first, range.second));

        /// loop through the iterators of range and collect values (points with similar tag number)
        for (auto i = range.first; i != range.second; ++i) {
            //std::move(i->second);
            boost::move(i->second);
            tmp.push_back(i->second);
        }
        total_points += tmp.size();
        result.push_back(std::move(tmp));
        umap_begin = range.second;
    }

    /// if there is output directory, write the tag partitions to the disk
    if (write_output){
        printf("Write partitions based on <%s> to the disk ... \n", AttributeName(int_tag, true));
        for (auto it=result.begin(); it != result.end(); ++it){
            int tag_nr = (it->begin())->Attribute(int_tag);

            if ((*it).size() > 2){

                string tmp_file;
                std::stringstream sstm;
                sstm << output << "/" << tag_nr << ".laser";
                tmp_file = sstm.str();
                (*it).Write(tmp_file.c_str(), false);
                sstm.clear();
            }
        }
    }

    //cout << "Total number of partitioned points: " << total_points << endl;
    return result;
};

void LaserPoints_info (LaserPoints &laserpoints, int lp_size){
    printf("LaserPoints size: %d \n", laserpoints.size ());
    vector<int> segment_numbers;
    segment_numbers = laserpoints.AttributeValues(SegmentNumberTag);  // vector of segment numbers
    sort(segment_numbers.begin (), segment_numbers.end ());
    printf("First SegmentNumber: %d , Last SegmentNumber: %d \n", segment_numbers.front (), segment_numbers.back ());
    printf("Number of segments: %d \n", segment_numbers.size ());
    LaserPoints lp_large_segments;
    vector <LaserPoints> lp_vec;
    char * root= nullptr;
    lp_vec = PartitionLpByTag (laserpoints, SegmentNumberTag, root);
    int count=0;
    for (auto &s : lp_vec) {
        if (s.size () > lp_size) {
            count++;
            lp_large_segments.AddPoints (s);
        }
    }
    printf("#segments bigger than %d: %d \n", lp_size, count);
    //lp_large_segments.Write("E:/publication_data/temp/lp_out.laser", false);

}

/// fast for large files
int Trim_doubleTag (LaserPoints &laserpoints, LaserPointTag double_tag, double trim_value, char* ascii_out, bool add_value){

    FILE *trimmed_file;
    //char str_root[500];
    //strcpy (str_root,root);
    //trimmed_file = fopen(strcat(str_root, "Ttrimmed.txt"),"w");
    trimmed_file = fopen(ascii_out, "w");
    fprintf(trimmed_file, "// x, y, z, time \n");

    int n_points = 0;
   /// trim_value = 1444700000.0; /// zeb1_timetag
    for(auto &p : laserpoints){
        printf("points counter: %d \r", n_points);
        if (p.HasAttribute (double_tag)){
            double tag_value, trimmed_tag;
            tag_value = p.DoubleAttribute (double_tag);
            if(add_value){
                trimmed_tag = tag_value + trim_value;
            }else trimmed_tag = tag_value - trim_value;
            p.SetDoubleAttribute (double_tag, trimmed_tag);
            n_points++;
        }
        fprintf(trimmed_file ,"%.3f,%.3f,%.3f,%.4f \n", p.X (), p.Y (), p.Z (), p.DoubleAttribute (double_tag));
    }
    return  n_points;
}

/// slow for large files
int Trim_doubleTag (const string &ascii_infile, int trim_column, double trim_value, const string &ascii_outfile, bool add_value){
    std::ifstream fs;
    fs.open (ascii_infile.c_str (), std::ifstream::in);
    if (!fs.is_open () || fs.fail ())
    {
        printf("Could not open file '%s'! Error : %s\n", ascii_infile.c_str (), strerror (errno));
        fs.close ();
        return (false);
    }

    /// open output file
    std::ofstream fs_out;
    fs_out.open (ascii_outfile.c_str (), std::ofstream::out);
    if (!fs_out.is_open () || fs_out.fail ())
    {
        printf("Could not open file '%s'! Error : %s\n", ascii_outfile.c_str (), strerror (errno));
        fs_out.close ();
        return (false);
    }

    string line;
    vector<string> st;

    /// skip file headers
    for (int i=0; i<1; i++){
        getline (fs, line);
    }

    int n_points =0;
    while (!fs.eof ()) {
        n_points++;
        printf("points counter: %d \r", n_points);

        getline(fs, line);
        // Ignore empty lines
        if (line.empty ())
            continue;

        // Tokenize the line
        boost::trim (line);
        boost::split (st, line, boost::is_any_of ("\t\r , "), boost::token_compress_on);

        if(st.size() < 3)
            continue;

        /// read x, y,z
        double x, y, z;
        x = float (atof (st[0].c_str ()));
        y = float (atof (st[1].c_str ()));
        z = float (atof (st[2].c_str ()));

        /// read the time and trim it
        char *ptr;
        double time;
        if (st[trim_column].c_str ()){
            time =  strtod (st[trim_column].c_str (), &ptr);
        }


        double trimmed_time;
        if(add_value){
            trimmed_time = time + trim_value;
        } else trimmed_time = time - trim_value;



        /// write to the file
        fs_out << std::fixed << std::setprecision(3) << x << ",";
        fs_out << std::fixed << std::setprecision(3) << y << ",";
        fs_out << std::fixed << std::setprecision(3) << z << ",";
        /// trimmed time
        fs_out << std::fixed << std::setprecision(4) << trimmed_time << std::endl;


        /// this is for conversion to laser format
        /// NOTE: there is no check if there is color or not
/*        int r, g, b;
        r = atoi (st[3].c_str ());
        g = atoi (st[4].c_str ());
        b = atoi (st[5].c_str ());

        LaserPoint p;
        p.X() = float (atof (st[0].c_str ()));
        p.Y() = float (atof (st[1].c_str ()));
        p.Z() = float (atof (st[2].c_str ()));

        if (st[3].c_str ()){
            p.SegmentNumber() = atoi (st[3].c_str ());
        }

        lp_out.push_back(p);*/

    }
    fs.close();
    fs_out.close ();

    printf ("# of points %d \n", n_points);


    /// debug
    //lp_out.Write(lp_outfile, false);
    return n_points;
}


int add_to_doubleTag (LaserPoints &laserpoints, LaserPointTag double_tag, double add_value){

    int n_points = 0;
    /// add_value = 1444700000.0; /// zeb1_timetag
    for(auto &p : laserpoints){
        if (p.HasAttribute (double_tag)){
            double tag_value, added_tag;
            tag_value = p.DoubleAttribute (double_tag);
            added_tag = tag_value + add_value;
            p.SetDoubleAttribute (double_tag, added_tag);
            n_points++;
        }
    }
    return  n_points;
}

void Normal_Flip (LaserPoints &laserpoints, LaserPoints &scanner_positions, int knn){

    /// storing to the disk for test:
    char str_root[500];
    FILE *laser_points_file= nullptr;
    FILE *laser_points_NFlip_file= nullptr;
    /// open a file for traj points
    char* out_root;
    out_root = (char*) "D:/test/normal_estimation/";
    strcpy(str_root, out_root);
    laser_points_file = fopen(strcat(str_root, "laser_points_file.txt"), "w");
    fprintf(laser_points_file, "X, Y, Z, Nx, Ny, Nz \n");

    strcpy(str_root, out_root);
    laser_points_NFlip_file = fopen(strcat(str_root, "laser_points_NFlip_file.txt"), "w");
    fprintf(laser_points_NFlip_file, "X, Y, Z, Nx, Ny, Nz \n");

    /// calculate normals for the points
    if(!laserpoints.HasAttribute (NormalXTag)){
        printf("Calculating Normals...wait \n");
        laserpoints.CalculateNormals (knn);
    }

    /// make a vector of trajectory time tags
    vector<double> trajectory_timetags_v;
    for (auto &sp : scanner_positions) trajectory_timetags_v.push_back (sp.DoubleAttribute (TimeTag));

    /// checking the normal direction and scanner position
    printf("Flipping Normals to the direction of scanner... \n");
    for(auto &p: laserpoints){

        /// store points in a txt file before flipping normals
        fprintf(laser_points_file, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f \n",
                p.X (), p.Y(), p.Z (), p.Normal ().X (), p.Normal ().Y (), p.Normal ().Z ());

        /// get the scanner position for p
        std::vector<double>::iterator   lower_traj_it;
        lower_traj_it = std::lower_bound(trajectory_timetags_v.begin(), trajectory_timetags_v.end(), p.DoubleAttribute (TimeTag));
        int scanner_position_index;
        scanner_position_index = lower_traj_it - trajectory_timetags_v.begin() -1;

        LaserPoint scanner_position;
        scanner_position = scanner_positions[scanner_position_index];

        /// make a vector from scanner_position to the p
        Vector3D p_sp(scanner_position.X () - p.X (),
                      scanner_position.Y () - p.Y (),
                      scanner_position.Z () - p.Z ()) ;
        /// check if the normal of the point and the vector from the point
        /// to the scanner position are the same side of the plane
        if(p_sp.DotProduct (p.Normal ()) < 0.0) p.FlipNormal ();

        fprintf(laser_points_NFlip_file, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f \n",
                p.X (), p.Y(), p.Z (), p.Normal ().X (), p.Normal ().Y (), p.Normal ().Z ());
    }

    fclose (laser_points_file);
    fclose(laser_points_NFlip_file);

    /// we want to flip normals for each segment
    /// if the trajectory is on one side of the segment's plane we flip all normals to that side
/*    vector<LaserPoints> segments;
    segments = PartitionLpByTag (laserpoints, SegmentNumberTag);
    for (auto &segment : segments){

    }*/

}

LaserPoints LabelPoints_To_Ver_Horizon (LaserPoints &laserpoints, double angle_threshold){

    double   PI;
    PI       = 3.14159;

    laserpoints.SetAttribute (LabelTag, 0);

    /// partition laser points to the segments
    vector<LaserPoints> segments;
    char* root = nullptr;
    segments = PartitionLpByTag (laserpoints, SegmentNumberTag, root);

    LaserPoints labeled_vert_horizon;
    for(auto &segment : segments){

        int segment_nr = segment[0].Attribute (SegmentNumberTag);
        /// fit a plane to the points of this segment
        Plane plane;
        plane = segment.FitPlane (segment_nr);

        /// get the plane normal and check if it is horizontal or not
        /// label laserpoints with horizontal planes as 1
        if (plane.IsHorizontal(angle_threshold * PI / 180)){ // radian
            segment.ConditionalReTag(0, 1, LabelTag, segment_nr , SegmentNumberTag); // Horizontal 1
        }

        /// label laserpoints with vertical planes as 2
        if (plane.IsVertical(angle_threshold * PI / 180)) { //  radian
            segment.ConditionalReTag(0, 2, LabelTag, segment_nr, SegmentNumberTag); // Vertical 2
        }
        labeled_vert_horizon.AddPoints (segment);
    }
    return labeled_vert_horizon;
}

/// partition points by a give vector of Z_peaks (e.g. extracted from histogram) and a tolerance
void PartitionPoints_By_Zpeaks(LaserPoints &laserpoints, vector<double> Z_peaks, double z_tolerance){

    sort(Z_peaks.begin (), Z_peaks.end ());
    int first_level =0;
    int last_level = Z_peaks.size () -1; /// number of building levels
    /// we assume Z-peaks are two by two couples as floor and ceiling
    for(auto &p: laserpoints){
        if(p.Z () <= Z_peaks.front ()) p.SetAttribute (LabelTag, first_level); //first level if it's under the lowest Z
        if(p.Z () >= Z_peaks.back ()) p.SetAttribute (LabelTag, last_level); //last level if it's bigger than highest Z
        for(int i=0; i< Z_peaks.size (); i+=2){
            if(p.Z () > (Z_peaks[i] - z_tolerance) && p.Z () <= (Z_peaks[i+1] + z_tolerance)){
                p.SetAttribute (LabelTag, i);
            }
        }
    }
}

/// temporary function to remove not necessary points in backpack data
/// remove points with a specific tag's value or without tag
void FilterPointsByTag(LaserPoints &laserpoints, LaserPointTag tag, int tag_value, LaserPoints &lp_filtered){
    //LaserPoints lp_filtered;
    int cnt=0;
    for (auto &p : laserpoints){
        if(!p.HasAttribute (tag) || p.Attribute (tag) == tag_value){
            cnt++;
        }else{
            lp_filtered.push_back (p);
        }
    }
    printf("# filtered points: %d \n", cnt);
}

void Segmentation_SurfaceGrowing(char *input, char * output, double max_dis_to_plane, double max_dist_to_surf){
/*void Segmentation_SurfaceGrowing(LaserPoints lp_input, LaserPoints &lp_segmented,
                                 double max_dis_to_plane, double max_dist_to_surf){*/
    SegmentationParameters *seg_parameter;
    seg_parameter = new SegmentationParameters;
    /// segmenting laserpoints
    seg_parameter -> MaxDistanceInComponent()  = 0.3;
    seg_parameter -> SeedNeighbourhoodRadius() = 1.0;
    seg_parameter -> MaxDistanceSeedPlane()    = max_dis_to_plane; // MaX Distance to the Plane
    seg_parameter -> GrowingRadius()           = 1.0;
    seg_parameter -> MaxDistanceSurface()      = max_dist_to_surf;

    /// if segmentation crashes set the compatibility of generated exe to windows7
    printf("segmentation process... \n ");
    LaserPoints lp;
    lp.Read(input);
    lp.SurfaceGrowing(*seg_parameter);
    lp.Write (output, false);

}

/// For using in ChangeDetection: find the location of changes in the original pointclouds
/// with wall, floor, ceiling labels and relabel them
/// find the result of changedetection in the original pointclouds and relabel them to walls, floor and ceiling
LaserPoints relabel_changed_points (LaserPoints &detected_changes, LaserPoints & original_labeled_points,
        double dist_threshold){

    LaserPoints relabeled_changes;
    /// build a kdtree of original_points laserpoints
    /// make a KNN from the original_points
    KNNFinder <LaserPoint> finder(original_labeled_points);
    for (auto &p : detected_changes){
        double dist;
        int inx;
        finder.FindIndexAndDistance (p, inx, dist, 1, EPS_DEFAULT);
        if (dist <= dist_threshold){
            p.Attribute (LabelTag) = original_labeled_points[inx].Attribute (LabelTag);
        }
        relabeled_changes.push_back (p);
    }
    return relabeled_changes;
}

/// merge segmented point clouds from a directory into one file and resegment them
/*NOTE: lp_master should be in a separate directory of the rest of the files, otherwise it would be duplicated.
 * Lp_master cann't be empty.*/
void merge_lp_segmented (LaserPoints &lp_master, char *input_directory, char *output_file){

    std::vector<std::pair<string, string>> files_v;
    files_v = RecursiveDirFileNameAndPath (input_directory);

    //auto it = std::begin(files_v);  /// this is for the case that masterfile is inside the directory
    //LaserPoints lp_master;
    // printf ("Reading first file as master file... \n");
    //lp_master.Read (it->first.c_str ()); /// first file can be the lp_master

    vector<int> segment_nu_v;
    segment_nu_v = lp_master.AttributeValues (SegmentNumberTag);
    std::sort (segment_nu_v.begin(), segment_nu_v.end());
    int last_segment_nu;
    if(!segment_nu_v.empty ()){
        last_segment_nu = segment_nu_v.back ();
    } else {
        printf("laser points segment number is empty, abort ... \n");
        exit(EXIT_FAILURE);
    }

    //printf ("Reading remaining files... \n");
    printf ("Reading directory files... \n");
    /// start with the remaining of the files to resegment and add to the previous one
    //++it;  /// this is for the case that masterfile is inside the directory
    //for(auto end=std::end(files_v); it!=end; ++it) {
    for (auto it = std::begin (files_v); it != std::end(files_v); it++){
        string file_path;
        file_path = it->first; // fielpath
        LaserPoints lp_tmp;
        vector<LaserPoints> lp_segments_v;
        lp_tmp.Read (file_path.c_str ());
        lp_segments_v = PartitionLpByTag (lp_tmp, SegmentNumberTag, input_directory);
        for(auto &lp_seg : lp_segments_v){
            if(lp_seg.HasAttribute (SegmentNumberTag)){
                int new_segment_nu = ++last_segment_nu;
                //int old_segment_nu = lp_seg[0].Attribute (SegmentNumberTag);
                lp_seg.SetAttribute (SegmentNumberTag, new_segment_nu);
                //lp_seg.ReTag (old_segment_nu, new_segment_nu, SegmentNumberTag);
                lp_master.AddPoints (lp_seg);
            }
        }
    }

    printf ("Last segment number: %d \n", last_segment_nu);
    segment_nu_v = lp_master.AttributeValues (SegmentNumberTag);
    printf("Total number of segments: %d \n", segment_nu_v.size ());
    printf("Total number of points: %d \n", lp_master.size ());
    lp_master.Write(output_file, false);
}

/*   renumber segment numbers  from 1 to n
 * This function is similar to ReTag function */
LaserPoints Renumber_points_byTag (LaserPoints &laserPoints, LaserPointTag laserPointTag, int start_number){
    LaserPoints lp_renumbered;
    vector<LaserPoints> lp_Tag_vec;
    char * root = nullptr;
    lp_Tag_vec = PartitionLpByTag (laserPoints, laserPointTag, root);
    int tag_nr =start_number;
    for(auto &tag_points : lp_Tag_vec){
        tag_points.SetAttribute (laserPointTag, tag_nr++);
        lp_renumbered.AddPoints (tag_points);
    }
}

LaserPoints Enforce_verticality (LaserPoints &laserPoints, int a_random_segment_number){
    laserPoints.SetAttribute(SegmentNumberTag, a_random_segment_number); // if we dont do this FitPLane fails just by using a random int
    Plane plane;
    //plane.Initialise();
    plane = laserPoints.FitPlane(laserPoints[0].SegmentNumber());
    Vector3D normal = plane.Normal();
    //Vector3D vecZ = Vector3D(0, 0, 1);
    //double angle = Angle(normal, vecZ);
    //double product = normal.DotProduct(vecZ);
    Vector3D verticalPlane_normal(normal.X(), normal.Y(), 0.0);
    Vector3D verticalPlane_point = laserPoints.Mean();
    Plane verticalPlane(verticalPlane_point, verticalPlane_normal);
    /// project points to the new vertical plane for double check
    LaserPoints pointsProjToVertPlane;
    for (auto &p: laserPoints){
        pointsProjToVertPlane.push_back(verticalPlane.Project(p.Position3DRef()));
    }
    //pointsProjToVertPlane.Write("/home/shayan/Drive_D/data/test/pointsProjToVertPlane.laser", false);
    /// to compare with the previous plane
/*    LaserPoints pointsProjToPlane;
    for (auto &p : laserPoints){
        pointsProjToPlane.push_back(plane.Project(p.Position3DRef()));
    }
    pointsProjToPlane.Write("/home/shayan/Drive_D/data/test/pointsProjToPlane.laser", false);*/

    return pointsProjToVertPlane;
}
