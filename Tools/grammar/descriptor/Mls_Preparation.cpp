//
// Created by NikoohematS on 25-7-2017.
//
#include <iostream>
#include <string>
#include "LaserPoints.h"
#include "boost/filesystem.hpp"
#include "boost/regex.hpp"
#include <unordered_map>
#include <boost/unordered_map.hpp>
#include "Annotator.h"
#include "Mls_Preparation.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/named_function_params.hpp>
#include <boost/graph/labeled_graph.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/algorithm/string.hpp>


LaserPoints segment_refinement(LaserPoints , int , double, int m=1);

bool comparison(const LaserPoint &p1, const LaserPoint &p2) {
    return p1.Attribute(SegmentNumberTag)<p2.Attribute(SegmentNumberTag);
}

bool compare_lp_segmenttag(const LaserPoints &lp1, const LaserPoints &lp2) {
    return lp1.begin()->Attribute(SegmentNumberTag) < lp2.begin()->Attribute(SegmentNumberTag);
}

bool compare_lp_labeltag(const LaserPoints &lp1, const LaserPoints &lp2) {
    return lp1.begin()->Attribute(LabelTag) < lp2.begin()->Attribute(LabelTag);
}

bool compare_lp_size(const LaserPoints &lp1, const LaserPoints &lp2) {
    return lp1.size() > lp2.size();  /// descending
}

bool compare_lp_label2tag(const LaserPoints &lp1, const LaserPoints &lp2) {
    return lp1.begin()->Attribute(Label2Tag) < lp2.begin()->Attribute(Label2Tag) ;
}

vector<string> RecursiveFilesPath (char *dirname, string filter_extension){
    vector<string> files_path;
    for ( boost::filesystem::recursive_directory_iterator end, dir(dirname);
          dir != end; ++dir ) {
        //std::cout << *dir << "\n";  // full path
        //std::cout << dir->path().filename() << "\n"; // just last bit
        if (!boost::filesystem::is_directory(*dir)){ // skip directory paths
            files_path.push_back(dir ->path().string());
        }
    }

    // collect just filtered files in a vector
    vector <string> filtered_filespath;
    vector <string>::iterator file_it;
    for (file_it = files_path.begin(); file_it != files_path.end(); file_it++){
        string extension;
        extension = boost::filesystem::extension(*file_it);
        if (extension == filter_extension){
            filtered_filespath.push_back(*file_it);
        }
    }
    return filtered_filespath;
}

void ReducePointsBatch(char *dirname, char *outputdir, double reduction_factor){

    /// extract laser filespath recursively
    vector <string> lasers_filespath;
    lasers_filespath = RecursiveFilesPath(dirname, ".laser");

    int lpoints_count_before=0, lpoints_count_after=0;
    vector<string>::iterator filepath_it;
    /// loop through laser files and reduce points and save to the disk
    for (filepath_it = lasers_filespath.begin(); filepath_it != lasers_filespath.end(); filepath_it++) {
        LaserPoints lpoints;

        lpoints.Read(filepath_it->c_str());
        lpoints_count_before += lpoints.size();

        /// *** Main function ************
        /// reduce points
        //MLS_preprocessing(lpoints, 0.05);   /// it has a bug, it crashes

        /// reduction using knn
        lpoints.ReduceData(2, 10, reduction_factor); // decrease points to 1/reduction_factor th using knn
        ///*******************************

        lpoints_count_after += lpoints.size();

        /// extract filename with its extension from the path
        boost::filesystem::path path(*filepath_it);  // ***.laser
        /// write the file in the disk again
        string reduced_file;
        std::stringstream sstm;
        sstm << outputdir << "/" <<  path.filename().string();
        reduced_file = sstm.str();
        lpoints.Write(reduced_file.c_str(), false);
        sstm.clear();
    }
    printf ("Number of points Before: %d, After: %d \n", lpoints_count_before, lpoints_count_after);
}

void MergeLaserFiles(char *dirname, char *outputdir){
    /// extract laser filespath recursively
    vector <string> lasers_filespath;
    lasers_filespath = RecursiveFilesPath(dirname, ".laser");

    LaserPoints modified_lpoints;
    vector<string>::iterator filepath_it;
    /// loop through laser files and add them together
    for (filepath_it = lasers_filespath.begin(); filepath_it != lasers_filespath.end(); filepath_it++) {
        LaserPoints lpoints;

        lpoints.Read(filepath_it->c_str());
        modified_lpoints.AddPoints(lpoints);
    }

    /// write the file in the disk
    string ouput_file;
    std::stringstream sstm;
    sstm << outputdir << "/all_lpoints.laser" ;
    ouput_file = sstm.str();
    modified_lpoints.Write(ouput_file.c_str(), false);
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

bool read_pcd_bin(const string &pcd_filename, LaserPoints &lp_out, char * lp_outfile)
{

    std::ifstream fs( pcd_filename.c_str(), std::ios::binary );

    if (!fs.is_open () || fs.fail ())
    {
        printf("Could not open file '%s'! Error : %s\n", pcd_filename.c_str (), strerror (errno));
        fs.close ();
        return (false);
    }
    // copies all data into buffer
    std::vector<char> buffer((std::istreambuf_iterator<char>(fs)),
                             (std::istreambuf_iterator<char>()));

    //TODO: parse the buffer to variables: x, y, z, rgb, L1, L2, ...
    /// tokenize the buffer of char, maybe using boost::tokenizer
/*    int test;
    inStream.read(reinterpret_cast<char*>(&test), sizeof(int));

    //Once again, we can create a function template to have a little better code :
    template<typename T>
    std::istream & binary_read(std::istream& stream, T& value){
        return stream.read(reinterpret_cast<char*>(&value), sizeof(T));
    }

    //and you can use it the same way :
    int test;
    binary_read(stream, test);*/

    /// then write to the laser format

    return (true);
}

/// read two laser files, add segmentnumbers from one to the origin
//// size of two laser points and the order should be the same
LaserPoints merge_attributes(LaserPoints &lp_origin, LaserPoints &lp_merged, const LaserPointTag &tag)
{
    /// check if the size of two laser points are the same
    if(lp_origin.size() != lp_merged.size()){
        printf("size of origin %d and lp_merged %d are not the same, Error! \n",
               lp_origin.size(), lp_merged.size() ,strerror(errno));
        return (1);
    }

    for (int i=0; i < lp_origin.size(); i++){
        LaserPoint p1, p2;
        p1 = lp_origin[i] , p2 = lp_merged[i];
/*        if(!((fabs(p1.X() - p2.X()) < 0.000001) ||
                (fabs(p1.Y() - p2.Y()) < 0.000001) ||
                (fabs(p1.Z() - p2.Z()) < 0.000001))) {*/
        // Compare coordinates
/*        if (p1.X() != p2.X() || p1.Y() != p2.Y() || p1.Z() != p2.Z()){
            printf ("WARNING! ... point %d is not equal! \n", i);
            printf ("(%f %f %f ) , (%f %f %f) \n", p1.X(), p1.Y(), p1.Z(), p2.X(), p2.Y(), p2.Z());
        }*/
        lp_origin[i].Attribute(tag) = lp_merged[i].Attribute(tag);
    }
    printf("size of origin: %d and lp_merged: %d  \n", lp_origin.size(), lp_merged.size() );

    return lp_origin;

}

/// segment laser points and reshape and merge segments with awkward TIN
void SegmentationAndRefinement(LaserPoints laserpoints, char* output_dir, int minsegmentsize,
                        double maxdistanceInComponent, double maxdistseedPlane,
                               bool do_segmentation, bool do_segment_refinement){

    char str_root[500];

    SegmentationParameters *seg_parameter;
    seg_parameter = new SegmentationParameters;
    /// segmenting laserpoints
    if (do_segmentation){
        seg_parameter -> MaxDistanceInComponent()  = maxdistanceInComponent;
        seg_parameter -> SeedNeighbourhoodRadius() = 1.0;
        seg_parameter -> MaxDistanceSeedPlane()    = maxdistseedPlane; // 0.05;
        seg_parameter -> GrowingRadius()           = 1.0;
        seg_parameter -> MaxDistanceSurface()      = maxdistseedPlane; //0.05;

        /// if segmentation crashes set the compatibility of generated exe to windows7
        printf("segmentation process... \n ");
        laserpoints.SurfaceGrowing(*seg_parameter);

        strcpy (str_root,output_dir);
        laserpoints.Write(strcat(str_root,"segmented.laser"), false);
    }

    if (do_segment_refinement){
        printf("Processing segments larger than %d points.\n", minsegmentsize);
        /// resegmenting laserpoints by removing longedges in the TIN and removing deformed segments
        LaserPoints refined_lp;
        refined_lp = segment_refinement(laserpoints, minsegmentsize, maxdistanceInComponent, 1);
        /// write the result
        strcpy (str_root,output_dir);
        refined_lp.Write(strcat(str_root,"segment_refined.laser"), false);

        /// renew laserpoints with the result of segment_refinement
        laserpoints = refined_lp;
    }

}

/// collect laserpoints per segment and return a vector of them,
/// computationally expensive: f(n) + f(nlogn)
vector <LaserPoints> CollectSegmentLaserpoints(LaserPoints &segmented_laserpoints,
                                               char* ouput_directory, bool verbose){

    if(verbose){
        vector<int>   segment_numbers;
        segment_numbers = segmented_laserpoints.AttributeValues(SegmentNumberTag);  // vector of segment numbers
        std::sort(segment_numbers.begin(), segment_numbers.end());
        printf ("Number of segments: %d\n", segment_numbers.size());
        if (segment_numbers.size() == 0) {
            printf("There is no segment number, exit... \n");
            exit(0);
        }
    }

    /// sort laserpoints based on segment number
    printf ("Sorting points by segment number... \n");
    std::sort(segmented_laserpoints.begin(), segmented_laserpoints.end(), comparison);

    vector <LaserPoints> segments_lp;
    map<LaserPoint, int> segmented_points;
    pair<LaserPoint, int> pair_segmentedpoint;
    int previous_segnr=0, segment_nr=0;
    int total_points=0;
    LaserPoints segment_lp_tmp;
    bool similar_seg=false;
    printf("Collecting laser points per segment... \n");
    for (int i=1; i < segmented_laserpoints.size(); i++){
        if (segmented_laserpoints[i].HasAttribute(SegmentNumberTag)){
            segment_nr = segmented_laserpoints[i].Attribute(SegmentNumberTag);
            //segmented_points.insert(std::pair<LaserPoint, int>(segmented_laserpoints[i], segment_nr));
            if (segment_nr == previous_segnr){
                segment_lp_tmp.push_back(segmented_laserpoints[i]);
                previous_segnr = segment_nr;

                /// for the last segment
                if (i == segmented_laserpoints.size()-1){
                    if (segment_lp_tmp.size() > 2){
                        segments_lp.push_back(segment_lp_tmp);
                        total_points +=segment_lp_tmp.size();
                    }

                    if(verbose && segment_lp_tmp.size() > 2){ /// write segments to the disk
                        string tmp_file;
                        std::stringstream sstm;
                        sstm << ouput_directory << "/" << previous_segnr << ".laser";
                        tmp_file = sstm.str();;
                        segment_lp_tmp.Write(tmp_file.c_str(), false);
                        sstm.clear();
                    }
                    segment_lp_tmp.ErasePoints();
                }
            } else{
                if (segment_lp_tmp.size() > 2){
                    segments_lp.push_back(segment_lp_tmp);
                    total_points +=segment_lp_tmp.size();
                }

                if(verbose && segment_lp_tmp.size() > 2){ /// write segments to the disk
                    string tmp_file;
                    std::stringstream sstm;
                    sstm << ouput_directory << "/" << previous_segnr << ".laser";
                    tmp_file = sstm.str();;
                    segment_lp_tmp.Write(tmp_file.c_str(), false);
                    sstm.clear();
                }

                segment_lp_tmp.ErasePoints();
                segment_lp_tmp.push_back(segmented_laserpoints[i]);
                previous_segnr = segment_nr;
            }
        }
    }


    printf("Total number of Points: %d \n", total_points);
    return segments_lp;
}

/// collect laserpoints per segment and return a vector of them,
std::vector<LaserPoints> LpToSegmentLpUsingBoost(LaserPoints const& lp, char* output) {

    printf("Partitioning points per segment... \n");
    /// if you don't have boost you can use std:unordered_multimap for newer compilers e.g. 2014
    typedef boost::unordered::unordered_multimap<int, LaserPoint> UnorderSegmentPoints;
    UnorderSegmentPoints umap;

    for (auto&& p : lp) {
        /// insert laser points and its key (segment number) as a pair to the multiumap
        umap.emplace(p.Attribute(SegmentNumberTag), p);
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

        /// loop through the iterators of range and collect points with similar segment number
        for (auto i = range.first; i != range.second; ++i) {
            //std::move(i->second);
            boost::move(i->second);
            tmp.push_back(i->second);
        }
        result.push_back(std::move(tmp));
        total_points += tmp.size();
        umap_begin = range.second;
    }

    /// if there is output directory, write the segments to the disk
    if (output){
        printf("Write partitions <segments> to the disk ... \n");
        for (auto it=result.begin(); it != result.end(); ++it){
            int seg_nr = (it->begin())->Attribute(SegmentNumberTag);

            if ((*it).size() > 2){

                string tmp_file;
                std::stringstream sstm;
                sstm << output << "/" << seg_nr << ".laser";
                tmp_file = sstm.str();
                (*it).Write(tmp_file.c_str(), false);
                sstm.clear();
            }
        }
    }

    cout << "Total number of partitioned points: " << total_points << endl;
    return result;
};

/// NOTE: This function has a bug, it doesn't save the segment number in the final results ***
/// This is similar to CollectSegmentsLp, computationally faster, it needs sorting: f(nlogn)
// pass by value - we need a copy anyway and we might get copy elision
vector <LaserPoints> PartitionBySegmentNumber(LaserPoints lp, char * ouput_directory){

    /// sort LaserPoints  based on segment number f(nlogn)
    printf ("Sorting points by segment number... \n");
    std::sort(lp.begin(), lp.end(), comparison);

    vector <LaserPoints> result;

    /*vector <LaserPoint>::iterator*/ auto first = lp.begin();
    /*vector <LaserPoint>::iterator*/ auto last = lp.end();
    int total_points=0;
    printf("Collecting laser points per segment... \n");
    /// because the range is sorted, we can partition it in linear time. f(n)
    while (first != last)
    {
        auto next = std::find_if(first, last, [&first](LaserPoint const& x)
        { return x.Attribute(SegmentNumberTag) != first->Attribute(SegmentNumberTag); });

        // let's move the items - that should speed things up a little
        // this is safe because we took a copy
        result.push_back(LaserPoints(std::make_move_iterator(first), std::make_move_iterator(next)));
        total_points += LaserPoints(std::make_move_iterator(first), std::make_move_iterator(next)).size();
        first = next;
    }

    if(ouput_directory){ /// write segments to the disk
        int seg_nr=0;
        for (vector<LaserPoints>::iterator lp_it=result.begin(); lp_it != result.end(); lp_it++){
            if (lp_it -> size() > 2){
                seg_nr++;
                //total_points += lp_it -> size();
                LaserPoint p;
                p = (*lp_it)[0];
                int segment_nr = p.Attribute(SegmentNumberTag);
                string tmp_file;
                std::stringstream sstm;
                sstm << ouput_directory << "/" << seg_nr << ".laser";
                tmp_file = sstm.str();
                (*lp_it).Write(tmp_file.c_str(), false);
                sstm.clear();
            }
        }
    }
    printf("# of points: %d \n", total_points);
    return result;
}



/// collect laserpoints per similar_tags and return a vector of them,
std::vector<LaserPoints> PartitionLpByTag(LaserPoints const& lp, LaserPointTag int_tag, char* output) {

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

        /// loop through the iterators of range and collect points with similar tag number
        for (auto i = range.first; i != range.second; ++i) {
            //std::move(i->second);
            boost::move(i->second);
            tmp.push_back(i->second);
        }
        result.push_back(std::move(tmp));
        total_points += tmp.size();
        umap_begin = range.second;
    }

    /// if there is output directory, write the tag partitions to the disk
    if (output){
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

/// Partition laserpoints first by tag1 and then by tag2
//TODO improve the algorithm by adding a pair<tag,tag2> to the multimap and then partition the multimap
vector <LaserPoints> PartitionLpByTag1Tag2(LaserPoints const& lp, LaserPointTag int_tag1,
                                           LaserPointTag int_tag2, char* output){
    int total_points=0;

    /// first make a vector collection of laserpoints based on tag1
    vector<LaserPoints> partition_byTtag1;
    partition_byTtag1 = PartitionLpByTag(lp, int_tag1, 0);

    /// for each partition of tag1 make a vector collection based on tag2
    std::vector<LaserPoints> result;
    for(auto partition1 : partition_byTtag1){

        vector<LaserPoints> partition_byTtag2;
        partition_byTtag2 = PartitionLpByTag(partition1, int_tag2, 0);
        result.insert(result.end(), partition_byTtag2.begin(), partition_byTtag2.end());
    }

    /// if there is output directory, write the tag partitions to the disk
    if (output){
        printf("Write partitions based on <%s and %s> to the disk ... \n",
               AttributeName(int_tag1, true), AttributeName(int_tag2, true));
        for (auto it=result.begin(); it != result.end(); ++it){

            /// generate partition name by tag1 and tag2
            string tag1_tag2;
            std::stringstream sstream;
            sstream << (*it)[0].Attribute(LabelTag) << "_" << (*it)[0].Attribute(Label2Tag);
            tag1_tag2 = sstream.str();

            if ((*it).size() > 2){
                total_points += (*it).size();
                string tmp_file;
                std::stringstream sstm;
                sstm << output << "/" << tag1_tag2 << ".laser";
                tmp_file = sstm.str();
                (*it).Write(tmp_file.c_str(), false);
                sstm.clear();
            }
        }
        cout << "Total number of written points: " << total_points << endl;
    }
    return result;
}


/// gives a report of each segment and its labels that represents objects
void SegmentToObject(LaserPoints &lp, char* output, bool out_segments_and_objects,
                     bool sort_output, bool verbose){

    /// record system time
    std::clock_t start;
    double duration;
    start = std::clock();

    vector<LaserPoints> vec_segments;
    char* segments_out;
    //vec_segments = CollectSegmentLaserpoints(lp, output, verbose); // too expensive
    segments_out = out_segments_and_objects ? output : 0;
    vec_segments = LpToSegmentLpUsingBoost(lp, segments_out); // when output is zero nothing will be written to the disk

    /// we sort vec_segments to have a more readable output, otherwise it's not necessary
    if(sort_output) sort(vec_segments.begin(), vec_segments.end(), compare_lp_segmenttag);

    /// time record
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    printf("Total segments partitioning time: %.3f m \n", duration/60);  //debug

    /// Store segment details based on its objects
    FILE *segments_report;
    char str_root[500];
    strcpy (str_root,output); // initialize the str_root with root string
    segments_report = fopen(strcat(str_root, "/segments_report.txt"),"w");
    fprintf(segments_report, "Segment No, Object (Percentage), Dominant Division (Percentage) \n");

    LaserPoints relabeledpoints;
    vector <pair<int, double>> vec_seg_confidence;
    for (auto segment = vec_segments.begin(); segment != vec_segments.end(); segment++){

        int seg_nr = (*segment)[0].Attribute(SegmentNumberTag);
        if (seg_nr < 0) seg_nr = -1;
        fprintf(segments_report, "%d \n", seg_nr);

        /// make a vector of laserpoints with similar labels in a segment
        vector <LaserPoints> vec_labelpartitions;
        vec_labelpartitions = PartitionLpByTag(*segment, LabelTag, 0);
        sort(vec_labelpartitions.begin(), vec_labelpartitions.end(), compare_lp_size); /// sort descending

        //// *** We relabel points in a segment based on the majority class of a label and
        //// majority class of a division
        int majority_division, majority_object;
        int majority_obj_count, majority_div_count;
        majority_object     = segment->MostFrequentAttributeValue(LabelTag, majority_obj_count);
        majority_division   = segment->MostFrequentAttributeValue(Label2Tag, majority_div_count);
        segment->SetAttribute(LabelTag, majority_object);
        segment->SetAttribute(Label2Tag, majority_division);
        relabeledpoints.AddPoints(*segment);
        /// confidence represent percentage of majority of object class
        auto segment_confidence = std::make_pair(seg_nr, majority_obj_count / (double) segment->size());
        vec_seg_confidence.push_back(segment_confidence);

        //printf("Segment Number and Size: %d, %d \n", seg_nr, segment -> size()); // debug
        /// loop through laserpoints partitioned by label in a segment
        for(auto it : vec_labelpartitions){ // "it" is a laserpoints with similar labels in a segment
            if(it.size() / (double) segment->size() > 0.05){
                //cout << "size lp_label:" << it.size() << endl;  // debug
                int division_count, labeltag, label2tag;
                //labeltag  = segment->MostFrequentAttributeValue(LabelTag, count1);
                label2tag = it.MostFrequentAttributeValue(Label2Tag, division_count);  /// dominant division in a label
                labeltag = it.begin()->Attribute(LabelTag);

                double count_percentage, count2_percentage;
                count_percentage = it.size() / (double)  segment->size();
                count2_percentage=  division_count / (double) segment->size();

                /// print out the report without class numbers translation
                if(!verbose){
                    fprintf(segments_report, "    %5d, %.2f, %d, %.2f \n",
                            labeltag, count_percentage, label2tag, count2_percentage);
                }

                /// translate class numbers to class names if verbose
                if(verbose){
                    ObjNameAndNumber object;
                    Annotator annotator;
                    object  = annotator.TranslateObjectLabel(labeltag); // Clutter_1
                    string obj_name_str;
                    std::stringstream sstream;
                    sstream << object.obj_name << "_" << object.obj_number;
                    obj_name_str = sstream.str();
                    fprintf(segments_report, "    %11s (%.2f) , %d (%.2f) \n",
                            obj_name_str.c_str(), count_percentage, label2tag, count2_percentage);
                    //sstream.clear();
                }
            }
        }
    }
    strcpy (str_root,output);
    relabeledpoints.Write(strcat(str_root, "/relabeled_segments.laser"), false);
    fclose(segments_report);


    /* second part of the code:  generate a report based on segments in each object */

    /// loop through relabeled points for each obj (labeltag && label2tag)
    /// and generate list of segments per object
    vector<LaserPoints> vec_objects;
    char* out_objects;
    out_objects = out_segments_and_objects ? output : 0;
    vec_objects = PartitionLpByTag1Tag2(relabeledpoints, LabelTag, Label2Tag, out_objects);

    /// if necessary sort the output by labeltag
    //if (sort_output) sort(vec_objects.begin(), vec_objects.end(), compare_lp_labeltag);
    if (sort_output) sort(vec_objects.begin(), vec_objects.end(), compare_lp_label2tag); //sort by division

    /// Store objects details based on their segments
    FILE *segments_report_per_object;
    strcpy (str_root,output); // initialize the str_root with root string
    segments_report_per_object = fopen(strcat(str_root, "/segments_report_per_object.txt"),"w");
    fprintf(segments_report_per_object, "Object ID (object name_no) --> segment_no (confidence) \n");

    for (auto obj : vec_objects){

        /// generate object name
        int obj_no, obj_div, labeltag, label2tag;
        labeltag  = obj[0].Attribute(LabelTag);
        label2tag = obj[0].Attribute(Label2Tag);
        ObjNameAndNumber object;
        Annotator annotator;
        object  = annotator.TranslateObjectLabel(labeltag);
        string obj_name_str;
        std::stringstream sstream;
        sstream << object.obj_name << "_" << object.obj_number; // Clutter_1
        obj_name_str = sstream.str();

        /// generate object name by label1 and label2
        string label1_label2;
        std::stringstream ss;
        ss << labeltag << "_" << label2tag; // 2001_150002
        label1_label2 = ss.str();

        fprintf(segments_report_per_object, "%13s (%11s ) --> ", label1_label2.c_str(), obj_name_str.c_str());
        // generate a list of segments in an object
        vector<int> segment_numbers;
        segment_numbers = obj.AttributeValues(SegmentNumberTag);
        double seg_confidence;
        /// for each segment extract segment confidence
        for(auto seg_no : segment_numbers){
            if (seg_no < 0) seg_no = -1;

            /// find the confidence for the segment in the vector of pairs of <seg_nr, confidence>
            auto it = find_if(vec_seg_confidence.begin(), vec_seg_confidence.end(),
                              [&seg_no](const pair<int, double>& p) { return p.first == seg_no; });

            //// **** printing to the file
            /// if there is just one segment
            if (segment_numbers.size() == 1 || seg_no == segment_numbers.back()) {
                fprintf(segments_report_per_object, "%d (%.2f)", seg_no, it->second);
            }

            /// if there is more than one segment
/*            if (segment_numbers.size() > 1 && seg_no != segment_numbers.back()){
                fprintf(segments_report_per_object, "%d (%.2f) | ", seg_no, it->second);
            }*/

            /// if there is more than one segment
            if (segment_numbers.size() > 1){
                fprintf(segments_report_per_object, " |%d (%.2f) ", seg_no, it->second);
            }
            ////////********

        }
        fprintf(segments_report_per_object, "\n");
        sstream.clear();
        ss.clear();
    }
    fclose(segments_report_per_object);

}














