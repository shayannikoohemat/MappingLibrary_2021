//
// Created by NikoohematS on 1-8-2017.
//

#ifndef DESCRIPTOR_MLS_PREPARATION_H
#define DESCRIPTOR_MLS_PREPARATION_H

#endif //DESCRIPTOR_MLS_PREPARATION_H

#include "LaserPoints.h"


bool comparison (const LaserPoint &p1, const LaserPoint &p2);

bool compare_lp_segmenttag(const LaserPoints& lp1,const LaserPoints& lp2);

bool compare_lp_labeltag(const LaserPoints& lp1,const LaserPoints& lp2);

bool compare_lp_label2tag(const LaserPoints& lp1,const LaserPoints& lp2);

bool compare_lp_size(const LaserPoints& lp1,const LaserPoints& lp2);

/// gives a report of each segments and its labels that represents objects
void SegmentToObject(LaserPoints &lp, char* output, bool out_segments_and_objects=false,
                     bool sort_output=true, bool verbose=true);

vector<string> RecursiveFilesPath (char *dirname, string filter_extension);

void ReducePointsBatch(char *dirname, char *outputdir, double reduction_factor);

void MergeLaserFiles(char *dirname, char *outputdir);

bool read_pcd_ascii(const string &pcd_filename, LaserPoints &lp_out, char * lp_outfile=0);

/// this is not done
bool read_pcd_bin(const string &pcd_filename, LaserPoints &lp_out, char * lp_outfile=0);

/// read two laser files, add segmentnumbers from one to the origin
//// size of two laser points and the order should be the same
LaserPoints merge_attributes(LaserPoints &lp_origin, LaserPoints &lp_merged, const LaserPointTag &tag);

/// segment laser points and reshape and merge segments with awkward TIN
void SegmentationAndRefinement(LaserPoints laserpoints, char* output_dir, int minsegmentsize,
                               double maxdistanceInComponent, double maxdistseedPlane,
                               bool do_segmentation=1, bool do_segment_refinement=1);


/// collect laserpoints per segment and return a vector of them,
/// computationally expensive: f(n) + f(nlogn)
vector <LaserPoints> CollectSegmentLaserpoints(LaserPoints &segmented_laserpoints,
                                               char* ouput_directory, bool verbose=false);

/// collect laserpoints per segment and return a vector of them,
std::vector<LaserPoints> LpToSegmentLpUsingBoost(LaserPoints const& lp, char* output=0);

/// NOTE: This function has a bug, it doesn't save the segment number in the final results ***
/// This is similar to CollectSegmentsLp, computationally faster, it needs sorting: f(nlogn)
// pass by value - we need a copy anyway and we might get copy elision
vector <LaserPoints> PartitionBySegmentNumber(LaserPoints lp, char * ouput_directory);

/// collect laserpoints per similar_tags and return a vector of them,
std::vector<LaserPoints> PartitionLpByTag(LaserPoints const& lp, LaserPointTag int_tag, char* output=0);

/// Partition laserpoints first by tag1 and then by tag2
vector <LaserPoints> PartitionLpByTag1Tag2(LaserPoints const& lp, LaserPointTag int_tag1,
                                           LaserPointTag int_tag2, char* output=0);





