//
// Created by NikoohematS on 23-11-2017.
//

#include <boost/geometry.hpp>

#ifndef INDOOR_GEOMETRY_REOCNSTRUCTION_POST_PROCESSING_H
#define INDOOR_GEOMETRY_REOCNSTRUCTION_POST_PROCESSING_H

#endif //INDOOR_GEOMETRY_REOCNSTRUCTION_POST_PROCESSING_H

void intersect_segments(const LaserPoints &segmented_lp, double start_threshold, double increase_step_size,
                        double stop_threshold, char* output_dir);

typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> boost_point;
typedef boost::geometry::model::polygon<boost_point> boost_polygon;
std::deque<boost_polygon> intersect_polygons(ObjectPoints const &poly1_corners, ObjectPoints const &poly2_corners);

vector<LaserPoints> filter_ceil_by_intersection(map<int, double> &horizon_segmetns_centroidheight_map,
                                                vector<LaserPoints> &candidate_ceil_segment_vec,
                                                const vector<pair<ObjectPoints, LineTopology>> &min_rectangle_ceil,
                                                vector<int> &not_ceiling_segments_nr_output,
                                                LaserPoints &not_ceiling_segments_lp_output,
                                                double intersection_percentage,
                                                bool is_floor=false);

/// for removing the points that are casued by occlusion_test process above each ceiling on the wall
/// not finished function
void filter_occlusion_result_by_ceiling (LaserPoints occlusion_result, LaserPoints lp_ceiling, char* root);


LaserPoints detect_false_openings (LaserPoints &wall_openings, double percentage);
