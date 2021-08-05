#include <LaserPoints.h>

// face selection and points association using coplanarity of segment and the face
LaserPoints associatePointsToFace3D(LaserPoints segments, int min_segsize,
                                                       double dist_threshold,
                                                       double area_threshld,
                                                       int min_points_for_face_selection,
                                                       ObjectPoints faces_v,
                                                       LineTopologies faces_e,
                                                       LineTopologies &selected_faces);

// face selection and points association using the linelabeltag from linetopology
LaserPoints associatePointsToFace3D_withTag(LaserPoints segments, int min_segsize,
                                                       double dist_threshold,
                                                       double area_threshld,
                                                       int min_points_for_face_selection,
                                                       ObjectPoints faces_v,
                                                       LineTopologies faces_e,
                                                       LineTopologies &selected_faces,
                                                       LineTopologies &faces_without_points);



