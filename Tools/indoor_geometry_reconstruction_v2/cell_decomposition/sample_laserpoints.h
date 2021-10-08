#include "LaserPoints.h"


LaserPoints Sample_Laserpoints_3D(ObjectPoints faces_vertices, LineTopologies faces_edges, char *root_dir);


LaserPoints Create_Triangle_Centroids (TIN &tin, ObjectPoints &vertices);

ObjectPoint Objpts_Centroid(ObjectPoints points);

LaserPoints Face_to_Voxel(ObjectPoints faces_vertices, LineTopologies faces_edges,
                          char *root_dir, double vox_l);

LaserPoints Face_to_Voxel_with_noise(ObjectPoints faces_vertices, LineTopologies faces_edges,
                          char *root_dir, double vox_l, double noise_level, LaserPointTag face_tag=ScanLineNumberTag);

LaserPoints Project_points_to_Plane(LaserPoints lp, const Plane &plane);

bool InsideSegmentBounds(Position3D pos, DataBoundsLaser db);




