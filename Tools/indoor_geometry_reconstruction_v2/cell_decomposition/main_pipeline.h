#include <iostream>
#include <LaserPoints.h>


///
/// \brief pipeline_v0
///
/// 1. create a 3d bbox around the data
/// 1.1 select the data with floor, wall, ceiling, beam, pillar
/// 2. segment the data (ransac or region growing)
/// 3. fit planes to segments and extend them to the bbox
/// 4. intesect planes to create faces
/// 5. split faces at intersections (cell decomposition)
/// 6. associate points to faces, points inside each face get the tag of the face
/// 7. face selection1: faces without points or with few points are selected
/// 8. face selection2: faces with small point coverage are selected
/// 9. face selection3: faces with very small area are selected (to be invalidated or merged)
/// 9. sample points in selected faces (add points to them)
/// 10. label points+faces to valid, invalid (inside but invalid) and outside
///
void pipeline_v0(char * proj_dir);
