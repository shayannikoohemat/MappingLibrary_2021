#include <LaserPoints.h>

#ifndef FACE_SELECTION_H
#define FACE_SELECTION_H


class FaceSelection
{
public:
    //FaceSelection();
    int faceSelection(LaserPoints segments, int min_segsize, double dist_threshold,
                                     double area_threshld,
                                     ObjectPoints faces_v,LineTopologies faces_e);

private:
    const double PI = 3.14159;

};

#endif // FACE_SELECTION_H
