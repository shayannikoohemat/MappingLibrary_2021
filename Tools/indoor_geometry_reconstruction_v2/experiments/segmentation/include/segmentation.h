
#include <LaserPoints.h>

#ifndef SEGMENTATION_SEGMENTATION_H
#define SEGMENTATION_SEGMENTATION_H

#endif //SEGMENTATION_SEGMENTATION_H

///
/// \param input_file
/// \param output_file
/// \param MaxDistToPlane
/// \param GrowingRadius
void surfacegrowing_test(char* input_file, char* output_file,
                         double MaxDistToPlane=0.05, double GrowingRadius=1.0);
///
/// \param input_file
/// \param output_file
/// \param MaxDistInComponent
/// \return
LaserPoints Connected_Component_Segmentation(char* input_file, char* output_file,
                                             double MaxDistInComponent=0.5);