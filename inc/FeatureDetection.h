
#ifndef _FEATURE_DETECTION_H_
#define _FEATURE_DETECTION_H_

#include <QtGlobal>
#include <vector>

/*******************************************************************************************************************************/
/* Calculate feature vector for a given RGB image */
/*******************************************************************************************************************************/
std::vector<double> calculateFeatureVector(const uchar input[], int xSize, int ySize);

void extendBorders(uchar input[], int xSize, int ySize, uchar output[], int delta);
void performSobelEdgeDetection(uchar input[], int xSize, int ySize, double threshold, double* G, double* angle);

#endif //  _FEATURE_DETECTION_H_

