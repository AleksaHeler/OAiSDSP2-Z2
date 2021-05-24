
#ifndef _FEATURE_DETECTION_H_
#define _FEATURE_DETECTION_H_

#include <QtGlobal>
#include <vector>

/*******************************************************************************************************************************/
/* Calculate feature vector for a given RGB image */
/*******************************************************************************************************************************/
std::vector<double> calculateFeatureVector(const uchar input[], int xSize, int ySize);

#endif //  _FEATURE_DETECTION_H_

