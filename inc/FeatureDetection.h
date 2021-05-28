#ifndef _FEATURE_DETECTION_H_
#define _FEATURE_DETECTION_H_

#include <QtGlobal>
#include <vector>

/*******************************************************************************************************************************/
/* Calculate feature vector for a given RGB image */
/*******************************************************************************************************************************/
std::vector<double> calculateFeatureVector(const uchar input[], int xSize, int ySize);

std::vector<double> calculateFeatureVectorHOG(const uchar input[], int xSize, int ySize);
std::vector<double> calculateFeatureVectorCustom(const uchar input[], int xSize, int ySize);


void extendBorders(uchar input[], int xSize, int ySize, uchar output[], int delta);
void performSuccessiveNFFilter(uchar input[], int xSize, int ySize, int stages);
void convolve2D(uchar image[], int xSize, int ySize, double* filterCoeff, int N);

void performSobelEdgeDetection(uchar input[], int xSize, int ySize, double* G, double* edgeAngle);
void performSobelEdgeDetection2(uchar input[], int xSize, int ySize, double* G, double* edgeAngle, double threshold);

#endif //  _FEATURE_DETECTION_H_