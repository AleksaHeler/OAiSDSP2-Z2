#include "FeatureDetection.h"
#include "ColorSpaces.h"
#include <list>
#include <cmath>
#include <iostream>

using namespace std;

#define PI 3.14159265358979323846

// Choose feature descriptor
#define HOG
//#define CUSTOM

// Chose parametars
#define BLOCK_SIZE 4
#define HISTOGRAM_SIZE 18 //only has effect when using HOG


void extendBorders(uchar input[], int xSize, int ySize, uchar output[], int delta)
{
	int newXSize = xSize + 2 * delta;
	int newYSize = ySize + 2 * delta;

	for (int i = 0; i < xSize; i++)
	{
		for (int j = 0; j < ySize; j++)
		{
			output[(j + delta) * newXSize + i + delta] = input[j * xSize + i];
		}
	}

	for (int i = 0; i < delta; i++)
	{
		memcpy(&output[i * newXSize + delta], &input[0], xSize);
		memcpy(&output[(ySize + delta + i) * newXSize + delta], &input[(ySize - 1) * xSize], xSize);
	}

	for (int i = 0; i < newYSize; i++)
	{
		memset(&output[i * newXSize], output[i * newXSize + delta], delta);
		memset(&output[i * newXSize + delta + xSize], output[i * newXSize + xSize + delta - 1], delta);
	}
}

void performSuccessiveNFFilter(uchar input[], int xSize, int ySize, int stages)
{
	double* coeff = new double[9];
	for (int i = 0; i < 9; i++)
	{
		coeff[i] = 1.0 / 9.0;
	}

	for (int i = 0; i < stages; i++)
	{
		convolve2D(input, xSize, ySize, coeff, 3);
	}
	delete[] coeff;
}

void convolve2D(uchar image[], int xSize, int ySize, double* filterCoeff, int N)
{
	int newXSize = xSize + (N - 1);
	int newYSize = ySize + (N - 1);

	uchar* extendedImage = new uchar[newXSize * newYSize];

	extendBorders(image, xSize, ySize, extendedImage, (N - 1) / 2);

	for (int i = 0; i < xSize; i++)
	{
		for (int j = 0; j < ySize; j++)
		{
			double accum = 0;
			for (int m = 0; m < N; m++)
			{
				for (int n = 0; n < N; n++)
				{

					accum += extendedImage[(j + n) * newXSize + i + m] * filterCoeff[(N - n) * N - m - 1];
				}
			}
			if (accum > 255.0)
				image[j * xSize + i] = 255;
			else if (accum < 0.0)
				image[j * xSize + i] = 0;
			else
				image[j * xSize + i] = floor(accum + 0.5);

		}
	}

	delete[] extendedImage;
}


void performSobelEdgeDetection(uchar input[], int xSize, int ySize, double* G, double* angle)
{
	int N = 3;
	int delta = (N - 1) / 2;

	//Declare Sobel coeffs for horizontal and vertical edges
	double hCoeff[] = { 0.25, 0.5, 0.25, 0, 0, 0,-0.25, -0.5, -0.25 };
	double vCoeff[] = { 0.25, 0, -0.25, 0.5, 0, -0.5, 0.25, 0, -0.25 };

	int newXSize = xSize + 2;
	int newYSize = ySize + 2;

	// Extend image
	uchar* extendedImage = new uchar[newXSize * newYSize];
	extendBorders(input, xSize, ySize, extendedImage, delta);

	// Perform edge detection
	for (int i = 0; i < xSize; i++)
	{
		for (int j = 0; j < ySize; j++)
		{
			// Apply Sobel horizontal and vertical filter
			double Gh = 0, Gv = 0;

			for (int m = 0; m < N; m++)
			{
				for (int n = 0; n < N; n++)
				{
					Gh += extendedImage[(j + n) * newXSize + i + m] * hCoeff[(N - n) * N - m - 1];
					Gv += extendedImage[(j + n) * newXSize + i + m] * vCoeff[(N - n) * N - m - 1];
				}
			}

			// Save magnitude
			G[j*xSize + i] = sqrt(Gh*Gh + Gv*Gv);

			// Save angle
			double edgeAngle = atan2(Gv, Gh);
			if (edgeAngle < 0) {
				edgeAngle += 2 * PI;
			}
			angle[j*xSize + i] = edgeAngle;
		}
	}

	delete[] extendedImage;	
}

vector<double> calculateFeatureVectorHOG(const uchar input[], int xSize, int ySize)
{
	vector<double> features;

	uchar* Y_buff = new uchar[xSize*ySize];
	char* U_buff = new char[xSize*ySize / 4];
	char* V_buff = new char[xSize*ySize / 4];
	RGBtoYUV420(input, xSize, ySize, Y_buff, U_buff, V_buff);
	
	double* G = new double[xSize*ySize];
	double* angle = new double[xSize*ySize];
	performSobelEdgeDetection(Y_buff, xSize, ySize, G, angle);

	int histogramSize = HISTOGRAM_SIZE;
	int blockSize = BLOCK_SIZE;

	//Calculating upper bounds of angle
	double* gr = new double[histogramSize];
	for (int i = 0; i < histogramSize; i++) {
		gr[i] = 2.0 * PI * (i + 1) / histogramSize;
	}

	double* histogram = new double[histogramSize];
	for (int i = 0; i < xSize; i += blockSize) {
		for (int j = 0; j < ySize; j += blockSize) {

			//Reseting histogram
			for (int p = 0; p < histogramSize; p++)
				histogram[p] = 0;

			//Filling histogram
			for (int k1 = i; k1 < i + blockSize; k1++) {
				for (int k2 = j; k2 < j + blockSize; k2++) {
					
					for (int p = 0; p < histogramSize; p++) {
						if (angle[k2*xSize + k1] < gr[p]) {
							histogram[p] += G[k2*xSize + k1];
							break;
						}
					}

				}
			}

			//Filling return value
			for (int p = 0; p < histogramSize; p++) {
				features.push_back(histogram[p]/(blockSize*blockSize));
			}
		}
	}

	delete[] histogram;
	delete[] gr;
	delete[] G;
	delete[] angle;
	delete[] Y_buff;
	delete[] U_buff;
	delete[] V_buff;
	
	return features;
}



std::vector<double> calculateFeatureVector(const uchar input[], int xSize, int ySize) {

#ifdef HOG
	return calculateFeatureVectorHOG(input, xSize, ySize);
#else
	return calculateFeatureVectorCustom(input, xSize, ySize);
#endif

}

void performSobelEdgeDetection2(uchar input[], int xSize, int ySize, double* G, double* angle, double threshold)
{
	int N = 3;
	int delta = (N - 1) / 2;

	//Declare Sobel coeffs for horizontal and vertical edges
	double hCoeff[] = { 0.25, 0.5, 0.25, 0, 0, 0,-0.25, -0.5, -0.25 };
	double vCoeff[] = { 0.25, 0, -0.25, 0.5, 0, -0.5, 0.25, 0, -0.25 };

	int newXSize = xSize + 2;
	int newYSize = ySize + 2;

	// Extend image
	uchar* extendedImage = new uchar[newXSize * newYSize];
	extendBorders(input, xSize, ySize, extendedImage, delta);

	// Perform edge detection
	for (int i = 0; i < xSize; i++)
	{
		for (int j = 0; j < ySize; j++)
		{
			// Apply Sobel horizontal and vertical filter
			double Gh = 0, Gv = 0;

			for (int m = 0; m < N; m++)
			{
				for (int n = 0; n < N; n++)
				{
					Gh += extendedImage[(j + n) * newXSize + i + m] * hCoeff[(N - n) * N - m - 1];
					Gv += extendedImage[(j + n) * newXSize + i + m] * vCoeff[(N - n) * N - m - 1];
				}
			}

			// Calculate global gradient and apply threshold
			G[j*xSize + i] = sqrt(Gh*Gh + Gv*Gv);
			if (G[j*xSize + i] < threshold) {
				G[j*xSize + i] = 0;
			}

			// Calculate and save angle
			double edgeAngle = atan2(Gv, Gh);
			if (edgeAngle < 0) {
				edgeAngle += PI;
			}

			if (edgeAngle > PI / 4 && edgeAngle < 3 * PI / 4) {
				angle[j*xSize + i] = 90;
			}
			else {
				angle[j*xSize + i] = 0;
			}

		}
	}

	delete[] extendedImage;
}

std::vector<double> calculateFeatureVectorCustom(const uchar input[], int xSize, int ySize) {

	vector<double> features;

	uchar* Y_buff = new uchar[xSize*ySize];
	char* U_buff = new char[xSize*ySize / 4];
	char* V_buff = new char[xSize*ySize / 4];
	RGBtoYUV420(input, xSize, ySize, Y_buff, U_buff, V_buff);

	performSuccessiveNFFilter(Y_buff, xSize, ySize, 3);

	double* G = new double[xSize*ySize];
	double* angle = new double[xSize*ySize];
	performSobelEdgeDetection2(Y_buff, xSize, ySize, G, angle, 0.65);

	double histogram[7] = { 0, 0, 0, 0, 0, 0, 0 };
	int blockSize = BLOCK_SIZE;
	for (int i = 0; i < xSize; i += blockSize) {
		for (int j = 0; j < ySize; j += blockSize) {

			//Reset histogram to 0
			for (int p = 0; p < 7; p++)
				histogram[p] = 0;

			//Fill histogram
			for (int k1 = i; k1 < i + blockSize; k1++) {
				for (int k2 = j; k2 < j + blockSize; k2++) {

					if (angle[k2*xSize + k1] == 90) { // |
						if (k1 < xSize / 2) { //left
							if (k2 < ySize / 2)  //top
								histogram[3] += G[k2*xSize + k1];
							else //bottom
								histogram[4] += G[k2*xSize + k1];
						}
						else { // right
							if (k2 < ySize / 2) //top
								histogram[5] += G[k2*xSize + k1];
							else //bottom
								histogram[6] += G[k2*xSize + k1];
						}
					}
					else { // -
						if (k2 < ySize / 3) //top
							histogram[0] += G[k2*xSize + k1];
						else if (k2 > ySize / 3 && k2 < ySize - (ySize / 3)) //middle
							histogram[1] += G[k2*xSize + k1];
						else //bottom
							histogram[2] += G[k2*xSize + k1];
					}
				
				}
			}

			//Filling return value
			for (int p = 0; p < 7; p++) {
				features.push_back(histogram[p]/64);
			}

		}
	}

	delete[] G;
	delete[] angle;
	delete[] Y_buff;
	delete[] U_buff;
	delete[] V_buff;
	
	return features;
}