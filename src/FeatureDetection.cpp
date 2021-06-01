#include "FeatureDetection.h"
#include "ColorSpaces.h"
#include <list>
#include <cmath>
#include <iostream>

using namespace std;

#define PI 3.14159265358979323846

//Choose feature descriptor
#define HOG
//#define CUSTOM

//Chose parametars
#define BLOCK_SIZE 8 //BLOCK_SIZE 4 => 4 * 4 = 16

//Only has effect when using HOG
#define HISTOGRAM_SIZE 9

//Only has effect when using CUSTOM
//default histogram size is 7 and size is defined in const variable in calculateFeatureVectorCustom
//changing positions won't result in better validation
#define HORIZONTAL_TOP 0
#define HORIZONTAL_MIDDLE 1
#define HORIZONTAL_BOTTOM 2
#define VERTICAL_LEFT_TOP 3
#define VERTICAL_LEFT_BOTTOM 4
#define VERTICAL_RIGHT_TOP 5
#define VERTICAL_RIGHT_BOTTOM 6

std::vector<double> calculateFeatureVector(const uchar input[], int xSize, int ySize) 
{

#if defined(HOG)
	return calculateFeatureVectorHOG(input, xSize, ySize);
#elif defined(CUSTOM)
	return calculateFeatureVectorCustom(input, xSize, ySize);
#endif

}

vector<double> calculateFeatureVectorHOG(const uchar input[], int xSize, int ySize)
{
	vector<double> features;

	uchar* G = new uchar[xSize*ySize];
	RGBtoLuminance(input, xSize, ySize, G);

	double* angle = new double[xSize*ySize];
	performSobelEdgeDetection(G, xSize, ySize, angle);

	//Calculating upper bounds of angle
	double gr[HISTOGRAM_SIZE];
	for (int i = 0; i < HISTOGRAM_SIZE; i++)
		gr[i] = 2.0 * PI * (i + 1) / HISTOGRAM_SIZE;

	double histogram[HISTOGRAM_SIZE];
	double sum = 0;
	//Splitting images in blocks BLOCK_SIZE*BLOCK_SIZE
	for (int i = 0; i < xSize; i += BLOCK_SIZE) {
		for (int j = 0; j < ySize; j += BLOCK_SIZE) {

			//Reseting histogram
			for (int p = 0; p < HISTOGRAM_SIZE; p++)
				histogram[p] = 0;

			//Getting block BLOCK_SIZE*BLOCK_SIZE
			for (int k1 = i; k1 < i + BLOCK_SIZE; k1++) {
				for (int k2 = j; k2 < j + BLOCK_SIZE; k2++) {

					//Used for normalisation
					sum += G[k2*xSize + k1] * G[k2*xSize + k1];

					//Filling histogram based on angle
					if (angle[k2*xSize + k1] >= gr[HISTOGRAM_SIZE - 1]) 
					{
						histogram[HISTOGRAM_SIZE - 1] = angle[k2*xSize + k1];
					}
					else
					{
						for (int p = 0; p < HISTOGRAM_SIZE; p++) 
						{
							if (angle[k2*xSize + k1] < gr[p]) 
							{
								histogram[p] += G[k2*xSize + k1];
								break;
							}
						}
					}

				}
			}

			//Filling return value
			for (int p = 0; p < HISTOGRAM_SIZE; p++)
				features.push_back(histogram[p]);

		}
	}

	//Normalisation
	sum = sqrt(sum);
	for (auto it = features.begin(); it != features.end(); it++)
		*it /= sum;

	delete[] G;
	delete[] angle;

	return features;
}

vector<double> calculateFeatureVectorCustom(const uchar input[], int xSize, int ySize)
{
	vector<double> features;

	uchar* G = new uchar[xSize*ySize];
	RGBtoLuminance(input, xSize, ySize, G);

	double* angle = new double[xSize*ySize];
	performCannyEdgeDetection(G, xSize, ySize, 0, 0.3, angle);

	const int histogramPostitions = 7;
	double histogram[histogramPostitions];
	double sum = 0;
	//Splitting images in blocks BLOCK_SIZE*BLOCK_SIZE
	for (int i = 0; i < xSize; i += BLOCK_SIZE) {
		for (int j = 0; j < ySize; j += BLOCK_SIZE) {

			//Reset histogram
			for (int p = 0; p < histogramPostitions; p++)
				histogram[p] = 0;

			//Getting block BLOCK_SIZE*BLOCK_SIZE
			for (int k1 = i; k1 < i + BLOCK_SIZE; k1++) {
				for (int k2 = j; k2 < j + BLOCK_SIZE; k2++) {

					//Used for normalisation
					sum += G[k2*xSize + k1] * G[k2*xSize + k1];

					//Filling histogram
					if (angle[k2*xSize + k1] == 90) { // |
						if (k1 < xSize / 2) { //left
							if (k2 < ySize / 2)  //top
								histogram[VERTICAL_LEFT_TOP] += G[k2*xSize + k1];
							else //bottom
								histogram[VERTICAL_LEFT_BOTTOM] += G[k2*xSize + k1];
						}
						else { // right
							if (k2 < ySize / 2) //top
								histogram[VERTICAL_RIGHT_TOP] += G[k2*xSize + k1];
							else //bottom
								histogram[VERTICAL_RIGHT_BOTTOM] += G[k2*xSize + k1];
						}
					}
					else { // -
						if (k2 < ySize / 3) //top
							histogram[HORIZONTAL_TOP] += G[k2*xSize + k1];
						else if (k2 > ySize / 3 && k2 < ySize - (ySize / 3)) //middle
							histogram[HORIZONTAL_MIDDLE] += G[k2*xSize + k1];
						else //bottom
							histogram[HORIZONTAL_BOTTOM] += G[k2*xSize + k1];
					}

				}
			}

			//Filling return value
			for (int p = 0; p < histogramPostitions; p++) {
				features.push_back(histogram[p]);
			}

		}
	}

	//Normalisation
	sum = sqrt(sum);
	for (auto it = features.begin(); it != features.end(); it++) {
		*it /= sum;
	}

	delete[] G;
	delete[] angle;

	return features;
}



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


void performSobelEdgeDetection(uchar input[], int xSize, int ySize, double* angle)
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

			// Calculate and save magnitude
			input[j*xSize + i] = sqrt(Gh*Gh + Gv*Gv);

			// Calculate and save angle
			double edgeAngle = atan2(Gv, Gh);
			if (edgeAngle < 0) 
				edgeAngle += 2 * PI;
			angle[j*xSize + i] = edgeAngle;
		}
	}

	delete[] extendedImage;	
}



void nonMaxSupression(double edgeMagnitude[], uchar edgeDirection[], int xSize, int ySize, double out[])
{
	for (int i = 0; i < xSize; i++)
	{
		for (int j = 0; j < ySize; j++)
		{
			double mag1 = 0;
			double mag2 = 0;
			switch (edgeDirection[j * xSize + i])
			{
			case 0:
				if (j > 0)
					mag1 = edgeMagnitude[(j - 1) * xSize + i];
				if (j < ySize - 1)
					mag2 = edgeMagnitude[(j + 1) * xSize + i];
				break;
			case 45:
				if (j > 0 && i > 0)
					mag1 = edgeMagnitude[(j - 1) * xSize + i - 1];
				if (j < ySize - 1 && i < xSize - 1)
					mag2 = edgeMagnitude[(j + 1) * xSize + i + 1];
				break;
			case 90:
				if (i > 0)
					mag1 = edgeMagnitude[(j)*xSize + i - 1];
				if (i < xSize - 1)
					mag2 = edgeMagnitude[(j)*xSize + i + 1];
				break;
			case 135:
				if (j > 0 && i < xSize - 1)
					mag1 = edgeMagnitude[(j - 1) * xSize + i + 1];
				if (j < ySize - 1 && i > 0)
					mag2 = edgeMagnitude[(j + 1) * xSize + i - 1];
				break;
			default:
				break;
			}

			double mag0 = edgeMagnitude[j * xSize + i];

			if (mag0 >= mag1 && mag0 >= mag2)
				out[j * xSize + i] = mag0;
			else
				out[j * xSize + i] = 0;
		}
	}

}

void performCannyEdgeDetection(uchar input[], int xSize, int ySize, double threshold1, double threshold2, double* angle)
{
	int N = 3;
	int delta = (N - 1) / 2;
	double hCoeff[] = { 0.25, 0.5, 0.25, 0, 0, 0,-0.25, -0.5, -0.25 };
	double vCoeff[] = { 0.25, 0, -0.25, 0.5, 0, -0.5, 0.25, 0, -0.25 };

	int newXSize = xSize + 2 * delta;
	int newYSize = ySize + 2 * delta;

	uchar* extendedImage = new uchar[newXSize * newYSize];

	double* edgeMagnitude = new double[xSize * ySize];
	uchar* edgeDirection = new uchar[xSize * ySize];
	double* nonMaxSup = new double[xSize * ySize];

	extendBorders(input, xSize, ySize, extendedImage, delta);

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

			// Calculate global gradient
			double G = sqrt(Gh*Gh + Gv*Gv);
			edgeMagnitude[j * xSize + i] = G;

			// Calculate edge angle
			double edgeAngle = atan2(Gv, Gh);
			if (edgeAngle < 0)
				edgeAngle += PI;
			if ((edgeAngle < PI / 8) || (edgeAngle > 7 * PI / 8))
				edgeDirection[j * xSize + i] = 0;
			else if (edgeAngle < 3 * PI / 8)
				edgeDirection[j * xSize + i] = 45;
			else if (edgeAngle < 5 * PI / 8)
				edgeDirection[j * xSize + i] = 90;
			else
				edgeDirection[j * xSize + i] = 135;

			// Calculate and save angle
			if (edgeAngle > PI / 4 && edgeAngle < 3 * PI / 4)
				angle[j*xSize + i] = 90;
			else
				angle[j*xSize + i] = 0;

		}
	}

	nonMaxSupression(edgeMagnitude, edgeDirection, xSize, ySize, nonMaxSup);

	/* Apply threshold */
	for (int i = 0; i < xSize; i++)
	{
		for (int j = 0; j < ySize; j++)
		{

			if (nonMaxSup[j * xSize + i] < threshold1) //lower bound
				input[j * xSize + i] = 0;
			else if (nonMaxSup[j * xSize + i] > threshold2) //upper bound
				input[j * xSize + i] = 255;
			else
			{
				double mag1 = 0;
				double mag2 = 0;

				switch (edgeDirection[j * xSize + i])
				{
				case 0:
					if (j > 0)
						mag1 = edgeMagnitude[(j - 1) * xSize + i];
					if (j < ySize - 1)
						mag2 = edgeMagnitude[(j + 1) * xSize + i];
					break;

				case 45:
					if (j > 0 && i > 0)
						mag1 = edgeMagnitude[(j - 1) * xSize + i - 1];
					if (j < ySize - 1 && i < xSize - 1)
						mag2 = edgeMagnitude[(j + 1) * xSize + i + 1];
					break;

				case 90:
					if (i > 0)
						mag1 = edgeMagnitude[(j)*xSize + i - 1];
					if (i < xSize - 1)
						mag2 = edgeMagnitude[(j)*xSize + i + 1];
					break;

				case 135:
					if (j > 0 && i < xSize - 1)
						mag1 = edgeMagnitude[(j - 1) * xSize + i + 1];
					if (j < ySize - 1 && i > 0)
						mag2 = edgeMagnitude[(j + 1) * xSize + i - 1];
					break;

				default:
					break;
				}

				if (mag1 > threshold2 || mag2 > threshold2)
					input[j * xSize + i] = edgeMagnitude[j * xSize + i];
				else
					input[j*xSize + i] = 0;
			}
		}
	}

	delete[] extendedImage;
	delete[] edgeDirection;
	delete[] edgeMagnitude;
	delete[] nonMaxSup;
}