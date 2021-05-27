#include "FeatureDetection.h"
#include "ColorSpaces.h"
#include <list>
#include <cmath>

#include <iostream>
using namespace std;

#define PI 3.14159265358979323846

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

			// Calculate global gradient and apply threshold
			G[j*xSize + i] = sqrt(Gh*Gh + Gv*Gv);

			double edgeAngle = atan2(Gv, Gh);
			//cout << "G = " << G[j*xSize + i] << endl;
			//cout << edgeAngle << " " << edgeAngle + 2 * PI << " " << edgeAngle * 180 / PI << " " << (edgeAngle + 2 * PI) * 180 / PI << endl;
			if (edgeAngle < 0) {
				edgeAngle += 2 * PI;
			}
			angle[j*xSize + i] = edgeAngle;
		}
	}


	delete[] extendedImage;	
}




vector<double> calculateFeatureVector(const uchar input[], int xSize, int ySize)
{
	vector<double> features;

	uchar* Y_buff = new uchar[xSize*ySize];
	char* U_buff = new char[xSize*ySize / 4];
	char* V_buff = new char[xSize*ySize / 4];
	RGBtoYUV420(input, xSize, ySize, Y_buff, U_buff, V_buff);
	
	double* G = new double[xSize*ySize];
	double* angle = new double[xSize*ySize];
	performSobelEdgeDetection(Y_buff, xSize, ySize, G, angle);

	double* gr = new double[9];
	for (int i = 0; i < 9; i++) {
		gr[i] = 2.0 * PI * (i + 1) / 9;
	}

	double sum = 0;
	double* histogram = new double[9];
	for (int i = 0; i < xSize; i += 8) {
		for (int j = 0; j < ySize; j += 8) {

			for (int p = 0; p < 9; p++)
				histogram[p] = 0;

			sum = 0;
			for (int k1 = i; k1 < i + 8; k1++) {
				for (int k2 = j; k2 < j + 8; k2++) {
					
					sum += G[k2*xSize + k1];
					for (int p = 0; p < 9; p++) {
						if (angle[k2*xSize + k1] < gr[p]) {
							//cout << angle[k2*xSize + k1] << " " << gr[p] << endl;
							histogram[p] += G[k2*xSize + k1];
							break;
						}
					}

				}
			}

			for (int p = 0; p < 9; p++) {
				features.push_back(histogram[p]/64);
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