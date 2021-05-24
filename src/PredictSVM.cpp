#include "PredictSVM.h"
#include "FeatureDetection.h"
#include <iostream>
#include <fstream>
#include <QDebug>

using namespace std;

bool predictSVM(const ImgDataBase& dataBase, std::string SVMModelFileName)
{
	qInfo() << "Load SVM model ...";
	
	svm_model* svmModel = svm_load_model(SVMModelFileName.c_str());
	if(svmModel == NULL)
	{
		qCritical() << "ERROR: Unable to load SVM model (" << SVMModelFileName.c_str() << ")";
		return false;
	}
	
	qInfo() << "DONE" << endl;

	qInfo() << "Do prediction...";
	int totalHits = 0;
	int imageIndex = 0;
	for (const DBImage& img : dataBase)
	{
		int predictedLabel = predictSingleImage(img.image->bits(), img.image->width(), img.image->height(), svmModel);

		if (predictedLabel == img.labelNumber)
		{
			totalHits++;
		}
		
		imageIndex++;
	}

	qInfo() << "DONE" << endl;

	qInfo() << "Prediction accuracy: " << totalHits*100.0 / dataBase.size() << "%";
	
	svm_free_and_destroy_model(&svmModel);

	return true;
}


int predictSingleImage(const uchar input[], int xSize, int ySize, svm_model* svmModel)
{
	vector<double> feature = calculateFeatureVector(input, xSize, ySize);

	svm_node* featureVector = new svm_node[feature.size()];
	
	int j = 0;

	for (int index = 0; index < feature.size(); index++)
	{
		if (feature[index] != 0)
		{
			featureVector[j] = { index+1, feature[index] };
			j++;
		}
	}
	featureVector[j] = { -1, 0.0 };
	
	double res = svm_predict(svmModel, featureVector);
	
	delete[] featureVector;

	return round(res);
}