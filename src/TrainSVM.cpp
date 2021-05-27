
#include <QDebug>
#include "TrainSVM.h"
#include "DataBase.h"
#include "svm.h"
#include <fstream>
#include "FeatureDetection.h"

using namespace std;

// modify this function to tune SVM classifier
static void setDefaultParams(svm_parameter& param)
{
	
	//  svm_type : set type of SVM:
	//		0 -- C-SVC		(multi-class classification)
	//		1 -- nu-SVC		(multi-class classification)
	//		2 -- one-class SVM
	//		3 -- epsilon-SVR(regression)
	//		4 -- nu-SVR		(regression)
	//	kernel_type : set type of kernel function
	//		0 -- linear: u'*v
	//		1 -- polynomial: (gamma*u'*v + coef0)^degree
	//		2 -- radial basis function: exp(-gamma*|u-v|^2)
	//		3 -- sigmoid: tanh(gamma*u'*v + coef0)
	//	degree : set degree in kernel function
	//	gamma : set gamma in kernel function 
	//	coef0 : set coef0 in kernel function (default 0)
	//	C : set the parameter C of C-SVC, epsilon-SVR, and nu-SVR (default 1)
	//	nu : set the parameter nu of nu-SVC, one-class SVM, and nu-SVR (default 0.5)
	//	cachesize : set cache memory size in MB 
	//	eps : set the epsilon in loss function of epsilon-SVR 
	//	p : set tolerance of termination criterion
	//	shrinking : whether to use the shrinking heuristics, 0 or 1 \n"
	//	probability : whether to train a SVC or SVR model for probability estimates, 0 or 1 \n"
	//	weight : set the parameter C of class i to weight*C, for C-SVC \n"

	param.svm_type = C_SVC;
	param.kernel_type = LINEAR;
	param.degree = 0;
	param.gamma = 1;
	param.coef0 = 0;
	param.C = 1;
	param.nu = 0;
	param.cache_size = 100;
	param.eps = 1;
	param.p = 0.1;
	param.shrinking = 1;
	param.probability = 0;
	param.nr_weight = 0;
	param.weight_label = NULL;
	param.weight = NULL;
}

bool trainSVM(const ImgDataBase& dataBase, std::string SVMModelFileName)
{
	svm_problem svmProblemSet;
	svm_parameter svmParam;

	// Create problem set
	svmProblemSet.l = dataBase.size();
	svmProblemSet.y = new double[svmProblemSet.l];
	svmProblemSet.x = new svm_node* [svmProblemSet.l];

	// Load SVM params
	setDefaultParams(svmParam);

	qInfo() << "Calculating features for SVM...";
	int i = 0;
	for (const DBImage& img : dataBase)
	{
		// Calculate features
		vector<double> feature = calculateFeatureVector(img.image->bits(), img.image->width(), img.image->height());
	
		// Prepare problem set based on label and calculated features
		svmProblemSet.y[i] = img.labelNumber;
		svmProblemSet.x[i] = new svm_node[feature.size()+1];
		
		int j = 0;
		for (int index = 0; index < feature.size(); index++)
		{
			if (feature[index] != 0)
			{
				svmProblemSet.x[i][j] = { index + 1, feature[index] };
				j++;
			}
		}
		svmProblemSet.x[i][j] = { -1, 0.0 };
		i++;
	}
	
	qInfo() << "DONE" << endl;
	
	qInfo() << "Training SVM model ...";

	// Check svm parameters 
	if (svm_check_parameter(&svmProblemSet, &svmParam))
	{
		qCritical() << "ERROR: invalid problem set!" << endl;
		return false;
	}
	
	// Invoke train function
	svm_model* model = svm_train(&svmProblemSet, &svmParam);
	
	qInfo() << "DONE" << endl;

	// Save model to file
	if(svm_save_model(SVMModelFileName.c_str(), model))
	{
		qWarning() <<"ERROR: can't save model to file";
	}
	
	// Free used memory
	svm_free_and_destroy_model(&model);

	delete[] svmProblemSet.y;
	delete[] svmProblemSet.x;

	return true;
}


bool crossValidateKFold(const ImgDataBase& dataBase, int K)
{
	svm_problem svmProblemSet;
	svm_parameter svmParam;

	// Create problem set 
	svmProblemSet.l = dataBase.size();
	svmProblemSet.y = new double[svmProblemSet.l];
	svmProblemSet.x = new svm_node*[svmProblemSet.l];

	// Load SVM params 
	setDefaultParams(svmParam);

	qInfo() << "Calculating features for SVM...";
	int i = 0;
	for (const DBImage& img : dataBase)
	{
		// Calculate features
		vector<double> feature = calculateFeatureVector(img.image->bits(), img.image->width(), img.image->height());

		// Prepare problem set based on label and calculated features
		svmProblemSet.y[i] = img.labelNumber;
		svmProblemSet.x[i] = new svm_node[feature.size() + 1];

		int j = 0;
		for (int index = 0; index < feature.size(); index++)
		{
			if (feature[index] != 0)
			{
				svmProblemSet.x[i][j] = { index + 1, feature[index] };
				j++;
			}
		}
		svmProblemSet.x[i][j] = { -1, 0.0 };
		i++;
	}


	qInfo() << "DONE" << endl;

	qInfo() << "Perform cross-validation, k = " << K << "...";

	// Prepare target vector
	int total_correct = 0;
	double *target = new double[svmProblemSet.l];

	// Check svm parameters 
	if (svm_check_parameter(&svmProblemSet, &svmParam))
	{
		qCritical() << "ERROR: invalid problem set!" << endl;
		return false;
	}

	// Invoke K-fold cross validation
	svm_cross_validation(&svmProblemSet, &svmParam, K, target);
	
	qInfo() << "DONE" << endl;

	// Calculate prediction accuracy in %
	for (int i = 0; i<svmProblemSet.l; i++)
		if (target[i] == svmProblemSet.y[i])
			++total_correct;
	qInfo() << "Cross Validation Accuracy = " << 100.0*total_correct / svmProblemSet.l << endl;
	

	// Free used memory
	delete[] target;
	delete[] svmProblemSet.y;
	delete[] svmProblemSet.x;

	return true;
}



