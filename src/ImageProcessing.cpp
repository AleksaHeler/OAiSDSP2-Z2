
#include "ImageProcessing.h"
#include "PredictSVM.h"
#include "svm.h"
#include <QDebug>
#include <QPainter>

static bool modelLoaded = false;
static svm_model* svmModel;
static const std::string pathSVMModelFile = "precalculated_models\\numbersSVM.model";

void imageProcessingFun( const QString& progName, QImage& outImgs, const QImage& inImgs, const QVector<double>& params)
{
	if (!modelLoaded)
	{
		svmModel = svm_load_model(pathSVMModelFile.c_str());
		if (svmModel == NULL)
		{
			return;
		}
		modelLoaded = true;

	}

	outImgs = QImage(inImgs);

	if (progName == "SVM Classification")
	{
		// Invoke prediction for input image
		int predicted_label = predictSingleImage(inImgs.bits(), outImgs.width(), outImgs.height(), svmModel);
		
		
		//Paint image label
		outImgs.fill(Qt::GlobalColor::black);
		QPainter p(&outImgs);
		p.setPen(QPen(Qt::green));
		p.setFont(QFont("Times", 16, QFont::Bold));
		QString number = QString::number(predicted_label);
		p.drawText(outImgs.rect(), Qt::AlignCenter, number);
	}
}

