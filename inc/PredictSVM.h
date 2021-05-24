#pragma once
#include <QImage>
#include "svm.h"
#include "DataBase.h"
#include <string>


bool predictSVM(const ImgDataBase& dataBase, std::string SVMModelFileName);

int predictSingleImage(const uchar input[], int xSize, int ySize, svm_model* svmModel);
