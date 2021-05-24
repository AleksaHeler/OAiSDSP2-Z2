#ifndef _TRAIN_SVM_H_
#define _TRAIN_SVM_H_

#include "DataBase.h"

bool trainSVM(const ImgDataBase& dataBase, std::string SVMModelFileName);

bool crossValidateKFold(const ImgDataBase& dataBase, int K);

#endif // _TRAIN_SVM_H_
