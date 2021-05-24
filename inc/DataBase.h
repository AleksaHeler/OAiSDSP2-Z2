#pragma once
#include <QImage>
#include <list>
#include <string>

typedef struct
{
	QImage* image;
	int labelNumber;
	std::string label;
} DBImage;

typedef std::list<DBImage> ImgDataBase;

bool readDatabase(ImgDataBase& dataBase, std::string databasePath, std::string dataBaseName);
