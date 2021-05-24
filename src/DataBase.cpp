#include "DataBase.h"
#include <fstream>
#include <QDebug>

using namespace std;

bool readDatabase(ImgDataBase& dataBase, string databasePath, string dataBaseName)
{
	ifstream dbFile(databasePath + dataBaseName);
	if (dbFile.is_open())
	{
		qInfo() << "Database opened succesefully!" << endl;
		qInfo() << "Reading images... ";
	
		string line;
		int linenum = 1;

		while (getline(dbFile, line, '|'))
		{
			// Load image name and label 
			DBImage img;
			img.label = line;
			img.labelNumber = stoi(line);
			getline(dbFile, line);

			// Open image and align dimension to be dividable by 8
			QImage input = QImage((databasePath + line).c_str());
			if (input.width() % 8 != 0 || input.height() % 8 != 0)
				input = input.scaled(input.width() + (8 - input.width() % 8), input.height() + (8 - input.height() % 8));

			img.image = new QImage(input.convertToFormat(QImage::Format_RGB888));

			if (img.image == NULL)
			{
				qCritical() << "Error reading database file at line" << linenum << ": " << line.c_str() << endl;
				return false;
			}
			
			dataBase.push_back(img);
			linenum++;
		}
		qInfo() << "DONE! " << endl;
		dbFile.close();
	}
	else
	{
		qCritical() << "Unable to open database file";
		return false;
	}

	return true;

}


