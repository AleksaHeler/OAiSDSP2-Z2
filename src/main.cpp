#include "MainWindow.h"

#include <QApplication>

int main(int argc, char *argv[]) {
	QApplication a(argc, argv);
	MainWindow* w = new MainWindow(QString("prjs/classification.imgdsp"), 0);
	w->showMaximized();

	return a.exec();
}