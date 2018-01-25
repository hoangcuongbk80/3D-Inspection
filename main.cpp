#include "threeD_Inspection.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	threeD_Inspection w;
	w.show();
	return a.exec();
}
