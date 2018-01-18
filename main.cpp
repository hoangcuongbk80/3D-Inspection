#include "3D_Inspection.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	3D_Inspection w;
	w.show();
	return a.exec();
}
