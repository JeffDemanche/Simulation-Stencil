#include <main.h>
#include "mainwindow.h"
#include <iostream>
#include <ctime>
#include <QCommandLineParser>

using namespace std;

QString meshFile;
float incompressibility;
float rigidity;
float phi;
float psi;
float density;
QString sphereFile;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    QCommandLineParser parser;
    parser.addHelpOption();
    parser.addPositionalArgument("mesh", "Mesh file");
    parser.addPositionalArgument("incompressibility", "Elastic incompressibility");
    parser.addPositionalArgument("rigidity", "Elastic rigidity");
    parser.addPositionalArgument("phi", "Phi (viscous incompressibility)");
    parser.addPositionalArgument("psi", "Psi (viscous rigidity)");
    parser.addPositionalArgument("density", "Uniform mesh density");
    parser.addPositionalArgument("sphere", "Sphere mesh file");

    parser.process(a);

    const QStringList args = parser.positionalArguments();
    if(args.size() < 4) {
        cerr << "Error: Wrong number of arguments" << endl;
        a.exit(1);
        return 1;
    }
    meshFile = args[0];
    incompressibility = args[1].toFloat();
    rigidity = args[2].toFloat();
    phi = args[3].toFloat();
    psi = args[4].toFloat();
    density = args[5].toFloat();
    sphereFile = args[6];

    MainWindow w;
    srand (static_cast <unsigned> (time(0)));
    // We cannot use w.showFullscreen() here because on Linux that creates the
    // window behind all other windows, so we have to set it to fullscreen after
    // it has been shown. 
    w.show();
    //w.setWindowState(w.windowState() | Qt::WindowFullScreen); // Comment out this line to have a windowed 800x600 game on startup.

    return a.exec();
}

