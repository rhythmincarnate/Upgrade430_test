#include "actuatorupgrade.h"
#include <QApplication>
#include "actuatorcontroller.h"
#include <QTimer>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ActuatorController::initController();
    ActuatorController::getInstance()->autoRecoginze();
    ActuatorUpgrade w;
    w.setWindowTitle("ActuatorUpgrade_430--INTERNAL USE ONLY");

    w.show();

    QTimer t;
    QObject::connect(&t,&QTimer::timeout,[&]{
        ActuatorController::getInstance()->processEvents();
    });
    t.start(5);
    return a.exec();
}
