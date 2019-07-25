#ifndef ACTUATORUPGRADE_H
#define ACTUATORUPGRADE_H

#include <QMainWindow>
#include <QWidget>//z

class ActuatorController;
class UpgradeUnit;
namespace Ui {
class ActuatorUpgrade;
//class QMainWindow;//z
}

class ActuatorUpgrade : public QMainWindow
{
    Q_OBJECT

public:
    explicit ActuatorUpgrade(QWidget *parent = 0);
    ~ActuatorUpgrade();
    void actuatorOperation(quint8 nId, quint8 nType);
    void onUpgradeCheck(uint64_t nId,uint16_t checkNum);
    void onUpgradeVersion(uint64_t nId,uint16_t version_430);//Zhang ADD 读取430的版本号
public slots:
    void onUpgradeUnitStateChanged(uint8_t status);
private slots:
    void on_allUpgrade_clicked();

private:
    void initUpgradeInfo();
private:
    Ui::ActuatorUpgrade *ui;
    ActuatorController * m_pController;
    QVector<UpgradeUnit * > m_upgradeUnits;
};

#endif // ACTUATORUPGRADE_H
