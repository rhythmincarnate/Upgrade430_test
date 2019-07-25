#include "actuatorupgrade.h"
#include "ui_actuatorupgrade.h"
#include "actuatorcontroller.h"
#include "upgradeunit.h"
#include <QFile>
#include <QMessageBox>
#include "upgradeform.h"
#include <QGridLayout>

#include <qdebug.h>
//#include <innfosproxy.h>
#include <qstring.h>
#include <qbytearray.h>
//uint16_t Version_430;
//__declspec(dllimport) uint16_t Version_430;

ActuatorUpgrade::ActuatorUpgrade(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ActuatorUpgrade)
{
    ui->setupUi(this);
    m_pController = ActuatorController::getInstance();
    m_pController->m_sOperationFinished->connect_member(this,&ActuatorUpgrade::actuatorOperation);
//    m_pController->m_upgradeCheck->connect_member(this,&ActuatorUpgrade::onUpgradeCheck);//
    m_pController->m_upgrade430Check->connect_member(this,&ActuatorUpgrade::onUpgradeCheck);//change for check_430
    m_pController->m_upgrade430Version->connect_member(this,&ActuatorUpgrade::onUpgradeVersion);//Zhang ADD

    QGridLayout * pLayout = new QGridLayout();
    ui->centralWidget->setLayout(pLayout);

//    /**/


}

ActuatorUpgrade::~ActuatorUpgrade()
{
    delete ui;
}

void ActuatorUpgrade::actuatorOperation(quint8 nId, quint8 nType)
{
    switch (nType) {
    case Actuator::Recognize_Finished:
        initUpgradeInfo();
        break;
    default:
        break;
    }
}

void ActuatorUpgrade::onUpgradeCheck(uint64_t nId, uint16_t checkNum)
{
//    QByteArray qbyte_version_430;//Zhang ADD
//    qbyte_version_430.push_back(m_pController->Ver_430);//Zhang ADD
//    qbyte_version_430.push_front(m_pController->Ver_430 >> 8);//Zhang ADD
//    qDebug() << "                                                      " << m_pController->Ver_430 << endl;
    qDebug() << "in onUpgradeCheck in onUpgradeCheck in onUpgradeCheck in onUpgradeCheck in onUpgradeCheck" << endl;
    qDebug() << "my checkNum" << checkNum << endl;
    for(int i=0;i<m_upgradeUnits.size();++i)
    {
        UpgradeUnit * pUnit  = m_upgradeUnits.at(i);
        if(pUnit->actuatorLongId() == nId)
        {
            if(pUnit->checkSum(checkNum))
            {
                pUnit->continueUpgrade();
//                Zhang ADD
//                ui->lineEdit_version->setText(qbyte_version_430.toHex());//Zhang ADD
            }
        }
    }
}

/*Zhang ADD*/
void ActuatorUpgrade::onUpgradeVersion(uint64_t nId, uint16_t version_430)
{
    QByteArray qbyte_version_430;//Zhang ADD
//    qbyte_version_430.push_back(m_pController->Ver_430);//Zhang ADD
//    qbyte_version_430.push_front(m_pController->Ver_430 >> 8);//Zhang ADD
    qbyte_version_430.push_back(version_430);//Zhang ADD
    qbyte_version_430.push_front(version_430 >> 8);//Zhang ADD
    ui->lineEdit_version->setText(qbyte_version_430.toHex());//Zhang ADD
}
/*ZhangADD*/

void ActuatorUpgrade::onUpgradeUnitStateChanged(uint8_t status)
{
//    UpgradeUnit * pUnit = qobject_cast<UpgradeUnit *>(sender());
//    switch (status) {
//    case UpgradeUnit::Upgrade_Timeout:
//    case UpgradeUnit::Upgrade_Failed:
//        QMessageBox::critical(this,"Error",tr("Id %1 upgrade failed! Error code %2 , please try again!")
//                              .arg(ActuatorController::toByteId(pUnit->actuatorLongId())).arg(status));
//        break;
//    case UpgradeUnit::Upgrade_Finished:
//        QMessageBox::information(this,"Tip",tr("Id %1 upgrade success!").arg(ActuatorController::toByteId(pUnit->actuatorLongId())));
//        break;
//    default:
//        break;
//    }
}

void ActuatorUpgrade::initUpgradeInfo()
{
    std::vector<uint8_t> idArray = m_pController->getActuatorIdArray();
    int nCount = 0;
    QGridLayout * pLayout = qobject_cast<QGridLayout *>(ui->centralWidget->layout());
    for(auto i : idArray)
    {
//        m_pController->getActuatorAttributeWithACK(i,Actuator::MODE_ID);

        UpgradeUnit *pUnit = new UpgradeUnit(ActuatorController::toLongId(i),this);
        m_upgradeUnits.push_back(pUnit);
        connect(pUnit,&UpgradeUnit::stateChanged,this,&ActuatorUpgrade::onUpgradeUnitStateChanged);
        UpgradeForm * pForm = new UpgradeForm(i);
        pLayout->addWidget(pForm,nCount/4,nCount%4);
        connect(pUnit,&UpgradeUnit::stateChanged,pForm,&UpgradeForm::onStateChanged);
        connect(pUnit,&UpgradeUnit::upgradePercent,pForm,&UpgradeForm::upgradePercent);
        connect(pForm,&UpgradeForm::startUpgrade,[=]{pUnit->startUpgrade();});
//            pUnit->startUpgrade();
        ++nCount;

    }
}

void ActuatorUpgrade::on_allUpgrade_clicked()
{
    QList<UpgradeForm *> list = ui->centralWidget->findChildren<UpgradeForm *>();
    foreach(UpgradeForm * form , list)
    {
        form->upgrade();
    }
    ui->allUpgrade->setEnabled(false);
}
