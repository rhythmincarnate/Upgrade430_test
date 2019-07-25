#ifndef UPGRADEUNIT_H
#define UPGRADEUNIT_H
#include <QString>
#include <QObject>
#include <QTimer>

#include <QTime>
#include <QApplication>
#include <QEventLoop>



class UpgradeUnit : public QObject
{
    Q_OBJECT
public:
    enum{
        Upgrade_Ready,
        Upgrade_Sending,
        Upgrade_WaitingACK,
        Upgrade_Finished,
        Upgrade_Failed,
        Upgrade_Unkown,
        Upgrade_Error,
        Upgrade_Timeout,
        Upgrade_Nofile,
    };
    UpgradeUnit(uint64_t actuatorId,QObject * parent=nullptr);
    uint64_t actuatorLongId()const{return m_actuatorId;}
    uint8_t currentState()const{return m_state;}
    void Retry();
    void startUpgrade();
    void continueUpgrade();
    bool checkSum(uint16_t checksum);

    /*Z*/
    QString int_2_char(int data_int);
    void sleep_msec(unsigned int msec);

private slots:
    void sendData();
private:
    void upgrade(uint32_t nLen);
    void changeState(uint8_t state);
signals:
    void stateChanged(uint8_t state);
    void upgradePercent(int percent);
private:
    QByteArray m_upgradeData;
    uint32_t m_sendIndex;
    uint32_t m_checkRemain;
    uint32_t m_sectionRemain;
    uint64_t m_actuatorId;
    uint8_t m_state;
    uint16_t m_checksum;
    QTimer * m_pSendTimer;
    QTimer * m_pWaitCheckTimer;
    int m_ignoreNum;//需要忽略的总和
};

#endif // UPGRADEUNIT_H
