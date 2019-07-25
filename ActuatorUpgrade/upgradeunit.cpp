#include "upgradeunit.h"
#include "actuatorcontroller.h"
#include <QFile>
#include <QDebug>
#include <QString>
#include <QApplication>

//#include <data_transition.h>

UpgradeUnit::UpgradeUnit(uint64_t actuatorId, QObject *parent):
    QObject(parent),
    m_actuatorId(actuatorId),
    m_sendIndex(0),
    m_checkRemain(0),
    m_state(Upgrade_Unkown),
    m_checksum(0),
    m_sectionRemain(0),
    m_ignoreNum(0)
{

    m_pSendTimer = new QTimer(this);
    connect(m_pSendTimer,&QTimer::timeout,this,&UpgradeUnit::sendData);
    m_pWaitCheckTimer = new QTimer(this);
    connect(m_pWaitCheckTimer,&QTimer::timeout,[=]{changeState(Upgrade_Timeout);});
    m_pWaitCheckTimer->setInterval(10000);
}

void UpgradeUnit::Retry()
{
    m_sendIndex = 0;
    changeState(Upgrade_Ready);
    startUpgrade();
}

void UpgradeUnit::startUpgrade()
{
//    QTimer * pTest = new QTimer(this);
//    pTest->setInterval(1);
//    connect(pTest,&QTimer::timeout,[=]{ActuatorController::getInstance()->regainAttrbute(m_actuatorId,qAbs(qrand())%Actuator::DATA_CNT);});
//    pTest->start();
//    return;
    QFile file(QApplication::applicationDirPath()+tr("\\%1.txt").arg(ActuatorController::toByteId(m_actuatorId)));
    m_upgradeData.clear();
    if(file.open(QFile::ReadOnly | QFile::Text))
    {
//        file.readLine();
//        QString content = file.readAll();
//        content.remove(' ');
//        content.remove('\n');
        /*430文件的读取和转换.因430文件和txt文件不同*/
        QString content;
        QByteArray buf, buf_addr, Buffer, Buffer_Addr, Buffer_Size;
        int buf_length[10] = {0};
        int Buffer_Length[10] = {0};
        int length = 0;
        int sections = 0;

        while (!file.atEnd())
        {
            buf = file.read(1);

            if (buf[0] == 'q')
            {
                buf_length[sections -1 ] = length;
                break;
            }
            if ( buf[0] == '@')
            {
                for (int i = 0; i < 4; i++)
                {
                    buf = file.read(1);
                    buf_addr = buf;
                    Buffer_Addr.append(buf_addr);
                }

                if (sections >= 1)
                {
                    buf_length[sections - 1] = length;
                    length = 0;
                }
                sections++;
            }
            else
            {

                if ((buf[0] != '\n') && (buf[0] != ' '))
                {
                    Buffer.append(buf[0]);
                    length++;
                }
            }
        }
        for (int uu = 0; uu < sections; uu++)
        {
            Buffer_Length[uu] = (buf_length[uu] / 2);

            Buffer_Size.push_back(Buffer_Length[uu] >> 8);
            Buffer_Size.push_back(Buffer_Length[uu] & 0xff);
        }
        qDebug() << "data " << Buffer << endl;
        qDebug() << "address " << Buffer_Addr << endl;
        for(int cd = 0; cd < sections; cd ++)
        {
            qDebug() << "size " << Buffer_Length[cd] << endl;
        }
        qDebug() << "Buffer_Size " << Buffer_Size.toHex() << endl;

        int ii_addr = 0;    //给地址赋值的临时变量

        QString str;
        int seek = 0;
        char ad = '0';  //'0' 0x30
        /**/
        QByteArray add;
        add.push_back(ad);
        /**/
        for (int ii = 0; ii < sections; ii++)
        {
            //Size  2byte
            str = int_2_char(Buffer_Length[ii] / 2);
//            content.push_back(str);//
            content.push_back(str.at(2));
            content.push_back(str.at(3));
            content.push_back(str.at(0));
            content.push_back(str.at(1));
//            qDebug() << "str str " << content << endl;
            //Address   2byte  ,but dsp's address is 4 byte,
            content.push_back(Buffer_Addr.data()[ii_addr + 2]);
            content.push_back(Buffer_Addr.data()[ii_addr + 3]);
            content.push_back(Buffer_Addr.data()[ii_addr + 0]);
            content.push_back(Buffer_Addr.data()[ii_addr + 1]);
            //
            content.push_back(add.data()[0]);
            content.push_back(add.data()[0]);
            content.push_back(add.data()[0]);
            content.push_back(add.data()[0]);
            ii_addr += 4;
            //Data
            for (int uu = 0; uu < (Buffer_Length[ii] * 2); uu++)
            {
                content.push_back(Buffer.data()[seek]);
                seek++;
            }
        }
//        char ad = '0';  //'0' 0x30
        //DSP.txt里的  AA 08 16字节 0x00 4字节 地址 这里的地址用不到 也用00
        for (int aa = 0; aa < 44; aa++)    //对应下面的m_sectionRemain = 22; 这里这22个字节是保留位用不到
        {
//            content.push_front(&ad);
            content.push_front(add.data()[0]);
        }
        //DSP.txt 结尾的 00 00
//        for (int hh = 0; hh < 4; hh++)
        for (int hh = 0; hh < (4 + 2); hh++)
        {
            content.push_back(add.data()[0]);
        }

        qDebug() << "contect" << content << endl;

//        /*430文件的读取和转换.因430文件和txt文件不同*/

        m_upgradeData = content.toLocal8Bit();

        if(m_sendIndex == 0)
        {
            //m_ignoreNum = -0xaa-0x08;
            //先发送启动升级指令
            std::vector<uint8_t> data;
            data.push_back(0x55);
            data.push_back(0x08);
            /*******************************************************************************/
//            ActuatorController::getInstance()->upgradeActuator(ActuatorController::toByteId(m_actuatorId),data);                            //!!!对应DSPv_0.5 发了两个0x97 0x5508
//            ActuatorController::getInstance()->upgradeActuator(ActuatorController::toByteId(m_actuatorId),data);                            //!!!
           ActuatorController::getInstance()->upgrade430(ActuatorController::toByteId(m_actuatorId),data);                                    //对应DSPv_0.5版本后的
           /********************************************************************************/
            m_sectionRemain = 22;
//            m_sectionRemain = 2;
            upgrade(m_sectionRemain);
        }
    }
    else
    {
        changeState(Upgrade_Nofile);
    }
    file.close();

}

void UpgradeUnit::continueUpgrade()
{
    if(m_sendIndex < m_upgradeData.size()-2 && currentState()==Upgrade_Ready)//原来的
//    if(m_sendIndex < m_upgradeData.size() && currentState()==Upgrade_Ready)//Zhang 改的
    {
        int nIdx = m_sendIndex;
//        uint8_t send = m_upgradeData.at(nIdx);
//        while ((send==' '|| send=='\n') && m_sendIndex < m_upgradeData.size()) {
//            ++ nIdx;
//        }
        if(m_sectionRemain > 0)
        {
//            upgrade(m_sectionRemain>2048?2048:m_sectionRemain);//最多2048个字节就要校验一次//
            upgrade(m_sectionRemain>512?512:m_sectionRemain);//最多512个字节就要校验一次    430 每512字节一块
        }
        else if(nIdx < m_upgradeData.size()-4)//原来的
//        else if(nIdx < m_upgradeData.size()-2)//Zhang 改的
        {
            uint16_t low = m_upgradeData.mid(m_sendIndex,2).toUInt(nullptr,16);
            uint16_t hight = m_upgradeData.mid(m_sendIndex+2,2).toUInt(nullptr,16);

            m_sectionRemain = (low + hight*(1<<8))*2;
            qDebug("len %x %x",low,hight);
            if(m_sectionRemain > 0)
            {
                m_sectionRemain += 6;//有数据发送加上地址和长度信息//
//                upgrade(m_sectionRemain>2054?2054:m_sectionRemain);//最多2048个字节就要校验一次,但是加了2字节长度和四字节地址
                upgrade(m_sectionRemain>(512 + 2 + 4)?(512 + 2 + 4):m_sectionRemain);//最多512个字节就要校验一次,但是加了2字节长度和4字节地址
            }
            else
            {
                //升级完成，发送最后00字节
                std::vector<uint8_t> data;
                data.push_back(0);
                data.push_back(0);
//                ActuatorController::getInstance()->upgradeActuator(ActuatorController::toByteId(m_actuatorId),data);
                ActuatorController::getInstance()->upgrade430(ActuatorController::toByteId(m_actuatorId),data);//430
                qDebug() << "In the end, send : " << data << endl;//z
                changeState(Upgrade_Finished);
                emit upgradePercent(100);
            }

        }
        else{//添加了下面的代码 原来没有
//            changeState(Upgrade_Finished);
//            upgradePercent(100);
//            sleep_msec(2000);
//            std::vector<uint8_t> dataa;
//            dataa.push_back(0);
//            dataa.push_back(0);
////                ActuatorController::getInstance()->upgradeActuator(ActuatorController::toByteId(m_actuatorId),data);
//            ActuatorController::getInstance()->upgrade430(ActuatorController::toByteId(m_actuatorId),dataa);//430
//            changeState(Upgrade_Finished);
//            emit upgradePercent(100);
        }
    }
}

bool UpgradeUnit::checkSum(uint16_t checksum)
{
    m_pWaitCheckTimer->stop();
    if(m_checksum != checksum)
    {
        changeState(Upgrade_Failed);
        qDebug() << "check failed!" << m_checksum << checksum;
        return false;
    }
    else
    {
        if(m_sendIndex < m_upgradeData.size())
        {
            changeState(Upgrade_Ready);
        }
        else {
            changeState(Upgrade_Finished);
        }
        return true;
    }
    return false;
}


void UpgradeUnit::sendData()
{
    if(m_sendIndex < m_upgradeData.size())
    {

        uint16_t send = m_upgradeData.mid(m_sendIndex,4).toUInt(nullptr,16);
        m_sendIndex += 4;
//        m_checksum += send;
        m_sectionRemain -= 2;
        std::vector<uint8_t> data;
        data.push_back(send>>8);
        data.push_back(send&0xff);
        if(m_ignoreNum < 0)
        {
            m_ignoreNum += data.at(0);
            m_ignoreNum += data.at(1);
        }
        else if( m_ignoreNum == 0)
        {
            m_checksum += data.at(0);
            m_checksum += data.at(1);
        }
        else {
            qDebug() << "ignore error " << m_ignoreNum;
            return ;
        }
        //send data
//        ActuatorController::getInstance()->upgradeActuator(ActuatorController::toByteId(m_actuatorId),data);
        ActuatorController::getInstance()->upgrade430(ActuatorController::toByteId(m_actuatorId),data);//430
        qDebug("Upgrade send %x %x %d",data.at(0),data.at(1),m_checkRemain);
        m_checkRemain -= 2;
        if(m_checkRemain == 0)
        {
            m_pSendTimer->stop();//stop and wait for checking
            changeState(Upgrade_WaitingACK);
            m_pWaitCheckTimer->start();
            emit upgradePercent(m_sendIndex*100/m_upgradeData.size());
        }

    }

}

void UpgradeUnit::upgrade(uint32_t nLen)
{
    m_checkRemain = nLen;
    m_pSendTimer->start(1);
    changeState(Upgrade_Sending);
}

void UpgradeUnit::changeState(uint8_t state)
{
    m_state = state;
    emit stateChanged(state);
}

QString UpgradeUnit::int_2_char(int data_int1)
{
    QString str;
    QByteArray buf,buf_1;
//    data_int1 = 0x1234;
    buf.push_back((data_int1 >> 12) & 0x000f);
    buf.push_back((data_int1 >> 8) & 0x000f);
    buf.push_back((data_int1 >> 4) & 0x000f);
    buf.push_back(data_int1 & 0x000f);

    for (int i = 0; i < 4; i++)
    {
        if (buf.data()[i] >= 0x00 && buf.data()[i] <= 0x09)
        {
            buf_1.push_back(buf[i] + 0x30);
        }
        else if (buf.data()[i] >= 0x0a && buf.data()[i] <= 0x0f)
        {
            buf_1.push_back((buf[i] - 0x0a) + 0x61);
        }
//        qDebug() << buf_1 << endl;
    }
    str.push_back(buf_1);
    qDebug() << "str " << str << endl;;
    return str;
}

void UpgradeUnit::sleep_msec(unsigned int msec)
{
    QTime reachTime = QTime::currentTime().addMSecs(msec);
    while(QTime::currentTime() < reachTime)
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}
