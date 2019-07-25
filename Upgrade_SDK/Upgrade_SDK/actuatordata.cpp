#include "actuatordata.h"
#include "mediator.h"
#include "innfosproxy.h"
#include "dataUtil.h"
#include "itimer.h"
#include <functional>
#include "stringformat.h"
#include <iostream>
#include <thread>
#include "actuatorcontroller.h"
using namespace std;
using namespace Actuator;
#define UNINITIALIZE_VALUE 0
#define TRY_CNT 3//

ActuatorAttribute unRevisable[] = {VOLTAGE,ACTUATOR_TEMPERATURE,INVERTER_TEMPERATURE,CURRENT_SCALE,VELOCITY_SCALE,FIRMWARE_VERSION,
                                  SN_ID,CUMULATIVE_TIME,COMMUNICATION_ID};

const int nMaxErrorCnt = 30;
ActuatorData::ActuatorData(const uint8_t nDeviceId, const uint32_t nDeviceMac,const uint32_t ipAddress) :
    m_nHeartFailCnt(0),
    m_nAutoRequestInterval(1000)
{
    initData();
    userRequestValue(DEVICE_ID,nDeviceId);
    userRequestValue(SN_ID,nDeviceMac);
    userRequestValue(COMMUNICATION_ID,ipAddress);
    setValue(DEVICE_ID,nDeviceId);
    setValue(SN_ID,nDeviceMac);
    setValue(COMMUNICATION_ID,ipAddress);
    m_pHeartTimer = new ITimer(1000,[=]{
        if(!isOnline())
        {
            if(getValue(ONLINE_STATUS) != Actuator::Status_Offline)
            {
                setValue(ONLINE_STATUS,Actuator::Status_Offline);
            }

            m_pValueTimer->stop();

        }
        else
        {

            if((uint8_t)getValue(Actuator::ACTUATOR_SWITCH)==Actuator::ACTUATOR_SWITCH_ON)//if error occur, do not check error again
                InnfosProxy::SendProxyWithLongId(longId(),D_CHECK_ERROR);
            ++ m_nHeartFailCnt;
#ifdef HAS_BRAKE
            InnfosProxy::SendProxyWithLongId(longId(),D_READ_ACTUATOR_BRAKE);
#endif
        }
        if(m_nHeartFailCnt > TRY_CNT-1)//只有当检测可能掉线才发心跳，如果检测到有数据收发默认在线
            InnfosProxy::SendProxyWithLongId(longId(),D_HANDSHAKE);
    },*mediator->ioContext());
    m_pValueTimer = new ITimer(m_nAutoRequestInterval,std::bind(&ActuatorData::requestActualValue,this),*mediator->ioContext());
}

ActuatorData::~ActuatorData()
{
    m_pHeartTimer->destroy();
    m_pValueTimer->destroy();
}


bool ActuatorData::deviceIdIsAvailable() const
{
    return m_userRequestData[DEVICE_ID] == m_motorData[DEVICE_ID];
}


void ActuatorData::changeDemand(double value)
{
    switch ((int)getValue(MODE_ID)) {
    case Mode_Cur:

        setValueByUser(CUR_IQ_SETTING,value);
        break;
    case Mode_Vel:
    case Mode_Profile_Vel:

        setValueByUser(VEL_SETTING,value);
        break;
    case Mode_Pos:
    case Mode_Profile_Pos:

        setValueByUser(POS_SETTING,value);
        break;
    default:
        break;
    }
}

void ActuatorData::switchAutoRequestActual(bool bStart)
{
    if(bStart)
    {
        m_pValueTimer->start(m_nAutoRequestInterval);
        //m_pHeartTimer->start(1000);
    }
    else
    {
        m_pValueTimer->stop();
        //m_pHeartTimer->stop();
    }
}

void ActuatorData::setAutoRequestInterval(uint32_t mSec)
{
    m_nAutoRequestInterval = mSec;
    m_pValueTimer->stop();
    m_pValueTimer->start(m_nAutoRequestInterval);
}

void ActuatorData::saveAllParams()
{
    InnfosProxy::SendProxyWithLongId(longId(),D_SAVE_PARAM);
}

void ActuatorData::clearHomingInfo()
{
    InnfosProxy::SendProxyWithLongId(longId(),D_CLEAR_HOMING);
}

void ActuatorData::setHomingOperationMode(const uint8_t nMode)
{
    InnfosProxy::SendProxyWithLongId(longId(),D_SET_HOMING_OPERATION,nMode);
}

void ActuatorData::openChartChannel(const int nChannelId)
{
    int nProxyId[Actuator::channel_cnt] = {D_CHANNEL1_OPEN,D_CHANNEL2_OPEN,D_CHANNEL3_OPEN,D_CHANNEL4_OPEN};
    InnfosProxy::SendProxyWithLongId(longId(),nProxyId[nChannelId]);
}

void ActuatorData::closeChartChannel(const int nChannelId)
{
    int nProxyId[Actuator::channel_cnt] = {D_CHANNEL1_CLOSE,D_CHANNEL2_CLOSE,D_CHANNEL3_CLOSE,D_CHANNEL4_CLOSE};
    InnfosProxy::SendProxyWithLongId(longId(),nProxyId[nChannelId]);
}

void ActuatorData::switchChartAllChannel(bool bOn)
{
    uint8_t nProxyId = bOn?D_CHART_OPEN:D_CHART_CLOSE;
    InnfosProxy::SendProxyWithLongId(longId(),nProxyId);
}

void ActuatorData::switchCalibrationVel(uint8_t nValue)
{
    InnfosProxy::SendProxyWithLongId(longId(),D_SWITCH_CALIBRATION_VEL,nValue);
}

void ActuatorData::switchCalibration(uint8_t nValue)
{
    InnfosProxy::SendProxyWithLongId(longId(),D_SWITCH_CALIBRATION,nValue);
}

void ActuatorData::startCalibration()
{
    InnfosProxy::SendProxyWithLongId(longId(),D_START_CALIBRATION);
}

void ActuatorData::requestSuccessfully(uint8_t nDataId)
{

    if(DATA_CNT > nDataId)
    {
        //qDebug() << "success" << nDataId << m_userRequestData[nDataId];
        setValue(nDataId,m_userRequestData[nDataId]);
    }
}

void ActuatorData::readStatus()
{
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_LOADER_VERSION);
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_MOTORS_SWITCH);
}

void ActuatorData::setValue(int nDataId, double value, bool bEmitSignal)
{
    if(DATA_CNT > nDataId)
    {
        if(nDataId == DEVICE_ID)
        {
            if(bEmitSignal)//先发送信号，避免修改id以后无法通知关联旧id的对象
                motorDataMgrInstance->dataChanged(longId(),(ActuatorAttribute)nDataId,
                                                  Mediator::toLongId(Mediator::toCommunicationId(longId()),(uint8_t)value));
            m_motorData[nDataId] = value;
        }
        else
        {
            m_motorData[nDataId] = value;
            if(bEmitSignal)
                motorDataMgrInstance->dataChanged(longId(),(ActuatorAttribute)nDataId,value);
        }
        m_dataStatus[nDataId] = DATA_NORMAL;
        //qDebug() << "set" << nDataId << m_motorData[nDataId];
    }
}

void ActuatorData::userRequestValue(int nDataId, double value)
{
    if(DATA_CNT > nDataId)
    {
        m_userRequestData[nDataId] = value;
        m_dataStatus[nDataId] = DATA_WAIT;
    }
}

void ActuatorData::initData()
{
    for(int i=CUR_IQ_SETTING;i<DATA_CNT;++i)
    {
        if(i != DEVICE_ID && i!= SN_ID && i!=COMMUNICATION_ID)//一旦初始化，再次初始化不要更改某些参数
        {
            m_motorData[i] = UNINITIALIZE_VALUE;
            m_userRequestData[i] = UNINITIALIZE_VALUE;
        }
        m_dataStatus[i] = DATA_WAIT;
    }
    userRequestValue(MODE_ID,Mode_Cur);
    userRequestValue(CURRENT_SCALE,curScale);
    userRequestValue(VELOCITY_SCALE,velScale);
    userRequestValue(ERROR_ID,Actuator::ERR_NONE);
    userRequestValue(CUR_IQ_SETTING,0);
    userRequestValue(CUR_ID_SETTING,0);
    userRequestValue(POS_SETTING,0);
    userRequestValue(VEL_SETTING,0);

    setValue(CURRENT_SCALE,curScale,false);
    setValue(VELOCITY_SCALE,velScale,false);
    setValue(ERROR_ID,Actuator::ERR_NONE,false);
    setValue(MODE_ID,Mode_Cur,false);
    setValue(CUR_IQ_SETTING,0,false);
    setValue(CUR_ID_SETTING,0,false);
    setValue(POS_SETTING,0,false);
    setValue(VEL_SETTING,0,false);
    setValue(ONLINE_STATUS,0);
}


void ActuatorData::activeModeSuccessfully()
{
    setValue(Actuator::ACTUATOR_BRAKE,1);//close break
    switch ((int)getValue(MODE_ID)) {
    case Mode_Cur:
        setValueByUser(CUR_IQ_SETTING,0,false);//reset demand
        break;
    case Mode_Vel:
        setValueByUser(VEL_SETTING,0,false);//reset demand
        break;
    case Mode_Pos:
        break;
    case Mode_Profile_Pos:
        break;
    case Mode_Profile_Vel:
        //setValueByUser(VEL_SET,0,false);//reset demand
        break;
    default:
        break;
    }

}

void ActuatorData::requestActualValue()
{
    if(getValue(Actuator::FIRMWARE_VERSION) >= 0x0401)
    {
        InnfosProxy::SendProxyWithLongId(longId(),D_READ_ACTUAL_CVP);
    }
    else
    {
        InnfosProxy::SendProxyWithLongId(longId(),D_READ_CUR_CURRENT);
        InnfosProxy::SendProxyWithLongId(longId(),D_READ_CUR_VELOCITY);
        InnfosProxy::SendProxyWithLongId(longId(),D_READ_CUR_POSITION);
    }
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_VOLTAGE);
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_TEMP_INVERTER);
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_TEMP_MOTOR);
}

void ActuatorData::setValidValue(const int nProxyId, double value)
{
    Directives proxy = (Directives)nProxyId;
    ActuatorAttribute id = DataUtil::convertToMotorDataId(proxy);
    if(id != DIRECTIVES_INVALID)
    {
        setValue(id,value);
    }
}

void ActuatorData::saveData()
{
//    QFileDialog dialog(nullptr,tr("Save as"),QDir::currentPath());
//    dialog.setFileMode(QFileDialog::AnyFile);
//    dialog.setAcceptMode(QFileDialog::AcceptSave);
//    dialog.setNameFilter(tr("Innfos(*.innfos)"));
//    if(dialog.exec() == QDialog::Accepted)
//    {
//        std::string path = dialog.selectedFiles().first();
//        if(path.size() > 0)
//        {
//            saveDataToFile(path);
//        }

//    }
}

void ActuatorData::loadData()
{
//    QFileDialog dialog(nullptr,tr("Load"),QDir::currentPath());
//    dialog.setFileMode(QFileDialog::ExistingFile);
//    dialog.setNameFilter(tr("Innfos(*.innfos)"));
//    if(dialog.exec() == QDialog::Accepted)
//    {
//        std::string path = dialog.selectedFiles().first();
//        if(path.size() > 0)
//        {
//            readDataFromFile(path);
//        }

//    }
}

void ActuatorData::reconnect()
{
    m_nHeartFailCnt = 0;
    if(m_pHeartTimer)
    {
        m_pHeartTimer->start();
    }
    if(m_pValueTimer)
    {
        m_pValueTimer->start();
    }
}

ActuatorMode ActuatorData::currentMode() const
{
    return (Actuator::ActuatorMode)((int)getValue(MODE_ID));
}

//void ActuatorData::saveDataToFile(std::string fileName)//todo
//{
//    QFile file(fileName);
//    if(!file.open(QFile::WriteOnly | QFile::Text | QFile::Truncate))
//    {
//        //qDebug() << tr("Cannot write file %1 %2").arg(fileName).arg(file.errorString());
//        return;
//    }

////    QJsonArray arr;
//////    for(int i=0;i<DATA_CNT;++i)
//////    {
//////        QJsonObject attr;
//////        attr[tr("Attr%1").arg(i)] = m_motorData[i];
//////        arr.emplace_back(attr);
//////    }
////    QJsonDocument saveDoc(arr);
////    file.write(saveDoc.toJson());
//    file.close();
//}

//void ActuatorData::readDataFromFile(std::string fileName)
//{
//    QFile file(fileName);
//    if(file.open(QIODevice::ReadOnly | QIODevice::Text))
//    {
//        QXmlStreamReader reader(&file);
//        while(!reader.atEnd())
//        {
//            QXmlStreamReader::TokenType nType = reader.readNext();
//            switch (nType)
//            {
//            case QXmlStreamReader::StartDocument:

//                break;
//            case QXmlStreamReader::StartElement:
//            {
//                std::string strElementName = reader.name().toString();
//                if(strElementName == "ServoData")
//                {
//                    QXmlStreamAttributes attributes = reader.attributes();
//                    if(attributes.hasAttribute("Version") && attributes.value("Version").toString() == "1.0")
//                    {
//                        readParams(&reader);
//                    }
//                }
//            }
//                break;
//            default:
//                break;
//            }
//        }
//        if(reader.hasError())
//        {
//            //qDebug() << tr("Errorinfo:%1 line:%2 column:%3 character offset:%4").arg(reader.errorString()).arg(reader.lineNumber()).arg(reader.columnNumber()).arg(reader.characterOffset());
//        }
//        file.close();
//    }


//}

//void ActuatorData::readParams(QXmlStreamReader *reader)
//{
//    while (!reader->atEnd()) {
//        reader->readNext();
//        if(reader->isStartElement())
//        {
//            std::string strElementName = reader->name().toString();
//            int nIdx = strElementName.indexOf("Attr");
//            if(nIdx >=0)
//            {
//                int nAttrId = strElementName.right(strElementName.size()-4);
//                if(nAttrId>=0 && nAttrId<DATA_CNT)
//                {
//                    double readValue = reader->readElementText().toDouble();
//                    if(m_motorData[nAttrId] != readValue)
//                    {
//                        m_motorData[nAttrId] = readValue;
//                        //emit dataChange((Motor_Data_Id)nAttrId);to do
//                    }

//                }
//            }
//        }
//    }
//}




uint8_t ActuatorData::deviceId() const
{
//    if(m_oldDeviceId >=0)
//        return m_oldDeviceId;
    return (uint8_t)getValue(DEVICE_ID);
}

uint64_t ActuatorData::longId() const
{
    return Mediator::toLongId(m_motorData[COMMUNICATION_ID],deviceId());
}

uint64_t ActuatorData::requestDeviceId() const
{
    return Mediator::toLongId(m_motorData[COMMUNICATION_ID],m_userRequestData[DEVICE_ID]);
}

uint32_t ActuatorData::deviceMac() const
{
    return (uint32_t)getValue(SN_ID);
}

bool ActuatorData::isRevisable(ActuatorAttribute nDataId) const
{

    for(int i=0;i<sizeof(unRevisable)/sizeof(ActuatorAttribute);++i)
    {
        if(unRevisable[i] == nDataId)
        {
            //std::cout << "Attribute " << nDataId << " can not be changed!" << std::endl;
            return false;
        }
    }
    return true;
}

void ActuatorData::setValueByProxy(const int nProxyId, double value)
{
    //responseHeart(true);
    if(!isOnline())
        responseHeart(true);
    else
        m_nHeartFailCnt = 0;

    switch (nProxyId) {
    case D_CHECK_ERROR:
    {
        if((uint16_t)value == (uint16_t)getValue(ERROR_ID))
            return;
        setValue(ERROR_ID,value);
        std::map<int,std::string> errorInfo;
        errorInfo.insert({Actuator::ERR_ACTUATOR_OVERVOLTAGE,string_format("ID:%d voltage:%f Overvoltage error!",deviceId(),getValue(VOLTAGE))});
        errorInfo.insert({Actuator::ERR_ACTUATOR_UNDERVOLTAGE,string_format("ID:%d voltage:%f Undervoltage error!",deviceId(),getValue(VOLTAGE))});
        errorInfo.insert({Actuator::ERR_ACTUATOR_LOCKED_ROTOR,string_format("ID:%d Locked-rotor error!",deviceId())});
        errorInfo.insert({Actuator::ERR_ACTUATOR_OVERHEATING,string_format("ID:%d Temperture:%f Overheating error!",deviceId(),getValue(ACTUATOR_TEMPERATURE))});
        errorInfo.insert({Actuator::ERR_ACTUATOR_READ_OR_WRITE,string_format("ID:%d Read or write params error!",deviceId())});
        errorInfo.insert({Actuator::ERR_ACTUATOR_MULTI_TURN,string_format("ID:%d Multi-turn count error!",deviceId())});
        errorInfo.insert({Actuator::ERR_INVERTOR_TEMPERATURE_SENSOR,string_format("ID:%d Invertor temperature sensor error!",deviceId())});
        errorInfo.insert({Actuator::ERR_ACTUATOR_TEMPERATURE_SENSOR,string_format("ID:%d Motor temperature sensor error!",deviceId())});
        errorInfo.insert({Actuator::ERR_CAN_COMMUNICATION,string_format("ID:%d Can communication error!",deviceId())});
        errorInfo.insert({Actuator::ERR_DRV_PROTECTION,string_format("ID:%d DRV protection error!",deviceId())});
        errorInfo.insert({Actuator::ERR_STEP_OVER,string_format("ID:%d Step is too big!",deviceId())});
        errorInfo.insert({Actuator::ERR_CODER_DISABLED,string_format("ID:%d Coder is disabled!",deviceId())});
        //std::list<int> keys = errorInfo.uniqueKeys();
        std::string errorStr;
        for(auto it=errorInfo.begin();it!=errorInfo.end();++it)
        {
            if((uint16_t)value&it->first)
            {
                errorStr += it->second;
                errorStr += "\n";
            }

        }
        if(errorStr.length() > 0)
        {
            motorDataMgrInstance->errorOccured(deviceId(),(uint16_t)value,errorStr);
        }

        m_errorHistory.emplace_back((uint16_t)value);
        regainData(Actuator::MODE_ID);
    }

        break;
    case D_READ_MOTORS_SWITCH:
        if((int)value == Actuator::ACTUATOR_SWITCH_ON && m_motorData[ACTUATOR_SWITCH]!=Actuator::ACTUATOR_SWITCH_ON)
        {
            requestAllValue();
        }
        setValue(ACTUATOR_SWITCH,(int)value);
        break;
    case D_READ_CURRENT_LIMIT:
        setValidValue(nProxyId,value*m_motorData[Actuator::CURRENT_SCALE]);
        break;
    case D_READ_VELOCITY_LIMIT:
        setValidValue(nProxyId,value*m_motorData[Actuator::VELOCITY_SCALE]);
        if((int)getValue(INIT_STATE) == Uninitlized)
        {
            setValue(INIT_STATE,Initlized);
        }
        std::cout << "read to velocity limit" << std::endl;
        break;
    case D_READ_RESERVE_0:
    case D_READ_RESERVE_1:
    case D_READ_RESERVE_2:
    case D_READ_RESERVE_3:
    case D_READ_RESERVE_4:
    case D_READ_RESERVE_5:
    case D_READ_RESERVE_6:
    case D_READ_RESERVE_7:
    case D_READ_RESERVE_8:
        setValidValue(nProxyId,value);
        //std::cout << "RESERVE" << deviceId() << " "<< nProxyId << " value " << value << std::endl;
        break;
    default:
        setValidValue(nProxyId,value);
        break;
    }
}

void ActuatorData::requestAllValue()
{
    setValue(MODE_ID,Mode_Cur);
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_VERSION);
//    activeMode(Mode_Cur);
//    for (int i=D_READ_CUR_CURRENT;i<=D_READ_CUR_POSITION;++i)
//    {
//        InnfosProxy::SendProxy(m_deviceId,i);
//    }

    for (int i=D_READ_CUR_CURRENT;i<=D_READ_CUR_POSITION;++i)
    {
        InnfosProxy::SendProxyWithLongId(longId(),i);
    }


    for (int i=D_READ_CUR_P;i<=D_READ_PROFILE_POS_DEC;++i)
    {
        InnfosProxy::SendProxyWithLongId(longId(),i);
    }

    for (int i=D_READ_PROFILE_VEL_MAX_SPEED;i<=D_READ_PROFILE_VEL_DEC;++i)
    {
        InnfosProxy::SendProxyWithLongId(longId(),i);
    }

    for (int i=D_READ_CURRENT_PID_MIN;i<=D_READ_POSITION_PID_MAX;++i)
    {
        InnfosProxy::SendProxyWithLongId(longId(),i);
    }

    for (int i=D_READ_CHART_THRESHOLD;i<=D_READ_CHART_FREQUENCY;++i)
    {
        InnfosProxy::SendProxyWithLongId(longId(),i);
    }
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_VOLTAGE);
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_CURRENT_SCALE);

    for (int i=D_READ_MAX_POS;i<=D_READ_MIN_POS;++i)
    {
        InnfosProxy::SendProxyWithLongId(longId(),i);
    }

    for (int i=D_READ_POS_OFFSET;i<=D_READ_HOMING_LIMIT;++i)
    {
        InnfosProxy::SendProxyWithLongId(longId(),i);
    }
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_LAST_STATE);
    //InnfosProxy::SendProxyWithLongId(longId(),D_READ_VERSION);
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_HOMING_CUR_MIN);
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_HOMING_CUR_MAX);

    InnfosProxy::SendProxyWithLongId(longId(),D_READ_FILTER_C_STATUS);
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_FILTER_C_VALUE);
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_FILTER_V_STATUS);
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_FILTER_V_VALUE);
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_FILTER_P_STATUS);
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_FILTER_P_VALUE);
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_INERTIA);
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_LOCK_ENERGY);
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_TEMP_MOTOR);
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_TEMP_INVERTER);
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_TEMP_PROTECT);
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_TEMP_RECOVERY);
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_INVERTER_TEMP_PROTECT);
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_INVERTER_TEMP_RECOVERY);
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_CALIBRATION_SWITCH);
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_CALIBRATION_ANGLE);
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_CURRENT_MAXSPEED);
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_MOTOR_MODE);
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_ACTUATOR_BRAKE);
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_CURRENT_LIMIT);
    InnfosProxy::SendProxyWithLongId(longId(),D_READ_VELOCITY_LIMIT);

    ITimer::singleShot(5000,[=]{
        if((int)getValue(INIT_STATE) == Uninitlized)
        {
            setValue(INIT_STATE,Initlized);
            std::cout << "Initlize timeout " << longId() << std::endl;
        }

    },*mediator->ioContext());
#ifndef NO_HEART_BEAT
    m_pHeartTimer->start(1000);
    //m_pValueTimer->start(m_nAutoRequestInterval);
#endif
}

double ActuatorData::getValue(Actuator::ActuatorAttribute nDataId) const
{
    //Q_ASSERT(nDataId < DATA_CNT);
    if(nDataId < DATA_CNT)
    {
        return m_motorData[nDataId];
    }
    return -1;
}

double ActuatorData::getUserRequestValue(Actuator::ActuatorAttribute nDataId) const
{
    //Q_ASSERT(nDataId < DATA_CNT);
    if(nDataId < DATA_CNT)
    {
        return m_userRequestData[nDataId];
    }
    return -1;
}

void ActuatorData::regainData(ActuatorAttribute nDataId)
{
    Directives proxyId = DataUtil::convertToReadProxyId(nDataId);
    if(proxyId != DIRECTIVES_INVALID)
    {
        InnfosProxy::SendProxyWithLongId(longId(),proxyId);
        m_dataStatus[nDataId] = DATA_WAIT;
    }
}

int ActuatorData::getDataStatus(ActuatorAttribute nDataId) const
{
    if(nDataId < DATA_CNT)
    {
        return m_dataStatus[nDataId];
    }
    return DATA_NONE;
}

void ActuatorData::requestFailed(ActuatorAttribute nDataId)
{
    if(nDataId < DATA_CNT)
    {
        m_dataStatus[nDataId] = DATA_FAILED;
    }
}

void ActuatorData::setValueByUser(Actuator::ActuatorAttribute nDataId, double value, bool bSendProxy)
{
    if (isRevisable(nDataId))
    {
        Directives pId = DataUtil::convertToSetProxyId(nDataId);
        //m_motorData[nDataId] = value;
        userRequestValue(nDataId,value);
        if(pId != DIRECTIVES_INVALID)
        {
            //
            if(bSendProxy)
            {
                switch (pId) {
                case D_SET_CHART_FREQUENCY:// these values are int16_t
                {
                    int16_t nValue = (int16_t)value;
                    InnfosProxy::SendProxyWithLongId(longId(),pId,nValue);
                }
                    break;
                case D_SET_LOCK_ENERGY://
                {
                    int32_t nValue = value*75.225;
                    InnfosProxy::SendProxyWithLongId(longId(),pId,nValue);
                }
                    break;
                case D_SET_FILTER_C_VALUE:
                case D_SET_FILTER_V_VALUE:
                case D_SET_FILTER_P_VALUE:
                case D_SET_TEMP_PROTECT:
                case D_SET_TEMP_RECOVERY:
                case D_SET_INVERTER_TEMP_PROTECT:
                case D_SET_INVERTER_TEMP_RECOVERY:
                {
                    int16_t nValue = value*(1<<8);
                    InnfosProxy::SendProxyWithLongId(longId(),pId,nValue);
                }
                    break;
                case D_SET_HOMING_LIMIT:
                case D_SET_FILTER_C_STATUS:
                case D_SET_FILTER_V_STATUS:
                case D_SET_FILTER_P_STATUS:
                case D_SWITCH_CALIBRATION:
                case D_SET_SWITCH_MOTORS:
                case D_SET_MODE:
                //case D_SET_DEVICE_ID:
                {
                    uint8_t invertValue = value;
                    InnfosProxy::SendProxyWithLongId(longId(),pId,invertValue);
                    ////qDebug()<<"send" << value.toInt() << getValue(ACTUATOR_SWITCH);
                }
                    break;
                case D_SET_DEVICE_ID:
                    InnfosProxy::SendProxyWithLongId(longId(),(uint32_t)getValue(Actuator::SN_ID),Actuator::D_SET_DEVICE_ID,uint8_t(value));
                    break;
                case D_CHECK_ERROR:
                    if(value == Actuator::ERR_NONE)//clear error
                    {
                        InnfosProxy::SendProxyWithLongId(longId(),D_CLEAR_ERROR);
                    }
                    break;
                case D_CHART_CLOSE:
                case D_CHART_OPEN:
                {
                    if(value == Actuator::CHART_SWITCH_ON)
                    {
                        InnfosProxy::SendProxyWithLongId(longId(),D_CHART_OPEN);
                    }
                    else
                    {
                        InnfosProxy::SendProxyWithLongId(longId(),D_CHART_CLOSE);
                    }
                }
                    break;
                case D_SET_CHART_THRESHOLD:
                {
                    int16_t nScaleValue = value*(1<<15);
                    InnfosProxy::SendProxyWithLongId(longId(),pId,nScaleValue);
                }
                    break;
                case D_SET_CURRENT:
                case D_SET_CURRENT_ID:
                {
                    int nScaleValue = value/m_motorData[CURRENT_SCALE]*(1<<24);
                    InnfosProxy::SendProxyWithLongId(longId(),pId,nScaleValue);
                }
                    break;
                case D_SET_VELOCITY:
                case D_SET_CURRENT_MAXSPEED:
                {
                    int nScaleValue = value/m_motorData[VELOCITY_SCALE]*(1<<24);
                    InnfosProxy::SendProxyWithLongId(longId(),pId,nScaleValue);
                }
                    break;
                case D_SET_PROFILE_POS_ACC:
                case D_SET_PROFILE_POS_DEC:
                case D_SET_PROFILE_POS_MAX_SPEED:
                case D_SET_PROFILE_VEL_ACC:
                case D_SET_PROFILE_VEL_DEC:
                case D_SET_PROFILE_VEL_MAX_SPEED:
                {
                    int nScaleValue = value*(1<<20)/60;
                    InnfosProxy::SendProxyWithLongId(longId(),pId,nScaleValue);
                }
                    break;
                case D_SET_CURRENT_LIMIT:
                {
                    int nScaleValue = value*(1<<24)/m_motorData[Actuator::CURRENT_SCALE];
                    InnfosProxy::SendProxyWithLongId(longId(),pId,nScaleValue);
                }
                    break;
                case D_SET_VELOCITY_LIMIT:
                {
                    int nScaleValue = value*(1<<24)/m_motorData[Actuator::VELOCITY_SCALE];
                    InnfosProxy::SendProxyWithLongId(longId(),pId,nScaleValue);
                }
                    break;
                default:
                {
                    int nScaleValue = value*(1<<24);
                    InnfosProxy::SendProxyWithLongId(longId(),pId,nScaleValue);
                }
                    break;
                }
            }
            else
            {
                //qDebug() << "user not send" << nDataId;
                requestSuccessfully(nDataId);//if do not need to send proxy, default is request successfully.
            }
        }
    }
}

void ActuatorData::responseHeart(bool bSuccess)
{
    if(bSuccess/* && isOnline()*/)
    {
        m_nHeartFailCnt = 0;
        if(getValue(ONLINE_STATUS) != Actuator::Status_Online)//这种情况只有断线重连才会判断成功，重连后m_nHeartFailCnt，但是电机数据在线状态还是断线
        {
            setValue(ONLINE_STATUS,Actuator::Status_Online);//
            InnfosProxy::SendProxyWithLongId(longId(),D_READ_MOTORS_SWITCH);
            InnfosProxy::SendProxyWithLongId(longId(),D_READ_MOTOR_MODE);
        }

    }

}

bool ActuatorData::isOnline() const
{
#ifdef MY_DEBUG
    return true;
#endif
    return m_nHeartFailCnt < 3;
}

//data manager

MotorDataMgr * MotorDataMgr::m_pMgr = nullptr;

MotorDataMgr* MotorDataMgr::getInstance()
{
    if(m_pMgr == nullptr)
        m_pMgr = new MotorDataMgr();
    return m_pMgr;
}


MotorDataMgr::MotorDataMgr()
{
//    connect(this,&MotorDataMgr::dataChanged,mediator,&Mediator::motorAttrChanged);
//    connect(this,&MotorDataMgr::errorOccured,mediator,&Mediator::errorOccur);
}

void MotorDataMgr::dataChanged(const uint64_t longId, const ActuatorAttribute attrId, double value)
{
    mediator->motorAttrChanged(longId,attrId,value);
}

void MotorDataMgr::errorOccured(const uint64_t longId, const uint16_t erroId, std::string errorInfo)
{
    mediator->errorOccur(longId,erroId,errorInfo);
}

MotorDataMgr::~MotorDataMgr()
{
    for(auto it = m_allMotorDatas.begin();it != m_allMotorDatas.end();++it)
    {
        delete *it;
    }
}

double MotorDataMgr::getMotorDataAttrValue(const uint64_t longId, const Actuator::ActuatorAttribute attrId) const
{
    ActuatorData * pData = getMotorDataByLongId(longId);
    if(pData)
        return pData->getValue(attrId);
    //qDebug()<< tr("getValue error DeviceId%1 AttrId%2").arg(nDeviceId).arg(attrId);
    return double();//null
}

bool MotorDataMgr::waitForACK(const ActuatorData *pData, const ActuatorAttribute attrId)
{
    int nWaitCnt = 0;
    int nDataStatus = ActuatorData::DATA_NONE;
    while (true) {
        ActuatorController::processEvents();
        nDataStatus = pData->getDataStatus(attrId);
        switch (nDataStatus)
        {
        case ActuatorData::DATA_NONE:
        case ActuatorData::DATA_FAILED:
            return false;
            break;
        case ActuatorData::DATA_NORMAL:
            //std::cout << "wait count:" << nWaitCnt << std::endl;
            return true;
            break;
        case ActuatorData::DATA_WAIT:
            if(nWaitCnt > 100)//等待超时，重发
            {
                nWaitCnt = 0;
                //std::cout << "wait timeout " << std::endl;
                return false;
            }
            ++nWaitCnt;
            break;
        default:
            return false;
            break;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
    return false;
}

double MotorDataMgr::getMotorDataAttrValueAsDouble(const uint64_t longId, const Actuator::ActuatorAttribute attrId) const
{
    bool bOK = true;
    double ret = getMotorDataAttrValue(longId,attrId);
    if(bOK)
        return ret;
    //qDebug()<< tr("getValue error DeviceId%1 AttrId%2").arg(nDeviceId).arg(attrId);
    return 0;
}

int32_t MotorDataMgr::getMotorDataAttrValueAsInt(const uint64_t longId, const Actuator::ActuatorAttribute attrId) const
{
    bool bOK = true;
    int32_t ret = getMotorDataAttrValue(longId,attrId);
    if(bOK)
        return ret;
    //qDebug()<< tr("getValue error DeviceId%1 AttrId%2").arg(nDeviceId).arg(attrId);
    return 0;
}

void MotorDataMgr::setMotorDataAttrByUser(const uint64_t longId, const Actuator::ActuatorAttribute attrId, double value, bool bSend)
{
    ActuatorData * pData = getMotorDataByLongId(longId);
    if(pData)
    {
        switch (attrId) {
        case ACTUATOR_SWITCH:
            if(/*pData->getValue(Actuator::ACTUATOR_SWITCH) != value.toInt()*/true)//if current value is equal to the target value,nothing will be done!
            {
                pData->setValueByUser(attrId,value,bSend);
            }
            break;
        default:
            pData->setValueByUser(attrId,value,bSend);
            break;
        }

    }
    else
    {
        //qDebug() << tr("Set Motor %1 Attri %2 value %3 failed! ").arg(nDeviceId).arg(attrId).arg(value.toDouble());
    }
}

bool MotorDataMgr::setMotorDataWithACK(const uint64_t longId, const ActuatorAttribute attrId, double value, bool bSend)
{
    ActuatorData * pData = getMotorDataByLongId(longId);
    if(pData)
    {
        //std::cout << "start " << (int)longId << " " << attrId << " " << value << std::endl;
        setMotorDataAttrByUser(longId,attrId,value,bSend);
        for(int i=0;i<3;++i)
        {
            if(!waitForACK(pData,attrId))
            {
                setMotorDataAttrByUser(longId,attrId,value,bSend);
                //std::cout << "retry " << (int)longId << " " << attrId << " " << value << std::endl;
            }
            else
            {
                return true;
            }
        }

    }

    return false;
}

double MotorDataMgr::regainAttrWithACK(const uint64_t longId, ActuatorAttribute attrId, bool *bSuccess)
{
    ActuatorData * pData = getMotorDataByLongId(longId);
    if(pData)
    {
        regainData(longId,attrId);
        for(int i=0;i<3;++i)
        {
            if(!waitForACK(pData,attrId))
            {
                regainData(longId,attrId);
            }
            else
            {
                if(bSuccess)
                    *bSuccess = true;
                return pData->getValue(attrId);
            }
        }

    }
    if(bSuccess)
        *bSuccess = false;
    return 0;
}

void MotorDataMgr::setMotorDataAttrByProxy(const uint64_t longId, int proxyId, double value)
{
    ActuatorData * pData = getMotorDataByLongId(longId);
    if(pData)
    {
        pData->setValueByProxy(proxyId,value);
    }
    else
    {
        //qDebug() << tr("Set Motor %1 Attri %2 value %3 failed! ").arg(nDeviceId).arg(proxyId).arg(value.toDouble());
    }
}

void MotorDataMgr::setMotorDataAttrInBatch(const std::list<uint64_t> idList, const Actuator::ActuatorAttribute attrId, double value, bool bSend)
{
    for(auto it = idList.begin();it != idList.end();++it)
    {
        setMotorDataAttrByUser(*it,attrId,value,bSend);
    }
}

void MotorDataMgr::AddMotorsData(std::multimap<std::pair<uint8_t, uint32_t>, uint32_t> dataMap)
{
    std::vector<uint8_t> duplication;
    for (auto iter = dataMap.begin(); iter != dataMap.end(); iter = dataMap.upper_bound(iter->first))
    {
        if(dataMap.count(iter->first) > 1)
        {
            auto res = dataMap.equal_range(iter->first);
            duplication.push_back(iter->first.first);
            for (auto it = res.first; it != res.second; ++it)
            {
                ActuatorData * pData = new ActuatorData(it->first.first,it->second,it->first.second);
                m_allMotorDatas.emplace_back(pData);
            }
        }

    }

    if(duplication.size() > 0)
    {
        for(uint8_t nDeviceId : duplication)
        {
            mediator->errorOccur(nDeviceId,Actuator::ERR_ID_UNUNIQUE,
                                 string_format("There are at least two Actuators with the same ID:%d,\n please check and set suitably!",nDeviceId));
        }
    }
    else
    {
        std::multimap<uint8_t,std::pair<uint32_t,uint32_t>> idOrderMap;

        for(auto it=dataMap.begin();it!=dataMap.end();++it)
        {
            idOrderMap.insert({it->first.first,{it->first.second,it->second}});
        }

        for(auto it=idOrderMap.begin();it!=idOrderMap.end();++it)
        {
            ActuatorData * pData = new ActuatorData(it->first,it->second.second,it->second.first);
            m_allMotorDatas.emplace_back(pData);
            pData->readStatus();
        }
    }


}

std::vector<uint64_t> MotorDataMgr::getLongIdArray() const
{
    std::vector<uint64_t> v;
    for(auto it: m_allMotorDatas)
    {
        v.push_back(it->longId());
    }
    return v;
}

std::vector<uint64_t> MotorDataMgr::getLongIdGroup(uint64_t longId) const
{
    std::vector<uint64_t> v;
    for(auto it: m_allMotorDatas)
    {
        if(Mediator::toCommunicationId(it->longId()) == Mediator::toCommunicationId(longId))
            v.push_back(it->longId());
    }
    return v;
}

std::vector<uint16_t> MotorDataMgr::motorErrorHistory(const uint64_t longId) const
{
    std::vector<uint16_t> errorList;
    ActuatorData * pData = getMotorDataByLongId(longId);
    if(pData)
        return pData->errorHistory();
    return errorList;
}

//bool MotorDataMgr::deviceIdHasExist(uint64_t longId)
//{
//    for(auto it = m_allMotorDatas.begin();it != m_allMotorDatas.end();++it)
//    {
//        ActuatorData * pData = *it;
//        if(pData->longId() == longId)
//            return true;
//    }
//    return false;
//}

//void MotorDataMgr::activeMotorModeSuccessfully(const uint64_t longId)
//{
//    ActuatorData * pData = getMotorDataByLongId(longId);
//    if(pData)
//    {
//        pData->activeModeSuccessfully();
//        //emit dataChanged(nDeviceId,Actuator::MODE_ID,pData->getValue(Actuator::MODE_ID));
//    }

//}

void MotorDataMgr::activeMotorModeInBatch(const std::vector<uint64_t> idList, const Actuator::ActuatorMode mode)
{
    for(auto it = idList.begin();it != idList.end();++it)
    {
        setMotorDataAttrByUser(*it,MODE_ID,mode);
        setMotorDataWithACK(*it,MODE_ID,mode);
    }
}

//void MotorDataMgr::regainAllData(const uint64_t longId)
//{
//    ActuatorData * pData = getMotorDataByLongId(longId);
//    if(pData)
//        pData->requestAllValue();
//}

void MotorDataMgr::regainData(const uint64_t longId, Actuator::ActuatorAttribute attrId)
{
    ActuatorData * pData = getMotorDataByLongId(longId);
    if(pData)
    {
        pData->regainData(attrId);
    }
}

void MotorDataMgr::responseHeart(const uint64_t longId, bool bSuccessfully)
{
    ActuatorData * pData = getMotorDataByLongId(longId);
    if(pData)
    {
        pData->responseHeart(bSuccessfully);
    }
}

void MotorDataMgr::switchAutoRequestActual(const uint64_t longId, bool bStart)
{
    ActuatorData * pData = getMotorDataByLongId(longId);
    if(pData && isUnique(pData))
    {
        pData->switchAutoRequestActual(bStart);
    }
}

void MotorDataMgr::setAutoRequestInterval(const uint64_t longId, uint32_t mSec)
{
    ActuatorData * pData = getMotorDataByLongId(longId);
    if(pData)
    {
        pData->setAutoRequestInterval(mSec);
    }
}

uint8_t MotorDataMgr::getOldDeviceId(const uint64_t newLongId)
{
    ActuatorData * pData = getMotorDataByNewId(newLongId);
    if(pData)
    {
        return pData->deviceId();
    }
    return 0;
}

void MotorDataMgr::saveAllParams(const uint64_t longId)
{
    ActuatorData * pData = getMotorDataByLongId(longId);
    if(pData)
    {
        pData->saveAllParams();
    }
}

void MotorDataMgr::clearHomingInfo(const uint64_t longId)
{
    ActuatorData * pData = getMotorDataByLongId(longId);
    if(pData)
    {
        pData->clearHomingInfo();
    }
}

void MotorDataMgr::setHomingOperationMode(const uint64_t longId, const uint8_t nMode)
{
    ActuatorData * pData = getMotorDataByLongId(longId);
    if(pData)
    {
        pData->setHomingOperationMode(nMode);
    }
}

void MotorDataMgr::openChartChannel(uint64_t longId, const int nChannelId)
{
    ActuatorData * pData = getMotorDataByLongId(longId);
    if(pData)
    {
        pData->openChartChannel(nChannelId);
    }
}

void MotorDataMgr::closeChartChannel(uint64_t longId, const int nChannelId)
{
    ActuatorData * pData = getMotorDataByLongId(longId);
    if(pData)
    {
        pData->closeChartChannel(nChannelId);
    }
}

void MotorDataMgr::switchChartAllChannel(uint64_t longId, bool bOn)
{
    ActuatorData * pData = getMotorDataByLongId(longId);
    if(pData)
    {
        pData->switchChartAllChannel(bOn);
    }
}

void MotorDataMgr::switchCalibrationVel(uint64_t longId, uint8_t nValue)
{
    ActuatorData * pData = getMotorDataByLongId(longId);
    if(pData)
    {
        pData->switchCalibrationVel(nValue);
    }
}

void MotorDataMgr::switchCalibration(uint64_t longId, uint8_t nValue)
{
    ActuatorData * pData = getMotorDataByLongId(longId);
    if(pData)
    {
        pData->switchCalibration(nValue);
    }
}

void MotorDataMgr::startCalibration(uint64_t longId)
{
    ActuatorData * pData = getMotorDataByLongId(longId);
    if(pData)
    {
        pData->startCalibration();
    }
}

void MotorDataMgr::requestSuccessfully(const uint64_t longId, const uint8_t nProxyId)
{
    ActuatorData * pData = getMotorDataByLongId(longId);
    if(nProxyId == D_SET_DEVICE_ID)
    {
        pData = getMotorDataByNewId(longId);
    }
    //qDebug() << "request successfully" << nDeviceId << nProxyId;
    if(pData)
    {
        uint8_t nDataId = DataUtil::convertToMotorDataId((Directives)nProxyId);
        switch (nProxyId)
        {
        case D_CLEAR_HOMING:
            pData->setValueByUser(POS_LIMITATION_MAXIMUM,0,false);
            pData->setValueByUser(POS_LIMITATION_MINIMUM,0,false);
            pData->setValueByUser(HOMING_POSITION,0,false);
            pData->requestSuccessfully(nDataId);
            //setProxyCallback(nProxyId,true);
            break;
        case D_CLEAR_ERROR:
//            pData->setValueByUser(ERROR_ID,Actuator::ERR_NONE,false);
//            pData->setValueByUser(MODE_ID,Mode_Cur,false);//mode change to current automatically

            //执行器抱闸和模式清除错误会变，重新获取
            ITimer::singleShot(300,[=]{//模式切换需要时间
                pData->regainData(Actuator::MODE_ID);
                pData->requestSuccessfully(nDataId);
                },*mediator->ioContext());
#ifdef HAS_BRAKE
            pData->regainData(Actuator::ACTUATOR_BRAKE);
#endif
            //emit setProxyCallback(nProxyId,true);
            break;
        case D_SET_MODE:
            pData->requestSuccessfully(nDataId);
            pData->activeModeSuccessfully();
            //emit setProxyCallback(nProxyId,true);
            break;
        case D_SET_SWITCH_MOTORS:
        {
            uint8_t nRequest = pData->getUserRequestValue(ACTUATOR_SWITCH);
            uint8_t nBefore = pData->getValue(ACTUATOR_SWITCH);
            if(nRequest==Actuator::ACTUATOR_SWITCH_ON && nBefore!=Actuator::ACTUATOR_SWITCH_ON)//user open motor
            {
                if(isUnique(pData))
                {
                    ITimer::singleShot(3000,[=]{pData->requestAllValue();},*mediator->ioContext());
                }
            }
            else if(nRequest==Actuator::ACTUATOR_SWITCH_OFF && nBefore!=Actuator::ACTUATOR_SWITCH_OFF)//user close motor
            {
                pData->switchAutoRequestActual(false);
                pData->initData();//关机后要把数据重置
            }
            pData->requestSuccessfully(nDataId);
        }

            break;
        case D_SET_DEVICE_ID:
            //mototor device is has changed, we need find motorData by new id.
            //qDebug() << "Change" << nDeviceId;
            if(pData)
            {
                //qDebug() << "Change2" << nDeviceId;
                pData->requestSuccessfully(nDataId);
                //emit setProxyCallback(nProxyId,true);
            }
            break;
        default:
            //emit setProxyCallback(nProxyId,true);
            pData->requestSuccessfully(nDataId);
            break;
        }
    }
}

void MotorDataMgr::requestFailed(const uint64_t longId, const uint8_t nProxyId)
{
     ActuatorData * pData = getMotorDataByLongId(longId);
     if(pData)
     {
         switch (nProxyId) {
         case D_SET_SWITCH_MOTORS:
         {
             uint8_t nRequest = pData->getUserRequestValue(ACTUATOR_SWITCH);
             if(nRequest == Actuator::ACTUATOR_SWITCH_OFF)//关机保存参数失败
             {
                mediator->errorOccur(longId,Actuator::ERR_SHUTDOWN_SAVING,string_format("Failed to save parameters when shutdown actuator %d!",longId));
                requestSuccessfully(longId,nProxyId);//默认执行器已经关机
             }
         }

             break;
         default:
             pData->requestFailed(DataUtil::convertToMotorDataId((Directives)nProxyId));
             break;
         }
     }
}
void MotorDataMgr::reconnect(uint64_t longId)
{
    ActuatorData * pData = getMotorDataByLongId(longId);
    if(pData)
    {
        pData->reconnect();
    }
}
void MotorDataMgr::clearError(uint64_t longId)
{
    ActuatorData * pData = getMotorDataByLongId(longId);
    if(pData)
    {
        uint32_t errorCode = pData->getValue(Actuator::ERROR_ID);
        if(errorCode&0x20)
        {
            //init 430
            InnfosProxy::SendProxyWithLongId(longId,D_INIT_430,(uint8_t)0);
        }
        pData->setValueByUser(ERROR_ID,Actuator::ERR_NONE);

    }


}

void MotorDataMgr::sendCmd(uint64_t longId, uint16_t cmdId)
{
    ActuatorData * pData = getMotorDataByLongId(longId);
    if(pData)
    {
        if(cmdId < DIRECTIVES_INVALID)
        {
            InnfosProxy::SendProxyWithLongId(longId,cmdId);
        }
    }
}

void MotorDataMgr::sendCmd(uint64_t longId, uint16_t cmdId, uint32_t value)
{
    ActuatorData * pData = getMotorDataByLongId(longId);
    if(pData)
    {
        if(cmdId < DIRECTIVES_INVALID)
        {
            InnfosProxy::SendProxyWithLongId(longId,cmdId,value);
        }
    }
}

uint64_t MotorDataMgr::getLongIdByDeviceId(uint8_t id) const
{
    ActuatorData * pData = getMotorDataById(id);
    if(pData)
    {
        return pData->longId();
    }
    return 0;
}

void MotorDataMgr::sendCmd(uint64_t longId, uint16_t cmdId, uint8_t value)
{
    ActuatorData * pData = getMotorDataByLongId(longId);
    if(pData)
    {
        if(cmdId < DIRECTIVES_INVALID)
        {
            InnfosProxy::SendProxyWithLongId(longId,cmdId,value);
        }
    }
}

ActuatorData *MotorDataMgr::getMotorDataById(const uint8_t nId) const
{
    for(auto it=m_allMotorDatas.begin();it!=m_allMotorDatas.end();++it)
    {
        ActuatorData * pData = *it;
        if(pData->deviceId() == nId)
            return pData;
    }
    return nullptr;
}

ActuatorData *MotorDataMgr::getMotorDataByLongId(const uint64_t longId) const
{
    for(auto it=m_allMotorDatas.begin();it!=m_allMotorDatas.end();++it)
    {
        ActuatorData * pData = *it;
        if(pData->longId() == longId)
            return pData;
    }
    return nullptr;
}

ActuatorData *MotorDataMgr::getMotorDataByNewId(const uint64_t longId) const
{
    for(auto it=m_allMotorDatas.begin();it!=m_allMotorDatas.end();++it)
    {
        ActuatorData * pData = *it;
        if(pData->requestDeviceId() == longId)
            return pData;
    }
    return nullptr;
}

ActuatorData *MotorDataMgr::getMotorDataByMac(const uint32_t nMac) const
{
    for(auto it=m_allMotorDatas.begin();it!=m_allMotorDatas.end();++it)
    {
        ActuatorData * pData = *it;
        if(pData->deviceMac() == nMac)
            return pData;
    }
    return nullptr;
}

bool MotorDataMgr::isUnique(const ActuatorData *pData) const
{
    for(auto it=m_allMotorDatas.begin();it!=m_allMotorDatas.end();++it)
    {
        ActuatorData * tmp = *it;
        if(pData->longId() == tmp->longId() && pData != tmp)
            return false;
    }
    return true;
}

//bool MotorDataMgr::checkIdUnique(std::map<uint8_t, uint32_t> dataMap) const
//{
////    return dataMap.keys().size() == dataMap.uniqueKeys().size();
//    return true;
//}

//void MotorDataMgr::handleUnuiqueError(std::map<uint8_t, uint32_t> dataMap)
//{
////    std::list<uint8_t> keys = dataMap.uniqueKeys();
////    for(int i=0;i<keys.size();++i)
////    {
////        if(dataMap.values(keys.at(i)).size() > 1)
////        {
////            errorOccured(0,Actuator::ERR_ID_UNUNIQUE,tr("There are at least two motors with the same ID:%1,\n please check and set suitably!").arg(keys.at(i)));
////            break;
////        }
////    }
//}
