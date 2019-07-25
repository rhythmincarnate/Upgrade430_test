#include "actuatorcontroller.h"
#include "innfosproxy.h"
#include "itimer.h"
#include <iostream>
#include "asio.hpp"
#include "mediator.h"
#include "stringformat.h"

//#define toLongId(x) motorDataMgrInstance->getLongIdByDeviceId(x)
using namespace Actuator;
using asio::ip::udp;

ActuatorController * ActuatorController::m_pInstance = nullptr;
udp::socket * pUpgradeSocket = nullptr;


ActuatorController::ActuatorController()
    :m_sOperationFinished(nullptr)
    ,m_sRequestBack(nullptr)
    ,m_sError(nullptr)
    ,m_sActuatorAttrChanged(nullptr)
    ,m_sNewChartStart(nullptr)
    ,m_sChartValueChange(nullptr)
    ,m_lConnectionIds(nullptr)
{
    m_sOperationFinished = new CSignal<uint8_t,uint8_t>;
    m_sOperationFinishedL = new CSignal<uint64_t,uint8_t>;
    m_sRequestBack = new CSignal<uint8_t,uint8_t,double>;
    m_sRequestBackL = new CSignal<uint64_t,uint8_t,double>;
    m_sError = new CSignal<uint8_t,uint16_t,std::string>;
    m_sErrorL = new CSignal<uint64_t,uint16_t,std::string>;
    m_sActuatorAttrChanged = new CSignal<uint8_t,uint8_t,double>;
    m_sActuatorAttrChangedL = new CSignal<uint64_t,uint8_t,double>;
    m_sNewChartStart = new CSignal<>;
    m_sChartValueChange = new CSignal<uint8_t,double>;
    m_lConnectionIds = new std::vector<int>;
    m_upgradeCheck = new CSignal<uint64_t,uint16_t>;
    m_upgrade430Check = new CSignal<uint64_t,uint16_t>;
    m_upgrade430Version = new CSignal<uint64_t,uint16_t>;//ADD
#ifdef IMU_ENABLE
    m_sQuaternion = new CSignal<uint8_t,double,double,double,double>;
    m_sQuaternionL = new CSignal<uint64_t,double,double,double,double>;
#endif

    m_lConnectionIds->push_back(mediator->m_sRequestBack.connect_member(this,&ActuatorController::onRequestCallback));
    m_lConnectionIds->push_back(mediator->m_sRecognizeFinished.connect_member(this,&ActuatorController::finishRecognizeCallback));
    m_lConnectionIds->push_back(mediator->m_sError.connect_member(this,&ActuatorController::errorOccur));
    m_lConnectionIds->push_back(mediator->m_sMotorAttrChanged.connect_member(this,&ActuatorController::motorAttrChanged));
    m_lConnectionIds->push_back(mediator->m_sChartValueChange.connect_member(this,&ActuatorController::chartValueChange));
    m_lConnectionIds->push_back(mediator->m_sNewChartStart.connect_member(this,&ActuatorController::startNewChart));
    m_lConnectionIds->push_back(mediator->m_upgradeCheck.connect_member(this,&ActuatorController::upgradeCheck));
    m_lConnectionIds->push_back(mediator->m_upgrade430Check.connect_member(this,&ActuatorController::upgrade430Check));
    m_lConnectionIds->push_back(mediator->m_upgrade430Version.connect_member(this,&ActuatorController::upgrade430Version));  //ADD
#ifdef IMU_ENABLE
    m_lConnectionIds->push_back(mediator->m_sQuaternion.connect_member(this,&ActuatorController::receiveQuaternion));
#endif

}

string ActuatorController::toString(uint64_t longId)
{
    return Mediator::toString(longId);
}

uint8_t ActuatorController::toByteId(uint64_t longId)
{
    return Mediator::toDeviceId(longId);
}

uint64_t ActuatorController::toLongId(string ipStr, uint8_t id)
{
    return Mediator::toLongId(asio::ip::address::from_string(ipStr).to_v4().to_uint(),id);
}

uint64_t ActuatorController::toLongId(uint8_t id)
{
    return motorDataMgrInstance->getLongIdByDeviceId(id);
}

void ActuatorController::initController(int nCommunicationType)
{
    if(m_pInstance != nullptr)
    {
        return;
    }
    mediator->initCommunication(nCommunicationType);
    m_pInstance= new ActuatorController();
}

ActuatorController *ActuatorController::getInstance()
{
    return m_pInstance;
}

void ActuatorController::processEvents()
{
    //QCoreApplication::processEvents();
    mediator->runOnce();
}

ActuatorController::~ActuatorController()
{
    mediator->m_sRequestBack.s_Disconnect(*m_lConnectionIds);
    mediator->m_sRecognizeFinished.s_Disconnect(*m_lConnectionIds);
    mediator->m_sError.s_Disconnect(*m_lConnectionIds);
    mediator->m_sMotorAttrChanged.s_Disconnect(*m_lConnectionIds);
    mediator->m_sChartValueChange.s_Disconnect(*m_lConnectionIds);
    mediator->m_sNewChartStart.s_Disconnect(*m_lConnectionIds);
    mediator->m_upgradeCheck.s_Disconnect(*m_lConnectionIds);
#ifdef IMU_ENABLE
    mediator->m_sQuaternion.s_Disconnect(*m_lConnectionIds);
    delete m_sQuaternion;
    delete m_sQuaternionL;
#endif
    //Mediator::destroyAllStaticObjects();
    delete m_sOperationFinished;
    delete m_sRequestBack;
    delete m_sError;
    delete m_sActuatorAttrChanged;
    delete m_sNewChartStart;
    delete m_sChartValueChange;
    delete m_lConnectionIds;
    delete m_sOperationFinishedL;
    delete m_sRequestBackL;
    delete m_sErrorL;
    delete m_sActuatorAttrChangedL;
    delete m_upgradeCheck;
    delete m_upgrade430Check;
    if(pUpgradeSocket)
    {
        asio::error_code ec;
        pUpgradeSocket->close(ec);
        delete pUpgradeSocket;
    }
}

void ActuatorController::autoRecoginze()
{
    mediator->autoRecognize();
}

bool ActuatorController::hasAvailableActuator() const
{
    return getActuatorIdArray().size() > 0;
}

void ActuatorController::finishRecognizeCallback()
{
    //qDebug()<<"finished";
    ITimer::singleShot(100,[=]{
        //qDebug() << "really finished";
        m_sOperationFinished->s_Emit(0,Actuator::Recognize_Finished);
        m_sOperationFinishedL->s_Emit(0,Actuator::Recognize_Finished);
    },*mediator->ioContext()); //delay to insure all requests have acknowledges
}

void ActuatorController::onRequestCallback(uint64_t longId, uint8_t nProxyId, double value)
{
    switch (nProxyId) {

    case Actuator::D_SAVE_PARAM:
        if(value > 0)
        {
            m_sOperationFinished->s_Emit(toByteId(longId),Actuator::Save_Params_Finished);
            m_sOperationFinishedL->s_Emit(longId,Actuator::Save_Params_Finished);
        }
        else
        {
            m_sOperationFinished->s_Emit(toByteId(longId),Actuator::Save_Params_Failed);
            m_sOperationFinishedL->s_Emit(longId,Actuator::Save_Params_Failed);
        }

        break;
    case Actuator::D_SET_SWITCH_MOTORS:
        if(motorDataMgrInstance->getMotorDataAttrValueAsInt(longId,Actuator::ACTUATOR_SWITCH) == Actuator::ACTUATOR_SWITCH_ON)
        {
//            ITimer::singleShot(4500,[=]{//等待3.5s,开机动作才完成
//                m_sOperationFinished->s_Emit(toByteId(longId),Actuator::Launch_Finished);
//                m_sOperationFinishedL->s_Emit(longId,Actuator::Launch_Finished);
//            },*mediator->ioContext());
        }
        else
        {
            m_sOperationFinished->s_Emit(toByteId(longId),Actuator::Close_Finished);
            m_sOperationFinishedL->s_Emit(longId,Actuator::Close_Finished);
        }
        break;
    default:
        m_sRequestBack->s_Emit(toByteId(longId),nProxyId,value);
        m_sRequestBackL->s_Emit(longId,nProxyId,value);
        break;
    }
}

void ActuatorController::errorOccur(uint64_t longId, uint16_t errorId, string errorStr)
{
    m_sError->s_Emit(toByteId(longId),errorId,errorStr);
    m_sErrorL->s_Emit(longId,errorId,errorStr);
}

void ActuatorController::motorAttrChanged(uint64_t longId, uint8_t attrId, double value)
{
    switch(attrId)
    {
    case Actuator::INIT_STATE:
        if(int(value) == Actuator::Initlized)
        {
            m_sOperationFinished->s_Emit(toByteId(longId),Actuator::Launch_Finished);
            m_sOperationFinishedL->s_Emit(longId,Actuator::Launch_Finished);
        }
        break;
    default:
        m_sActuatorAttrChanged->s_Emit(toByteId(longId),attrId,value);
        m_sActuatorAttrChangedL->s_Emit(longId,attrId,value);
        break;
    }

}

void ActuatorController::startNewChart()
{
    m_sNewChartStart->s_Emit();
}

void ActuatorController::chartValueChange(uint8_t channelId, double value)
{
    m_sChartValueChange->s_Emit(channelId,value);
}

void ActuatorController::upgradeCheck(uint64_t longId, uint16_t checkNum)
{
    m_upgradeCheck->s_Emit(longId,checkNum);
}

void ActuatorController::upgrade430Check(uint64_t longId, uint16_t checkNum)
{
    m_upgrade430Check->s_Emit(longId,checkNum);
}

/**/
void ActuatorController::upgrade430Version(uint64_t longId, uint16_t version_430)
{
    m_upgrade430Version->s_Emit(longId,version_430);
}
/**/
vector<uint8_t> ActuatorController::getActuatorIdArray() const
{
    vector<uint8_t> motorKeyVector;
    vector<uint64_t> motorsVector = motorDataMgrInstance->getLongIdArray();
    for(auto it: motorsVector)
    {
        motorKeyVector.emplace_back(Mediator::toDeviceId(it));
    }
    return motorKeyVector;
}

vector<uint64_t> ActuatorController::getActuatorLongIdArray() const
{
    return motorDataMgrInstance->getLongIdArray();
}

vector<uint64_t> ActuatorController::getActuatorIdGroup(uint64_t longId) const
{
    return motorDataMgrInstance->getLongIdGroup(longId);
}

vector<uint64_t> ActuatorController::getActuatorIdGroup(string name)
{
    return getActuatorIdGroup(toLongId(name,0));
}

void ActuatorController::activeActuatorMode(vector<uint8_t> idArray, const Actuator::ActuatorMode nMode)
{
    for(int i=0;i<idArray.size();++i)
    {
        motorDataMgrInstance->setMotorDataWithACK(toLongId(idArray[i]),Actuator::MODE_ID,nMode);
    }
}

void ActuatorController::activateActuatorMode(vector<uint64_t> longIdArray, const ActuatorMode nMode)
{
    for(int i=0;i<longIdArray.size();++i)
    {
        motorDataMgrInstance->setMotorDataWithACK(longIdArray[i],Actuator::MODE_ID,nMode);
    }
}

void ActuatorController::launchAllActuators()
{
    vector<uint64_t> motorsVector = motorDataMgrInstance->getLongIdArray();
    for(int i=0;i<motorsVector.size();++i)
    {
        motorDataMgrInstance->setMotorDataWithACK(motorsVector[i],Actuator::ACTUATOR_SWITCH,Actuator::ACTUATOR_SWITCH_ON);
    }

}

void ActuatorController::closeAllActuators()
{
    vector<uint64_t> motorsVector = motorDataMgrInstance->getLongIdArray();
    for(int i=0;i<motorsVector.size();++i)
    {
        motorDataMgrInstance->setMotorDataWithACK(motorsVector[i],Actuator::ACTUATOR_SWITCH,Actuator::ACTUATOR_SWITCH_OFF);
    }
}

void ActuatorController::launchActuator(uint8_t id)
{
//    bool bRet = motorDataMgrInstance->setMotorDataWithACK(id,Actuator::ACTUATOR_SWITCH,Actuator::ACTUATOR_SWITCH_ON);
//    //std::cout << "success :" << bRet << std::endl;
    launchActuator(toLongId(id));
}

void ActuatorController::launchActuator(uint64_t longId)
{
    motorDataMgrInstance->setMotorDataWithACK(longId,Actuator::ACTUATOR_SWITCH,Actuator::ACTUATOR_SWITCH_ON);
}

void ActuatorController::closeActuator(uint8_t id)
{
//    motorDataMgrInstance->setMotorDataWithACK(id,Actuator::ACTUATOR_SWITCH,Actuator::ACTUATOR_SWITCH_OFF);
    closeActuator(toLongId(id));
}

void ActuatorController::closeActuator(uint64_t longId)
{
    motorDataMgrInstance->setMotorDataWithACK(longId,Actuator::ACTUATOR_SWITCH,Actuator::ACTUATOR_SWITCH_OFF);
}

void ActuatorController::switchAutoRefresh(uint8_t id, bool bOpen)
{
//    motorDataMgrInstance->switchAutoRequestActual(id,bOpen);
    switchAutoRefresh(toLongId(id),bOpen);
}

void ActuatorController::switchAutoRefresh(uint64_t longId, bool bOpen)
{
    motorDataMgrInstance->switchAutoRequestActual(longId,bOpen);
}

void ActuatorController::setAutoRefreshInterval(uint8_t id, uint32_t mSec)
{
//    motorDataMgrInstance->setAutoRequestInterval(id,mSec);
    setAutoRefreshInterval(toLongId(id),mSec);
}

void ActuatorController::setAutoRefreshInterval(uint64_t longId, uint32_t mSec)
{
    motorDataMgrInstance->setAutoRequestInterval(longId,mSec);
}

void ActuatorController::setPosition(uint8_t id, double pos)
{
    setPosition(toLongId(id),pos);
}

void ActuatorController::setPosition(uint64_t longId, double pos)
{
    motorDataMgrInstance->setMotorDataAttrByUser(longId,Actuator::POS_SETTING,pos);
}

void ActuatorController::setVelocity(uint8_t id, double vel)
{
    setVelocity(toLongId(id),vel);
}

void ActuatorController::setVelocity(uint64_t longId, double vel)
{
    motorDataMgrInstance->setMotorDataAttrByUser(longId,Actuator::VEL_SETTING,vel);
}

void ActuatorController::setCurrent(uint8_t id, double current)
{
//    motorDataMgrInstance->setMotorDataAttrByUser(id,Actuator::CUR_IQ_SETTING,current);
    setCurrent(toLongId(id),current);
}

void ActuatorController::setCurrent(uint64_t longId, double current)
{
    motorDataMgrInstance->setMotorDataAttrByUser(longId,Actuator::CUR_IQ_SETTING,current);
}

double ActuatorController::getPosition(uint8_t id, bool bRefresh) const
{
    return getPosition(toLongId(id),bRefresh);
}

double ActuatorController::getPosition(uint64_t longId, bool bRefresh) const
{
    if(bRefresh)
        motorDataMgrInstance->regainData(longId,Actuator::ACTUAL_POSITION);
    return motorDataMgrInstance->getMotorDataAttrValueAsDouble(longId,Actuator::ACTUAL_POSITION);
}

double ActuatorController::getVelocity(uint8_t id, bool bRefresh) const
{
    return getVelocity(toLongId(id),bRefresh);
}

double ActuatorController::getVelocity(uint64_t longId, bool bRefresh) const
{
    if(bRefresh)
        motorDataMgrInstance->regainData(longId,Actuator::ACTUAL_VELOCITY);
    return motorDataMgrInstance->getMotorDataAttrValueAsDouble(longId,Actuator::ACTUAL_VELOCITY);
}

double ActuatorController::getCurrent(uint8_t id, bool bRefresh) const
{
    return getCurrent(toLongId(id),bRefresh);
}

double ActuatorController::getCurrent(uint64_t longId, bool bRefresh) const
{
    if(bRefresh)
        motorDataMgrInstance->regainData(longId,Actuator::ACTUAL_CURRENT);
    return motorDataMgrInstance->getMotorDataAttrValueAsDouble(longId,Actuator::ACTUAL_CURRENT);
}

void ActuatorController::setActuatorAttribute(uint8_t id, ActuatorAttribute attrId, double value)
{
    setActuatorAttribute(toLongId(id),attrId,value);
}

void ActuatorController::setActuatorAttribute(uint64_t longId, Actuator::ActuatorAttribute attrId, double value)
{
    motorDataMgrInstance->setMotorDataAttrByUser(longId,attrId,value);
}

double ActuatorController::getActuatorAttribute(uint8_t id, ActuatorAttribute attrId) const
{
    return getActuatorAttribute(toLongId(id),attrId);
}

double ActuatorController::getActuatorAttribute(uint64_t longId, Actuator::ActuatorAttribute attrId) const
{
    return motorDataMgrInstance->getMotorDataAttrValueAsDouble(longId,attrId);
}

void ActuatorController::saveAllParams(uint8_t id)
{
    saveAllParams(toLongId(id));
}

void ActuatorController::saveAllParams(uint64_t id)
{
    motorDataMgrInstance->saveAllParams(id);
}

void ActuatorController::clearHomingInfo(uint8_t id)
{
    clearHomingInfo(toLongId(id));
}

void ActuatorController::clearHomingInfo(uint64_t longId)
{
    motorDataMgrInstance->clearHomingInfo(longId);
}

void ActuatorController::setHomingOperationMode(uint8_t id, uint8_t nMode)
{
    setHomingOperationMode(toLongId(id),nMode);
}

void ActuatorController::setHomingOperationMode(uint64_t longId, uint8_t nMode)
{
    motorDataMgrInstance->setHomingOperationMode(longId,nMode);
}

void ActuatorController::setMinPosLimit(uint8_t id)
{
    setMinPosLimit(toLongId(id));
}

void ActuatorController::setMinPosLimit(uint64_t longId)
{
    motorDataMgrInstance->sendCmd(longId,Actuator::D_SET_HOMING_MIN);
}

void ActuatorController::setMinPosLimit(uint8_t id, double posValue)
{
    setMinPosLimit(toLongId(id),posValue);
}

void ActuatorController::setMinPosLimit(uint64_t longId, double posValue)
{
    motorDataMgrInstance->setMotorDataAttrByUser(longId,Actuator::POS_LIMITATION_MINIMUM,posValue);
}

void ActuatorController::setMaxPosLimit(uint8_t id)
{
    setMaxPosLimit(toLongId(id));
}

void ActuatorController::setMaxPosLimit(uint64_t longId)
{
    motorDataMgrInstance->sendCmd(longId,Actuator::D_SET_HOMING_MAX);
}

void ActuatorController::setMaxPosLimit(uint8_t id, double posValue)
{
    setMaxPosLimit(toLongId(id),posValue);
}

void ActuatorController::setMaxPosLimit(uint64_t longId, double posValue)
{
    motorDataMgrInstance->setMotorDataAttrByUser(longId,Actuator::POS_LIMITATION_MAXIMUM,posValue);
}

void ActuatorController::setHomingPosition(uint8_t id, double posValue)
{
    setHomingPosition(toLongId(id),posValue);
}

void ActuatorController::setHomingPosition(uint64_t longId, double posValue)
{
    motorDataMgrInstance->setMotorDataAttrByUser(longId,Actuator::HOMING_POSITION,posValue);
}

void ActuatorController::openChartChannel(uint8_t id, uint8_t nChannelId)
{
    openChartChannel(toLongId(id),nChannelId);
}

void ActuatorController::openChartChannel(uint64_t longId, uint8_t nChannelId)
{
    motorDataMgrInstance->openChartChannel(longId,nChannelId);
}

void ActuatorController::closeChartChannel(uint8_t id, uint8_t nChannelId)
{
    closeChartChannel(toLongId(id),nChannelId);
}

void ActuatorController::closeChartChannel(uint64_t longId, uint8_t nChannelId)
{
    motorDataMgrInstance->closeChartChannel(longId,nChannelId);
}

void ActuatorController::switchChartAllChannel(uint8_t id, bool bOn)
{
    switchAutoRefresh(toLongId(id),bOn);
}

void ActuatorController::switchChartAllChannel(uint64_t longId, bool bOn)
{
    motorDataMgrInstance->switchChartAllChannel(longId,bOn);
}

void ActuatorController::setCurrentChartMode(uint8_t id, uint8_t mode)
{
    setCurrentChartMode(toLongId(id),mode);
}

void ActuatorController::setCurrentChartMode(uint64_t longId, uint8_t mode)
{
    motorDataMgrInstance->sendCmd(longId,Actuator::D_SET_CUR_TRIGGER_MODE,mode);
}

void ActuatorController::regainAttrbute(uint8_t id, uint8_t attrId)
{
    regainAttrbute(toLongId(id),attrId);
}


void ActuatorController::regainAttrbute(uint64_t longId, uint8_t attrId)
{
    motorDataMgrInstance->regainData(longId,(Actuator::ActuatorAttribute)attrId);
}

vector<uint16_t> ActuatorController::getErrorHistory(uint8_t id)
{
    return getErrorHistory(toLongId(id));
}

vector<uint16_t> ActuatorController::getErrorHistory(uint64_t longId)
{
    std::vector<uint16_t> errors = motorDataMgrInstance->motorErrorHistory(longId);
    vector<uint16_t> history;
    for(int i=0;i<errors.size();++i)
    {
        history.push_back(errors.at(i));
    }
    return history;
}

void ActuatorController::reconnect(uint8_t id)
{
    reconnect(toLongId(id));
}
void ActuatorController::reconnect(uint64_t longId)
{
    motorDataMgrInstance->reconnect(longId);
}

void ActuatorController::clearError(uint8_t id)
{
    clearError(toLongId(id));
}
void ActuatorController::clearError(uint64_t longId)
{
    motorDataMgrInstance->clearError(longId);
}

string ActuatorController::versionString() const
{
    return mediator->versionString();
}

double ActuatorController::getMaxCurrent(uint8_t id) const
{
    return getMaxCurrent(toLongId(id));
}

double ActuatorController::getMaxCurrent(uint64_t longId) const
{
    return getActuatorAttribute(longId,Actuator::CURRENT_SCALE);
}

double ActuatorController::getMaxVelocity(uint8_t id) const
{
    return getMaxVelocity(toLongId(id));
}

double ActuatorController::getMaxVelocity(uint64_t longId) const
{
    return getActuatorAttribute(longId,Actuator::VELOCITY_SCALE);
}

double ActuatorController::getMaxOutputCurrent(uint8_t id) const
{
    return getMaxOutputCurrent(toLongId(id));
}

double ActuatorController::getMaxOutputCurrent(uint64_t longId) const
{
    return getActuatorAttribute(longId,Actuator::VEL_OUTPUT_LIMITATION_MAXIMUM)*getMaxCurrent(longId);
}

bool ActuatorController::setMaxOutputCurrent(uint8_t id, double maxOutputCurrent)
{
    return setMaxOutputCurrent(toLongId(id),maxOutputCurrent);
}

bool ActuatorController::setMaxOutputCurrent(uint64_t longId, double maxOutputCurrent)
{
    if(maxOutputCurrent <= getMinOutputCurrent(longId) || maxOutputCurrent > getMaxCurrent(longId))
    {
        std::cerr << "setMaxOutputCurrent maxOutputCurrent:" << maxOutputCurrent << " is invalid!" << std::endl;
        return false;
    }
    setActuatorAttribute(longId,Actuator::VEL_OUTPUT_LIMITATION_MAXIMUM,maxOutputCurrent/getMaxCurrent(longId));
    return true;
}

double ActuatorController::getMinOutputCurrent(uint8_t id) const
{
    return getMinOutputCurrent(toLongId(id));
}

double ActuatorController::getMinOutputCurrent(uint64_t longId) const
{
    return getMaxCurrent(longId)*getActuatorAttribute(longId,Actuator::VEL_OUTPUT_LIMITATION_MINIMUM);
}

bool ActuatorController::setMinOutputCurrent(uint8_t id, double minOutputCurrent)
{
    return setMinOutputCurrent(toLongId(id),minOutputCurrent);
}

bool ActuatorController::setMinOutputCurrent(uint64_t longId, double minOutputCurrent)
{
    if(minOutputCurrent < -getMaxCurrent(longId) || minOutputCurrent >= getMaxOutputCurrent(longId))
    {
        std::cerr << "setMinOutputCurrent minOutputCurrent:" << minOutputCurrent << " is invalid!" << std::endl;
        return false;
    }
    setActuatorAttribute(longId,Actuator::VEL_OUTPUT_LIMITATION_MINIMUM,minOutputCurrent/getMaxCurrent(longId));
    return true;
}

double ActuatorController::getMaxOutputVelocity(uint8_t id) const
{
    return getMaxOutputVelocity(toLongId(id));
}

double ActuatorController::getMaxOutputVelocity(uint64_t longId) const
{
    return getActuatorAttribute(longId,Actuator::POS_OUTPUT_LIMITATION_MAXIMUM)*getMaxVelocity(longId);
}

bool ActuatorController::setMaxOutputVelocity(uint8_t id, double maxOutputVelocity)
{
    return setMaxOutputVelocity(toLongId(id),maxOutputVelocity);
}

bool ActuatorController::setMaxOutputVelocity(uint64_t id, double maxOutputVelocity)
{
    if(maxOutputVelocity <= getMinOutputVelocity(id) || maxOutputVelocity > getMaxVelocity(id))
    {
        std::cerr << "setMaxOutputVelocity maxOutputVelocity:" << maxOutputVelocity << " is invalid!" << std::endl;
        return false;
    }
    setActuatorAttribute(id,Actuator::POS_OUTPUT_LIMITATION_MAXIMUM,maxOutputVelocity/getMaxVelocity(id));
    return true;
}

double ActuatorController::getMinOutputVelocity(uint8_t id) const
{
    return getMinOutputVelocity(toLongId(id));
}

double ActuatorController::getMinOutputVelocity(uint64_t longId) const
{
    return getMaxVelocity(longId)*getActuatorAttribute(longId,Actuator::POS_OUTPUT_LIMITATION_MINIMUM);
}

bool ActuatorController::setMinOutputVelocity(uint8_t id, double minOutputVelocity)
{
    return setMinOutputVelocity(toLongId(id),minOutputVelocity);
}

bool ActuatorController::setMinOutputVelocity(uint64_t longId, double minOutputVelocity)
{
    if(minOutputVelocity < -getMaxVelocity(longId) || minOutputVelocity >= getMaxOutputVelocity(longId))
    {
        std::cerr << "setMinOutputVelocity minOutputVelocity:" << minOutputVelocity << " is invalid!" << std::endl;
        return false;
    }
    setActuatorAttribute(longId,Actuator::POS_OUTPUT_LIMITATION_MINIMUM,minOutputVelocity/getMaxVelocity(longId));
    return true;
}

void ActuatorController::activeActuatorMode(uint8_t id, const ActuatorMode nMode)
{
    activateActuatorMode(toLongId(id),nMode);
}

void ActuatorController::activateActuatorMode(uint64_t id, const ActuatorMode nMode)
{
    activateActuatorMode(std::vector<uint64_t>({id}),nMode);
}

double ActuatorController::getActuatorAttributeWithACK(uint8_t id, ActuatorAttribute attrId, bool *bSuccess)
{
    return getActuatorAttributeWithACK(toLongId(id),attrId,bSuccess);
}

double ActuatorController::getActuatorAttributeWithACK(uint64_t longId, ActuatorAttribute attrId, bool *bSuccess)
{
    return motorDataMgrInstance->regainAttrWithACK(longId,attrId,bSuccess);
}

bool ActuatorController::setActuatorAttributeWithACK(uint8_t id, ActuatorAttribute attrId, double value)
{
    return setActuatorAttributeWithACK(toLongId(id),attrId,value);
}

bool ActuatorController::setActuatorAttributeWithACK(uint64_t longId, ActuatorAttribute attrId, double value)
{
    return motorDataMgrInstance->setMotorDataWithACK(longId,attrId,value) ;
}

void ActuatorController::getCVPValue(uint8_t id)
{
    motorDataMgrInstance->sendCmd(toLongId(id),Actuator::D_READ_ACTUAL_CVP);
}

void ActuatorController::getCVPValue(uint64_t longId)
{
    motorDataMgrInstance->sendCmd(toLongId(longId),Actuator::D_READ_ACTUAL_CVP);
}

void ActuatorController::switchCalibrationVel(uint64_t longId, uint8_t nValue)
{
    motorDataMgrInstance->switchCalibrationVel(longId,nValue);
}

void ActuatorController::switchCalibration(uint64_t longId, uint8_t nValue)
{
    motorDataMgrInstance->switchCalibration(longId,nValue);
}

void ActuatorController::startCalibration(uint64_t longId)
{
    motorDataMgrInstance->startCalibration(longId);
}

void ActuatorController::sendCmd(uint64_t longId, uint16_t cmdId, uint32_t value)
{
    motorDataMgrInstance->sendCmd(longId,cmdId,value);
}

void ActuatorController::upgradeActuator(uint8_t id, std::vector<uint8_t> &data)
{
    mediator->upgradeActuator(toLongId(id),data);
}

void ActuatorController::upgrade430(uint8_t id, std::vector<uint8_t> &data)
{
    mediator->upgrade430(toLongId(id),data);
}

std::vector<uint32_t> ActuatorController::allCommunicationUnits() const
{
    return mediator->allCommunicationUnits();
}

#ifdef IMU_ENABLE
void ActuatorController::requestAllQuaternions(uint64_t longId)
{
    mediator->requestQuaternion(2,longId);
}

void ActuatorController::requestSingleQuaternion(uint64_t longId)
{
    mediator->requestQuaternion(1,longId);
}

//void ActuatorController::requestQuaternion(const string &target)
//{
//    mediator->requestQuaternion(1,target);
//}

void ActuatorController::receiveQuaternion(uint64_t nIMUId, double w, double x, double y, double z)
{
    m_sQuaternion->s_Emit(toByteId(nIMUId),w,x,y,z);
    m_sQuaternionL->s_Emit(nIMUId,w,x,y,z);
}

string ActuatorController::readSoftwareVersion(string &ip) const
{
    uint16_t version = mediator->readSoftwareVersion(asio::ip::address::from_string(ip).to_v4().to_uint());
    string verStr("unknown");
    if(version > 0)
        verStr = string_format("soft version %d.%d",version>>8,version&0xff);
    return verStr;
}

string ActuatorController::readHardwareVersion(string &ip) const
{
    uint16_t version = mediator->readHardwareVersion(asio::ip::address::from_string(ip).to_v4().to_uint());
    string verStr("unknown");
    if(version > 0)
        verStr = string_format("hard version %d.%d",version>>8,version&0xff);
    return verStr;
}

bool ActuatorController::enterPannelBootloader(const string &target)
{
    return mediator->enterPannelBootloader(target);
}

bool ActuatorController::upgradePannel(const string &target, uint32_t port ,const std::vector<uint8_t> &upgradeData)
{
    std::vector<uint8_t> writeRequest{0x00,0x02,0x32,0x33,0x34,0x2e,0x74,0x78,0x74,0x00,0x6f,0x63,0x74,0x65,0x74,0x00,0x74,0x73,0x69,0x7a,0x65,0x00};//0x00 0x02是写入请求，数据为234.txt octet tsize
    //加上数据长度
    uint32_t count = upgradeData.size();
    std::string countStr(std::to_string(count));
    for(int i=0;i<countStr.size();++i)
    {
        writeRequest.push_back(countStr.c_str()[i]);
    }
    writeRequest.push_back(0x00);
    uint32_t remotePort = 0;
    std::vector<uint8_t> ret = sendToPannel(target,port,remotePort,writeRequest);
    if(ret.size() < 2 || ret.at(1)!=04)//request failed
        return false;

    uint32_t nIdx = 0;
    uint8_t nPack = 0;
    while (nIdx < upgradeData.size()) {
        std::vector<uint8_t> sendData{0x00,0x03,0x00,++nPack};
        int nSize = upgradeData.size()-nIdx > 512 ? 512:upgradeData.size()-nIdx;
        for(int i=0;i<nSize;++i)
        {
            sendData.emplace_back(upgradeData[i+nIdx]);
        }
        nIdx += nSize;
        std::vector<uint8_t> ret = sendToPannel(target,remotePort,sendData);
        if(ret.size() < 4 || ret.at(1)!=04 || ret.at(3) != nPack)//send failed
            return false;
    }
    return true;
}

std::vector<uint8_t> ActuatorController::sendToPannel(const string &target, uint32_t port, uint32_t &remotePort, const std::vector<uint8_t> &data)
{
    if(!pUpgradeSocket)
        pUpgradeSocket = new udp::socket(*mediator->ioContext(),udp::endpoint(udp::v4(),0));
    asio::error_code ec;
    pUpgradeSocket->send_to(asio::buffer(data),udp::endpoint(asio::ip::address::from_string(target),port),0,ec);
    if(ec)
    {
        std::cout << ec.message() << std::endl;
        return {0};
    }
    std::vector<uint8_t> receiveBuf(1024);
    udp::endpoint receiveEndpoint;
    pUpgradeSocket->receive_from(asio::buffer(receiveBuf),receiveEndpoint);
    remotePort = receiveEndpoint.port();
    std::cout << receiveEndpoint.address().to_string() << remotePort << std::endl;
//    send.close();
    return receiveBuf;
}

std::vector<uint8_t> ActuatorController::sendToPannel(const string &target, uint32_t port, const std::vector<uint8_t> &data)
{
    if(!pUpgradeSocket)
        pUpgradeSocket = new udp::socket(*mediator->ioContext(),udp::endpoint(udp::v4(),0));
    pUpgradeSocket->send_to(asio::buffer(data),udp::endpoint(asio::ip::address::from_string(target),port));
    std::vector<uint8_t> receiveBuf(1024);
    std::cout << "222 " <<target << port << data.size()<<std::endl;
    pUpgradeSocket->receive(asio::buffer(receiveBuf));
//    send.close();
    return receiveBuf;
}

bool ActuatorController::erasePannel(const string &target)
{
    if(!pUpgradeSocket)
        pUpgradeSocket = new udp::socket(*mediator->ioContext(),udp::endpoint(udp::v4(),0));

    std::vector<uint8_t> writeRequest{0x00,0x02,0x32,0x33,0x34,0x2e,0x74,0x78,0x74,0x00,0x6f,0x63,0x74,0x65,0x74,0x00,0x74,0x73,0x69,0x7a,0x65,0x00};//0x00 0x02是写入请求，数据为234.txt octet tsize
    //加上数据长度
    uint32_t count = 4;
    std::string countStr(std::to_string(count));
    for(int i=0;i<countStr.size();++i)
    {
        writeRequest.push_back(countStr.c_str()[i]);
    }
    writeRequest.push_back(0x00);
    uint32_t remotePort = 0;
    std::vector<uint8_t> ret = sendToPannel(target,69,remotePort,writeRequest);
    if(ret.size() < 2 || ret.at(1)!=04)//request failed
        return false;
    std::vector<uint8_t> data{0x00,0x03,0x00,0x01,0xff,0xff,0xff,0xff};
    pUpgradeSocket->send_to(asio::buffer(data),udp::endpoint(asio::ip::address::from_string(target),0));
    return true;
}

int ActuatorController::Ver_430 = 10;
#endif




