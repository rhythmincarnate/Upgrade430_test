#include "AbstractAutoRecognize.h"
#include "mediator.h"
#include "EthernetAutoRecognize.h"
#include "SerialAutoRecognize.h"
#include "stringformat.h"

AbstractAutoRecognize *AbstractAutoRecognize::getInstance(int nType)
{
    AbstractAutoRecognize * pInstance = nullptr;
    if(nType == Actuator::Via_Ethernet)
    {
        pInstance = new EthernetAutoRecognize();
    }
    else
    {
        pInstance = new SerialAutoRecognize();
    }
    return pInstance;
}

AbstractAutoRecognize::AbstractAutoRecognize():
    m_bUniqueId(true)
{

}

AbstractAutoRecognize::~AbstractAutoRecognize()
{

}

void AbstractAutoRecognize::clearInfo()
{
    m_motorsInfo.clear();
    m_ipMotors.clear();
}

void AbstractAutoRecognize::addMototInfo(uint8_t nDeviceId, uint32_t nDeviceMac, uint32_t communicationId)
{
//    if(m_bUniqueId)
//    {
//        std::pair<std::multimap<uint32_t,std::pair<uint8_t,uint32_t>>::iterator,std::multimap<uint32_t,std::pair<uint8_t,uint32_t>>::iterator> mRet;
//        mRet = m_ipMotors.equal_range(communicationId);
//        for(std::multimap<uint32_t,std::pair<uint8_t,uint32_t>>::iterator it = mRet.first;it!=mRet.second;++it)
//        {
//            if(it->second.first==nDeviceId)
//            {
//                mediator->errorOccur(nDeviceId,Actuator::ERR_ID_UNUNIQUE,
//                                     string_format("There are at least two motors with the same ID:%d,\n please check and set suitably!",nDeviceId));
//                m_bUniqueId = false;
//                m_ipMotors.clear();
//                break;
//            }
//        }

//        if(m_bUniqueId)
//        {
//            m_ipMotors.insert({communicationId,{nDeviceId,nDeviceMac}});
//        }
//    }
    m_ipMotors.insert({{nDeviceId,communicationId},nDeviceMac});

}
