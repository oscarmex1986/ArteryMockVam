//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 
#ifndef EXAMPLESERVICE_H_
#define EXAMPLESERVICE_H_

#include "artery/application/ItsG5Service.h"
#include <omnetpp/simtime.h>
#include "artery/application/ItsG5BaseService.h"
#include "artery/utility/Geometry.h"
#include <vanetza/asn1/cam.hpp>
#include <vanetza/btp/data_interface.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <string> //OAM



namespace artery
{

class NetworkInterfaceTable;
class Timer;
class VehicleDataProvider;

class ExampleService : public ItsG5BaseService
{
    public:
        ExampleService();
        ~ExampleService();
        void initialize() override;
        void indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>) override;
        void trigger() override;



    private:
        
        void checkTriggeringConditions(const omnetpp::SimTime&);
        bool checkHeadingDelta() const;
        bool checkPositionDelta() const;
        bool checkSpeedDelta() const;
        void sendExa(const omnetpp::SimTime&); //SendCam
        omnetpp::SimTime genExaDcc(); //genCamDcc

        ChannelNumber mPrimaryChannel = channel::CCH;
        const NetworkInterfaceTable* mNetworkInterfaceTable = nullptr;
        const VehicleDataProvider* mVehicleDataProvider;
        const Timer* mTimer;
        LocalDynamicMap* mLocalDynamicMap = nullptr;

        omnetpp::cModule* mHost = nullptr;

        omnetpp::SimTime mGenCamMin;
        omnetpp::SimTime mGenCamMean;
        omnetpp::SimTime mGenCamMax;
        omnetpp::SimTime mGenCam;
        unsigned mGenCamLowDynamicsCounter;
        unsigned mGenCamLowDynamicsLimit;
        Position mLastCamPosition;
        vanetza::units::Velocity mLastCamSpeed;
        vanetza::units::Angle mLastCamHeading;
        omnetpp::SimTime mLastCamTimestamp;
        omnetpp::SimTime mLastLowCamTimestamp;
        vanetza::units::Angle mHeadingDelta;
        vanetza::units::Length mPositionDelta;
        vanetza::units::Velocity mSpeedDelta;
        bool mDccRestriction;
        bool mFixedRate;
        unsigned long contTx=0; //OAM Contador de CAMs
        long int myId; //OAM Id para el LLDM
        const char* nodeName; //OAM Nombre del nodo para el archivo con las métricas
        const char* carOrRsu; //OAM Nombre del nodo para el archivo con las métricas
        int nodeIndex;
        double delayCamDcc;
        int flagDcc=1;
        omnetpp::cMessage* m_self_msg;
        //int messageID;
        int recs = 0;
        int txs = 0;
        vanetza::geonet::Area buildDestinationArea(const VehicleDataProvider&);
        double waitMilliSeconds;
        bool multiHop;
        int flagtxd = 0;

};

vanetza::asn1::Cam createExampleMessage(const VehicleDataProvider&, uint16_t genDeltaTime);
void addExaLowFrequencyContainer(vanetza::asn1::Cam&, unsigned pathHistoryLength = 0);

} // namespace artery

#endif /* EXAMPLESERVICE_H_ */
