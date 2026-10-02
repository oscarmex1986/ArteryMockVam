/*
* Artery V2X Simulation Framework
* Copyright 2014-2019 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#include "ExampleService.h"
#include "artery/application/CaObject.h"
#include "artery/application/CaService.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/MultiChannelPolicy.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/utility/simtime_cast.h"
#include "veins/base/utils/Coord.h"
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <omnetpp/cexception.h>
#include <vanetza/btp/ports.hpp>
#include <vanetza/dcc/transmission.hpp>
#include <vanetza/dcc/transmit_rate_control.hpp>
#include <vanetza/facilities/cam_functions.hpp>
#include <chrono>
#include <string> //OAM
#include <random> //OAM
#include <iostream> //OAM
#include <stdio.h> //OAM
#include "veins/base/utils/FindModule.h" //OAM
using namespace std;




namespace artery
{

using namespace omnetpp;


auto microdegree2 = vanetza::units::degree * boost::units::si::micro;
auto decidegree2 = vanetza::units::degree * boost::units::si::deci;
auto degree_per_second2 = vanetza::units::degree / vanetza::units::si::second;
auto centimeter_per_second2 = vanetza::units::si::meter_per_second * boost::units::si::centi;

//OAM VARIABLES PARA MÉTRICAS DE CASERVICE BEGIN
int flagExa = 1; //OAM Flag en 1 para registrar las distancias y en 0 para no registrar
int headerFlagExa = 1;
int flagCounterExa = 1; //
int messageIDExa;
double meanWait = 3;

// OAM GUARDAR LOS DATOS DEL VEHICULO EN EL CENTRO DEL DESTINO
//const VehicleDataProvider* brokenVehicleVdp;

static const int numCars = 40000 + 1; //OAM para LLDM
static const int numVehicles = 504;
static const int centralNode = 150; //OAM 103 PARA 300C, 46 PARA NODO ESQUINA;
long double lldm2[numCars][2];
double recTimeExa = 100;
double endTimeExa = 160;
int messageCounterExa = 3;
long double x_ce = -36917700; // X Nuevos Ministerios
long double y_ce = 404412510; // Y Nuevos Ministerios
long double x_sw = -37050130; // X Quevedo
long double y_sw = 404319490; // Y Quevedo
long double x_ne = -36828270; // X Chamartin
long double y_ne = 404541580; // Y Chamartin
long double x_nw = -37027140; // X Tetuan
long double y_nw = 404510460; // Y Tetuan
long double x_se = -36753220; // X Salamanca
long double y_se = 404292130; // Y Salamanca
long double x_hwy_early = 98388380; // X HWY EARLY
long double y_hwy_early = 54374250; // Y HWY EARLY
long double x_hwy_late = 98388400; // X HWY LATE
long double y_hwy_late = 54374250; // Y HWY LATE
long double x_dhwy_early = 98702280; // X HWY EARLY
long double y_dhwy_early = 54373830; // Y HWY EARLY
long double x_dhwy_late = 98702290; // X HWY LATE
long double y_dhwy_late = 54373830; // Y HWY LATE
long double xlim2a = 98603640;
long double xlim2b = 98611880;
long double xlim2c = 99128000;
long double xlim2d = 99150360;
long double ylim2 = 5437300;
long double xpinchado = x_hwy_late; // 98605230	 Offset 98388380 Center 98605230 MadridSJDLC -36917710
long double ypinchado = y_hwy_late; // 54373960 Offset 54374250 Center 54373960 MadridSJDLC 404412520
long double xpinchadofin = -36917690; 
long double ypinchadofin = 404412470;
// -36917690	404412470 98388380	 Offset Y 54374250
long double xdest = x_dhwy_late;
long double ydest =	y_dhwy_late;
/*
Pinchado
Early 98388380
Late 98388400

Dest
Early 98702280
Late 98702290

98388400	54374250
98702290	54373830

98388400	54374250
98702280	54373830

98388400	54374250
98702290	54373830




*/

/* COORDINADAS MADRID
120-150 segundos
Quevedo		x -37050130	y 404319490
Centro 		x -36917700	y 404412510
Chamartin 	x -36828270 y 404541580
Tetuan		x -37027140 y 404510460
Salamanca	x -36804840 y 404326090

*/

long double destinationX = 98900410; // 98927400
long double destinationY = 54293640; // 54248590


//(myX>=98918690 && myX<=99020330)||(myX>=99128000 && myX<=99150360))&&( myY >= 54290000)
FILE *myfile3;
FILE *myfile32;
FILE *dumpDistances;
//OAM END


static const simsignal_t scSignalExaReceived = cComponent::registerSignal("ExaReceived");
static const simsignal_t scSignalExaSent = cComponent::registerSignal("ExaSent");
static const auto scLowFrequencyContainerInterval = std::chrono::milliseconds(2000);//OAM Original 500 Obligar a que envíe un CAM completo con el contenedor de baja frecuencia
//static const simsignal_t SentExa = cComponent::registerSignal("SentExa"); //OAM Registrar el tiempo de generación del CAM

template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
	boost::units::quantity<U> v { q };
	return std::round(v.value());
}

SpeedValue_t buildSpeedValueExa(const vanetza::units::Velocity& v)
{
	static const vanetza::units::Velocity lower { 0.0 * boost::units::si::meter_per_second };
	static const vanetza::units::Velocity upper { 163.82 * boost::units::si::meter_per_second };

	SpeedValue_t speed = SpeedValue_unavailable;
	if (v >= upper) {
		speed = 16382; // see CDD A.74 (TS 102 894 v1.2.1)
	} else if (v >= lower) {
		speed = round(v, centimeter_per_second2) * SpeedValue_oneCentimeterPerSec;
	}
	return speed;
}

Define_Module(ExampleService)

ExampleService::ExampleService() :
		mVehicleDataProvider(nullptr),
		mTimer(nullptr),
		mGenCamMin { 100, SIMTIME_MS }, //OAM Original 100
		mGenCamMax { 5000, SIMTIME_MS },//OAM Original 1000
		mGenCamMean { 1000, SIMTIME_MS },//OAM Nuevo Par
		mGenCam(mGenCamMax),
		mGenCamLowDynamicsCounter(0),
		mGenCamLowDynamicsLimit(3) //OAM default 3
{
}

void ExampleService::initialize()
{
	
	//OAM Inicializar el array que registra las distancias hacia el nodo central
	mHost = veins::FindModule<>::findHost(this);
	//nodeName = findHost()->getFullName();
	nodeName = mHost->getFullName();
	nodeName = &nodeName[5];
	std::string hostName(nodeName);
	hostName = hostName.substr(0,hostName.size()-1);
	nodeIndex = std::stoi (hostName);
	if (headerFlagExa == 1){
		headerFlagExa = 0;
		myfile3 = fopen("RecExa.csv", "w");
		fprintf(myfile3, "%s, %s, %s, %s, %s, %s, %s, %s, %s, %s\n", "RxNodeName", "RxStationID", "X", "Y", "timestamp_rx", "TxStationID", "messageID", "distance", "side","AboveBelow");
		fclose(myfile3);
		myfile32 = fopen("SentExa.csv", "w");
		fprintf(myfile32,"%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n", "nodeName", "TxStationID", "messageID", "timestamp_gen","x","y", "delayDcc", "250L", "500L", "750L", "1000L", "1250L", "1500L", "1750L", "2000L", "2250L", "250R", "500R", "750R", "1000R", "1250R", "1500R", "1750R", "2000R", "2250R", "2500R", "2750R", "3000R", "3250R", "3500R", "3750R", "4000R");
		fclose(myfile32);
		dumpDistances = fopen("Distancias.csv", "w");
		fprintf(dumpDistances,"%s,%s,%s,%s,%s,%s\n","TxNodeIndex","RxNodeName","messageID","Distance","timestamp_gen","side");
		fclose(dumpDistances);
		
	}
	//if (nodeIndex==centralNode) flagCounter = 1; //OAM Si queremos métricas para más nodos como el 1 que es muy extremo, colocarlo en este if
	//OAM End
	/**/
	ItsG5BaseService::initialize();
	mNetworkInterfaceTable = &getFacilities().get_const<NetworkInterfaceTable>();
	mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
	mTimer = &getFacilities().get_const<Timer>();
	mLocalDynamicMap = &getFacilities().get_mutable<artery::LocalDynamicMap>();

	// avoid unreasonable high elapsed time values for newly inserted vehicles
	mLastCamTimestamp = simTime();

	// first generated CAM shall include the low frequency container
	mLastLowCamTimestamp = mLastCamTimestamp - artery::simtime_cast(scLowFrequencyContainerInterval);
	
	// generation rate boundaries
	mGenCamMin = par("minInterval");
	mGenCamMax = par("maxInterval");
	mGenCamMean = par("meanInterval");
	mGenCam = exponential(mGenCamMean,0);


	// vehicle dynamics thresholds
	mHeadingDelta = vanetza::units::Angle { par("headingDelta").doubleValue() * vanetza::units::degree };
	mPositionDelta = par("positionDelta").doubleValue() * vanetza::units::si::meter;
	mSpeedDelta = par("speedDelta").doubleValue() * vanetza::units::si::meter_per_second;

	mDccRestriction = par("withDccRestriction");
	mFixedRate = par("fixedRate");

	// look up primary channel for CA
	mPrimaryChannel = getFacilities().get_const<MultiChannelPolicy>().primaryChannel(vanetza::aid::CA);

	//m_self_msg = new cMessage("Example Service");
	//subscribe(scSignalExaReceived);
	multiHop = par("multiHop");

	//scheduleAt(simTime() + 3.0, m_self_msg);
}


void ExampleService::trigger()
{
	Enter_Method("trigger");
	//OAM actualizar la ubicación propia en el LLDM - Begin
	lldm2[nodeIndex][0] = round(mVehicleDataProvider->longitude(), microdegree2) * Longitude_oneMicrodegreeEast;
	lldm2[nodeIndex][1] = round(mVehicleDataProvider->latitude(), microdegree2) * Latitude_oneMicrodegreeNorth;
	//OAM End
	//if(SIMTIME_DBL(simTime())>recTimeExa &&(lldm2[nodeIndex][0]>=xlim2a && lldm2[nodeIndex][0]<=xlim2b)&&( lldm2[nodeIndex][1] >= ylim2)){
	//if(SIMTIME_DBL(simTime())>recTimeExa && SIMTIME_DBL(simTime())<endTimeExa && (lldm2[nodeIndex][0]==xpinchado && lldm2[nodeIndex][1] <= ypinchado && lldm2[nodeIndex][0] <= xpinchadofin && lldm2[nodeIndex][1] >= ypinchadofin)){
		//OAM End
		//if (lldm2[nodeIndex][0] == xdest && lldm2[nodeIndex][1] == ydest){
		//	brokenVehicleVdp = mVehicleDataProvider;
		//}

		if (SIMTIME_DBL(simTime())>recTimeExa && SIMTIME_DBL(simTime())<endTimeExa){
		//if(SIMTIME_DBL(simTime())>recTimeExa && SIMTIME_DBL(simTime())<endTimeExa && (lldm2[nodeIndex][0]==xpinchado && lldm2[nodeIndex][1] == ypinchado )){
		//if (nodeIndex == 0 && SIMTIME_DBL(simTime())>recTimeExa && SIMTIME_DBL(simTime())<endTimeExa){
			//if (nodeIndex < numVehicles){
				//flagtxd=1;
				
				checkTriggeringConditions(simTime());
			//}
		}
	
}

void ExampleService::indicate(const vanetza::btp::DataIndication& ind, std::unique_ptr<vanetza::UpPacket> packet)
{
	Enter_Method("indicate");
	Asn1PacketVisitor<vanetza::asn1::Cam> visitor;
	const vanetza::asn1::Cam* cam = boost::apply_visitor(visitor, *packet);
	const auto egoStationID = getFacilities().get_const<VehicleDataProvider>().station_id();
	
	if (cam && cam->validate()) {
		CaObject obj = visitor.shared_wrapper;
		emit(scSignalExaReceived, &obj);
		mLocalDynamicMap->updateAwareness(obj);
		//OAM Registrar la distancia del nodo hacia el nodo central - Begin
		long double myX = round(mVehicleDataProvider->longitude(), microdegree2) * Longitude_oneMicrodegreeEast;
		long double myY = round(mVehicleDataProvider->latitude(), microdegree2) * Latitude_oneMicrodegreeNorth;
		const vanetza::asn1::Cam& msg = obj.asn1();
		if (msg->header.stationID != myId){
			
				long double visitorX = msg->cam.camParameters.basicContainer.referencePosition.longitude;
				
				long double visitorY = msg->cam.camParameters.basicContainer.referencePosition.latitude;

				myId = mVehicleDataProvider->station_id();
				//long double distance = (sqrt(pow((myX - visitorX)/1.18, 2) + pow((myY - visitorY)/0.90, 2)))/100;
				long double distance = (sqrt(pow((myX - visitorX), 2) + pow((myY - visitorY), 2)))/100;
				distance = distance * 1.12;
				myfile3 = fopen("RecExa.csv", "a");
				if (myX < xdest){
					if((myY < ydest)){
						fprintf(myfile3, "%s, %li, %Lf, %Lf, %f, %lu, %li, %Lf,%s,%s\n", findHost()->getFullName(), myId,myX,myY, SIMTIME_DBL(simTime()),msg->header.stationID, msg->header.messageID, distance,"L","B");
					} else {
						fprintf(myfile3, "%s, %li, %Lf, %Lf, %f, %lu, %li, %Lf,%s,%s\n", findHost()->getFullName(), myId,myX,myY, SIMTIME_DBL(simTime()),msg->header.stationID, msg->header.messageID, distance,"L","A");	
					}
					fclose(myfile3); 
				} else  {
					if((myY < ydest)){
						fprintf(myfile3, "%s, %li, %Lf, %Lf, %f, %lu, %li, %Lf,%s,%s\n", findHost()->getFullName(), myId,myX,myY, SIMTIME_DBL(simTime()),msg->header.stationID, msg->header.messageID, distance,"R","B");
					} else {
						fprintf(myfile3, "%s, %li, %Lf, %Lf, %f, %lu, %li, %Lf,%s,%s\n", findHost()->getFullName(), myId,myX,myY, SIMTIME_DBL(simTime()),msg->header.stationID, msg->header.messageID, distance,"R","A");	
					}
					fclose(myfile3);
				}
		}//OAM End
		
	}

}

ExampleService::~ExampleService()
{
	cancelAndDelete(m_self_msg);
}

void ExampleService::checkTriggeringConditions(const SimTime& T_now)
{
	
	// provide variables named like in EN 302 637-2 V1.3.2 (section 6.1.3)
	//SimTime& T_GenCam = mGenCam;
	const SimTime& T_GenCamMin = mGenCamMin;
	const SimTime& T_GenCamMax = mGenCamMax;
	const SimTime T_elapsed = T_now - mLastCamTimestamp;
	//const SimTime T_GenCamDcc = mDccRestriction ? genExaDcc() : mGenCamMin;

	//OAM RANDOMIZAR EL TIEMPO
	const SimTime T_GenCamMean = mGenCam;	

	//OAM End
	delayCamDcc =  SIMTIME_DBL(T_GenCamMean);

	//BEGIN ORIGINAL
	
	if (nodeIndex <= numVehicles) {
	if (T_elapsed >= T_GenCamMin) { //OAM Desacoplar del feedback DCC sin alterar Min/Max Gen
		if (mFixedRate) {

			sendExa(T_now);
		} else if (checkHeadingDelta() || checkPositionDelta() || checkSpeedDelta()) {
			//std::cout << "Dynamics" ;
			sendExa(T_now);
			
	//		T_GenCam = std::min(T_elapsed, T_GenCamMax); /*< if middleware update interval is too long */
	//		mGenCamLowDynamicsCounter = 0;
		} else if (T_elapsed >= T_GenCamMax) {//OAM Original T_GenCam
			sendExa(T_now);
	//		if (++mGenCamLowDynamicsCounter >= mGenCamLowDynamicsLimit) {
	//			T_GenCam = T_GenCamMax;
	//		}
		}//END ORIGINAL
	}	
	//BEGIN OAM MODIFIED
	//if (T_elapsed >= T_GenCamDcc) {
	//	if (checkHeadingDelta() || checkPositionDelta() || checkSpeedDelta()) {
	//		sendCam(T_now);
			
	//		T_GenCam = std::min(T_elapsed, T_GenCamMax); /*< if middleware update interval is too long */
	//		mGenCamLowDynamicsCounter = 0;
	//	} 
	//} else if (T_elapsed >= T_GenCam) {//OAM Original T_GenCam
	//		sendCam(T_now);
	//		if (++mGenCamLowDynamicsCounter >= mGenCamLowDynamicsLimit) {
	//			T_GenCam = T_GenCamMax;
	//		}
	} //END MODIFIED
}

bool ExampleService::checkHeadingDelta() const
{
	return !vanetza::facilities::similar_heading(mLastCamHeading, mVehicleDataProvider->heading(), mHeadingDelta);
}

bool ExampleService::checkPositionDelta() const
{
	return (distance(mLastCamPosition, mVehicleDataProvider->position()) > mPositionDelta);
}

bool ExampleService::checkSpeedDelta() const
{
	return abs(mLastCamSpeed - mVehicleDataProvider->speed()) > mSpeedDelta;
}

void ExampleService::sendExa(const SimTime& T_now)
{
	uint16_t genDeltaTimeMod = countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
	auto cam = createExampleMessage(*mVehicleDataProvider, genDeltaTimeMod);

	using vanetza::units::si::seconds;

	mLastCamPosition = mVehicleDataProvider->position();
	mLastCamSpeed = mVehicleDataProvider->speed();
	mLastCamHeading = mVehicleDataProvider->heading();
	mLastCamTimestamp = T_now;
	if (T_now - mLastLowCamTimestamp >= artery::simtime_cast(scLowFrequencyContainerInterval)) {
		addLowFrequencyContainer(cam, par("pathHistoryLength"));
		mLastLowCamTimestamp = T_now;
	}

	using namespace vanetza;
	

	btp::DataRequestB request;
	request.destination_port = host_cast<ExampleService::port_type>(getPortNumber());
	request.gn.its_aid = aid::CA;
	//request.gn.transport_type = geonet::TransportType::SHB;
	request.gn.maximum_lifetime = vanetza::geonet::Lifetime { vanetza::geonet::Lifetime::Base::One_Second, 10 }; //OAM originally 10 secs
	request.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
	if(multiHop){
		request.gn.transport_type = vanetza::geonet::TransportType::GBC;
		//request.gn.destination = buildDestinationArea(*mVehicleDataProvider);
		//request.gn.destination = buildDestinationArea(*brokenVehicleVdp);
		//geonet::DataRequest::Repetition repetition;
		//repetition.interval = 1 * seconds;
		//repetition.maximum = 30 * seconds;
		//request.gn.repetition = repetition;
	} else {
		request.gn.transport_type = vanetza::geonet::TransportType::SHB;
	}
	request.gn.communication_profile = vanetza::geonet::CommunicationProfile::ITS_G5;
	
	

	long double myX = round(mVehicleDataProvider->longitude(), microdegree2) * Longitude_oneMicrodegreeEast;
	long double myY = round(mVehicleDataProvider->latitude(), microdegree2) * Latitude_oneMicrodegreeNorth;
	
	if(SIMTIME_DBL(simTime())>recTimeExa ){
		int to250L = 0;
		int to500L = 0;
		int to750L = 0;
		int to1000L = 0;
		int to1250L = 0;
		int to1500L = 0;
		int to1750L = 0;
		int to2000L = 0;
		int to2250L = 0;
		int to2500L = 0;
		int to2750L = 0;
		int to3000L = 0;
		int to3250L = 0;
		int to3500L = 0;
		int to3750L = 0;
		int to4000L = 0;
		int to250R = -1;
		int to500R = 0;
		int to750R = 0;
		int to1000R = 0;
		int to1250R = 0;
		int to1500R = 0;
		int to1750R = 0;
		int to2000R = 0;
		int to2250R = 0;
		int to2500R = 0;
		int to2750R = 0;
		int to3000R = 0;
		int to3250R = 0;
		int to3500R = 0;
		int to3750R = 0;
		int to4000R = 0;
		
		
		for (int i = 0; i < numCars; i++){
			
			long double visitorX = lldm2[i][0];
			long double visitorY = lldm2[i][1];
			if (visitorX < xdest){
				//long double distance = (sqrt(pow((myX - visitorX)/1.18, 2) + pow((myY - visitorY)/0.90, 2)))/100;
				long double distance = (sqrt(pow((myX - visitorX), 2) + pow((myY - visitorY), 2)))/100;
				distance = distance * 1.12;
				if (distance >= 0 && distance <= 250) to250L++;
				if (distance > 250 && distance <= 500) to500L++;
				if (distance > 500 && distance <= 750) to750L++;
				if (distance > 750 && distance <= 1000) to1000L++;
				if (distance > 1000 && distance <= 1250) to1250L++;
				if (distance > 1250 && distance <= 1500) to1500L++;
				if (distance > 1500 && distance <= 1750) to1750L++;
				if (distance > 1750 && distance <= 2000) to2000L++;
				if (distance > 2000 && distance <= 2250) to2250L++;
				if (distance > 2250 && distance <= 2500) to2500L++;
				if (distance > 2500 && distance <= 2750) to2750L++;
				if (distance > 2750 && distance <= 3000) to3000L++;
				if (distance > 3000 && distance <= 3250) to3250L++;
				if (distance > 3250 && distance <= 3500) to3500L++;
				if (distance > 3500 && distance <= 3750) to3750L++;
				if (distance > 3750 && distance <= 4000) to4000L++;

				if (distance <= 4000 && nodeIndex!=i){
					//dumpDistances = fopen("Distancias.csv", "a");
					//fprintf(dumpDistances,"node[%i],node[%i],%i,%Lf,%f,%s\n", nodeIndex, i, messageCounterExa, distance, SIMTIME_DBL(simTime()),"L");
					//fclose(dumpDistances);
				}
			} else {
				//long double distance = (sqrt(pow((myX - visitorX)/1.18, 2) + pow((myY - visitorY)/0.90, 2)))/100;
				long double distance = (sqrt(pow((myX - visitorX), 2) + pow((myY - visitorY), 2)))/100;
				distance = distance * 1.12;
				if (distance >= 0 && distance <= 250) to250R++;
				if (distance > 250 && distance <= 500) to500R++;
				if (distance > 500 && distance <= 750) to750R++;
				if (distance > 750 && distance <= 1000) to1000R++;
				if (distance > 1000 && distance <= 1250) to1250R++;
				if (distance > 1250 && distance <= 1500) to1500R++;
				if (distance > 1500 && distance <= 1750) to1750R++;
				if (distance > 1750 && distance <= 2000) to2000R++;
				if (distance > 2000 && distance <= 2250) to2250R++;
				if (distance > 2250 && distance <= 2500) to2500R++;
				if (distance > 2500 && distance <= 2750) to2750R++;
				if (distance > 2750 && distance <= 3000) to3000R++;
				if (distance > 3000 && distance <= 3250) to3250R++;
				if (distance > 3250 && distance <= 3500) to3500R++;
				if (distance > 3500 && distance <= 3750) to3750R++;
				if (distance > 3750 && distance <= 4000) to4000R++;

				if (distance <= 4000 && nodeIndex!=i){
					//dumpDistances = fopen("Distancias.csv", "a");
					//fprintf(dumpDistances,"node[%i],node[%i],%i,%Lf,%f,%s\n", nodeIndex, i, messageCounterExa,distance, SIMTIME_DBL(simTime()),"R");
					//fclose(dumpDistances);
				}
			
			}
		} 
		myId = mVehicleDataProvider->station_id();
		myfile32 = fopen("SentExa.csv","a");
		fprintf(myfile32,"%s, %li, %i, %f, %Lf, %Lf, %f, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i\n", findHost()->getFullName(), myId, messageIDExa, SIMTIME_DBL(T_now), myX, myY, delayCamDcc,to250L, to500L, to750L, to1000L, to1250L, to1500L, to1750L, to2000L, to2250L,to250R, to500R, to750R, to1000R, to1250R, to1500R, to1750R, to2000R, to2250R, to2500R, to2750R, to3000R, to3250R, to3500R, to3750R, to4000R);
		fclose(myfile32);
	}
	//OAM End
	
	//if (flagtxd == 0){
	//std::cout << "Transmitting Generated Denm" ;
	CaObject obj(std::move(cam));
	emit(scSignalExaSent, &obj);
		//OAM Envía señal que registra en un vector la generación de un CAM - BEGIN
	//contTx = contTx + 1;
		//emit(SentExa, contTx);
		//OAM END
	using CamByteBuffer = convertible::byte_buffer_impl<asn1::Cam> ;
	std::unique_ptr<vanetza::geonet::DownPacket> payload { new vanetza::geonet::DownPacket() };
	std::unique_ptr<convertible::byte_buffer> buffer { new CamByteBuffer(obj.shared_ptr()) };
	payload->layer(OsiLayer::Application) = std::move(buffer);
	this->request(request, std::move(payload));
	
	//mGenCamMin = mGenCamMax;
	flagtxd = 1;
	//} 
	/**/
}

SimTime ExampleService::genExaDcc()
{
	// network interface may not be ready yet during initialization, so look it up at this later point
	auto netifc = mNetworkInterfaceTable->select(mPrimaryChannel);
	vanetza::dcc::TransmitRateThrottle* trc = netifc ? netifc->getDccEntity().getTransmitRateThrottle() : nullptr;
	if (!trc) {
		throw cRuntimeError("No DCC TRC found for CA's primary channel %i", mPrimaryChannel);
	}

	static const vanetza::dcc::TransmissionLite ca_tx(vanetza::dcc::Profile::DP1, 0);
	vanetza::Clock::duration interval = trc->interval(ca_tx);
	SimTime dcc { std::chrono::duration_cast<std::chrono::milliseconds>(interval).count(), SIMTIME_MS };
	//return std::min(mGenCamMax, std::max(mGenCamMin, dcc));
	return std::min(mGenCamMax, std::max(mGenCamMin, {100, SIMTIME_MS})); //OAM Obligar a generar cada 100 ms
}

vanetza::asn1::Cam createExampleMessage(const VehicleDataProvider& vdp, uint16_t genDeltaTime)
{
	vanetza::asn1::Cam message;

	ItsPduHeader_t& header = (*message).header;
	header.protocolVersion = 1;
	//OAM Identificar cada mensaje con una ID diferente - Begin
	long double myX = round(vdp.longitude(), microdegree2) * Longitude_oneMicrodegreeEast;
	long double myY = round(vdp.latitude(), microdegree2) * Latitude_oneMicrodegreeNorth;
	//if(SIMTIME_DBL(simTime())>recTimeExa &&(myX>=xlim2a && myX<=xlim2b)&&( myY >= ylim2)){
	if(SIMTIME_DBL(simTime())>recTimeExa ){
		header.messageID = messageCounterExa;
		messageIDExa = messageCounterExa;
		messageCounterExa = messageCounterExa + 1;
	} else {
		header.messageID = ItsPduHeader__messageID_cam;
		
		messageIDExa = header.messageID;
	}
	//OAM End/**/
	//header.messageID = ItsPduHeader__messageID_cam; // OAM Linea original de Message ID
	
	header.stationID = vdp.station_id();

	CoopAwareness_t& cam = (*message).cam;
	cam.generationDeltaTime = genDeltaTime * GenerationDeltaTime_oneMilliSec;
	BasicContainer_t& basic = cam.camParameters.basicContainer;
	HighFrequencyContainer_t& hfc = cam.camParameters.highFrequencyContainer;

	basic.stationType = StationType_passengerCar;
	basic.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
	basic.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
	basic.referencePosition.longitude = round(vdp.longitude(), microdegree2) * Longitude_oneMicrodegreeEast;
	basic.referencePosition.latitude = round(vdp.latitude(), microdegree2) * Latitude_oneMicrodegreeNorth;
    //basic.referencePosition.longitude = round(brokenVehicleVdp->longitude(), microdegree2) * Longitude_oneMicrodegreeEast;
	//basic.referencePosition.latitude = round(brokenVehicleVdp->latitude(), microdegree2) * Latitude_oneMicrodegreeNorth;

	basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
	basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence =
			SemiAxisLength_unavailable;
	basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence =
			SemiAxisLength_unavailable;

	hfc.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
	BasicVehicleContainerHighFrequency& bvc = hfc.choice.basicVehicleContainerHighFrequency;
	bvc.heading.headingValue = round(vdp.heading(), decidegree2);
	bvc.heading.headingConfidence = HeadingConfidence_equalOrWithinOneDegree;
	bvc.speed.speedValue = round(vdp.speed(), centimeter_per_second2) * SpeedValue_oneCentimeterPerSec;
	bvc.speed.speedConfidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec * 3;
	bvc.driveDirection = vdp.speed().value() >= 0.0 ?
			DriveDirection_forward : DriveDirection_backward;
	const double lonAccelValue = vdp.acceleration() / vanetza::units::si::meter_per_second_squared;
	// extreme speed changes can occur when SUMO swaps vehicles between lanes (speed is swapped as well)
	if (lonAccelValue >= -160.0 && lonAccelValue <= 161.0) {
		bvc.longitudinalAcceleration.longitudinalAccelerationValue = lonAccelValue * LongitudinalAccelerationValue_pointOneMeterPerSecSquaredForward;
	} else {
		bvc.longitudinalAcceleration.longitudinalAccelerationValue = LongitudinalAccelerationValue_unavailable;
	}
	bvc.longitudinalAcceleration.longitudinalAccelerationConfidence = AccelerationConfidence_unavailable;
	bvc.curvature.curvatureValue = abs(vdp.curvature() / vanetza::units::reciprocal_metre) * 10000.0;
	if (bvc.curvature.curvatureValue >= 1023) {
		bvc.curvature.curvatureValue = 1023;
	}
	bvc.curvature.curvatureConfidence = CurvatureConfidence_unavailable;
	bvc.curvatureCalculationMode = CurvatureCalculationMode_yawRateUsed;
	bvc.yawRate.yawRateValue = round(vdp.yaw_rate(), degree_per_second2) * YawRateValue_degSec_000_01ToLeft * 100.0;
	if (abs(bvc.yawRate.yawRateValue) >= YawRateValue_unavailable) {
		bvc.yawRate.yawRateValue = YawRateValue_unavailable;
	}
	bvc.vehicleLength.vehicleLengthValue = VehicleLengthValue_unavailable;
	bvc.vehicleLength.vehicleLengthConfidenceIndication =
			VehicleLengthConfidenceIndication_noTrailerPresent;
	bvc.vehicleWidth = VehicleWidth_unavailable;

	std::string error;
	if (!message.validate(error)) {
		throw cRuntimeError("Invalid High Frequency CAM: %s", error.c_str());
	}

	return message;
}

void addExaLowFrequencyContainer(vanetza::asn1::Cam& message, unsigned pathHistoryLength)
{
	if (pathHistoryLength > 40) {
		EV_WARN << "path history can contain 40 elements at maximum";
		pathHistoryLength = 40;
	}

	LowFrequencyContainer_t*& lfc = message->cam.camParameters.lowFrequencyContainer;
	lfc = vanetza::asn1::allocate<LowFrequencyContainer_t>();
	lfc->present = LowFrequencyContainer_PR_basicVehicleContainerLowFrequency;
	BasicVehicleContainerLowFrequency& bvc = lfc->choice.basicVehicleContainerLowFrequency;
	bvc.vehicleRole = VehicleRole_default;
	bvc.exteriorLights.buf = static_cast<uint8_t*>(vanetza::asn1::allocate(1));
	assert(nullptr != bvc.exteriorLights.buf);
	bvc.exteriorLights.size = 1;
	bvc.exteriorLights.buf[0] |= 1 << (7 - ExteriorLights_daytimeRunningLightsOn);

	for (int i = 0; i < pathHistoryLength; ++i) {
		PathPoint* pathPoint = vanetza::asn1::allocate<PathPoint>();
		pathPoint->pathDeltaTime = vanetza::asn1::allocate<PathDeltaTime_t>();
		*(pathPoint->pathDeltaTime) = 0;
		pathPoint->pathPosition.deltaLatitude = DeltaLatitude_unavailable;
		pathPoint->pathPosition.deltaLongitude = DeltaLongitude_unavailable;
		pathPoint->pathPosition.deltaAltitude = DeltaAltitude_unavailable;
		ASN_SEQUENCE_ADD(&bvc.pathHistory, pathPoint);
	}

	std::string error;
	if (!message.validate(error)) {
		throw cRuntimeError("Invalid Low Frequency CAM: %s", error.c_str());
	}
}

vanetza::geonet::Area ExampleService::buildDestinationArea(const VehicleDataProvider& vdp)
{
    namespace geonet = vanetza::geonet;
    using vanetza::units::si::seconds;
    using vanetza::units::si::meter;
    using vanetza::units::si::meters;
    static const auto microdegree = vanetza::units::si::micro * vanetza::units::degree;
    /*
    geonet::Area area;
    geonet::Circle destination_shape;
    destination_shape.r = 500.0 * meter; //1784.0
    area.shape = destination_shape;
    area.position.latitude = vdp.latitude();
    area.position.longitude = vdp.longitude();
	*/
	vanetza::geonet::Area area;
	vanetza::geonet::Rectangle destination_shape;
	destination_shape.a = 100.0 * meter;
	destination_shape.b = 1000.00 * meter;
	area.shape = destination_shape;
	area.position.latitude = vdp.latitude();
    area.position.longitude = vdp.longitude();
    
    //area.position.latitude = areaDeInteres.position.latitude;
    //area.position.longitude = areaDeInteres.position.latitude; 

    return area;
    
}


} // namespace artery