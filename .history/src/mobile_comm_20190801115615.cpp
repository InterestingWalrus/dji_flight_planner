#include "m100_flight_planner/mobile_comm.h"

using namespace DJI::OSDK;



MobileComm::MobileComm()
{
    mobile_data_service = nh.serviceClient<dji_sdk::SendMobileData>("dji_sdk/send_data_to_mobile");
        
}


void MobileComm::SendDataToMobile(unsigned char* data_to_mobile)
{
    dji_sdk::SendMobileData mobile_data;

    mobile_data.request.data.resize(10);
    memcpy(&mobile_data.request.data[0], data_to_mobile, 10 );

    mobile_data_service.call(mobile_data);

    bool mobile_response = mobile_data.response.result;
   

   // ROS_INFO_STREAM("Sent: " <<  mobile_data.request.data[0]<< mobile_data.request.data[1]);

   std::cout<<  static_cast<unsigned>(mobile_data.request.data[1]) << std::endl;
   
    ROS_INFO("SEND DATA TO MOBILE RESPONSE %i", mobile_response);

}

// Overloaded Function

void MobileComm::SendDataToMobile(AckReturnToMobile returnMobileAck)
{
    dji_sdk::SendMobileData mobile_data;

    mobile_data.request.data.resize(sizeof(AckReturnToMobile));
    memcpy(&mobile_data.request.data[0], (uint8_t *)(&returnMobileAck), sizeof(AckReturnToMobile));

    mobile_data_service.call(mobile_data);

    bool mobile_response = mobile_data.response.result;
      
   // ROS_INFO("SEND DATA TO MOBILE RESPONSE %i", mobile_response);

}



