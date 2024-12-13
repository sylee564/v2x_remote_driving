#include "ControlCmdReceiver.h"

ControlCmdReceiver::ControlCmdReceiver() {
    init_rosthread_msg_data();

}

void ControlCmdReceiver::create_and_run_ros_thread() {
    its_thread = std::thread(&rosloop_vehicle::ros_run_spin_loop<tod_msgs::ControlCmd>,
                             &its_ros_loop, "ControlCmdReceiverNode", "control_cmd_data",
                             &its_control_cmd_msg, &ros_terminated);
    its_thread_packet = std::thread(&rosloop_vehicle::ros_run_spin_loop<tod_msgs::PacketInfo>,
                             &its_ros_loop, "PacketInfoNode", "packets_info",
                             &its_packet_info_msg, &ros_terminated);
}

void ControlCmdReceiver::wait_for_ros_thread_to_join() {
    its_thread.join();
    its_thread_packet.join();   
}

void ControlCmdReceiver::callback_tod_network_client(const driver_tcpip_msgs::tcp_packet::ConstPtr& packet_msg) {
    
    // if(packet_msg.data.empty())
    //     printf("data empty!!!\n");
    // if(tod_status == tod_msgs::Status::TOD_STATUS_TELEOPERATION){
	if(packet_msg->data.size()>20){
		tod_msgs::PacketInfo tmp_packet;
		V2x_App_Hdr *hdr = (V2x_App_Hdr *)packet_msg->data.data();

		parsing_msgs((uint8_t *)packet_msg->data.data(), packet_msg->data.size());
		// std::copy(packet_msg.data.begin(), packet_msg.data.end(), m_buffer);
		tmp_packet.distance = calculate_distance(rx_latitude, rx_longitude, tx_latitude, tx_longitude);
		tmp_packet.latencyUsec = packet_msg->time_stamp - tx_time;
		tmp_packet.modem_latency = rx_modem_time - tx_modem_time;

		++number_received_packages;
		tmp_packet.seqNum = number_received_packages;
		tmp_packet.sizeBit = ntohs(hdr->len);
		tmp_packet.rx_v2x_db.eDeviceType = DB_V2X_DEVICE_TYPE_OBU;
		tmp_packet.rx_v2x_db.eTeleCommType = DB_V2X_TELECOMM_TYPE_5G_PC5;
		tmp_packet.rx_v2x_db.unDeviceId = 1;
		tmp_packet.rx_v2x_db.ulTimeStamp = packet_msg->time_stamp;
		tmp_packet.rx_v2x_db.eServiceId = DB_V2X_SERVICE_ID_REMOTE_DRIVING;
		tmp_packet.rx_v2x_db.eActionType = DB_V2X_ACTION_TYPE_RESPONSE;
		tmp_packet.rx_v2x_db.eRegionId = DB_V2X_REGION_ID_CHEONGJU;
		tmp_packet.rx_v2x_db.ePayloadType = DB_V2X_PAYLOAD_TYPE_REMOTE_DRIVING;
		
		if(rx_commId == DB_V2X_COMM_ID_V2I)
			tmp_packet.rx_v2x_db.eCommId = DB_V2X_COMM_ID_I2V;
		else if(rx_commId == DB_V2X_COMM_ID_V2V)
			tmp_packet.rx_v2x_db.eCommId = DB_V2X_COMM_ID_V2V;
		else if(rx_commId == DB_V2X_COMM_ID_I2V)
			tmp_packet.rx_v2x_db.eCommId = DB_V2X_COMM_ID_V2I;
		else
			tmp_packet.rx_v2x_db.eCommId = DB_V2X_COMM_ID_N2V;

		tmp_packet.rx_v2x_db.usDbVer = 1;
		tmp_packet.rx_v2x_db.usHwVer = 2;
		tmp_packet.rx_v2x_db.usSwVer = 3;
		tmp_packet.rx_v2x_db.ulPayloadLength = packet_msg->data.size();
		tmp_packet.tx_modem_status = its_tx_modem_status;
		tmp_packet.rx_modem_status = its_rx_modem_status;
		tmp_packet.tx_comm_status = its_tx_comm_status;
		tmp_packet.rx_comm_status = its_rx_comm_status;
		set_rosthread_msg_packet_info(tmp_packet);
		if ( number_received_packages >= USHRT_MAX ) {
			number_received_packages = 0;
		}
	}
    // }
    else if(packet_msg->data.size()>1 && packet_msg->data.size()<20 ){
        V2x_App_Hdr *hdr = (V2x_App_Hdr *)packet_msg->data.data();
        V2x_App_WSC *wsc = (V2x_App_WSC *)hdr->data;
        printf("Header payload ID : %04x\n", ntohs(hdr->payload_id));
        printf("action : %d, PSID : %d\n", wsc->action_result, ntohl(wsc->psid));
    }
}


void ControlCmdReceiver::callback_tod_status(const tod_msgs::Status::ConstPtr &status_msg) {
    tod_status = status_msg->tod_status;
}

void ControlCmdReceiver::parsing_msgs(uint8_t *msg, int len)
{
	int i, overall_len, package_len, package_remain_len;
	uint16_t *crc, cal_crc;
	void *p;
	TLVC_Overall *p_overall = NULL;
	V2x_App_Hdr *hdr = (V2x_App_Hdr *)msg;
	V2x_App_RxMsg *rx_msg = (V2x_App_RxMsg *)hdr->data;
	int psid = ntohl(rx_msg->psid);
	int flag_extensible_msg = 0;

	if (len > 0)
	{
		p_overall = (TLVC_Overall *)rx_msg->data;
	}
	else
	{
		printf("[Error] Extensible Message Lenth : %d\n", len);
        return;
	}
	

	if (ntohl(p_overall->type) != EM_PT_OVERALL)
	{
		printf("[Error] Overall Type - %u, need - %u\n", ntohl(p_overall->type), EM_PT_OVERALL);
		return ;
	}

	overall_len = ntohs(p_overall->len);		// V, C 길이
	package_remain_len = package_len = p_overall->len_package;
	// printf("Overall Package - Version : %d / Length : %d\n", p_overall->version, overall_len);
	// printf("Number of Packages = %d / All Length of Package= %d\n", p_overall->num_package,
	// 														ntohs(package_len));
	cal_crc = CalcCRC16((uint8_t*)p_overall, overall_len + 4);	// T, L, V 길이
	if(cal_crc != ntohs(p_overall->crc))
		printf("[Error] CRC Error : %04X / need : %04X\n", ntohs(p_overall->crc), cal_crc);

	p = (uint8_t*)p_overall + sizeof(TLVC_Overall);	// next TLVC
	

	for (i=0; i<p_overall->num_package; i++)
	{
		V2x_App_Ext_TLVC *tlvc = (V2x_App_Ext_TLVC *)p;
		int tlvc_len = ntohs(tlvc->len);
		uint32_t tlvc_type = ntohl(tlvc->type);

		if (package_remain_len < tlvc_len)
		{
			printf("[ERROR] Remain Length - %d\n", tlvc_len);
			break;
		}
			
		if (tlvc_type == EM_PT_STATUS)
		{
			
			// printf("Package : %d (Status Package)\n", i+1);
			parse_ext_status(p);
			
		}
		else
		{
			tod_msgs::ControlCmd tmp;
			ros::serialization::IStream stream(tlvc->data, tlvc->len-2);
			ros::serialization::Serializer<tod_msgs::ControlCmd>::read(stream, tmp);
			set_rosthread_msg_cotrol_cmd(tmp);
			// printf("Package : %d\n\tPSID : %d, TLV lenth : %d\n", i+1, tlvc_type, tlvc_len + 6);
		}

		p = p + tlvc_len + 6; // 6: T, L ?���?
		package_remain_len = package_remain_len - tlvc_len - 6; // 6: T, L ?���?
	}

}

void ControlCmdReceiver::parse_ext_status(void *p)
{
	TLVC_STATUS_Tx_ModemUnit *p_tx_modem;
	TLVC_STATUS_Rx_ModemUnit *p_rx_modem;
	TLVC_STATUS_CommUnit *p_comm;
	TLVC_STATUS_ControlUnit *p_control;
	uint8_t *dev_type = (uint8_t*)p + 6;	// T, L ?��?�� dev_type 존재
	char buf[32];
	uint16_t *crc, cal_crc;

	switch(*dev_type)
	{
		case eStatusDevType_ObuModem:
		{
			p_tx_modem = (TLVC_STATUS_Tx_ModemUnit*)p;
			p_rx_modem = (TLVC_STATUS_Rx_ModemUnit*)p;
			int tx_rx = p_tx_modem->tx_rx;

			if (tx_rx == eStatusTxRx_Tx)
			{
				tx_latitude = (double)htonl(p_tx_modem->latitude)/1000000;
				tx_longitude = (double)htonl(p_tx_modem->longitude)/1000000;
				tx_modem_time = ntohll(p_tx_modem->timestamp);

				its_tx_modem_status.dev_type = p_tx_modem->dev_type;
				its_tx_modem_status.tx_rx = p_tx_modem->tx_rx;
				its_tx_modem_status.dev_id = htonl(p_tx_modem->dev_id);
				its_tx_modem_status.hw_ver = htons(p_tx_modem->hw_ver);
				its_tx_modem_status.sw_ver = htons(p_tx_modem->sw_ver);
				its_tx_modem_status.timestamp = ntohll(p_tx_modem->timestamp);
				its_tx_modem_status.tx_power = p_tx_modem->tx_power;
				its_tx_modem_status.freq = htons(p_tx_modem->freq);
				its_tx_modem_status.bandwidth = p_tx_modem->bandwidth;
				its_tx_modem_status.scs =  p_tx_modem->mcs;
				its_tx_modem_status.mcs = p_tx_modem->scs;
				its_tx_modem_status.latitude = (float)htonl(p_tx_modem->latitude)/1000000;
				its_tx_modem_status.longitude = (float)htonl(p_tx_modem->longitude)/1000000;
				// tx_modem_time = be64toh(p_tx_modem->timestamp);
				// printf("\tLatitude - %d, Longitude - %d\n", htonl(p_tx_modem->latitude), htonl(p_tx_modem->longitude));
				// sprintf(buf, "%lu", be64toh(p_tx_modem->timestamp));
				// printf("\tTimestamp - %s\n", buf);
				cal_crc = CalcCRC16((uint8_t*)p_tx_modem, htons(p_tx_modem->len) + 4);	// T, L, V 길이
				if(cal_crc != ntohs(p_tx_modem->crc))
					printf("[Error] CRC Error : %04X / need : %04X\n", ntohs(p_tx_modem->crc), cal_crc);
			}
			else if (tx_rx == eStatusTxRx_Rx)
			{
				rx_latitude = (double)htonl(p_rx_modem->latitude)/1000000;
				rx_longitude = (double)htonl(p_rx_modem->longitude)/1000000;
				rx_modem_time = ntohll(p_rx_modem->timestamp);

				its_rx_modem_status.dev_type = p_rx_modem->dev_type;
				its_rx_modem_status.tx_rx = p_rx_modem->tx_rx;
				its_rx_modem_status.dev_id = htonl(p_rx_modem->dev_id);
				its_rx_modem_status.hw_ver = htons(p_rx_modem->hw_ver);
				its_rx_modem_status.sw_ver = htons(p_rx_modem->sw_ver);
				its_rx_modem_status.timestamp = ntohll(p_rx_modem->timestamp);
				its_rx_modem_status.rssi = p_rx_modem->rssi;
				its_rx_modem_status.rcpi = p_rx_modem->rcpi;
				its_rx_modem_status.latitude = (float)htonl(p_rx_modem->latitude)/1000000;
				its_rx_modem_status.longitude = (float)htonl(p_rx_modem->longitude)/1000000;
				// rx_modem_time = be64toh(p_rx_modem->timestamp);
				// printf("\tLatitude - %d, Longitude - %d\n", htonl(p_rx_modem->latitude), htonl(p_rx_modem->longitude));
				// sprintf(buf, "%lu", be64toh(p_rx_modem->timestamp));
				// printf("\tTimestamp - %s\n", buf);
				cal_crc = CalcCRC16((uint8_t*)p_rx_modem, htons(p_rx_modem->len) + 4);	// T, L, V 길이
				if(cal_crc != ntohs(p_rx_modem->crc))
					printf("[Error] CRC Error : %04X / need : %04X\n", ntohs(p_rx_modem->crc), cal_crc);
				}
			// else
			// {
			// 	printf("Tx_Rx Type Error - %d\n", tx_rx);
			// 	return;
			// }
			
			break;
		}
		case eStatusDevType_RsuModem:
		{
			p_tx_modem = (TLVC_STATUS_Tx_ModemUnit*)p;
			p_rx_modem = (TLVC_STATUS_Rx_ModemUnit*)p;
			int tx_rx = p_tx_modem->tx_rx;

			if (tx_rx == eStatusTxRx_Tx)
			{
				tx_latitude = (double)htonl(p_tx_modem->latitude)/1000000;
				tx_longitude = (double)htonl(p_tx_modem->longitude)/1000000;
				tx_modem_time = ntohll(p_tx_modem->timestamp);

				its_tx_modem_status.dev_type = p_tx_modem->dev_type;
				its_tx_modem_status.tx_rx = p_tx_modem->tx_rx;
				its_tx_modem_status.dev_id = htonl(p_tx_modem->dev_id);
				its_tx_modem_status.hw_ver = htons(p_tx_modem->hw_ver);
				its_tx_modem_status.sw_ver = htons(p_tx_modem->sw_ver);
				its_tx_modem_status.timestamp = ntohll(p_tx_modem->timestamp);
				its_tx_modem_status.tx_power = p_tx_modem->tx_power;
				its_tx_modem_status.freq = htons(p_tx_modem->freq);
				its_tx_modem_status.bandwidth = p_tx_modem->bandwidth;
				its_tx_modem_status.scs =  p_tx_modem->mcs;
				its_tx_modem_status.mcs = p_tx_modem->scs;
				its_tx_modem_status.latitude = (float)htonl(p_tx_modem->latitude)/1000000;
				its_tx_modem_status.longitude = (float)htonl(p_tx_modem->longitude)/1000000;
				// tx_modem_time = be64toh(p_tx_modem->timestamp);
				// printf("\tLatitude - %d, Longitude - %d\n", htonl(p_tx_modem->latitude), htonl(p_tx_modem->longitude));
				// sprintf(buf, "%lu", be64toh(p_tx_modem->timestamp));
				// printf("\tTimestamp - %s\n", buf);
				cal_crc = CalcCRC16((uint8_t*)p_tx_modem, htons(p_tx_modem->len) + 4);	// T, L, V 길이
				if(cal_crc != ntohs(p_tx_modem->crc))
					printf("[Error] CRC Error : %04X / need : %04X\n", ntohs(p_tx_modem->crc), cal_crc);
				}
			else if (tx_rx == eStatusTxRx_Rx)
			{
				rx_latitude = (double)htonl(p_rx_modem->latitude)/1000000;
				rx_longitude = (double)htonl(p_rx_modem->longitude)/1000000;
				rx_modem_time = ntohll(p_rx_modem->timestamp);

				its_rx_modem_status.dev_type = p_rx_modem->dev_type;
				its_rx_modem_status.tx_rx = p_rx_modem->tx_rx;
				its_rx_modem_status.dev_id = htonl(p_rx_modem->dev_id);
				its_rx_modem_status.hw_ver = htons(p_rx_modem->hw_ver);
				its_rx_modem_status.sw_ver = htons(p_rx_modem->sw_ver);
				its_rx_modem_status.timestamp = ntohll(p_rx_modem->timestamp);
				its_rx_modem_status.rssi = p_rx_modem->rssi;
				its_rx_modem_status.rcpi = p_rx_modem->rcpi;
				its_rx_modem_status.latitude = (float)htonl(p_rx_modem->latitude)/1000000;
				its_rx_modem_status.longitude = (float)htonl(p_rx_modem->longitude)/1000000;
				// rx_modem_time = be64toh(p_rx_modem->timestamp);
				// printf("\tLatitude - %d, Longitude - %d\n", htonl(p_rx_modem->latitude), htonl(p_rx_modem->longitude));
				// sprintf(buf, "%lu", be64toh(p_rx_modem->timestamp));
				// printf("\tTimestamp - %s\n", buf);
				cal_crc = CalcCRC16((uint8_t*)p_rx_modem, htons(p_rx_modem->len) + 4);	// T, L, V 길이
				if(cal_crc != ntohs(p_rx_modem->crc))
					printf("[Error] CRC Error : %04X / need : %04X\n", ntohs(p_rx_modem->crc), cal_crc);
				}
			else
			{
				printf("Tx_Rx Type Error - %d\n", tx_rx);
				return;
			}

			break;
		}
		case eStatusDevType_Obu:
		{
			p_comm = (TLVC_STATUS_CommUnit*)p;
			int tx_rx = p_comm->tx_rx;
			if (tx_rx == eStatusTxRx_Tx){
				its_tx_comm_status.dev_type = p_comm->dev_type;
				its_tx_comm_status.tx_rx = p_comm->tx_rx;
				its_tx_comm_status.dev_id = htonl(p_comm->dev_id);
				its_tx_comm_status.hw_ver = htons(p_comm->hw_ver);
				its_tx_comm_status.sw_ver = htons(p_comm->sw_ver);
				its_tx_comm_status.timestamp = ntohll(p_comm->timestamp);
			}
			else if (tx_rx == eStatusTxRx_Rx){
				its_rx_comm_status.dev_type = p_comm->dev_type;
				its_rx_comm_status.tx_rx = p_comm->tx_rx;
				its_rx_comm_status.dev_id = htonl(p_comm->dev_id);
				its_rx_comm_status.hw_ver = htons(p_comm->hw_ver);
				its_rx_comm_status.sw_ver = htons(p_comm->sw_ver);
				its_rx_comm_status.timestamp = ntohll(p_comm->timestamp);
			}
			cal_crc = CalcCRC16((uint8_t*)p_comm, htons(p_comm->len) + 4);	// T, L, V 길이
			if(cal_crc != ntohs(p_comm->crc))
				printf("[Error] CRC Error : %04X / need : %04X\n", ntohs(p_comm->crc), cal_crc);
			break;
		}

		case eStatusDevType_Rsu:
		{
			p_comm = (TLVC_STATUS_CommUnit*)p;
			int tx_rx = p_comm->tx_rx;

			if (tx_rx == eStatusTxRx_Tx){
				its_tx_comm_status.dev_type = p_comm->dev_type;
				its_tx_comm_status.tx_rx = p_comm->tx_rx;
				its_tx_comm_status.dev_id = htonl(p_comm->dev_id);
				its_tx_comm_status.hw_ver = htons(p_comm->hw_ver);
				its_tx_comm_status.sw_ver = htons(p_comm->sw_ver);
				its_tx_comm_status.timestamp = ntohll(p_comm->timestamp);
			}
			else if (tx_rx == eStatusTxRx_Rx){
				its_rx_comm_status.dev_type = p_comm->dev_type;
				its_rx_comm_status.tx_rx = p_comm->tx_rx;
				its_rx_comm_status.dev_id = htonl(p_comm->dev_id);
				its_rx_comm_status.hw_ver = htons(p_comm->hw_ver);
				its_rx_comm_status.sw_ver = htons(p_comm->sw_ver);
				its_rx_comm_status.timestamp = ntohll(p_comm->timestamp);
			}

			cal_crc = CalcCRC16((uint8_t*)p_comm, htons(p_comm->len) + 4);	// T, L, V 길이
			if(cal_crc != ntohs(p_comm->crc))
				printf("[Error] CRC Error : %04X / need : %04X\n", ntohs(p_comm->crc), cal_crc);
			break;
		}

	//   case eStatusDevType_RsuControl:
	//   {
	//   	p_control = (TLVC_STATUS_ControlUnit*)p;

	// 	printf("\tRsu Control : %s\n", (p_control->tx_rx == eStatusTxRx_Tx)?"Tx":"Rx");
	// 	printf("\tDevice ID : %u\n", htonl(p_control->dev_id));
	// 	printf("\tVersion - HW : %d / SW : %d\n", htons(p_control->hw_ver), htons(p_control->sw_ver));
	// 	sprintf(buf, "%lu", be64toh(p_control->timestamp));
	// 	printf("\tTimestamp - %s\n", buf);
	// 	cal_crc = CalcCRC16((uint8_t*)p_control, htons(p_control->len) + 4);	// T, L, V 길이
	// 	if(cal_crc != ntohs(p_control->crc))
	// 		printf("[Error] CRC Error : %04X / need : %04X\n", ntohs(p_control->crc), cal_crc);
	//   	break;
	//   }

	  default:
	  {
	  	// printf("[Error] Unknown Dev Type - %d\n", *dev_type);
	  	break;
	  }
	}
}

void ControlCmdReceiver::init_rosthread_msg_data() {
    its_mutex.lock();
    its_control_cmd_msg.control.steering_angle = 0.0;
    its_control_cmd_msg.control.throttle = 0.0;
    its_control_cmd_msg.control.brake = 0.0;
    its_control_cmd_msg.shift.data = 0;
    its_control_cmd_msg.indicator.data = 0;
    its_control_cmd_msg.velocity = 0.0;
    its_control_cmd_msg.acceleration = 0.0;
    its_control_cmd_msg.steer_mode = 0;
    its_control_cmd_msg.drive_mode = 0;
    its_control_cmd_msg.remote_flag = 0;
    its_control_cmd_msg.stream_flag = 0;
    its_control_cmd_msg.aeb_flag = 0;

    its_mutex.unlock();
    update_vehicle_time_stamp();
}


void ControlCmdReceiver::set_rosthread_msg_cotrol_cmd(const tod_msgs::ControlCmd& msg) {
    its_mutex.lock();
    // its_control_cmd_msg.header = msg.header;
    its_control_cmd_msg.operator_id = msg.operator_id;
    its_control_cmd_msg.control.steering_angle = msg.control.steering_angle;
    its_control_cmd_msg.control.throttle = msg.control.throttle;
    its_control_cmd_msg.control.brake = msg.control.brake;
    its_control_cmd_msg.shift.data = msg.shift.data;
    its_control_cmd_msg.indicator.data = msg.indicator.data;
    
    its_control_cmd_msg.velocity = msg.velocity;
    its_control_cmd_msg.acceleration = msg.acceleration;

    its_control_cmd_msg.steer_mode = msg.steer_mode;
    its_control_cmd_msg.drive_mode = msg.drive_mode;

    its_control_cmd_msg.remote_flag = msg.remote_flag;
    its_control_cmd_msg.stream_flag = msg.stream_flag;
    its_control_cmd_msg.aeb_flag = msg.aeb_flag;

	rx_commId = msg.v2x_db.eCommId;
	tx_time = msg.v2x_db.ulTimeStamp;
    its_mutex.unlock();
    update_vehicle_time_stamp();
}


void ControlCmdReceiver::set_rosthread_msg_packet_info(const tod_msgs::PacketInfo& msg) {
    its_mutex.lock();
    // its_packet_info_msg.header = msg.header;
    its_packet_info_msg.header.stamp = ros::Time::now();

    its_packet_info_msg.latencyUsec = msg.latencyUsec;
    its_packet_info_msg.sizeBit = msg.sizeBit;
    its_packet_info_msg.seqNum = msg.seqNum;
    its_packet_info_msg.distance = msg.distance;
	its_packet_info_msg.rx_v2x_db = msg.rx_v2x_db;
	its_packet_info_msg.modem_latency = msg.modem_latency;
	its_packet_info_msg.tx_modem_status = msg.tx_modem_status;
	its_packet_info_msg.rx_modem_status = msg.rx_modem_status;
	its_packet_info_msg.tx_comm_status = msg.tx_comm_status;
	its_packet_info_msg.rx_comm_status = msg.rx_comm_status; 

    its_mutex.unlock();
    update_vehicle_time_stamp();
}

uint16_t ControlCmdReceiver::calculate_distance(const double latitude1, const double longitude1,
                        const double latitude2, const double longitude2)
{
  const double earths_radius = 6371;

  // Get the difference between our two points then convert the difference into radians
  double lat_delta = (latitude2 - latitude1)*M_PI/180;
  double lon_delta = (longitude2 - longitude1)*M_PI/180;;

  double converted_lat1 = (latitude1)*M_PI/180;;
  double converted_lat2 = (latitude2)*M_PI/180;;

  double a =
      pow(sin(lat_delta / 2), 2) + cos(converted_lat1) * cos(converted_lat2) * pow(sin(lon_delta / 2), 2);

  auto d = 2 * atan2(sqrt(a), sqrt(1 - a))*earths_radius;
  return (uint16_t)round(d*1000);
}

// void ControlCmdReceiver::change_tod_status_msg_if_disconnected(bool disconnection_detected) {
//     if (disconnection_detected) {
//         its_mutex.lock();
//         its_status_msg.tod_status = tod_msgs::Status::TOD_STATUS_IDLE;
//         its_mutex.unlock();
//         update_vehicle_time_stamp();
//     }
// }

// tod_msgs::ControlCmd ControlCmdReceiver::get_its_status_msg() {
//     return its_status_msg;
// }

void ControlCmdReceiver::update_vehicle_time_stamp() {
    ros::Time::init();
    its_mutex.lock();
    // its_control_cmd_msg.header.stamp = ros::Time::now();
    its_mutex.unlock();
}

uint16_t ControlCmdReceiver::CalcCRC16(const uint8_t * pucaData, uint16_t usLenData) {
    uint8_t ucCrcTableIdx;
    uint16_t usCRC = 0x0000;

	while (usLenData--)
	{
		ucCrcTableIdx = (usCRC >> 8) ^ *pucaData++;
		usCRC = (usCRC << 8) ^ g_usaCRC16Table[ucCrcTableIdx];
	}
	return usCRC;
}