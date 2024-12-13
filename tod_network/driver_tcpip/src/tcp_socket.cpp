#include "tcp_socket.hpp"

#include "endpoint.hpp"

#include <driver_tcpip_msgs/tcp_packet.h>

#include <boost/asio/placeholders.hpp>


using namespace driver_tcpip;

// CONSTRUCTORS
tcp_socket_t::tcp_socket_t(boost::asio::ip::tcp::socket* socket, uint32_t id)
    : socket_t(id, protocol_t::TCP)
{
    // Store socket.
    tcp_socket_t::m_socket = socket;

    // Start ROS publishers and services.
    tcp_socket_t::start_ros();

    // Start asynchronously receiving.
    tcp_socket_t::async_rx();

    // Log socket opening.
    ROS_INFO_STREAM("tcp socket " << id << " opened successfully");
}
tcp_socket_t::~tcp_socket_t()
{
    // Close the socket.
    tcp_socket_t::close();

    // Free the socket.
    delete tcp_socket_t::m_socket;
}

// CONTROL
void tcp_socket_t::close()
{
    if(tcp_socket_t::m_socket->is_open())
    {
        // Stop publishers and services.
        tcp_socket_t::stop_ros();

        // Close the ASIO socket.
        boost::system::error_code error;
        tcp_socket_t::m_socket->close(error);
        if(error)
        {
            ROS_ERROR_STREAM("failed to close udp socket " << tcp_socket_t::m_id << " (" << error.message() << ")");
        }
        else
        {
            ROS_INFO_STREAM("tcp socket " << tcp_socket_t::m_id << " closed successfully");
        }
    }
}

// PROPERTIES
driver_tcpip_msgs::tcp_socket tcp_socket_t::description() const
{
    // Create output message.
    driver_tcpip_msgs::tcp_socket description;

    // Populate message.
    description.id = tcp_socket_t::m_id;
    description.local_endpoint = endpoint::to_ros(tcp_socket_t::m_socket->local_endpoint());
    description.remote_endpoint = endpoint::to_ros(tcp_socket_t::m_socket->remote_endpoint());

    return description;
}
bool tcp_socket_t::is_open() const
{
    return tcp_socket_t::m_socket->is_open();
}
template <class Msg> 
void tcp_socket_t::tx_callback(const Msg& message)
{
    if(tod_status == tod_msgs::Status::TOD_STATUS_TELEOPERATION){
        uint32_t send_len; 
        uint8_t buf[MAX_TX_PACKET_TO_RSU];
        memset(buf, 0, sizeof(buf));
        // V2x_App_Hdr* hdr = (V2x_App_Hdr*)tx_buffer.data();
        V2x_App_Hdr* hdr = (V2x_App_Hdr*)buf;
        ros::SerializedMessage serMsg = ros::serialization::serializeMessage(message);
        uint16_t *com_id = (uint16_t*)serMsg.message_start+12;
        uint16_t *crc16;
        uint16_t calc_crc16;
        V2x_App_TxMsg* tx_msg = (V2x_App_TxMsg*)hdr->data;
        TLVC_Overall *p_overall;
        V2x_App_Ext_TLVC *p_ssov;
        // tod_msgs::ControlCmd* control_msg = (tod_msgs::ControlCmd*) p_dummy->data;
        TLVC_STATUS_CommUnit *p_status;
        

        uint64_t keti_time = sys_timer();
        uint16_t package_len;
        p_overall = (TLVC_Overall*)tx_msg->data;
        p_overall->type = htonl(EM_PT_OVERALL);
        p_overall->len = htons(sizeof(TLVC_Overall) - 6);
        p_overall->magic[0] = 'E';
        p_overall->magic[1] = 'M';
        p_overall->magic[2] = 'O';
        p_overall->magic[3] = 'P';
        p_overall->version = 1;
        p_overall->num_package = 0;
        p_overall->len_package = 0;

        package_len = ntohs(p_overall->len_package);

        p_ssov  = (V2x_App_Ext_TLVC *)((uint8_t*)p_overall + sizeof(TLVC_Overall) + package_len);

        p_overall->num_package++;
        package_len = package_len + 8 + serMsg.num_bytes; // T = 4, L=2, C=2 , V = size
        p_overall->len_package = htons(package_len);
        p_overall->crc = htons(CalcCRC16((uint8_t*)p_overall, sizeof(TLVC_Overall) - 2));	// TLVC -C

        p_ssov->type = htonl(EM_PT_SSOV);
        p_ssov->len = htons(serMsg.num_bytes + 2);		// V=size, C=2

        // std::memcpy(com_id, serMsg.message_start+32, sizeof(com_id));
        std::memcpy(serMsg.message_start+8, &keti_time, sizeof(keti_time));
        std::memcpy(p_ssov->data, serMsg.message_start, serMsg.num_bytes);

        crc16 = (uint16_t*)((uint8_t*)p_ssov + 6 + serMsg.num_bytes);
        *crc16 = htons(CalcCRC16((uint8_t*)p_ssov, serMsg.num_bytes+6 - 2));	//  TLVC - C

        // AddExtStatusData(p_overall, eStatusTxRx_Tx);
        
        memcpy(hdr->magic, V2X_INF_EXT_MAGIC, sizeof(hdr->magic));
        // send_len = 16 + sizeof(TLVC_Overall) + ntohs(p_overall->len_package);	        // 16 : header(10) + psid(4) + crc(2)
        hdr->len = htons(10 + sizeof(TLVC_Overall) + ntohs(p_overall->len_package));	// seq(2) + payload id(2) + crc(2) + psid(4)

        hdr->seq = htons(seq);
        hdr->payload_id = htons(ePayloadId_TxMsg);
        
        switch (*com_id)
        {
        case 1:
            tx_msg->psid = htonl(EM_V2V_MSG);
            break;
            
        case 2:
            tx_msg->psid = htonl(EM_V2I_MSG);
            break;
        case 5:
            tx_msg->psid = htonl(EM_I2V_MSG);
            break;
        default:
            break;
        }

        send_len = ntohs(hdr->len) + 6;		// magic 4byte, lenth 2byte
        // printf("TLVC size : %ld, size : %d, len_pack: %d \n",sizeof(TLVC_Overall), send_len, ntohs(p_overall->len_package));
        crc16 = (uint16_t*)&buf[send_len-2];
        calc_crc16 = CalcCRC16(buf + SIZE_MAGIC_NUMBER_OF_HEADER, send_len - 6);		// magic(4), crc(2)
        *crc16 = htons(calc_crc16);

        // printf("#%d-----send length : %ld\n", seq, serMsg.num_bytes);

        seq++;
        if(seq>65535) seq = 0;
        tcp_socket_t::m_socket->async_send(boost::asio::buffer(buf, send_len),
                                            boost::bind(&tcp_socket_t::handle_send, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    }
    // usleep(10 * 1000);
}

void tcp_socket_t::handle_send(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    if(error)
    {
         // An error occured.

        // Check if port is being closed and rx was aborted.
        if(error == boost::asio::error::operation_aborted)
        {
            // Quit receiving.
            return;
        }

        // Check if remote closed the connection.
        if(error == boost::asio::error::connection_reset || error == boost::asio::error::eof)
        {
            // Indicate remote has closed.
            ROS_INFO_STREAM("tcp socket " << tcp_socket_t::m_id << " disconnect by peer");
            // Close socket.
            tcp_socket_t::close();

            // Quit receiving.
            return;
        }

        // Otherwise, report error.
        ROS_ERROR_STREAM("tcp socket " << tcp_socket_t::m_id << " asynchrounous send failed (" << error.message() << ")");
    }    
}

void tcp_socket_t::callback_status(const tod_msgs::Status::ConstPtr& status)
{
    tod_status = status->tod_status;
}
// void tcp_socket_t::AddExtStatusData(TLVC_Overall *p_overall, int tx_rx)
// {
// 	TLVC_STATUS_CommUnit *p_status;
// 	struct timeval now;
// 	struct tm *tm;
// 	uint64_t keti_time;
// 	uint16_t package_len = ntohs(p_overall->len_package);
//     // char buf[32];
// 	p_status  = (TLVC_STATUS_CommUnit *)((uint8_t*)p_overall + sizeof(TLVC_Overall) + package_len);

// 	p_overall->num_package++;
// 	package_len += sizeof(TLVC_STATUS_CommUnit);
// 	p_overall->len_package = htons(package_len);
// 	p_overall->crc = htons(CalcCRC16((uint8_t*)p_overall, sizeof(TLVC_Overall) - 2));	//

// 	p_status->type = htonl(EM_PT_STATUS);
// 	p_status->len = htons(sizeof(TLVC_STATUS_CommUnit) - 6);
// 	p_status->dev_type = eStatusDevType_RsuEnd;
// 	p_status->tx_rx = tx_rx;
// 	p_status->dev_id = htonl(1);
// 	p_status->hw_ver = htons(2);
// 	p_status->sw_ver = htons(3);


// 	p_status->timestamp = htobe64(sys_timer());

// 	p_status->crc = htons(CalcCRC16((uint8_t*)p_status, sizeof(TLVC_STATUS_CommUnit) - 2));	// TLVC ì¤? CRCë§? ? œ?™¸

// 	// return true;
// }

// ASIO SOCKET
void tcp_socket_t::async_rx()
{
    // Start an asynchronous receive.
    tcp_socket_t::m_socket->async_receive(boost::asio::buffer(tcp_socket_t::m_buffer),
                                          boost::bind(&tcp_socket_t::rx_callback, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}
void tcp_socket_t::rx_callback(const boost::system::error_code& error, std::size_t bytes_read)
{
    // Check for recieve errors.
    if(!error)
    {
        // Publish the message.
        driver_tcpip_msgs::tcp_packet message;
        message.data.assign(tcp_socket_t::m_buffer.begin(), tcp_socket_t::m_buffer.begin() + bytes_read);
		message.time_stamp = sys_timer();
		std::cout << "[time : "<<message.time_stamp<<"]"<<" size : "<<bytes_read<<std::endl;
		if(psid_register_flag){
			psid_result(tcp_socket_t::m_buffer.begin(), bytes_read);
			psid_register_flag = false;
		}
        tcp_socket_t::m_publisher_rx.publish(message);
    }
    else
    {
        // An error occured.

        // Check if port is being closed and rx was aborted.
        if(error == boost::asio::error::operation_aborted)
        {
            // Quit receiving.
            return;
        }

        // Check if remote closed the connection.
        if(error == boost::asio::error::connection_reset || error == boost::asio::error::eof)
        {
            // Indicate remote has closed.
            ROS_INFO_STREAM("tcp socket " << tcp_socket_t::m_id << " disconnect by peer");
            // Close socket.
            tcp_socket_t::close();

            // Quit receiving.
            return;
        }

        // Otherwise, report error.
        ROS_ERROR_STREAM("tcp socket " << tcp_socket_t::m_id << " asynchrounous receive failed (" << error.message() << ")");
    }

    // Continue receiving data.
    tcp_socket_t::async_rx();
}


// ROS
void tcp_socket_t::start_ros()
{
    // Get private node handle.
    ros::NodeHandle private_node("~");


    if (!private_node.getParam(ros::this_node::getName() + "/isVehicle", isVehicle))
        ROS_ERROR_STREAM(ros::this_node::getName() << ": Could not get param /isVehicle - using "<< isVehicle );
    // Create base topic.
    std::string topic_base = "sockets/" + std::to_string(tcp_socket_t::m_id);
    // Create TX service.
    tcp_socket_t::m_service_tx = private_node.advertiseService(topic_base + "/tx", &tcp_socket_t::service_tx, this);
    // Create RX publisher.
    tcp_socket_t::m_publisher_rx = private_node.advertise<driver_tcpip_msgs::tcp_packet>(topic_base + "/rx", 10);
    // Create TX subscriber.
	if(isVehicle){
		tcp_socket_t::m_subscriber_tx = private_node.subscribe("/Vehicle/kona/probe_vehicle_data", 1, &tcp_socket_t::tx_callback<tod_msgs::ProbeVehicleData>, this);
		tcp_socket_t::m_subscriber_status = private_node.subscribe("/Vehicle/Manager/status_msg", 1, &tcp_socket_t::callback_status, this);
	}
	else{
		tcp_socket_t::m_subscriber_tx = private_node.subscribe("/Operator/Control/control_cmd_data", 1, &tcp_socket_t::tx_callback<tod_msgs::ControlCmd>, this);
		tcp_socket_t::m_subscriber_status = private_node.subscribe("/Operator/Manager/status_msg", 1, &tcp_socket_t::callback_status, this);
    }
}
void tcp_socket_t::stop_ros()
{
    tcp_socket_t::m_service_tx.shutdown();
    tcp_socket_t::m_publisher_rx.shutdown();
    tcp_socket_t::m_subscriber_tx.shutdown();
}

bool tcp_socket_t::service_tx(driver_tcpip_msgs::send_tcpRequest& request, driver_tcpip_msgs::send_tcpResponse& response)
{
    // Try sending the message.
    boost::system::error_code error;
    tcp_socket_t::m_socket->send(boost::asio::buffer(request.packet.data), 0, error);
	psid_register_flag = true;
    // Indicate if the message was sent successfully.
    return !error;
}

void tcp_socket_t::psid_result(uint8_t *msg, int len)
{
	int i, overall_len, package_len, package_remain_len;
	uint16_t *crc, cal_crc;
	void *p;
	TLVC_Overall *p_overall = NULL;
	V2x_App_Hdr *hdr = (V2x_App_Hdr *)msg;
	V2x_App_RxMsg *rx_msg = (V2x_App_RxMsg *)hdr->data;
	// uint32_t psid = ntohl(hdr->psid);
	uint32_t psid = ntohl(rx_msg->psid);
	uint16_t payload_id = ntohs(hdr->payload_id);
	int flag_extensible_msg = 0;
	if(payload_id == ePayloadId_RxMsg){
		V2x_App_WSC *wsc = (V2x_App_WSC *)hdr->data;
		switch (wsc->action_result)
		{
		case eWSCActionResult_Fail:
		{
			printf("WSC Result Fail : %d\n", wsc->action_result);
			break;
		}
		case eWSCActionResult_Add_OK:
		{
			printf("WSC Result Add OK: %d\n", wsc->action_result);
			break;
		}
		case eWSCActionResult_Already_Add:
		{
			printf("WSC Result Already Add: %d\n", wsc->action_result);
			break;
		}
		case eWSCActionResult_Del_OK:
		{
			printf("WSC Result Delete OK: %d\n", wsc->action_result);
			break;
		}
		case eWSCActionResult_Not_Exist_Del:
		{
			printf("WSC Result Not Exist delete: %d\n", wsc->action_result);
			break;
		}
		default:
			break;
		}
		printf("Get Normal Message - PSID(%d)\n", ntohl(wsc->psid));
	}

}

uint64_t tcp_socket_t::sys_timer(){
	FILE *r_fp;
	char buff[32];
	int rtn = 0;
	uint64_t time_stamp=0;
	memset(buff, 0x00, sizeof(buff));
	r_fp = popen("date +%Y%m%d%H%M%S%5N", "r");
	if(!r_fp)
	{
		printf("popen ERROR (%d)\n", errno);
	}
	rtn=fread(buff, sizeof(char), sizeof(buff), r_fp);

	// if(rtn>0){
	// 	printf("READ : %s\n", buff);
		
	// }
	time_stamp = strtoul(buff, NULL, 10);
	// printf("time : %lu\n", time_stamp);
	pclose(r_fp);

	return time_stamp;
}

uint16_t tcp_socket_t::CalcCRC16(const unsigned char * pucaData, unsigned short usLenData) {
    unsigned char ucCrcTableIdx;
    unsigned short usCRC = 0x0000;

	while (usLenData--)
	{
		ucCrcTableIdx = (usCRC >> 8) ^ *pucaData++;
		usCRC = (usCRC << 8) ^ g_usaCRC16Table[ucCrcTableIdx];
	}
	return usCRC;
}

