#include "tod_network/mqtt_client.h"

namespace tod_network {

MqttClient::MqttClient(const std::string& serverAddress, const std::string& clientId)
    : _serverAddress(serverAddress), _clientId(clientId), _client(_serverAddress, _clientId),
    _clientIsConnected(false), _iactionListener(*this) {
    connectMqttClient();
}

unsigned int MqttClient::connectMqttClient() {
    _connOpts.set_keep_alive_interval(20);
    _connOpts.set_clean_session(true);
    _connOpts.set_connect_timeout(4);
    _client.set_callback(*this);
    try {
        _client.connect(_connOpts, nullptr, _iactionListener)->wait_for(2000);
    } catch (const mqtt::exception&) {
        std::cerr << "Client " << _clientId << " unable to connect to MQTT server " << _serverAddress << std::endl;
        return -1;
    }
    return 0;
}

int MqttClient::subscribe(const std::string& topic /* /Vehicle/VehicleBridge/Manager/connection_status */,
                          const int& qos, void (*fp)(mqtt::const_message_ptr) /*&callbackFoo*/) {
    _topic = topic;
    _qos = qos;
    _fp = fp;
    try {
        _subTokenPtr = _client.subscribe(_topic, _qos);
        _subTokenPtr->wait();
    }
    catch (const mqtt::exception& exc) {
        std::cerr << exc.what() << "\nClient " << _clientId.c_str() << "\n" << std::endl;
        return 1;
    }
    return 0;
}

int MqttClient::publish(const std::string& topic /*"/Vehicle/VehicleBridge/Manager/connection_status"*/,
                        const int& qos /*0, 1 or 2*/, const char* msg, size_t length) {
    _topic = topic;
    _qos = qos;
    mqtt::message_ptr pubmsg = mqtt::make_message(_topic, msg, length);
    pubmsg->set_qos(_qos);
    _client.publish(pubmsg);
    return 0;
}

int MqttClient::disconnect() {
    try {
        _client.connect(_connOpts);
        _client.disconnect()->wait();
        _clientIsConnected = false;
    } catch (const mqtt::exception& exc) {
        std::cerr << exc.what() << "\nClient " << _clientId.c_str() << "\n" << std::endl;
        return 1;
    }
    return 0;
}
}; // namespace tod_network