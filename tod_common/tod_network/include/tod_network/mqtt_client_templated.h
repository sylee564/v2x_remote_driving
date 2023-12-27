#pragma once
#include <mqtt/async_client.h>

namespace tod_network {

template <class T>
class MqttClientTemplated : public virtual mqtt::callback {
public:
    MqttClientTemplated() = delete;
    MqttClientTemplated(const std::string& serverAddress, const std::string& clientId)
        : _serverAddress(serverAddress), _clientId(clientId), _client(_serverAddress, _clientId),
        _clientIsConnected(false), _iactionListener(*this) {
        connectMqttClient();
    }

    int subscribe(const std::string& topic /* /Vehicle/VehicleBridge/Manager/connection_status */,
                  const int& qos /* 0, 1 or 2 */,
                  void (T::*fp)(mqtt::const_message_ptr)/* &T::callback_function */,
                  T* t /* 'this' or '&T' */) {
        _topic = topic;
        _qos = qos;
        _fp = fp;
        _t = t;
        try {
            _subTokenPtr = _client.subscribe(_topic, _qos);
            _subTokenPtr->wait();
        } catch (const mqtt::exception& exc) {
            std::cerr << exc.what() << "\nClient " << _clientId.c_str() << "\n" << std::endl;
            return 1;
        }
        return 0;
    }

    int publish(const std::string& topic /* /Vehicle/VehicleBridge/Manager/connection_status */,
                const int& qos /* 0, 1 or 2 */, const char* msg, size_t length) {
        _topic = topic;
        _qos = qos;
        mqtt::message_ptr pubmsg = mqtt::make_message(_topic, msg, length);
        pubmsg->set_qos(_qos);
        _client.publish(pubmsg);
        return 0;
    }

    int disconnect() {
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

    bool is_connected() { return _clientIsConnected; }


private:
    std::string _serverAddress;
    std::string _clientId;
    mqtt::connect_options _connOpts;
    mqtt::async_client _client;
    std::string _topic;
    mqtt::token_ptr _subTokenPtr;
    int _qos;
    void (T::*_fp)(mqtt::const_message_ptr msg) = nullptr;
    T* _t = nullptr;
    bool _clientIsConnected;

    unsigned int connectMqttClient() {
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

    void connected(const std::string& cause) override { /* add manually by overriding if necessary */ }
    void connection_lost(const std::string& cause) override { /* add manually by overriding if necessary */ }
    void message_arrived(mqtt::const_message_ptr msg) override {
        if (!(msg->get_payload_str() == "Last will and testament.")) (*_t.*_fp)(msg);
    }
    void delivery_complete(mqtt::delivery_token_ptr tok) override { /* add manually by overriding if necessary */ }

    class iactionListener : public virtual mqtt::iaction_listener {
        void on_failure(const mqtt::token& tok) override {
            _parent._clientIsConnected = false;
        }
        void on_success(const mqtt::token& tok) override {
            _parent._clientIsConnected = true;
        }
        MqttClientTemplated<T>& _parent;
    public:
        explicit iactionListener(MqttClientTemplated<T>& parent) : _parent(parent) {}
    } _iactionListener;
};

}; // namespace tod_network