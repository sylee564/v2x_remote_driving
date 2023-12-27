#pragma once
#include <mqtt/async_client.h>

namespace tod_network {

class MqttClient : public virtual mqtt::callback {
public:
    MqttClient() = delete;
    MqttClient(const std::string &serverAddress, const std::string &clientId);
    ~MqttClient() = default;

    int subscribe(const std::string &topic /* /Vehicle/VehicleBridge/Manager/connection_status */,
        const int& qos /*0, 1 or 2*/, void (*fp)(mqtt::const_message_ptr) /* &callback_function */);
    int publish(const std::string &topic /* /Vehicle/VehicleBridge/Manager/connection_status */,
        const int& qos /*0, 1 or 2*/, const char* msg, size_t length);
    int disconnect();
    bool is_connected() { return _clientIsConnected; }

private:
    std::string _serverAddress;
    std::string _clientId;
    mqtt::connect_options _connOpts;
    mqtt::async_client _client;
    std::string _topic;
    mqtt::token_ptr _subTokenPtr;
    bool _clientIsConnected;
    int _qos;
    void (*_fp)(mqtt::const_message_ptr msg);

    unsigned int connectMqttClient();
    void connected(const std::string &cause) override { /* add manually by overriding if necessary */ }
    void connection_lost(const std::string &cause) override { /* add manually by overriding if necessary */ }
    void delivery_complete(mqtt::delivery_token_ptr tok) override {/*add manually by overriding if necessary*/}
    void message_arrived(mqtt::const_message_ptr msg) override {
        if (!(msg->get_payload_str() == "Last will and testament.")) _fp( msg );
    }

    class iactionListener : public virtual mqtt::iaction_listener {
    public:
        explicit iactionListener(MqttClient& parent) : _parent(parent) {}
    private:
        void on_failure(const mqtt::token& tok) override { _parent._clientIsConnected = false; }
        void on_success(const mqtt::token& tok) override { _parent._clientIsConnected = true; }
        MqttClient& _parent;
    } _iactionListener;
};

}; // namespace tod_network