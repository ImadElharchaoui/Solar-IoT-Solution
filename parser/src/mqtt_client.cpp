// Paho MQTT C++ async wrapper

#include "mqtt_client.h"
#include <cstdio>


MqttClient::MqttClient(const MqttConfig& cfg) : cfg_(cfg) {
    const std::string uri = "tcp://" + cfg_.host + ":" + std::to_string(cfg_.port);
    client_ = std::make_unique<mqtt::async_client>(uri, cfg_.client_id);
}

MqttClient::~MqttClient() {
    if (connected_) disconnect();
}


bool MqttClient::connect() {
    auto opts_builder = mqtt::connect_options_builder()
                            .keep_alive_interval(std::chrono::seconds(cfg_.keepalive_s))
                            .clean_session(true)
                            .automatic_reconnect(false);

    if (!cfg_.username.empty())
        opts_builder.user_name(cfg_.username).password(cfg_.password);

    if (!cfg_.lwt_topic.empty()) {
        opts_builder.will(mqtt::message(cfg_.lwt_topic, cfg_.lwt_payload, /*qos=*/0, cfg_.lwt_retain));
    }

    try {
        client_->connect(opts_builder.finalize())->wait();
        connected_ = true;
        fprintf(stderr, "[mqtt] connected to %s:%d (client_id='%s')\n", cfg_.host.c_str(), cfg_.port, cfg_.client_id.c_str());
        return true;
    }
    catch (const mqtt::exception& e) {
        fprintf(stderr, "[mqtt] connect() failed: %s\n", e.what());
        return false;
    }
}


void MqttClient::disconnect() {
    if (!connected_) return;
    try {
        client_->disconnect()->wait();
    }
    catch (const mqtt::exception& e) {
        fprintf(stderr, "[mqtt] disconnect() error: %s\n", e.what());
    }
    connected_ = false;
}


bool MqttClient::publish(const std::string& topic,
                          const std::string& payload,
                          bool               retain,
                          int                qos) {
    if (!connected_) {
        fprintf(stderr, "[mqtt] publish() called while not connected\n");
        return false;
    }
    try {
        auto msg = mqtt::make_message(topic, payload, qos, retain);
        auto tok = client_->publish(msg);
        if (!tok->wait_for(std::chrono::milliseconds(cfg_.publish_timeout_ms))) {
            fprintf(stderr, "[mqtt] publish timed out on '%s'\n", topic.c_str());
            return false;
        }
        return true;
    }
    catch (const mqtt::exception& e) {
        fprintf(stderr, "[mqtt] publish() failed on '%s': %s\n", topic.c_str(), e.what());
        return false;
    }
}
