// Paho MQTT C++ async wrapper

#include "mqtt_client.h"

#include <iostream>
#include <utility>

MqttClient::MqttClient(MqttConfig cfg) : cfg_(std::move(cfg)) {
    const std::string URI = "tcp://" + cfg_.host + ":" + std::to_string(cfg_.port);
    client_               = std::make_unique<mqtt::async_client>(URI, cfg_.client_id);
}

MqttClient::~MqttClient() {
    if (connected_) {
        disconnect();
    }
}

void MqttClient::install_message_callback() {
    client_->set_message_callback([this](const mqtt::const_message_ptr &msg) {
        const std::string      TOPIC   = msg->get_topic();
        const std::string      PAYLOAD = msg->to_string();
        const std::string_view TOPIC_SV(TOPIC);

        for (const auto &[filter, cb] : subscriptions_) {
            const std::string_view FILTER_SV(filter);
            bool                   match = false;
            if (!FILTER_SV.empty() && FILTER_SV.back() == '#') {
                const std::string_view PREFIX = FILTER_SV.substr(0, FILTER_SV.size() - 1);
                match                         = (TOPIC_SV.substr(0, PREFIX.size()) == PREFIX);
            } else {
                match = (TOPIC_SV == FILTER_SV);
            }
            if (match) {
                cb(TOPIC, PAYLOAD);
            }
        }
    });
}

auto MqttClient::connect() -> bool {
    auto opts_builder = mqtt::connect_options_builder()
                            .keep_alive_interval(std::chrono::seconds(cfg_.keepalive_s))
                            .clean_session(true)
                            .automatic_reconnect(false);

    if (!cfg_.username.empty()) {
        opts_builder.user_name(cfg_.username).password(cfg_.password);
    }

    if (!cfg_.lwt_topic.empty()) {
        opts_builder.will(
            mqtt::message(cfg_.lwt_topic, cfg_.lwt_payload, /*qos=*/0, cfg_.lwt_retain));
    }

    try {
        install_message_callback();
        client_->connect(opts_builder.finalize())->wait();
        connected_ = true;
        std::cerr << "[mqtt] connected to " << cfg_.host << ":" << cfg_.port << " (client_id='"
                  << cfg_.client_id << "')\n";
        return true;
    } catch (const mqtt::exception &e) {
        std::cerr << "[mqtt] connect() failed: " << e.what() << "\n";
        return false;
    }
}

void MqttClient::disconnect() {
    if (!connected_) {
        return;
    }
    try {
        client_->disconnect()->wait();
    } catch (const mqtt::exception &e) {
        std::cerr << "[mqtt] disconnect() error: " << e.what() << "\n";
    }
    connected_ = false;
}

auto MqttClient::publish(const std::string &topic, const std::string &payload, bool retain, int qos)
    -> bool {
    if (!connected_) {
        std::cerr << "[mqtt] publish() called while not connected\n";
        return false;
    }
    try {
        auto msg = mqtt::make_message(topic, payload, qos, retain);
        auto tok = client_->publish(msg);
        if (!tok->wait_for(std::chrono::milliseconds(cfg_.publish_timeout_ms))) {
            std::cerr << "[mqtt] publish timed out on '" << topic << "'\n";
            return false;
        }
        return true;
    } catch (const mqtt::exception &e) {
        std::cerr << "[mqtt] publish() failed on '" << topic << "': " << e.what() << "\n";
        return false;
    }
}

auto MqttClient::subscribe(const std::string &topic, int qos, MessageCallback cb) -> bool {
    if (!connected_) {
        std::cerr << "[mqtt] subscribe() called while not connected\n";
        return false;
    }
    try {
        client_->subscribe(topic, qos)
            ->wait_for(std::chrono::milliseconds(cfg_.subscribe_timeout_ms));
        subscriptions_.emplace_back(topic, std::move(cb));
        std::cerr << "[mqtt] subscribed to '" << topic << "' (QoS " << qos << ")\n";
        return true;
    } catch (const mqtt::exception &e) {
        std::cerr << "[mqtt] subscribe() failed on '" << topic << "': " << e.what() << "\n";
        return false;
    }
}
