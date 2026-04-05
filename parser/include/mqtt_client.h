#pragma once
// mqtt_client.h - Paho MQTT C++ async wrapper

#include <functional>
#include <memory>
#include <string>

#include "mqtt/async_client.h"

struct MqttConfig {
    std::string host = "localhost";
    int         port = 1883;
    std::string client_id;
    std::string username;
    std::string password;
    int         keepalive_s = 60;

    // Last Will Testament - broker fires this on ungraceful disconnect
    // (power loss, crash, WAN outage).  Leave lwt_topic empty to disable.
    std::string lwt_topic;
    std::string lwt_payload = "0";
    bool        lwt_retain  = true;

    int publish_timeout_ms   = 5000;
    int subscribe_timeout_ms = 5000;
};

// Signature for inbound message callbacks set via subscribe().
// Called on the Paho consumer thread - implementations must be thread-safe.
using MessageCallback = std::function<void(const std::string &topic, const std::string &payload)>;

class MqttClient {
  public:
    explicit MqttClient(MqttConfig cfg);
    ~MqttClient();

    auto               connect() -> bool;
    void               disconnect();
    [[nodiscard]] auto connected() const -> bool {
        return connected_;
    }

    /**
     * Publish a message and block until acknowledged (up to publish_timeout_ms).
     *
     * @param topic   MQTT topic string
     * @param payload UTF-8 payload
     * @param retain  whether the broker should retain this message
     * @param qos     0 = fire-and-forget, 1 = at-least-once (default 0)
     */
    auto
    publish(const std::string &topic, const std::string &payload, bool retain = false, int qos = 0)
        -> bool;

    /**
     * Subscribe to a topic and register a callback for inbound messages.
     * May be called multiple times for different topics.
     * All subscriptions share one underlying Paho callback - each inbound
     * message is dispatched to every registered callback; callbacks should
     * filter on topic themselves.
     *
     * @param topic   MQTT topic filter (wildcards + / # supported)
     * @param qos     0 or 1
     * @param cb      called on the Paho consumer thread for matching messages
     */
    auto subscribe(const std::string &topic, int qos, MessageCallback cb) -> bool;

  private:
    MqttConfig                          cfg_;
    std::unique_ptr<mqtt::async_client> client_;
    bool                                connected_ = false;

    // All registered subscriptions - iterated on every inbound message
    std::vector<std::pair<std::string, MessageCallback>> subscriptions_;

    // Installed once after connect(); dispatches to subscriptions_
    void install_message_callback();
};
