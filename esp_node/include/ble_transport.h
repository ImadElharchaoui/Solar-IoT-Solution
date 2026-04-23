#pragma once
/**
 * ble_transport.h
 *
 * BLE packet framing over NimBLE GATT characteristics.
 *
 * Packet wire format:
 *   [0]     MSG_TYPE   (1 byte, identifies the proto message type)
 *   [1-2]   PAYLOAD_LEN (uint16 little-endian, byte count of proto data)
 *   [3..N]  PAYLOAD     (raw protobuf bytes)
 *
 * Because BLE MTU is limited (~512 bytes with NimBLE), the assembled frame
 * is chunked into notifications of at most BLE_CHUNK_SIZE bytes each.
 * Each notification is prefixed with a 1-byte sequence tag:
 *   0x00  CHUNK_FIRST  - first (or only) chunk of a new message
 *   0x01  CHUNK_CONT   - continuation chunk
 *   0x80  CHUNK_LAST   - final chunk of a multi-chunk message
 *
 * Single messages that fit in one notification still use CHUNK_FIRST.
 */

#include <Arduino.h>
#include <NimBLECharacteristic.h>

// Message type tags (same values used by rpi_ble_gateway.py)
enum MpptMsgType : uint8_t {
    MSG_TELEMETRY    = 0x01,
    MSG_FAULT_STATUS = 0x02,
    MSG_DEVICE_INFO  = 0x03,
    MSG_SETTINGS     = 0x04,
    MSG_DATALOG      = 0x05,
    MSG_ACK          = 0x11,
    MSG_CMD          = 0x10,
};

// Chunk sequence tags
static constexpr uint8_t CHUNK_FIRST = 0x00;
static constexpr uint8_t CHUNK_CONT  = 0x01;
static constexpr uint8_t CHUNK_LAST  = 0x80;

// Maximum bytes of payload data per BLE notification.
// Calculated as: 512 (Max MTU) - 3 (GATT overhead) - 1 (Tag) = 508 bytes.
static constexpr size_t BLE_CHUNK_SIZE = 508;

// Send a proto payload over a BLE Notify characteristic.
void ble_send(NimBLECharacteristic *tx_char,
              MpptMsgType           msg_type,
              const uint8_t        *payload,
              size_t                len);

// Reassembles chunked BLE Write values into complete framed messages.
class BleReassembler {
public:
    BleReassembler() { reset(); }

    // Feed one BLE Write chunk. Returns true when a complete message is ready.
    bool feed(const uint8_t *data, size_t len);

    MpptMsgType  msg_type()    const { return _msg_type; }
    const uint8_t *payload()   const { return _buf + 3; }   // skip 3-byte header
    size_t        payload_len() const { return _payload_len; }

    void reset();

private:
    static constexpr size_t BUF_SIZE = 4096;
    uint8_t     _buf[BUF_SIZE];
    size_t      _len        = 0;
    bool        _busy       = false;
    MpptMsgType _msg_type   = MSG_TELEMETRY;
    size_t      _payload_len= 0;
};
