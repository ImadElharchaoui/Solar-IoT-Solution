#include "ble_transport.h"
#include <NimBLEService.h>
#include <NimBLEServer.h>
#include <cstring>

void ble_send(NimBLECharacteristic *tx_char,
              MpptMsgType           msg_type,
              const uint8_t        *payload,
              size_t                len)
{
    if (!tx_char) {
        printf("[BLE] tx_char is null");
        return;
    }
    
    // In NimBLE 2.0+, getSubscribedCount() is removed. 
    // notify() handles checking for subscribers internally.
    // We check if anyone is connected at all.
    if (tx_char->getService()->getServer()->getConnectedCount() == 0) {
        // printf("[BLE] no connected peers, dropping");
        return;
    }

    // Build framed packet: [msg_type(1)] [payload_len_le(2)] [payload(N)]
    const size_t total = 3 + len;
    uint8_t frame[total];
    frame[0] = static_cast<uint8_t>(msg_type);
    frame[1] = static_cast<uint8_t>(len & 0xFF);
    frame[2] = static_cast<uint8_t>((len >> 8) & 0xFF);
    memcpy(frame + 3, payload, len);

    // Get actual MTU for this connection. GATT overhead is 3 bytes.
    // NimBLE default is 23 if not negotiated.
    uint16_t peer_mtu = 23;
    auto peer_ids = tx_char->getService()->getServer()->getPeerDevices();
    if (!peer_ids.empty()) {
        peer_mtu = tx_char->getService()->getServer()->getPeerMTU(peer_ids[0]);
    }
    
    if (peer_mtu < 23) peer_mtu = 23; 
    
    // Log MTU on first chunk if it's still default, to help diagnose sync issues
    static uint16_t last_mtu = 0;
    if (peer_mtu != last_mtu) {
        printf("[BLE] MTU updated: %d (Payload=%d)\n", peer_mtu, peer_mtu - 3);
        last_mtu = peer_mtu;
    }
    
    const size_t chunk_max = peer_mtu - 4; // MTU - 3 (GATT) - 1 (Tag)

    size_t offset = 0;
    bool   first  = true;

    while (offset < total) {
        const size_t remaining = total - offset;
        const size_t take      = (remaining < chunk_max) ? remaining : chunk_max;
        const bool   is_last   = (offset + take >= total);

        uint8_t pkt[chunk_max + 1];

        // Single-chunk messages use CHUNK_FIRST only (per spec).
        // Multi-chunk: first=CHUNK_FIRST, middle=CHUNK_CONT, last=CHUNK_LAST.
        if (first && is_last)  pkt[0] = CHUNK_FIRST; // single chunk
        else if (first)        pkt[0] = CHUNK_FIRST;
        else if (is_last)      pkt[0] = CHUNK_LAST;
        else                   pkt[0] = CHUNK_CONT;

        memcpy(pkt + 1, frame + offset, take);
        tx_char->setValue(pkt, static_cast<uint16_t>(1 + take));
        tx_char->notify(); // actually transmit the notification

        offset += take;
        first   = false;

        // Brief yield between chunks so BLE stack can flush.
        // NimBLE queues notifications but a tight loop can overflow the queue.
        if (!is_last) {
            delay(10);
        }
    }
}

void BleReassembler::reset()
{
    _len         = 0;
    _busy        = false;
    _payload_len = 0;
}

bool BleReassembler::feed(const uint8_t *data, size_t len)
{
    if (len == 0) return false;

    const uint8_t  tag  = data[0];
    const uint8_t *body = data + 1;
    const size_t   blen = len - 1;

    if (tag == CHUNK_FIRST) {
        _len  = 0;
        _busy = true;
    } else if (!_busy) {
        // Stray CONT or LAST without a preceding FIRST - ignore
        return false;
    }

    // Append body, guard against overflow
    const size_t space = BUF_SIZE - _len;
    const size_t copy  = (blen < space) ? blen : space;
    memcpy(_buf + _len, body, copy);
    _len += copy;

    // Frame is complete on CHUNK_FIRST (single-chunk) or CHUNK_LAST
    if (tag == CHUNK_FIRST || tag == CHUNK_LAST) {
        if (_len < 3) { reset(); return false; }
        _msg_type    = static_cast<MpptMsgType>(_buf[0]);
        _payload_len = static_cast<size_t>(_buf[1]) | (static_cast<size_t>(_buf[2]) << 8);
        _busy        = false;
        return (_payload_len + 3 <= _len);
    }

    return false; // CHUNK_CONT - waiting for more
}
