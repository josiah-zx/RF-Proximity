#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include "config.h"

#define MSG_CANDIDATES 1
#define MSG_RSSI       2
#define MSG_TARGET     3

#define MAGIC 0x52465333
#define VERSION 3

#define TOP_N 5

const unsigned long CANDIDATE_INTERVAL_MS = 8000;
const unsigned long RSSI_INTERVAL_MS = 750;
const unsigned long TARGET_TIMEOUT_MS = 20000;

const uint16_t FULL_SCAN_MS = 90;
const uint16_t TARGET_SCAN_MS = 120;

const float EMA_ALPHA = 0.35;

uint8_t controllerMac[6] = CONTROLLER_MAC;
const uint8_t nodeId = SENSOR_NODE_ID;

struct __attribute__((packed)) WifiCandidate {
    uint8_t bssid[6];
    int16_t rssi;
    uint8_t channel;
    char ssid[33];
};

struct __attribute__((packed)) CandidatePacket {
    uint32_t magic;
    uint8_t version;
    uint8_t type;
    uint8_t nodeId;
    uint8_t count;
    uint32_t seq;
    uint32_t ms;
    WifiCandidate candidates[TOP_N];
};

struct __attribute__((packed)) RssiPacket {
    uint32_t magic;
    uint8_t version;
    uint8_t type;
    uint8_t nodeId;

    bool targetValid;
    bool seen;

    int16_t rawRssi;
    int16_t emaRssi;

    uint8_t channel;
    uint8_t bssid[6];

    uint32_t seq;
    uint32_t ms;

    char ssid[33];
};

struct __attribute__((packed)) TargetPacket {
    uint32_t magic;
    uint8_t version;
    uint8_t type;

    bool targetValid;

    uint8_t channel;
    uint8_t bssid[6];

    uint32_t seq;
    uint32_t ms;

    char ssid[33];
};

esp_now_peer_info_t controllerPeer;

TargetPacket target;
bool hasTarget = false;
unsigned long lastTargetMs = 0;

unsigned long lastCandidateScanMs = 0;
unsigned long lastRssiScanMs = 0;

uint32_t candidateSeq = 0;
uint32_t rssiSeq = 0;

bool hasEma = false;
float emaRssi = -999;

const char* sideName() {
    if (nodeId == NODE_LEFT) return "L";
    if (nodeId == NODE_RIGHT) return "R";
    return "?";
}

bool sameMac(const uint8_t a[6], const uint8_t b[6]) {
    return memcmp(a, b, 6) == 0;
}

String macString(const uint8_t mac[6]) {
    char s[18];
    snprintf(s, sizeof(s), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(s);
}

void setEspNowChannel() {
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
}

void insertCandidate(WifiCandidate c, WifiCandidate list[], int &count) {
    int pos = count;

    for (int i = 0; i < count; i++) {
        if (c.rssi > list[i].rssi) {
            pos = i;
            break;
        }
    }

    if (pos >= TOP_N) return;

    if (count < TOP_N) count++;

    for (int i = count - 1; i > pos; i--) {
        list[i] = list[i - 1];
    }

    list[pos] = c;
}

void sendCandidates() {
    CandidatePacket pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.magic = MAGIC;
    pkt.version = VERSION;
    pkt.type = MSG_CANDIDATES;
    pkt.nodeId = nodeId;
    pkt.seq = candidateSeq++;
    pkt.ms = millis();

    WifiCandidate top[TOP_N];
    memset(top, 0, sizeof(top));
    int topCount = 0;

    int n = WiFi.scanNetworks(false, true, false, FULL_SCAN_MS);

    for (int i = 0; i < n; i++) {
        String ssid = WiFi.SSID(i);
        if (ssid.length() == 0) continue;

        WifiCandidate c;
        memset(&c, 0, sizeof(c));

        memcpy(c.bssid, WiFi.BSSID(i), 6);
        c.rssi = WiFi.RSSI(i);
        c.channel = WiFi.channel(i);
        strncpy(c.ssid, ssid.c_str(), sizeof(c.ssid) - 1);

        insertCandidate(c, top, topCount);
    }

    WiFi.scanDelete();

    pkt.count = topCount;
    for (int i = 0; i < topCount; i++) {
        pkt.candidates[i] = top[i];
    }

    setEspNowChannel();
    esp_now_send(controllerMac, (uint8_t*)&pkt, sizeof(pkt));

    Serial.print(sideName());
    Serial.print(" cand n=");
    Serial.print(topCount);

    if (topCount > 0) {
        Serial.print(" top=");
        Serial.print(top[0].ssid);
        Serial.print(" ");
        Serial.print(top[0].rssi);
        Serial.print(" ch=");
        Serial.print(top[0].channel);
    }

    Serial.println();
}

int16_t updateEma(bool seen, int raw) {
    if (!seen) {
        hasEma = false;
        emaRssi = -999;
        return -999;
    }

    if (!hasEma) {
        emaRssi = raw;
        hasEma = true;
    } else {
        emaRssi = EMA_ALPHA * raw + (1.0 - EMA_ALPHA) * emaRssi;
    }

    return (int16_t)round(emaRssi);
}

void sendRssi(bool seen, int raw, int16_t ema) {
    RssiPacket pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.magic = MAGIC;
    pkt.version = VERSION;
    pkt.type = MSG_RSSI;
    pkt.nodeId = nodeId;

    pkt.targetValid = hasTarget;
    pkt.seen = seen;
    pkt.rawRssi = seen ? raw : -999;
    pkt.emaRssi = seen ? ema : -999;

    pkt.channel = target.channel;
    memcpy(pkt.bssid, target.bssid, 6);

    pkt.seq = rssiSeq++;
    pkt.ms = millis();

    strncpy(pkt.ssid, target.ssid, sizeof(pkt.ssid) - 1);

    setEspNowChannel();
    esp_now_send(controllerMac, (uint8_t*)&pkt, sizeof(pkt));
}

void scanTarget() {
    if (!hasTarget) return;

    if (millis() - lastTargetMs > TARGET_TIMEOUT_MS) {
        hasTarget = false;
        hasEma = false;
        Serial.println("target timeout");
        return;
    }

    bool seen = false;
    int raw = -999;

    int n = WiFi.scanNetworks(false, true, false, TARGET_SCAN_MS, target.channel);

    for (int i = 0; i < n; i++) {
        String ssid = WiFi.SSID(i);
        if (ssid.length() == 0) continue;

        if (sameMac(WiFi.BSSID(i), target.bssid)) {
            raw = WiFi.RSSI(i);
            seen = true;
            break;
        }
    }

    WiFi.scanDelete();

    int16_t ema = updateEma(seen, raw);
    sendRssi(seen, raw, ema);

    Serial.print(sideName());
    Serial.print(" rssi seen=");
    Serial.print(seen ? 1 : 0);

    if (seen) {
        Serial.print(" raw=");
        Serial.print(raw);
        Serial.print(" ema=");
        Serial.print(ema);
    }

    Serial.println();
}

void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len != sizeof(TargetPacket)) return;

    TargetPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));

    if (pkt.magic != MAGIC) return;
    if (pkt.version != VERSION) return;
    if (pkt.type != MSG_TARGET) return;

    bool changed =
            !hasTarget ||
            !sameMac(target.bssid, pkt.bssid) ||
            target.channel != pkt.channel;

    target = pkt;
    hasTarget = pkt.targetValid;
    lastTargetMs = millis();

    if (changed) {
        hasEma = false;

        Serial.print("target=");
        if (hasTarget) {
            Serial.print(target.ssid);
            Serial.print(" ");
            Serial.print(macString(target.bssid));
            Serial.print(" ch=");
            Serial.println(target.channel);
        } else {
            Serial.println("none");
        }
    }
}

void addControllerPeer() {
    memset(&controllerPeer, 0, sizeof(controllerPeer));
    memcpy(controllerPeer.peer_addr, controllerMac, 6);
    controllerPeer.channel = ESPNOW_CHANNEL;
    controllerPeer.encrypt = false;

    if (esp_now_add_peer(&controllerPeer) != ESP_OK) {
        Serial.println("peer add fail");
        while (true) delay(1000);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(200);

    setEspNowChannel();

    Serial.print("sensor ");
    Serial.print(sideName());
    Serial.print(" mac=");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("espnow init fail");
        while (true) delay(1000);
    }

    esp_now_register_recv_cb(onReceive);
    addControllerPeer();

    Serial.println("ready");
}

void loop() {
    unsigned long now = millis();

    if (now - lastCandidateScanMs >= CANDIDATE_INTERVAL_MS) {
        lastCandidateScanMs = now;
        sendCandidates();
    }

    if (hasTarget && now - lastRssiScanMs >= RSSI_INTERVAL_MS) {
        lastRssiScanMs = now;
        scanTarget();
    }

    delay(10);
}