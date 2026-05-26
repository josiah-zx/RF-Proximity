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

const unsigned long PRINT_INTERVAL_MS = 500;
const unsigned long CANDIDATE_TIMEOUT_MS = 12000;
const unsigned long RSSI_TIMEOUT_MS = 3000;
const unsigned long TARGET_REPEAT_MS = 2000;

const int DELTA_DB = 6;
const int HIGH_DELTA_DB = 12;

const int STRONG_DBM = -55;
const int CLOSE_DBM = -45;

const int L_PWM = 4;
const int L_LOW = 5;
const int R_PWM = 6;
const int R_LOW = 7;

const int PWM_HZ = 1000;
const int PWM_BITS = 8;

const int DUTY_MED = 120;
const int DUTY_HIGH = 170;
const int DUTY_CLOSE = 220;

const unsigned long PULSE_MED_MS = 70;
const unsigned long PULSE_HIGH_MS = 90;
const unsigned long PULSE_CLOSE_MS = 120;

const unsigned long GAP_MED_MS = 900;
const unsigned long GAP_HIGH_MS = 450;
const unsigned long GAP_CLOSE_MS = 250;

const bool HAPTICS = true;
const bool STARTUP_TEST = true;

uint8_t leftMac[6] = LEFT_MAC;
uint8_t rightMac[6] = RIGHT_MAC;

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

struct CandidateState {
    bool valid;
    uint8_t count;
    WifiCandidate candidates[TOP_N];
    uint32_t seq;
    unsigned long rxMs;
};

struct RssiState {
    bool valid;
    bool targetValid;
    bool seen;

    int16_t rawRssi;
    int16_t emaRssi;

    uint8_t channel;
    uint8_t bssid[6];

    uint32_t seq;
    unsigned long rxMs;

    char ssid[33];
};

CandidateState leftCand;
CandidateState rightCand;

RssiState leftRssi;
RssiState rightRssi;

TargetPacket target;
bool hasTarget = false;

esp_now_peer_info_t leftPeer;
esp_now_peer_info_t rightPeer;

uint32_t targetSeq = 0;

unsigned long lastPrintMs = 0;
unsigned long lastTargetTxMs = 0;

bool motorOn = false;
int motorSide = 0;
unsigned long motorOffMs = 0;
unsigned long nextPulseMs = 0;

enum Decision {
    D_NONE,
    D_LEFT,
    D_RIGHT,
    D_CENTER,
    D_NO_TARGET,
    D_STALE
};

bool sameMac(const uint8_t a[6], const uint8_t b[6]) {
    return memcmp(a, b, 6) == 0;
}

String macString(const uint8_t mac[6]) {
    char s[18];
    snprintf(s, sizeof(s), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(s);
}

bool freshCand(const CandidateState &s) {
    return s.valid && millis() - s.rxMs <= CANDIDATE_TIMEOUT_MS;
}

bool freshRssi(const RssiState &s) {
    return s.valid && millis() - s.rxMs <= RSSI_TIMEOUT_MS;
}

void setEspNowChannel() {
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
}

void updateCandidates(CandidateState &s, const CandidatePacket &pkt) {
    s.valid = true;
    s.count = min((int)pkt.count, TOP_N);
    s.seq = pkt.seq;
    s.rxMs = millis();

    memset(s.candidates, 0, sizeof(s.candidates));

    for (int i = 0; i < s.count; i++) {
        s.candidates[i] = pkt.candidates[i];
    }
}

void updateRssi(RssiState &s, const RssiPacket &pkt) {
    s.valid = true;
    s.targetValid = pkt.targetValid;
    s.seen = pkt.seen;

    s.rawRssi = pkt.rawRssi;
    s.emaRssi = pkt.emaRssi;

    s.channel = pkt.channel;
    memcpy(s.bssid, pkt.bssid, 6);

    s.seq = pkt.seq;
    s.rxMs = millis();

    memset(s.ssid, 0, sizeof(s.ssid));
    strncpy(s.ssid, pkt.ssid, sizeof(s.ssid) - 1);
}

void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len < 6) return;

    uint32_t magic;
    memcpy(&magic, data, sizeof(magic));
    if (magic != MAGIC) return;

    uint8_t version = data[4];
    uint8_t type = data[5];

    if (version != VERSION) return;

    if (type == MSG_CANDIDATES && len == sizeof(CandidatePacket)) {
        CandidatePacket pkt;
        memcpy(&pkt, data, sizeof(pkt));

        if (pkt.nodeId == NODE_LEFT) updateCandidates(leftCand, pkt);
        if (pkt.nodeId == NODE_RIGHT) updateCandidates(rightCand, pkt);
    }

    if (type == MSG_RSSI && len == sizeof(RssiPacket)) {
        RssiPacket pkt;
        memcpy(&pkt, data, sizeof(pkt));

        if (pkt.nodeId == NODE_LEFT) updateRssi(leftRssi, pkt);
        if (pkt.nodeId == NODE_RIGHT) updateRssi(rightRssi, pkt);
    }
}

bool chooseTarget(WifiCandidate &out) {
    if (!freshCand(leftCand) || !freshCand(rightCand)) return false;

    bool found = false;
    int bestScore = -999;

    for (int i = 0; i < leftCand.count; i++) {
        for (int j = 0; j < rightCand.count; j++) {
            WifiCandidate l = leftCand.candidates[i];
            WifiCandidate r = rightCand.candidates[j];

            if (!sameMac(l.bssid, r.bssid)) continue;

            int score = min(l.rssi, r.rssi);

            if (!found || score > bestScore) {
                found = true;
                bestScore = score;

                out = l;
                out.rssi = (l.rssi + r.rssi) / 2;
            }
        }
    }

    return found;
}

void sendTarget() {
    target.magic = MAGIC;
    target.version = VERSION;
    target.type = MSG_TARGET;
    target.seq = targetSeq++;
    target.ms = millis();

    setEspNowChannel();

    esp_now_send(leftMac, (uint8_t*)&target, sizeof(target));
    delay(10);
    esp_now_send(rightMac, (uint8_t*)&target, sizeof(target));
}

void updateTarget() {
    WifiCandidate chosen;

    if (!chooseTarget(chosen)) return;

    bool changed =
            !hasTarget ||
            !sameMac(target.bssid, chosen.bssid) ||
            target.channel != chosen.channel;

    if (!changed) return;

    memset(&target, 0, sizeof(target));

    target.magic = MAGIC;
    target.version = VERSION;
    target.type = MSG_TARGET;
    target.targetValid = true;
    target.channel = chosen.channel;
    memcpy(target.bssid, chosen.bssid, 6);
    strncpy(target.ssid, chosen.ssid, sizeof(target.ssid) - 1);

    hasTarget = true;

    Serial.print("target ");
    Serial.print(target.ssid);
    Serial.print(" ");
    Serial.print(macString(target.bssid));
    Serial.print(" ch=");
    Serial.println(target.channel);

    sendTarget();
    lastTargetTxMs = millis();
}

Decision getDecision(int &delta, int &strongest) {
    delta = 0;
    strongest = -999;

    if (!hasTarget) return D_NO_TARGET;

    if (!freshRssi(leftRssi) || !freshRssi(rightRssi)) return D_STALE;
    if (!leftRssi.seen || !rightRssi.seen) return D_STALE;

    if (!sameMac(leftRssi.bssid, target.bssid)) return D_STALE;
    if (!sameMac(rightRssi.bssid, target.bssid)) return D_STALE;

    delta = leftRssi.emaRssi - rightRssi.emaRssi;
    strongest = max(leftRssi.emaRssi, rightRssi.emaRssi);

    if (delta >= DELTA_DB) return D_LEFT;
    if (delta <= -DELTA_DB) return D_RIGHT;
    return D_CENTER;
}

const char* decisionString(Decision d) {
    switch (d) {
        case D_LEFT: return "left";
        case D_RIGHT: return "right";
        case D_CENTER: return "center";
        case D_NO_TARGET: return "no_target";
        case D_STALE: return "stale";
        default: return "none";
    }
}

void motorsOff() {
    ledcWrite(L_PWM, 0);
    ledcWrite(R_PWM, 0);

    digitalWrite(L_LOW, LOW);
    digitalWrite(R_LOW, LOW);

    motorOn = false;
    motorSide = 0;
}

void pulse(int side, int duty) {
    motorsOff();

    if (!HAPTICS) return;

    if (side < 0) {
        digitalWrite(L_LOW, LOW);
        ledcWrite(L_PWM, duty);
        motorSide = -1;
    }

    if (side > 0) {
        digitalWrite(R_LOW, LOW);
        ledcWrite(R_PWM, duty);
        motorSide = 1;
    }

    motorOn = true;
}

void updateHaptics() {
    unsigned long now = millis();

    if (motorOn && now >= motorOffMs) {
        motorsOff();
    }

    if (!HAPTICS || now < nextPulseMs) return;

    int delta;
    int strongest;
    Decision d = getDecision(delta, strongest);

    if (d != D_LEFT && d != D_RIGHT) return;

    int duty = DUTY_MED;
    unsigned long pulseMs = PULSE_MED_MS;
    unsigned long gapMs = GAP_MED_MS;

    if (strongest >= CLOSE_DBM) {
        duty = DUTY_CLOSE;
        pulseMs = PULSE_CLOSE_MS;
        gapMs = GAP_CLOSE_MS;
    } else if (abs(delta) >= HIGH_DELTA_DB || strongest >= STRONG_DBM) {
        duty = DUTY_HIGH;
        pulseMs = PULSE_HIGH_MS;
        gapMs = GAP_HIGH_MS;
    }

    int side = (d == D_LEFT) ? -1 : 1;

    pulse(side, duty);
    motorOffMs = now + pulseMs;
    nextPulseMs = now + gapMs;
}

void hapticTest() {
    if (!STARTUP_TEST || !HAPTICS) return;

    Serial.println("motor_test");

    pulse(-1, DUTY_HIGH);
    delay(180);
    motorsOff();
    delay(300);

    pulse(1, DUTY_HIGH);
    delay(180);
    motorsOff();
    delay(300);

    for (int i = 0; i < 2; i++) {
        pulse(-1, DUTY_MED);
        delay(90);
        motorsOff();
        delay(120);

        pulse(1, DUTY_MED);
        delay(90);
        motorsOff();
        delay(120);
    }

    motorsOff();
}

void setupHaptics() {
    pinMode(L_LOW, OUTPUT);
    pinMode(R_LOW, OUTPUT);

    digitalWrite(L_LOW, LOW);
    digitalWrite(R_LOW, LOW);

    ledcAttach(L_PWM, PWM_HZ, PWM_BITS);
    ledcAttach(R_PWM, PWM_HZ, PWM_BITS);

    motorsOff();
}

void addPeer(uint8_t mac[6], esp_now_peer_info_t &peer) {
    memset(&peer, 0, sizeof(peer));
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;

    esp_err_t ok = esp_now_add_peer(&peer);

    Serial.print("peer ");
    Serial.print(macString(mac));
    Serial.print(" ");
    Serial.println(ok == ESP_OK ? "ok" : "fail");
}

void setupEspNow() {
    if (esp_now_init() != ESP_OK) {
        Serial.println("espnow init fail");
        while (true) delay(1000);
    }

    esp_now_register_recv_cb(onReceive);

    addPeer(leftMac, leftPeer);
    addPeer(rightMac, rightPeer);
}

void printRssi(const char* name, const RssiState &s) {
    Serial.print(name);
    Serial.print("=");

    if (!s.valid) {
        Serial.print("nodata");
        return;
    }

    if (!freshRssi(s)) {
        Serial.print("stale");
        return;
    }

    if (!s.seen) {
        Serial.print("miss");
        return;
    }

    Serial.print(s.emaRssi);
    Serial.print("/");
    Serial.print(s.rawRssi);
}

void printDebug() {
    int delta;
    int strongest;
    Decision d = getDecision(delta, strongest);

    Serial.print("t=");

    if (hasTarget) {
        Serial.print(target.ssid);
        Serial.print(" ch=");
        Serial.print(target.channel);
    } else {
        Serial.print("none");
    }

    Serial.print(" | ");
    printRssi("L", leftRssi);

    Serial.print(" ");
    printRssi("R", rightRssi);

    Serial.print(" | d=");
    if (d == D_LEFT || d == D_RIGHT || d == D_CENTER) {
        Serial.print(delta);
    } else {
        Serial.print("na");
    }

    Serial.print(" | dir=");
    Serial.print(decisionString(d));

    Serial.print(" | m=");
    if (motorOn) {
        Serial.print(motorSide < 0 ? "L" : "R");
    } else {
        Serial.print("off");
    }

    Serial.println();
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(200);

    setEspNowChannel();

    Serial.print("controller mac=");
    Serial.println(WiFi.macAddress());

    setupHaptics();
    hapticTest();

    setupEspNow();

    Serial.println("ready");
}

void loop() {
    unsigned long now = millis();

    updateTarget();

    if (hasTarget && now - lastTargetTxMs >= TARGET_REPEAT_MS) {
        sendTarget();
        lastTargetTxMs = now;
    }

    updateHaptics();

    if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
        lastPrintMs = now;
        printDebug();
    }

    delay(10);
}