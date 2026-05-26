#pragma once

#define ESPNOW_CHANNEL 6

#define NODE_LEFT  1
#define NODE_RIGHT 2

// IMPORTANT: Change these example MAC addresses to that of your own for every .ino's config.h.

#define CONTROLLER_MAC {0x44, 0x1B, 0xF6, 0xFE, 0x71, 0x60}
#define LEFT_MAC       {0xE0, 0x72, 0xA1, 0xD5, 0x27, 0x2C}
#define RIGHT_MAC      {0x44, 0x1B, 0xF6, 0xFE, 0x68, 0xC8}

// IMPORTANT: always make sure to change this node ID per-sensor before flashing
// value goes unused in controller sketch, but needed for sensors
#define SENSOR_NODE_ID NODE_LEFT
// #define SENSOR_NODE_ID NODE_RIGHT