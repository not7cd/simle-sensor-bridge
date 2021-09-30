#pragma once

#define CHECK_BIT(var,pos) ((var & pos) == pos)

static const uint8_t ROCKET_STATUS_ARMED = 0x01;
static const uint8_t ROCKET_STATUS_1 = 0x02;
static const uint8_t ROCKET_STATUS_2 = 0x04;
static const uint8_t ROCKET_STATUS_3 = 0x08;
static const uint8_t ROCKET_STATUS_4 = 0x10;
static const uint8_t ROCKET_STATUS_5 = 0x20;
static const uint8_t ROCKET_STATUS_6 = 0x40;
static const uint8_t ROCKET_STATUS_7 = 0x80;
