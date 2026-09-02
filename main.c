// 单文件版：OLED 显示「鸿蒙先锋号」 + SHT20 温湿度 + AP3216C 光/接近（多线程 + 信号量）
// 已合并原 hal_bsp_ssd1306.c / hal_bsp_sht20.c / hal_bsp_ap3216c.c / fonts.h / I2c_Ssd1306.c 全部内容，仅此一个 .c
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_errno.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_i2c.h"
#include "wifiiot_i2c_ex.h"
#include "wifiiot_uart.h"
#include "hi_time.h"
#include "hi_uart.h"   // 波特率自适应需要 hi_uart_read_timeout（带超时读）

/* ===== WiFi / MQTT 配置（烧录前改成自己的值）===== */
#include "wifi_device.h"      // WifiConnect / GetStationIP（见 wifi_connect.c）
#include "MQTTClient.h"       // paho MQTT 客户端（LiteOS 移植）

#define WIFI_SSID         "BUAA-WIFI"            // 校园网 SSID（北航）
#define WIFI_PASSWORD     ""                    // BUAA-WIFI 是开放网络，密码留空（OPEN 模式）
#define MQTT_BROKER_ADDR  "broker.emqx.io"       // 公网 MQTT broker（默认）；需小车所在网络能上外网。BUAA-WIFI 要网页认证→上不了外网，可改用手机热点
#define MQTT_BROKER_PORT  (1883)                // MQTT 端口
#define MQTT_CLIENT_ID    "QST_Pioneer_3861"    // 客户端 ID（同一 broker 上需唯一）
#define MQTT_SUB_TOPIC    "car/control"         // 订阅：手机/电脑往此主题发 W/A/S/D... 控车
#define MQTT_PUB_TOPIC    "car/status"          // 发布：小车状态上报（可选）

// ===================== 字模 =====================
static const unsigned char F6x8[][6] =
{
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00},// sp
	{0x00, 0x00, 0x00, 0x2f, 0x00, 0x00},// !
	{0x00, 0x00, 0x07, 0x00, 0x07, 0x00},// "
	{0x00, 0x14, 0x7f, 0x14, 0x7f, 0x14},// #
	{0x00, 0x24, 0x2a, 0x7f, 0x2a, 0x12},// $
	{0x00, 0x62, 0x64, 0x08, 0x13, 0x23},// %
	{0x00, 0x36, 0x49, 0x55, 0x22, 0x50},// &
	{0x00, 0x00, 0x05, 0x03, 0x00, 0x00},// '
	{0x00, 0x00, 0x1c, 0x22, 0x41, 0x00},// (
	{0x00, 0x00, 0x41, 0x22, 0x1c, 0x00},// )
	{0x00, 0x14, 0x08, 0x3E, 0x08, 0x14},// *
	{0x00, 0x08, 0x08, 0x3E, 0x08, 0x08},// +
	{0x00, 0x00, 0x00, 0xA0, 0x60, 0x00},// ,
	{0x00, 0x08, 0x08, 0x08, 0x08, 0x08},// -
	{0x00, 0x00, 0x60, 0x60, 0x00, 0x00},// .
	{0x00, 0x20, 0x10, 0x08, 0x04, 0x02},// /
	{0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E},// 0
	{0x00, 0x00, 0x42, 0x7F, 0x40, 0x00},// 1
	{0x00, 0x42, 0x61, 0x51, 0x49, 0x46},// 2
	{0x00, 0x21, 0x41, 0x45, 0x4B, 0x31},// 3
	{0x00, 0x18, 0x14, 0x12, 0x7F, 0x10},// 4
	{0x00, 0x27, 0x45, 0x45, 0x45, 0x39},// 5
	{0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30},// 6
	{0x00, 0x01, 0x71, 0x09, 0x05, 0x03},// 7
	{0x00, 0x36, 0x49, 0x49, 0x49, 0x36},// 8
	{0x00, 0x06, 0x49, 0x49, 0x29, 0x1E},// 9
	{0x00, 0x00, 0x36, 0x36, 0x00, 0x00},// :
	{0x00, 0x00, 0x56, 0x36, 0x00, 0x00},// ;
	{0x00, 0x08, 0x14, 0x22, 0x41, 0x00},// <
	{0x00, 0x14, 0x14, 0x14, 0x14, 0x14},// =
	{0x00, 0x00, 0x41, 0x22, 0x14, 0x08},// >
	{0x00, 0x02, 0x01, 0x51, 0x09, 0x06},// ?
	{0x00, 0x32, 0x49, 0x59, 0x51, 0x3E},// @
	{0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C},// A
	{0x00, 0x7F, 0x49, 0x49, 0x49, 0x36},// B
	{0x00, 0x3E, 0x41, 0x41, 0x41, 0x22},// C
	{0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C},// D
	{0x00, 0x7F, 0x49, 0x49, 0x49, 0x41},// E
	{0x00, 0x7F, 0x09, 0x09, 0x09, 0x01},// F
	{0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A},// G
	{0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F},// H
	{0x00, 0x00, 0x41, 0x7F, 0x41, 0x00},// I
	{0x00, 0x20, 0x40, 0x41, 0x3F, 0x01},// J
	{0x00, 0x7F, 0x08, 0x14, 0x22, 0x41},// K
	{0x00, 0x7F, 0x40, 0x40, 0x40, 0x40},// L
	{0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F},// M
	{0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F},// N
	{0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E},// O
	{0x00, 0x7F, 0x09, 0x09, 0x09, 0x06},// P
	{0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E},// Q
	{0x00, 0x7F, 0x09, 0x19, 0x29, 0x46},// R
	{0x00, 0x46, 0x49, 0x49, 0x49, 0x31},// S
	{0x00, 0x01, 0x01, 0x7F, 0x01, 0x01},// T
	{0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F},// U
	{0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F},// V
	{0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F},// W
	{0x00, 0x63, 0x14, 0x08, 0x14, 0x63},// X
	{0x00, 0x07, 0x08, 0x70, 0x08, 0x07},// Y
	{0x00, 0x61, 0x51, 0x49, 0x45, 0x43},// Z
	{0x00, 0x00, 0x7F, 0x41, 0x41, 0x00},// [
	{0x00, 0x55, 0x2A, 0x55, 0x2A, 0x55},// 55
	{0x00, 0x00, 0x41, 0x41, 0x7F, 0x00},// ]
	{0x00, 0x04, 0x02, 0x01, 0x02, 0x04},// ^
	{0x00, 0x40, 0x40, 0x40, 0x40, 0x40},// _
	{0x00, 0x00, 0x01, 0x02, 0x04, 0x00},// '
	{0x00, 0x20, 0x54, 0x54, 0x54, 0x78},// a
	{0x00, 0x7F, 0x48, 0x44, 0x44, 0x38},// b
	{0x00, 0x38, 0x44, 0x44, 0x44, 0x20},// c
	{0x00, 0x38, 0x44, 0x44, 0x48, 0x7F},// d
	{0x00, 0x38, 0x54, 0x54, 0x54, 0x18},// e
	{0x00, 0x08, 0x7E, 0x09, 0x01, 0x02},// f
	{0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C},// g
	{0x00, 0x7F, 0x08, 0x04, 0x04, 0x78},// h
	{0x00, 0x00, 0x44, 0x7D, 0x40, 0x00},// i
	{0x00, 0x40, 0x80, 0x84, 0x7D, 0x00},// j
	{0x00, 0x7F, 0x10, 0x28, 0x44, 0x00},// k
	{0x00, 0x00, 0x41, 0x7F, 0x40, 0x00},// l
	{0x00, 0x7C, 0x04, 0x18, 0x04, 0x78},// m
	{0x00, 0x7C, 0x08, 0x04, 0x04, 0x78},// n
	{0x00, 0x38, 0x44, 0x44, 0x44, 0x38},// o
	{0x00, 0xFC, 0x24, 0x24, 0x24, 0x18},// p
	{0x00, 0x18, 0x24, 0x24, 0x18, 0xFC},// q
	{0x00, 0x7C, 0x08, 0x04, 0x04, 0x08},// r
	{0x00, 0x48, 0x54, 0x54, 0x54, 0x20},// s
	{0x00, 0x04, 0x3F, 0x44, 0x40, 0x20},// t
	{0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C},// u
	{0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C},// v
	{0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C},// w
	{0x00, 0x44, 0x28, 0x10, 0x28, 0x44},// x
	{0x00, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C},// y
	{0x00, 0x44, 0x64, 0x54, 0x4C, 0x44},// z
	{0x14, 0x14, 0x14, 0x14, 0x14, 0x14},// horiz lines
};
static const unsigned char F8X16[] =
{
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,// 0
  0x00,0x00,0x00,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x33,0x30,0x00,0x00,0x00,//! 1
  0x00,0x10,0x0C,0x06,0x10,0x0C,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//" 2
  0x40,0xC0,0x78,0x40,0xC0,0x78,0x40,0x00,0x04,0x3F,0x04,0x04,0x3F,0x04,0x04,0x00,//# 3
  0x00,0x70,0x88,0xFC,0x08,0x30,0x00,0x00,0x00,0x18,0x20,0xFF,0x21,0x1E,0x00,0x00,//$ 4
  0xF0,0x08,0xF0,0x00,0xE0,0x18,0x00,0x00,0x00,0x21,0x1C,0x03,0x1E,0x21,0x1E,0x00,//% 5
  0x00,0xF0,0x08,0x88,0x70,0x00,0x00,0x00,0x1E,0x21,0x23,0x24,0x19,0x27,0x21,0x10,//& 6
  0x10,0x16,0x0E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//' 7
  0x00,0x00,0x00,0xE0,0x18,0x04,0x02,0x00,0x00,0x00,0x00,0x07,0x18,0x20,0x40,0x00,//( 8
  0x00,0x02,0x04,0x18,0xE0,0x00,0x00,0x00,0x00,0x40,0x20,0x18,0x07,0x00,0x00,0x00,//) 9
  0x40,0x40,0x80,0xF0,0x80,0x40,0x40,0x00,0x02,0x02,0x01,0x0F,0x01,0x02,0x02,0x00,//* 10
  0x00,0x00,0x00,0xF0,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x1F,0x01,0x01,0x01,0x00,//+ 11
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xB0,0x70,0x00,0x00,0x00,0x00,0x00,//, 12
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,//- 13
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x00,0x00,0x00,//. 14
  0x00,0x00,0x00,0x00,0x80,0x60,0x18,0x04,0x00,0x60,0x18,0x06,0x01,0x00,0x00,0x00,/// 15
  0x00,0xE0,0x10,0x08,0x08,0x10,0xE0,0x00,0x00,0x0F,0x10,0x20,0x20,0x10,0x0F,0x00,//0 16
  0x00,0x10,0x10,0xF8,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//1 17
  0x00,0x70,0x08,0x08,0x08,0x88,0x70,0x00,0x00,0x30,0x28,0x24,0x22,0x21,0x30,0x00,//2 18
  0x00,0x30,0x08,0x88,0x88,0x48,0x30,0x00,0x00,0x18,0x20,0x20,0x20,0x11,0x0E,0x00,//3 19
  0x00,0x00,0xC0,0x20,0x10,0xF8,0x00,0x00,0x00,0x07,0x04,0x24,0x24,0x3F,0x24,0x00,//4 20
  0x00,0xF8,0x08,0x88,0x88,0x08,0x08,0x00,0x00,0x19,0x21,0x20,0x20,0x11,0x0E,0x00,//5 21
  0x00,0xE0,0x10,0x88,0x88,0x18,0x00,0x00,0x00,0x0F,0x11,0x20,0x20,0x11,0x0E,0x00,//6 22
  0x00,0x38,0x08,0x08,0xC8,0x38,0x08,0x00,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00,//7 23
  0x00,0x70,0x88,0x08,0x08,0x88,0x70,0x00,0x00,0x1C,0x22,0x21,0x21,0x22,0x1C,0x00,//8 24
  0x00,0xE0,0x10,0x08,0x08,0x10,0xE0,0x00,0x00,0x00,0x31,0x22,0x22,0x11,0x0F,0x00,//9 25
  0x00,0x00,0x00,0xC0,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x00,//: 26
  0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x60,0x00,0x00,0x00,0x00,//; 27
  0x00,0x00,0x80,0x40,0x20,0x10,0x08,0x00,0x00,0x01,0x02,0x04,0x08,0x10,0x20,0x00,//< 28
  0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x00,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x00,//= 29
  0x00,0x08,0x10,0x20,0x40,0x80,0x00,0x00,0x00,0x20,0x10,0x08,0x04,0x02,0x01,0x00,//> 30
  0x00,0x70,0x48,0x08,0x08,0x08,0xF0,0x00,0x00,0x00,0x00,0x30,0x36,0x01,0x00,0x00,//? 31
  0xC0,0x30,0xC8,0x28,0xE8,0x10,0xE0,0x00,0x07,0x18,0x27,0x24,0x23,0x14,0x0B,0x00,//@ 32
  0x00,0x00,0xC0,0x38,0xE0,0x00,0x00,0x00,0x20,0x3C,0x23,0x02,0x02,0x27,0x38,0x20,//A 33
  0x08,0xF8,0x88,0x88,0x88,0x70,0x00,0x00,0x20,0x3F,0x20,0x20,0x20,0x11,0x0E,0x00,//B 34
  0xC0,0x30,0x08,0x08,0x08,0x08,0x38,0x00,0x07,0x18,0x20,0x20,0x20,0x10,0x08,0x00,//C 35
  0x08,0xF8,0x08,0x08,0x08,0x10,0xE0,0x00,0x20,0x3F,0x20,0x20,0x20,0x10,0x0F,0x00,//D 36
  0x08,0xF8,0x88,0x88,0xE8,0x08,0x10,0x00,0x20,0x3F,0x20,0x20,0x23,0x20,0x18,0x00,//E 37
  0x08,0xF8,0x88,0x88,0xE8,0x08,0x10,0x00,0x20,0x3F,0x20,0x00,0x03,0x00,0x00,0x00,//F 38
  0xC0,0x30,0x08,0x08,0x08,0x38,0x00,0x00,0x07,0x18,0x20,0x20,0x22,0x1E,0x02,0x00,//G 39
  0x08,0xF8,0x08,0x00,0x00,0x08,0xF8,0x08,0x20,0x3F,0x21,0x01,0x01,0x21,0x3F,0x20,//H 40
  0x00,0x08,0x08,0xF8,0x08,0x08,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//I 41
  0x00,0x00,0x08,0x08,0xF8,0x08,0x08,0x00,0xC0,0x80,0x80,0x80,0x7F,0x00,0x00,0x00,//J 42
  0x08,0xF8,0x88,0xC0,0x28,0x18,0x08,0x00,0x20,0x3F,0x20,0x01,0x26,0x38,0x20,0x00,//K 43
  0x08,0xF8,0x08,0x00,0x00,0x00,0x00,0x00,0x20,0x3F,0x20,0x20,0x20,0x20,0x30,0x00,//L 44
  0x08,0xF8,0xF8,0x00,0xF8,0xF8,0x08,0x00,0x20,0x3F,0x00,0x3F,0x00,0x3F,0x20,0x00,//M 45
  0x08,0xF8,0x30,0xC0,0x00,0x08,0xF8,0x08,0x20,0x3F,0x20,0x00,0x07,0x18,0x3F,0x00,//N 46
  0xE0,0x10,0x08,0x08,0x08,0x10,0xE0,0x00,0x0F,0x10,0x20,0x20,0x20,0x10,0x0F,0x00,//O 47
  0x08,0xF8,0x08,0x08,0x08,0x08,0xF0,0x00,0x20,0x3F,0x21,0x01,0x01,0x01,0x00,0x00,//P 48
  0xE0,0x10,0x08,0x08,0x08,0x10,0xE0,0x00,0x0F,0x18,0x24,0x24,0x38,0x50,0x4F,0x00,//Q 49
  0x08,0xF8,0x88,0x88,0x88,0x88,0x70,0x00,0x20,0x3F,0x20,0x00,0x03,0x0C,0x30,0x20,//R 50
  0x00,0x70,0x88,0x08,0x08,0x08,0x38,0x00,0x00,0x38,0x20,0x21,0x21,0x22,0x1C,0x00,//S 51
  0x18,0x08,0x08,0xF8,0x08,0x08,0x18,0x00,0x00,0x00,0x20,0x3F,0x20,0x00,0x00,0x00,//T 52
  0x08,0xF8,0x08,0x00,0x00,0x08,0xF8,0x08,0x00,0x1F,0x20,0x20,0x20,0x20,0x1F,0x00,//U 53
  0x08,0x78,0x88,0x00,0x00,0xC8,0x38,0x08,0x00,0x00,0x07,0x38,0x0E,0x01,0x00,0x00,//V 54
  0xF8,0x08,0x00,0xF8,0x00,0x08,0xF8,0x00,0x03,0x3C,0x07,0x00,0x07,0x3C,0x03,0x00,//W 55
  0x08,0x18,0x68,0x80,0x80,0x68,0x18,0x08,0x20,0x30,0x2C,0x03,0x03,0x2C,0x30,0x20,//X 56
  0x08,0x38,0xC8,0x00,0xC8,0x38,0x08,0x00,0x00,0x00,0x20,0x3F,0x20,0x00,0x00,0x00,//Y 57
  0x10,0x08,0x08,0x08,0xC8,0x38,0x08,0x00,0x20,0x38,0x26,0x21,0x20,0x20,0x18,0x00,//Z 58
  0x00,0x00,0x00,0xFE,0x02,0x02,0x02,0x00,0x00,0x00,0x00,0x7F,0x40,0x40,0x40,0x00,//[ 59
  0x00,0x0C,0x30,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x06,0x38,0xC0,0x00,//\ 60
  0x00,0x02,0x02,0x02,0xFE,0x00,0x00,0x00,0x00,0x40,0x40,0x40,0x7F,0x00,0x00,0x00,//] 61
  0x00,0x00,0x04,0x02,0x02,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//^ 62
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,//_ 63
  0x00,0x02,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//` 64
  0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x19,0x24,0x22,0x22,0x22,0x3F,0x20,//a 65
  0x08,0xF8,0x00,0x80,0x80,0x00,0x00,0x00,0x00,0x3F,0x11,0x20,0x20,0x11,0x0E,0x00,//b 66
  0x00,0x00,0x00,0x80,0x80,0x80,0x00,0x00,0x00,0x0E,0x11,0x20,0x20,0x20,0x11,0x00,//c 67
  0x00,0x00,0x00,0x80,0x80,0x88,0xF8,0x00,0x00,0x0E,0x11,0x20,0x20,0x10,0x3F,0x20,//d 68
  0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x1F,0x22,0x22,0x22,0x22,0x13,0x00,//e 69
  0x00,0x80,0x80,0xF0,0x88,0x88,0x88,0x18,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//f 70
  0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x6B,0x94,0x94,0x94,0x93,0x60,0x00,//g 71
  0x08,0xF8,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x3F,0x21,0x00,0x00,0x20,0x3F,0x20,//h 72
  0x00,0x80,0x98,0x98,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//i 73
  0x00,0x00,0x00,0x80,0x98,0x98,0x00,0x00,0x00,0xC0,0x80,0x80,0x80,0x7F,0x00,0x00,//j 74
  0x08,0xF8,0x00,0x00,0x80,0x80,0x80,0x00,0x20,0x3F,0x24,0x02,0x2D,0x30,0x20,0x00,//k 75
  0x00,0x08,0x08,0xF8,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//l 76
  0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x20,0x3F,0x20,0x00,0x3F,0x20,0x00,0x3F,//m 77
  0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x3F,0x21,0x00,0x00,0x20,0x3F,0x20,//n 78
  0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x1F,0x20,0x20,0x20,0x20,0x1F,0x00,//o 79
  0x80,0x80,0x00,0x80,0x80,0x00,0x00,0x00,0x80,0xFF,0xA1,0x20,0x20,0x11,0x0E,0x00,//p 80
  0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x0E,0x11,0x20,0x20,0xA0,0xFF,0x80,//q 81
  0x80,0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x20,0x20,0x3F,0x21,0x20,0x00,0x01,0x00,//r 82
  0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x33,0x24,0x24,0x24,0x24,0x19,0x00,//s 83
  0x00,0x80,0x80,0xE0,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x1F,0x20,0x20,0x00,0x00,//t 84
  0x80,0x80,0x00,0x00,0x00,0x80,0x80,0x00,0x00,0x1F,0x20,0x20,0x20,0x10,0x3F,0x20,//u 85
  0x80,0x80,0x80,0x00,0x00,0x80,0x80,0x80,0x00,0x01,0x0E,0x30,0x08,0x06,0x01,0x00,//v 86
  0x80,0x80,0x00,0x80,0x00,0x80,0x80,0x80,0x0F,0x30,0x0C,0x03,0x0C,0x30,0x0F,0x00,//w 87
  0x00,0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x31,0x2E,0x0E,0x31,0x20,0x00,//x 88
  0x80,0x80,0x80,0x00,0x00,0x80,0x80,0x80,0x80,0x81,0x8E,0x70,0x18,0x06,0x01,0x00,//y 89
  0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x21,0x30,0x2C,0x22,0x21,0x30,0x00,//z 90
  0x00,0x00,0x00,0x00,0x80,0x7C,0x02,0x02,0x00,0x00,0x00,0x00,0x00,0x3F,0x40,0x40,//{ 91
  0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,//| 92
  0x00,0x02,0x02,0x7C,0x80,0x00,0x00,0x00,0x00,0x40,0x40,0x3F,0x00,0x00,0x00,0x00,//} 93
  0x00,0x06,0x01,0x01,0x02,0x02,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//~ 94
};
// 16x16 中文字模：鸿蒙先锋号（顺序：鸿、蒙、先、锋、号）
static const unsigned char F16X16_CHN[][32] = {
    {0x48,0x26,0x00,0x10,0x10,0x1F,0x10,0x10,0x00,0x3F,0x68,0xA4,0x21,0x3F,0x00,0x00,0x20,0x7E,0x80,0x08,0x0C,0xF8,0x10,0x10,0x00,0xC8,0x48,0x48,0x4A,0x41,0x7E,0x00},
    {0x44,0x59,0x51,0x51,0xF5,0x55,0x55,0x55,0x55,0x55,0xF5,0x51,0x51,0x55,0x58,0x00,0x00,0x0A,0x4A,0x52,0x54,0xA4,0xCA,0x51,0x3E,0x10,0x28,0x24,0x44,0x02,0x02,0x00},
    {0x01,0x05,0x09,0x71,0x11,0x11,0x11,0xFF,0x11,0x11,0x11,0x11,0x11,0x01,0x01,0x00,0x01,0x01,0x02,0x04,0x18,0xE0,0x00,0x00,0x00,0xFC,0x02,0x02,0x02,0x02,0x0E,0x00},
    {0x04,0x08,0x34,0xE7,0x24,0x24,0x01,0x09,0x32,0xEA,0x25,0x2A,0x32,0x21,0x01,0x00,0x80,0x80,0x80,0xFE,0x84,0x88,0x08,0x08,0xA8,0xA8,0xFF,0xA8,0xA8,0x08,0x08,0x00},
    {0x02,0x02,0x02,0xFA,0x8B,0x8A,0x8A,0x8A,0x8A,0x8A,0x8A,0xFA,0x02,0x02,0x02,0x00,0x00,0x00,0x00,0xC0,0x40,0x40,0x40,0x40,0x44,0x42,0x44,0x78,0x00,0x00,0x00,0x00},
};

// ===================== SSD1306 OLED 驱动 =====================
#define SSD1306_I2C_ADDR (0x78)
#define SSD1306_I2C_IDX  0

static osMutexId_t g_i2cMutex = NULL;  // 保护 I2C0：避免 OLED 写与传感器读写多线程并发冲突

static uint32_t SSD1306_SendData(uint8_t *data, size_t size)
{
  if (g_i2cMutex) osMutexAcquire(g_i2cMutex, osWaitForever);
  WifiIotI2cData i2cData = {0};
  i2cData.sendBuf = data;
  i2cData.sendLen = size;
  uint32_t r = I2cWrite(SSD1306_I2C_IDX, SSD1306_I2C_ADDR, &i2cData);
  if (g_i2cMutex) osMutexRelease(g_i2cMutex);
  return r;
}

static uint32_t SSD1306_WriteCmd(uint8_t byte)
{
  uint8_t buffer[] = {0x00, byte};
  return SSD1306_SendData(buffer, sizeof(buffer));
}

static uint32_t SSD1306_WriteData(uint8_t byte)
{
  uint8_t buffer[] = {0x40, byte};
  return SSD1306_SendData(buffer, sizeof(buffer));
}

void SSD1306_SetPos(uint8_t x, uint8_t y);  // 前向声明，供 SSD1306_Init 调用

uint32_t SSD1306_Init(void)
{
    uint32_t result;
    GpioInit();
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_10, WIFI_IOT_IO_FUNC_GPIO_10_I2C0_SDA);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_9, WIFI_IOT_IO_FUNC_GPIO_9_I2C0_SCL);
    result = I2cInit(WIFI_IOT_I2C_IDX_0, 400000);
    I2cSetBaudrate(WIFI_IOT_I2C_IDX_0, 400000);
    if (result != 0) {
      printf("I2C SSD1306 Init status is 0x%x!!!\n", result);
      return result;
    }
    usleep(100 * 1000);
    SSD1306_WriteCmd(0xAE);
    SSD1306_WriteCmd(0x20);
    SSD1306_WriteCmd(0x10);
    SSD1306_WriteCmd(0xb0);
    SSD1306_WriteCmd(0xc8);
    SSD1306_WriteCmd(0x00);
    SSD1306_WriteCmd(0x10);
    SSD1306_WriteCmd(0x40);
    SSD1306_WriteCmd(0x81);
    SSD1306_WriteCmd(0xff);
    SSD1306_WriteCmd(0xa1);
    SSD1306_WriteCmd(0xa6);
    SSD1306_WriteCmd(0xa8);
    SSD1306_WriteCmd(0x1F);   // multiplex ratio=32（0.91寸 128×32 屏）；原 0x3F=64 是 128×64 参数，会让 32 行屏糊成一团
    SSD1306_WriteCmd(0xa4);
    SSD1306_WriteCmd(0xd3);
    SSD1306_WriteCmd(0x00);
    SSD1306_WriteCmd(0xd5);
    SSD1306_WriteCmd(0xf0);
    SSD1306_WriteCmd(0xd9);
    SSD1306_WriteCmd(0x22);   // 预充电周期（SSD1306 上电默认值，绝大多数 128×32 屏适用）
    SSD1306_WriteCmd(0xda);
    SSD1306_WriteCmd(0x02);   // COM pins=sequential（适配 32 行屏）；原 0x12 是 64 行配置
    SSD1306_WriteCmd(0xdb);
    SSD1306_WriteCmd(0x20);
    SSD1306_WriteCmd(0x8d);
    SSD1306_WriteCmd(0x14);
    SSD1306_WriteCmd(0xAF);
    SSD1306_SetPos(0,0);
    printf("I2C SSD1306 Init is succeeded!!!\r\n");
    return 0;
}

void SSD1306_SetPos(uint8_t x, uint8_t y)
{
  SSD1306_WriteCmd(0xb0 + y);
  SSD1306_WriteCmd(((x & 0xf0) >> 4) | 0x10);
  SSD1306_WriteCmd((x & 0x0f));
}

void SSD1306_Fill(uint8_t fill_Data)
{
  unsigned char m, n;
  for (m = 0; m < 4; m++) {   // 128×32 屏只有 4 页，只清 4 页（原 8 页是 64 行屏写法）
    SSD1306_WriteCmd(0xb0 + m);
    SSD1306_WriteCmd(0x00);
    SSD1306_WriteCmd(0x10);
    for (n = 0; n < 128; n++)
      SSD1306_WriteData(fill_Data);
  }
}

void SSD1306_CLS(void)
{
  SSD1306_Fill(0x00);
}

void SSD1306_ShowStr(uint8_t x, uint8_t y, uint8_t ch[], uint8_t TextSize)
{
  unsigned char c = 0, i = 0, j = 0;
  switch (TextSize) {
  case 8:
    while (ch[j] != '\0') {
      // 只渲染可打印 ASCII(32~126)：其他字节(控制符、中文的高字节)会令 ch[j]-32
      // 超出 F6x8 的 95 项范围导致越界读，这里直接跳过。
      if (ch[j] < 32 || ch[j] > 126) { j++; continue; }
      c = ch[j] - 32;
      if (x > 126) { x = 0; y++; }
      SSD1306_SetPos(x, y);
      for (i = 0; i < 6; i++) SSD1306_WriteData(F6x8[c][i]);
      x += 6; j++;
    }
    break;
  case 16:
    y *= 2;
    while (ch[j] != '\0') {
      // 同上，避免 F8X16 越界读
      if (ch[j] < 32 || ch[j] > 126) { j++; continue; }
      c = ch[j] - 32;
      if (x > 120) { x = 0; y++; }
      SSD1306_SetPos(x, y);
      for (i = 0; i < 8; i++) SSD1306_WriteData(F8X16[c * 16 + i]);
      SSD1306_SetPos(x, y + 1);
      for (i = 0; i < 8; i++) SSD1306_WriteData(F8X16[c * 16 + i + 8]);
      x += 8; j++;
    }
    break;
  }
}

void SSD1306_ShowChineseStr(uint8_t x, uint8_t y, uint8_t *str, uint8_t count)
{
    (void)str;  // 字模按固定顺序取，不使用 str 内容（消 -Werror）
    uint8_t i, k;
    y *= 2;
    for (k = 0; k < count; k++) {
        if (x > 128 - 16) { x = 0; y += 2; }
        SSD1306_SetPos(x, y);
        for (i = 0; i < 16; i++) SSD1306_WriteData(F16X16_CHN[k][i]);
        SSD1306_SetPos(x, y + 1);
        for (i = 0; i < 16; i++) SSD1306_WriteData(F16X16_CHN[k][i + 16]);
        x += 16;
    }
}

// ===================== SHT20 温湿度 =====================
#define SHT20_NoHoldMaster_Temp_REG_ADDR 0xF3
#define SHT20_NoHoldMaster_Humi_REG_ADDR 0xF5
#define SHT20_SW_REG_ADDR 0xFE

// 【门槛破解1】I2C 地址用 8 位 0x80（不是手册上的 7 位 0x40）
#define SHT20_I2C_ADDR (0x80)
#define SHT20_I2C_IDX  0

static uint32_t SHT20_WriteByte(uint8_t byte)
{
    uint8_t buf[] = {byte};
    WifiIotI2cData d = {0};
    d.sendBuf = buf;
    d.sendLen = 1;
    return I2cWrite(SHT20_I2C_IDX, SHT20_I2C_ADDR, &d);
}

static uint32_t SHT20_Recv(uint8_t *data, size_t size)
{
    WifiIotI2cData d = {0};
    d.receiveBuf = data;
    d.receiveLen = size;
    return I2cRead(SHT20_I2C_IDX, SHT20_I2C_ADDR, &d);
}

uint32_t SHT20_ReadData(float *temp, float *humi)
{
    uint32_t result;
    uint8_t buf[4] = {0};

    // 【优化】SHT20 工作在 NoHoldMaster 模式：发完测量命令后由芯片内部完成转换，
    // 这段等待期 I2C 总线是空闲的，完全没必要一直攥着互斥锁。
    // 改为「发命令(加锁) → 等待转换(释放锁) → 读结果(加锁)」的分段式加锁：
    // 锁的持有时间从 135ms(85+50) 降到 4 个毫秒级事务，OLED 刷屏不再被长时间阻塞。
    if (g_i2cMutex) osMutexAcquire(g_i2cMutex, osWaitForever);
    result = SHT20_WriteByte(SHT20_NoHoldMaster_Temp_REG_ADDR);
    if (g_i2cMutex) osMutexRelease(g_i2cMutex);
    if (result != 0) return result;

    usleep(85 * 1000);   // 温度转换等待（不持锁，期间其他设备可用 I2C）

    if (g_i2cMutex) osMutexAcquire(g_i2cMutex, osWaitForever);
    result = SHT20_Recv(buf, 3);
    if (g_i2cMutex) osMutexRelease(g_i2cMutex);
    if (result != 0) return result;
    // 【门槛破解2】SHT20 返回 16 位, 低 2 位是状态位, 必须 & 0xFFFC
    uint16_t t_raw = (uint16_t)(((uint16_t)buf[0] << 8) | buf[1]) & 0xFFFC;
    *temp = 175.72f * (t_raw / 65536.0f) - 46.85f;

    if (g_i2cMutex) osMutexAcquire(g_i2cMutex, osWaitForever);
    result = SHT20_WriteByte(SHT20_NoHoldMaster_Humi_REG_ADDR);
    if (g_i2cMutex) osMutexRelease(g_i2cMutex);
    if (result != 0) return result;

    usleep(50 * 1000);   // 湿度转换等待（不持锁）

    if (g_i2cMutex) osMutexAcquire(g_i2cMutex, osWaitForever);
    result = SHT20_Recv(buf, 3);
    if (g_i2cMutex) osMutexRelease(g_i2cMutex);
    if (result != 0) return result;
    uint16_t h_raw = (uint16_t)(((uint16_t)buf[0] << 8) | buf[1]) & 0xFFFC;
    *humi = 125.0f * (h_raw / 65536.0f) - 6.0f;
    return 0;
}

// 注意：I2C0 已由 SSD1306_Init() 初始化, 这里只做 SHT20 软复位, 不要再调 I2cInit
uint32_t SHT20_Init(void)
{
    uint32_t result;
    if (g_i2cMutex) osMutexAcquire(g_i2cMutex, osWaitForever);
    result = SHT20_WriteByte(SHT20_SW_REG_ADDR);
    if (g_i2cMutex) osMutexRelease(g_i2cMutex);
    if (result != 0) return result;
    usleep(100 * 1000);
    return 0;
}

// ===================== AP3216C 环境光/接近传感器 =====================
// 老师图里标 0x1E（7 位），本代码库 HAL 收 8 位地址 → 0x1E << 1 = 0x3C
#define AP3216C_I2C_ADDR (0x3C)
#define AP3216C_I2C_IDX  0

#define AP3216C_SYSTEM_ADDR 0x00
#define AP3216C_IR_L_ADDR   0x0A
#define AP3216C_IR_H_ADDR   0x0B
#define AP3216C_ALS_L_ADDR  0x0C
#define AP3216C_ALS_H_ADDR  0x0D
#define AP3216C_PS_L_ADDR   0x0E
#define AP3216C_PS_H_ADDR   0x0F

static uint32_t AP3216C_WriteByteData(uint8_t byte)
{
    WifiIotI2cData i2cData = {0};
    i2cData.sendBuf = &byte;
    i2cData.sendLen = 1;
    return I2cWrite(AP3216C_I2C_IDX, AP3216C_I2C_ADDR, &i2cData);
}

static uint32_t AP3216C_RecvData(uint8_t *data, size_t size)
{
    WifiIotI2cData i2cData = {0};
    i2cData.receiveBuf = data;
    i2cData.receiveLen = size;
    return I2cRead(AP3216C_I2C_IDX, AP3216C_I2C_ADDR, &i2cData);
}

static uint32_t AP3216C_WriteCmdByteData(uint8_t regAddr, uint8_t byte)
{
    uint8_t buffer[] = {regAddr, byte};
    WifiIotI2cData i2cData = {0};
    i2cData.sendBuf = buffer;
    i2cData.sendLen = 2;
    return I2cWrite(AP3216C_I2C_IDX, AP3216C_I2C_ADDR, &i2cData);
}

static uint32_t AP3216C_ReadRegByteData(uint8_t regAddr, uint8_t *byte)
{
    uint32_t result;
    uint8_t buffer[2] = {0};

    result = AP3216C_WriteByteData(regAddr);
    if (result != 0) return result;

    result = AP3216C_RecvData(buffer, 1);
    if (result != 0) return result;

    *byte = buffer[0];
    return 0;
}

uint32_t AP3216C_ReadData(uint16_t *irData, uint16_t *alsData, uint16_t *psData)
{
    uint32_t result;
    uint8_t data_H = 0, data_L = 0;

    if (g_i2cMutex) osMutexAcquire(g_i2cMutex, osWaitForever);

    // IR 10-bit
    result = AP3216C_ReadRegByteData(AP3216C_IR_L_ADDR, &data_L);
    if (result != 0) { if (g_i2cMutex) osMutexRelease(g_i2cMutex); return result; }
    result = AP3216C_ReadRegByteData(AP3216C_IR_H_ADDR, &data_H);
    if (result != 0) { if (g_i2cMutex) osMutexRelease(g_i2cMutex); return result; }
    if (data_L & 0x80) *irData = 0;
    else *irData = ((uint16_t)data_H << 2) | (data_L & 0x03);

    // ALS 16-bit
    result = AP3216C_ReadRegByteData(AP3216C_ALS_L_ADDR, &data_L);
    if (result != 0) { if (g_i2cMutex) osMutexRelease(g_i2cMutex); return result; }
    result = AP3216C_ReadRegByteData(AP3216C_ALS_H_ADDR, &data_H);
    if (result != 0) { if (g_i2cMutex) osMutexRelease(g_i2cMutex); return result; }
    *alsData = ((uint16_t)data_H << 8) | data_L;

    // PS 10-bit
    result = AP3216C_ReadRegByteData(AP3216C_PS_L_ADDR, &data_L);
    if (result != 0) { if (g_i2cMutex) osMutexRelease(g_i2cMutex); return result; }
    result = AP3216C_ReadRegByteData(AP3216C_PS_H_ADDR, &data_H);
    if (result != 0) { if (g_i2cMutex) osMutexRelease(g_i2cMutex); return result; }
    if (data_L & 0x40) *psData = 0;
    else *psData = ((uint16_t)(data_H & 0x3F) << 4) | (data_L & 0x0F);

    if (g_i2cMutex) osMutexRelease(g_i2cMutex);
    return 0;
}

// I2C0 已由 SSD1306_Init() 初始化，这里只发复位+模式命令
uint32_t AP3216C_Init(void)
{
    uint32_t result;

    if (g_i2cMutex) osMutexAcquire(g_i2cMutex, osWaitForever);
    result = AP3216C_WriteCmdByteData(AP3216C_SYSTEM_ADDR, 0x04);  // 复位
    if (result != 0) { if (g_i2cMutex) osMutexRelease(g_i2cMutex); return result; }
    if (g_i2cMutex) osMutexRelease(g_i2cMutex);

    usleep(5000);

    if (g_i2cMutex) osMutexAcquire(g_i2cMutex, osWaitForever);
    result = AP3216C_WriteCmdByteData(AP3216C_SYSTEM_ADDR, 0x03);  // 开启 ALS + PS + IR
    if (g_i2cMutex) osMutexRelease(g_i2cMutex);
    if (result != 0) return result;

    return 0;
}

// ===================== UART1 蓝牙（BLE / JDY-16） =====================
// 硬件：IO00=UART1_TXD, IO01=UART1_RXD（连 JDY-16 蓝牙模块）
// 【波特率自适应】JDY-16 出厂波特率不唯一（常见 9600，也有 115200），而系统 peripheral_init()
//   默认把 UART1 配成 115200（用于 AT/调试）。若模块波特率与 UART1 不一致，收到的数据全是乱码，
//   表现就是"手机能连上但发指令没反应"。这里不猜：启动时依次尝试候选波特率，
//   哪个能收到手机数据就锁定哪个（扫描逻辑见 BLE_RecvThread）。
//   注：UartInit 只改波特率、不碰引脚（底层 hi_uart_init 不设置 IO 复用），故每次设完波特率
//   都要重新 IoSetFunc，确保 GPIO_0/1 最终复用为 UART1。
#define BLE_UART_IDX  WIFI_IOT_UART_IDX_1

// 候选波特率：系统 peripheral_init() 已把 UART1 默认配成 115200 给 JDY-16 用，
// 所以模块最可能是 115200 —— 把它放第一个，上电即锁，不必等 3s 扫描。
// 若模块实是其他波特率，再依次补试（9600 出厂也常见）。
static const unsigned int g_bleBaudCands[] = {115200, 9600, 19200, 38400, 57600, 4800, 2400};
#define BLE_BAUD_CAND_NUM  ((int)(sizeof(g_bleBaudCands) / sizeof(g_bleBaudCands[0])))
static int g_bleBaudIdx = 0;      // 当前尝试的候选下标
static int g_bleBaudTried = 0;    // 已切换过的候选数（防 UartInit 反复分配 RX 缓冲泄漏）
static int g_bleBaudLocked = 0;   // 1=已锁定（收到过有效数据），不再切换

// 设置蓝牙 UART1 波特率 + 重新固定引脚
static void BLE_SetBaud(unsigned int baud)
{
    WifiIotUartAttribute attr = {0};
    attr.baudRate = baud;
    attr.dataBits = WIFI_IOT_UART_DATA_BIT_8;
    attr.stopBits = WIFI_IOT_UART_STOP_BIT_1;
    attr.parity   = WIFI_IOT_UART_PARITY_NONE;

    if (UartInit(BLE_UART_IDX, &attr, NULL) != 0) {
        printf("BLE set baud %u FAILED\r\n", baud);
    }
    // 每次改波特率后重新固定引脚（部分驱动 init 会重置 IO 复用）
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_0, WIFI_IOT_IO_FUNC_GPIO_0_UART1_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_1, WIFI_IOT_IO_FUNC_GPIO_1_UART1_RXD);
}

static void BLE_UartInit(void)
{
    // 从第一个候选开始（115200，与系统 peripheral_init 默认 UART1 波特率一致，最可能是模块真实波特率），
    // 由 BLE_RecvThread 负责扫描并锁定（若模块实是其他波特率，手机发指令会触发切换到对应候选）
    BLE_SetBaud(g_bleBaudCands[g_bleBaudIdx]);
    printf("BLE UART1 ready (auto-baud, start %u, pin GPIO0/1, JDY-16)\r\n",
           g_bleBaudCands[g_bleBaudIdx]);
}

// ===================== UART2 与 STM32 通信 + 双电机控制 =====================
// 硬件：IO11=UART2_TXD, IO12=UART2_RXD（连 STM32）
// 协议（6 字节）：0xFC | A_dir | A_speed | B_dir | B_speed | 0xFD
//   dir  : 1=正转, 0=反转
//   speed: 0~150（限幅，超过按 150 发）
#define MOTOR_UART_IDX   WIFI_IOT_UART_IDX_2
#define MOTOR_BAUDRATE   115200
#define MOTOR_SPEED_MAX  150
#define MOTOR_FRAME_HEAD 0xFC
#define MOTOR_FRAME_TAIL 0xFD

static int g_motorUartReady = 0;   // UART2 初始化成功标志

// 与蓝牙 UART1 不同，UART2 系统没有初始化过，这里必须主动 UartInit
static void Motor_UartInit(void)
{
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD);

    WifiIotUartAttribute attr = {0};
    attr.baudRate = MOTOR_BAUDRATE;
    attr.dataBits = WIFI_IOT_UART_DATA_BIT_8;
    attr.stopBits = WIFI_IOT_UART_STOP_BIT_1;
    attr.parity   = WIFI_IOT_UART_PARITY_NONE;

    if (UartInit(MOTOR_UART_IDX, &attr, NULL) != 0) {
        g_motorUartReady = 0;
        printf("Motor UART2 init fail\r\n");
    } else {
        g_motorUartReady = 1;
        printf("Motor UART2 ready (115200)\r\n");
    }
}

// 双电机控制：motorA=左轮速度, motorB=右轮速度，范围 -150 ~ +150
// 协议方向位对齐老师 robot_l9110s.c：dir=0 正转，dir=1 反转（负速度 -> dir=1）
static void stm32_motor_control(int motorA, int motorB)
{
    unsigned char buf[6];
    unsigned char aDir = 0, bDir = 0;   // 0=正转, 1=反转

    if (!g_motorUartReady) {
        return;   // UART2 未就绪时不发，避免访问无效驱动导致异常
    }

    // 方向分离：负速度 -> 反转(dir=1)，并取绝对值
    if (motorA < 0) { aDir = 1; motorA = -motorA; }
    if (motorB < 0) { bDir = 1; motorB = -motorB; }

    // 限幅，防止超出 STM32 侧协议范围
    if (motorA > MOTOR_SPEED_MAX) motorA = MOTOR_SPEED_MAX;
    if (motorB > MOTOR_SPEED_MAX) motorB = MOTOR_SPEED_MAX;

    // 组帧：帧头 + 左轮(方向,速度) + 右轮(方向,速度) + 帧尾
    buf[0] = MOTOR_FRAME_HEAD;
    buf[1] = aDir;
    buf[2] = (unsigned char)motorA;
    buf[3] = bDir;
    buf[4] = (unsigned char)motorB;
    buf[5] = MOTOR_FRAME_TAIL;

    UartWrite(MOTOR_UART_IDX, buf, sizeof(buf));
}

// 运动封装（对齐老师 robot_l9110s.c：正转=前进，反转=后退）
// 若实测方向相反，把下面各函数的正负号整体取反即可（只改这一处）。
// 无参数版：统一使用 CAR_DEFAULT_SPEED 油门；需要自定义速度时直接调 stm32_motor_control()。
#define CAR_DEFAULT_SPEED 60   // 默认油门（约 40%）

static void car_forward(void)
{
    stm32_motor_control(CAR_DEFAULT_SPEED, CAR_DEFAULT_SPEED);      // 两轮正转 = 前进
}
static void car_backward(void)
{
    stm32_motor_control(-CAR_DEFAULT_SPEED, -CAR_DEFAULT_SPEED);    // 两轮反转 = 后退
}
static void car_left(void)
{
    stm32_motor_control(-CAR_DEFAULT_SPEED, CAR_DEFAULT_SPEED);     // 左退右进 = 车头左转
}
static void car_right(void)
{
    stm32_motor_control(CAR_DEFAULT_SPEED, -CAR_DEFAULT_SPEED);     // 左进右退 = 车头右转
}
static void car_stop(void)
{
    stm32_motor_control(0, 0);
}

// ===================== WS2812 内置 LED（IO06，车前后各 6 颗共 12 颗） =====================
#define WS_PIN      WIFI_IOT_IO_NAME_GPIO_6
#define WS_COUNT    12

// 时序 nop 参数
// 【已修正】Hi3861 的 CPU 主频 CONFIG_CPU_CLOCK = 160MHz（见 hi3861_platform_base.h，
// hi_cpu.h 亦注明 "Default CPU clock is 160M"），即 1 个时钟周期约 6.25ns。
// 原参数按 40MHz(≈25ns/循环)估算，在 160MHz 下实际电平宽度只有目标值的 1/4
// （0码高电平仅 ~87ns，规格要求 ~350ns）→ WS2812 无法识别数据，灯不亮或严重偏色。
// 现按 160MHz 重算：ws_nops 每次循环约 2 个周期(≈12.5ns)，对应
//   0码高 350ns→28   0码低 800ns→64   1码高 700ns→56   1码低 600ns→48
// 若实测颜色仍有偏差，把这 4 个值按同比例微调即可。
#define WS_NOP_T0H  28   // 0码高电平 ~350ns
#define WS_NOP_T0L  64   // 0码低电平 ~800ns
#define WS_NOP_T1H  56   // 1码高电平 ~700ns
#define WS_NOP_T1L  48   // 1码低电平 ~600ns

static void ws_nops(unsigned int n)
{
    while (n--) {
        __asm__ volatile("nop");
    }
}

static void ws_send_byte(uint8_t b)
{
    for (int i = 7; i >= 0; i--) {
        if (b & (1u << i)) {
            GpioSetOutputVal(WS_PIN, WIFI_IOT_GPIO_VALUE1);
            ws_nops(WS_NOP_T1H);
            GpioSetOutputVal(WS_PIN, WIFI_IOT_GPIO_VALUE0);
            ws_nops(WS_NOP_T1L);
        } else {
            GpioSetOutputVal(WS_PIN, WIFI_IOT_GPIO_VALUE1);
            ws_nops(WS_NOP_T0H);
            GpioSetOutputVal(WS_PIN, WIFI_IOT_GPIO_VALUE0);
            ws_nops(WS_NOP_T0L);
        }
    }
}

// 注意：WS2812 的数据顺序是 GRB，不是 RGB
static void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b)
{
    osKernelLock();  // 锁调度，保证整串数据时序不被任务切换打断
    for (int i = 0; i < WS_COUNT; i++) {
        ws_send_byte(g);
        ws_send_byte(r);
        ws_send_byte(b);
    }
    osKernelUnlock();
    GpioSetOutputVal(WS_PIN, WIFI_IOT_GPIO_VALUE0);
    hi_udelay(60);   // 复位信号：低电平保持 > 50us
}

static void WS2812_Init(void)
{
    IoSetFunc(WS_PIN, WIFI_IOT_IO_FUNC_GPIO_6_GPIO);
    GpioSetDir(WS_PIN, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetOutputVal(WS_PIN, WIFI_IOT_GPIO_VALUE0);
    WS2812_SetAll(0, 0, 0);
}

// ===================== HC-SR04 超声波（IO07=TRIG, IO08=ECHO） =====================
#define HC_TRIG  WIFI_IOT_IO_NAME_GPIO_7
#define HC_ECHO  WIFI_IOT_IO_NAME_GPIO_8

static void HCSR04_Init(void)
{
    IoSetFunc(HC_TRIG, WIFI_IOT_IO_FUNC_GPIO_7_GPIO);
    GpioSetDir(HC_TRIG, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetOutputVal(HC_TRIG, WIFI_IOT_GPIO_VALUE0);

    IoSetFunc(HC_ECHO, WIFI_IOT_IO_FUNC_GPIO_8_GPIO);
    GpioSetDir(HC_ECHO, WIFI_IOT_GPIO_DIR_IN);
}

// 返回距离(cm)；返回 <0 表示超时 / 没测到
// 与原版驱动的区别：加了超时保护，模块没接或故障时不会 while(1) 卡死整个线程
static float HCSR04_GetDistance(void)
{
    WifiIotGpioValue v = WIFI_IOT_GPIO_VALUE0;
    unsigned long long t0 = 0, t1 = 0, due;

    // TRIG 发一个 >=10us 的高脉冲
    GpioSetOutputVal(HC_TRIG, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(20);
    GpioSetOutputVal(HC_TRIG, WIFI_IOT_GPIO_VALUE0);

    // 等 ECHO 变高（最多等 40ms，对应约 6.8m；空旷无回波时避免长时间阻塞线程）
    due = hi_get_us() + 40000;
    while (hi_get_us() < due) {
        GpioGetInputVal(HC_ECHO, &v);
        if (v == WIFI_IOT_GPIO_VALUE1) { t0 = hi_get_us(); break; }
    }
    if (t0 == 0) return -1.0f;

    // 等 ECHO 变低，得到高电平持续时间（最多等 40ms）
    due = hi_get_us() + 40000;
    while (hi_get_us() < due) {
        GpioGetInputVal(HC_ECHO, &v);
        if (v == WIFI_IOT_GPIO_VALUE0) { t1 = hi_get_us(); break; }
    }
    if (t1 == 0) return -1.0f;

    // 距离 = 高电平时间(us) * 0.034 / 2
    return (float)((double)(t1 - t0) * 0.034 / 2.0);
}

// ===================== TCRT5000 车底检测（CGQ1=IO13, CGQ2=IO14） =====================
// 原理图：CGQ1/CGQ2 是两个 TCRT5000 红外对管（车底），经 LM393 U22 比较器输出 TC_OUT_L/R -> IO13/IO14。
// 极性（对齐老师 trace_model.c）：低电平=有反射(地面/白面)，高电平=无反射(黑线/悬空)。
// 同一组传感器两种用途：巡线时高电平=压黑线；防掉落时高电平=车底悬空。
#define TC_L  WIFI_IOT_IO_NAME_GPIO_13
#define TC_R  WIFI_IOT_IO_NAME_GPIO_14

static void TCRT_Init(void)
{
    IoSetFunc(TC_L, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    GpioSetDir(TC_L, WIFI_IOT_GPIO_DIR_IN);
    IoSetFunc(TC_R, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    GpioSetDir(TC_R, WIFI_IOT_GPIO_DIR_IN);
}

// 读车底检测：1=检测到无反射(黑线/悬空)，0=有反射(地面正常)
static void TCRT_Read(uint8_t *leftNoReflect, uint8_t *rightNoReflect)
{
    WifiIotGpioValue vL = WIFI_IOT_GPIO_VALUE0;
    WifiIotGpioValue vR = WIFI_IOT_GPIO_VALUE0;
    *leftNoReflect = (GpioGetInputVal(TC_L, &vL) == 0 && vL == WIFI_IOT_GPIO_VALUE1) ? 1 : 0;
    *rightNoReflect = (GpioGetInputVal(TC_R, &vR) == 0 && vR == WIFI_IOT_GPIO_VALUE1) ? 1 : 0;
}

// ===================== 内置 LED 联动逻辑 =====================
#define DIST_NEAR_CM   20    // 距离小于 20cm 视为"靠近"
#define ALS_DARK_LUX   100   // 光照小于 100 视为"太暗"

// 0=灭 1=靠近(红) 2=太暗(白)；只在状态变化时才刷灯，避免频繁占用 CPU
static void Led_Update(float dist, uint16_t als, int alsValid, int *lastState)
{
    int want;
    if (dist >= 0.0f && dist < DIST_NEAR_CM) {
        want = 1;                 // 靠近：红色警示（优先级最高）
    } else if (alsValid && als < ALS_DARK_LUX) {
        want = 2;                 // 太暗：白色照明（仅当光感有效才判断）
    } else {
        want = 0;                 // 正常：灭
    }

    if (want != *lastState) {
        *lastState = want;
        if (want == 1) {
            WS2812_SetAll(255, 0, 0);
        } else if (want == 2) {
            WS2812_SetAll(255, 255, 255);
        } else {
            WS2812_SetAll(0, 0, 0);
        }
    }
}

// ===================== 多线程 + 信号量（复刻老师架构） =====================
static float g_temp = 0;
static float g_humi = 0;
static uint16_t g_ir = 0;
static uint16_t g_als = 0;
static int g_alsValid = 0;   // AP3216C 是否读到有效光照数据
static uint16_t g_ps = 0;
static float g_distance = -1.0f;   // 超声波距离(cm)，<0 表示没测到
static int g_ledState = -1;        // 内置 LED 当前状态（见 Led_Update）
static uint8_t g_leftBlack = 0;    // 1=左车底检测识别到无反射/黑线（IO13，高电平有效）
static uint8_t g_rightBlack = 0;   // 1=右车底检测识别到无反射/黑线（IO14，高电平有效）
static osSemaphoreId_t g_semRead = NULL;
static osSemaphoreId_t g_semDisp = NULL;
static volatile int g_i2cReady = 0;   // I2C 设备(OLED/SHT20/AP3216C)初始化完成标志

// thread1：周期释放两个信号量（生产者）
static void Sensor_ProducerThread(void *arg)
{
    (void)arg;
    while (1) {
        if (g_semRead) osSemaphoreRelease(g_semRead);
        if (g_semDisp) osSemaphoreRelease(g_semDisp);
        osDelay(300);  // LiteOS-M tick=1ms，故 300 tick = 300ms；传感器约 3.3Hz 刷新
    }
}

// thread2：读 SHT20 + AP3216C -> 串口打印
// 【门槛破解3】Hi3861 的 printf 默认不支持 %f，改用整数放大 ×100 拼两位小数
static void Sensor_ReaderThread(void *arg)
{
    (void)arg;
    while (1) {
        osSemaphoreAcquire(g_semRead, osWaitForever);

        float temp = 0, humi = 0;
        uint16_t ir = 0, als = 0, ps = 0;

        // 仅当 I2C 设备初始化完成才读（避免 I2C 卡死拖垮整个线程）
        if (g_i2cReady) {
            if (SHT20_ReadData(&temp, &humi) == 0) {
                g_temp = temp;
                g_humi = humi;
            } else {
                printf("SHT20 read fail\r\n");
            }

            if (AP3216C_ReadData(&ir, &als, &ps) == 0) {
                g_ir = ir;
                g_als = als;
                g_ps = ps;
                g_alsValid = 1;   // 只有真正读到数据才认为光照值有效
            } else {
                g_alsValid = 0;
                printf("AP3216C read fail\r\n");
            }
        }

        // 超声波测距（GPIO 操作，不占 I2C 总线，无需加互斥锁）
        float dist = HCSR04_GetDistance();
        g_distance = dist;

        // 红外巡线（IO13/IO14 输入，高电平=无反射=黑线/悬空，与 TCRT_Read 实现一致）
        TCRT_Read(&g_leftBlack, &g_rightBlack);

        int ti = (int)(temp * 100.0f + (temp >= 0 ? 0.5f : -0.5f));
        int hi = (int)(humi * 100.0f + (humi >= 0 ? 0.5f : -0.5f));
        int ta = (ti < 0) ? -ti : ti;  // 负数时取绝对值, 符号单独拼
        printf("T=%s%d.%02dC H=%d.%02d%% L=%u P=%u I=%u D=%dcm TCRT=%s%s\r\n",
               (ti < 0) ? "-" : "", ta / 100, ta % 100, hi / 100, hi % 100,
               als, ps, ir, (int)(dist < 0 ? -1 : dist),
               g_leftBlack ? "L" : "-", g_rightBlack ? "R" : "-");
    }
}

// thread3：把最新数据刷新到 OLED（读与显示解耦）
static void Sensor_DisplayThread(void *arg)
{
    (void)arg;
    char line[32];
    // 清行缓冲：128 像素 / 6 像素每字符 ≈ 21.3，取 21 字符（21×6=126<128），
    // 避免越界（页地址模式下列地址到 127 会回绕到本页列 0，覆盖行首字符）
    char blank[22];
    memset(blank, ' ', 21);
    blank[21] = '\0';

    // 防御 GPIO_6 复用冲突：系统 app_io_init() 默认把 GPIO_6 配成 UART1_TXD（AT 口默认脚），
    // 而 GPIO_6 同时是 WS2812 灯带数据线。若系统初始化晚于本应用执行，GPIO_6 会被抢成
    // UART1_TXD —— 灯不亮，且 UART1 发数据时波形串到灯带导致乱闪。
    // 任务运行时（系统初始化早已结束）明确设回普通 GPIO 输出，确保灯带受控。
    IoSetFunc(WS_PIN, WIFI_IOT_IO_FUNC_GPIO_6_GPIO);
    GpioSetDir(WS_PIN, WIFI_IOT_GPIO_DIR_OUT);

    while (1) {
        osSemaphoreAcquire(g_semDisp, osWaitForever);

        int ti = (int)(g_temp * 100.0f + (g_temp >= 0 ? 0.5f : -0.5f));
        int hi = (int)(g_humi * 100.0f + (g_humi >= 0 ? 0.5f : -0.5f));
        int ta = (ti < 0) ? -ti : ti;

        // 仅当 I2C 就绪才刷 OLED（I2C 卡死时跳过显示，不影响其他功能）
        if (g_i2cReady) {
            // 第 2 页：温湿度
            snprintf(line, sizeof(line), "T:%s%d.%02dC H:%d.%02d%%",
                     (ti < 0) ? "-" : "", ta / 100, ta % 100, hi / 100, hi % 100);
            SSD1306_ShowStr(0, 2, (uint8_t *)blank, 8);
            SSD1306_ShowStr(0, 2, (uint8_t *)line, 8);

            // 第 3 页：光照 / 接近 / 红外
            snprintf(line, sizeof(line), "L:%u P:%u I:%u", g_als, g_ps, g_ir);
            SSD1306_ShowStr(0, 3, (uint8_t *)blank, 8);
            SSD1306_ShowStr(0, 3, (uint8_t *)line, 8);
        }

        // 内置 LED 联动：靠近亮红 / 太暗亮白（状态变化时才刷灯）
        Led_Update(g_distance, g_als, g_alsValid, &g_ledState);
    }
}

// thread4：蓝牙接收（从零重写）—— 逐字节解析，避免粘包丢指令
//   查询指令：T=温湿度, L=光感, R=巡线
//   运动指令：O=停止, W=前进, S=后退, A=左转, D=右转, I=定速前进(100), K=定速前进(150)（避开 T/L/R 不冲突）

// 处理手机发来的一个字节指令（逐字节，粘包也不丢）
// 判断是否为合法指令字节（仅用于波特率锁定的判定）
// 注意：只认真正的指令字母 W/A/S/D/O/I/K/T/L/R（含大小写）。
//       【之前把 \r \n 空格 也算合法 → 波特率不匹配时模块吐出的乱码若恰好是
//       0x0D/0x0A/0x20 就会误锁在错误波特率上，导致之后永远收不到真指令。】
//       行结束符/空格在 BLE_HandleChar 里仍作为无操作字节正常忽略，这里只收紧锁定判据。
static int BLE_IsValidCmd(unsigned char c)
{
    switch (c) {
        case 'W': case 'w': case 'A': case 'a': case 'S': case 's':
        case 'D': case 'd': case 'O': case 'o': case 'I': case 'i':
        case 'K': case 'k': case 'T': case 't': case 'L': case 'l':
        case 'R': case 'r':
            return 1;
        default:
            return 0;
    }
}

static void BLE_HandleChar(unsigned char ch)
{
    char resp[64];
    int n;

    // [DEBUG] 收到任意字节都打到调试串口，用于定位「连上收不到」：
    //   插 USB 看调试串口，手机发 W 应出现 "BLE RX: 0x57"
    //   - 有这行但车不动 → 解析/执行问题（代码层）
    //   - 完全没有这行 → 模块波特率/接线/透传问题（硬件层，与代码无关）
    //   稳定后可把下面这行注释掉，避免刷屏。
    printf("BLE RX: 0x%02X\r\n", ch);

    switch (ch) {
        // ---- 查询指令 ----
        case 'T':
        case 't': {
            int ti = (int)(g_temp * 100.0f + (g_temp >= 0 ? 0.5f : -0.5f));
            int hi = (int)(g_humi * 100.0f + (g_humi >= 0 ? 0.5f : -0.5f));
            int ta = (ti < 0) ? -ti : ti;
            int ha = (hi < 0) ? -hi : hi;
            n = snprintf(resp, sizeof(resp), "T=%s%d.%02dC H=%s%d.%02d%%\r\n",
                         (ti < 0) ? "-" : "", ta / 100, ta % 100,
                         (hi < 0) ? "-" : "", ha / 100, ha % 100);
            if (n > 0) UartWrite(BLE_UART_IDX, (unsigned char *)resp, (unsigned int)n);
            break;
        }
        case 'L':
        case 'l':
            n = snprintf(resp, sizeof(resp), "L=%u P=%u I=%u\r\n", g_als, g_ps, g_ir);
            if (n > 0) UartWrite(BLE_UART_IDX, (unsigned char *)resp, (unsigned int)n);
            break;
        case 'R':
        case 'r':
            n = snprintf(resp, sizeof(resp), "TCRT=%s%s\r\n",
                         g_leftBlack ? "L" : "-", g_rightBlack ? "R" : "-");
            if (n > 0) UartWrite(BLE_UART_IDX, (unsigned char *)resp, (unsigned int)n);
            break;

        // ---- 运动指令（蓝牙遥控，封版） ----
        // O=停止  W=前进  S=后退  A=左转  D=右转  I=定速前进100  K=定速前进150
        case 'O':
        case 'o':
            car_stop();
            break;
        case 'W':
        case 'w':
            car_forward();
            break;
        case 'S':
        case 's':
            car_backward();
            break;
        case 'A':
        case 'a':
            car_left();
            break;
        case 'D':
        case 'd':
            car_right();
            break;
        case 'I':
        case 'i':
            stm32_motor_control(100, 100);   // 定速前进 100
            break;
        case 'K':
        case 'k':
            stm32_motor_control(150, 150);   // 定速前进 150（最快）
            break;

        // ---- 忽略回车换行等无意义字节 ----
        case '\r':
        case '\n':
        case 0:
            break;

        default:
            break;   // 未知指令忽略（不回显，避免手机端刷屏）
    }
}

static void BLE_RecvThread(void *arg)
{
    (void)arg;
    unsigned char buf[64];
    int noDataTicks = 0;   // 连续无数据次数（每次约 200ms，见下面 read timeout）

    while (1) {
        // 用带超时的读：无数据时 200ms 后返回 0。
        // 不用普通 UartRead —— 它会按 rx_block 配置阻塞，导致"超时切换波特率"逻辑卡死走不到。
        int len = hi_uart_read_timeout((hi_uart_idx)BLE_UART_IDX, buf, sizeof(buf), 200);

        if (len > 0) {
            // 只有收到"合法指令字节"才认为波特率对并锁定（乱码不锁定）
            int valid = 0;
            for (int i = 0; i < len; i++) {
                if (BLE_IsValidCmd(buf[i])) { valid = 1; break; }
            }
            if (valid && !g_bleBaudLocked) {
                g_bleBaudLocked = 1;
                printf("BLE baud LOCKED: %u (rx %d bytes)\r\n",
                       g_bleBaudCands[g_bleBaudIdx], len);
            }
            for (int i = 0; i < len; i++) {
                BLE_HandleChar(buf[i]);   // 逐字节解析，粘包也不丢指令
            }
            noDataTicks = 0;
            continue;
        }

        // 无数据：未锁定则每 3 秒切到下一个候选波特率（BLE_SetBaud 内部会重设引脚）
        // 【防泄漏】BLE_SetBaud 每次都会 UartInit 重新分配 RX 环形缓冲。若永远收不到有效数据，
        // 每 3s 调一次会持续泄漏内存。故：把所有候选各试一遍(约 21s)后仍无有效数据，
        // 就提交到最可能的 115200 并停止扫描，不再反复 UartInit。
        if (!g_bleBaudLocked) {
            if (++noDataTicks >= 15) {   // 15 × 200ms = 3s
                noDataTicks = 0;
                if (g_bleBaudTried < BLE_BAUD_CAND_NUM) {
                    g_bleBaudTried++;
                    g_bleBaudIdx = (g_bleBaudIdx + 1) % BLE_BAUD_CAND_NUM;
                    BLE_SetBaud(g_bleBaudCands[g_bleBaudIdx]);
                    printf("BLE baud try: %u\r\n", g_bleBaudCands[g_bleBaudIdx]);
                } else if (g_bleBaudTried == BLE_BAUD_CAND_NUM) {
                    g_bleBaudTried++;   // 仅提交一次
                    g_bleBaudIdx = 0;   // 提交到 115200（系统默认，最可能为模块真实波特率）
                    BLE_SetBaud(g_bleBaudCands[0]);
                    printf("BLE auto-baud done: commit 115200 (send any cmd to lock)\r\n");
                }
                // g_bleBaudTried > CAND_NUM：停止扫描，保持 115200 等待锁定
            }
        }
    }
}

// thread5：I2C 设备初始化（独立线程）—— I2C 若卡死只卡本线程，不影响蓝牙/电机/其他
static void I2c_InitThread(void *arg)
{
    (void)arg;

    SSD1306_Init();
    SSD1306_CLS();
    if (SHT20_Init() != 0) {
        printf("SHT20 init fail\r\n");
    }
    if (AP3216C_Init() != 0) {
        printf("AP3216C init fail\r\n");
    }
    SSD1306_ShowChineseStr(24, 0, (uint8_t *)"鸿蒙先锋号", 5);

    g_i2cReady = 1;   // 置位标志，Reader/Display 线程才真正开始读写 I2C
    printf("=== I2C READY ===\r\n");

    while (1) {
        osDelay(1000);   // 初始化完成后挂起，不再做事
    }
}

// 入口：只做不依赖 I2C 的初始化 + 创建所有线程；I2C 初始化交给独立线程（thread5）
/* ===================== WiFi + MQTT 远程控车 ===================== */
extern int WifiConnect(const char *ssid, const char *psk);
extern int GetStationIP(char *ipbuf, int buflen);

static MQTTClient g_mqttClient;
static Network   g_mqttNet;
static unsigned char g_mqttSendBuf[256];
static unsigned char g_mqttReadBuf[1024];

// MQTT 收到指令回调：payload 首字符即遥控指令（与蓝牙同一套），复用运动函数
static void MQTT_MessageArrived(MessageData *data)
{
    if (data == NULL || data->message == NULL) return;
    int len = (int)data->message->payloadlen;
    char *payload = (char *)data->message->payload;
    if (len <= 0 || payload == NULL) return;

    printf("MQTT RX on %.*s: %.*s\r\n",
           data->topicName->lenstring.len, data->topicName->lenstring.data,
           len, payload);

    char ch = payload[0];
    switch (ch) {
        case 'O': case 'o': car_stop();                    break;
        case 'W': case 'w': car_forward();                 break;
        case 'S': case 's': car_backward();               break;
        case 'A': case 'a': car_left();                    break;
        case 'D': case 'd': car_right();                   break;
        case 'I': case 'i': stm32_motor_control(100, 100); break;
        case 'K': case 'k': stm32_motor_control(150, 150); break;
        default: break;   // 其它字符忽略
    }
}

static void WifiMqttThread(void *arg)
{
    (void)arg;
    int rc;

    printf("=== WiFi+MQTT thread start ===\r\n");

    // 1) 连 WiFi（PSK 方式）
    if (WifiConnect(WIFI_SSID, WIFI_PASSWORD) != 0) {
        printf("!!! WifiConnect failed, MQTT disabled\r\n");
        return;
    }
    char ip[16] = {0};
    if (GetStationIP(ip, sizeof(ip)) == 0) {
        printf("WiFi connected, IP=%s\r\n", ip);
    } else {
        printf("WiFi connected, IP unknown\r\n");
    }

    // 2) 初始化 paho 网络与客户端
    NetworkInit(&g_mqttNet);
    MQTTClientInit(&g_mqttClient, &g_mqttNet, 30000,
                   g_mqttSendBuf, sizeof(g_mqttSendBuf),
                   g_mqttReadBuf, sizeof(g_mqttReadBuf));

    // 3) 连接 broker（用局部数组避免字符串字面量传 char* 告警）
    char broker_addr[64];
    strncpy(broker_addr, MQTT_BROKER_ADDR, sizeof(broker_addr) - 1);
    broker_addr[sizeof(broker_addr) - 1] = '\0';
    if ((rc = NetworkConnect(&g_mqttNet, broker_addr, MQTT_BROKER_PORT)) != 0) {
        printf("!!! MQTT NetworkConnect failed: %d\r\n", rc);
        return;
    }

    // 启动后台保活/收消息线程（MQTTLiteOS 实现；返回值 0=失败，但不阻塞后续）
    if (MQTTStartTask(&g_mqttClient) == 0) {
        printf("WARN MQTTStartTask failed\r\n");
    }

    MQTTPacket_connectData connectData = MQTTPacket_connectData_initializer;
    connectData.MQTTVersion = 4;   // 3.1.1
    char client_id[32];
    strncpy(client_id, MQTT_CLIENT_ID, sizeof(client_id) - 1);
    client_id[sizeof(client_id) - 1] = '\0';
    connectData.clientID.cstring = client_id;
    if ((rc = MQTTConnect(&g_mqttClient, &connectData)) != 0) {
        printf("!!! MQTTConnect failed: %d\r\n", rc);
        return;
    }
    printf("MQTT connected to %s:%d\r\n", broker_addr, (int)MQTT_BROKER_PORT);

    // 4) 订阅控制主题
    if ((rc = MQTTSubscribe(&g_mqttClient, MQTT_SUB_TOPIC, QOS1, MQTT_MessageArrived)) != 0) {
        printf("!!! MQTTSubscribe failed: %d\r\n", rc);
    } else {
        printf("MQTT subscribed: %s\r\n", MQTT_SUB_TOPIC);
    }

    // 5) 后台线程自动维持连接与接收；主循环可在此周期上报状态
    while (1) {
        osDelay(1000);
        // 如需上报：MQTTPublish(g_mqttClient, MQTT_PUB_TOPIC, &msg);
    }
}

static void I2c_Ssd1306_Demo(void)
{
    printf("=== APP INIT START ===\r\n");

    // 显式初始化 GPIO 模块：确保在调用任何 GpioSetFunc/GpioSetDir 之前 GPIO 子系统已就绪。
    // （此前 GpioInit 只在 SSD1306_Init 内调用，而本线程的 WS2812/HCSR04/TCRT/UART 初始化更早执行 GPIO 操作，
    //  属于隐式依赖系统 peripheral_init 已调过 GpioInit。这里显式调一次，消除该依赖；重复调用幂等。）
    GpioInit();

    g_i2cMutex = osMutexNew(NULL);
    printf("=== mutex done ===\r\n");

    // ① 纯 GPIO 外设（不依赖 I2C）
    WS2812_Init();   // 内置 WS2812 灯带（IO06）
    HCSR04_Init();   // 超声波 HC-SR04（IO07=TRIG, IO08=ECHO）
    TCRT_Init();     // 车底检测 CGQ1/CGQ2（IO13=左，IO14=右，T/R 查询与日志用）
    printf("=== GPIO init done ===\r\n");

    // ② UART 外设
    BLE_UartInit();      // UART1 蓝牙（IO00/01）
    Motor_UartInit();    // UART2 与 STM32 通信（IO11=TXD, IO12=RXD）
    car_stop();          // 上电先发一帧停止指令，避免 STM32 残留状态导致乱跑
    printf("=== UART init done ===\r\n");

    // ③ 创建信号量与全部线程
    g_semRead = osSemaphoreNew(1, 0, NULL);
    g_semDisp = osSemaphoreNew(1, 0, NULL);

    osThreadAttr_t attr = {0};
    attr.stack_size = 1024 * 4;
    attr.priority = osPriorityNormal;

    // Hi3861 的 LiteOS-M 对任务数有上限(默认 LOSCFG_BASE_CORE_TSK_LIMIT=64),
    // 若创建失败 osThreadNew 返回 NULL 且静默不跑, 这里逐一校验以免「功能莫名失效」。
    osThreadId_t tid;
#define SAFE_NEW(tname, fn) do { \
        attr.name = tname; \
        tid = osThreadNew((osThreadFunc_t)(fn), NULL, &attr); \
        if (tid == NULL) printf("!!! THREAD CREATE FAILED: %s (task limit or stack OOM?)\r\n", tname); \
    } while (0)

    SAFE_NEW("thread1", Sensor_ProducerThread);
    SAFE_NEW("thread2", Sensor_ReaderThread);
    SAFE_NEW("thread3", Sensor_DisplayThread);
    SAFE_NEW("thread4", BLE_RecvThread);
    SAFE_NEW("thread5", I2c_InitThread);   // I2C 初始化独立线程
    SAFE_NEW("thread6", WifiMqttThread);    // WiFi+MQTT 远程控车

    printf("=== ALL THREADS STARTED ===\r\n");
}

// 用 SYS_RUN 而非 APP_FEATURE_INIT：APP_FEATURE_INIT 依赖 SAMGR 运行时动态扫描，
// 精简固件里可能不被调用导致任务全不起；SYS_RUN 注册进 .zinitcall.run2.init，
// 内核完成基础硬件初始化后由 main() 无条件直接调用，确保任务 100% 随开机直启。
SYS_RUN(I2c_Ssd1306_Demo);
