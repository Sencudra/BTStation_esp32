#pragma once
// номер станции
#define EEPROM_STATION_NUMBER   "ST#" // int

// Bluetooth имя
#define EEPROM_STATION_NAME   "STNAME" // string

// Bluetooth pin
#define EEPROM_STATION_PIN   "STPIN" // int

// номер режима работы станции
#define EEPROM_STATION_MODE     "STMODE#" // int

// коэфф. пересчета значения ADC в вольты = 0,00587
#define EEPROM_VOLTAGE_KOEFF    "VKOEF" // float

// усиление сигнала RFID
#define EEPROM_GAIN             "GAIN" // int

// тип чипа, с которым должна работать станция
#define EEPROM_CHIP_TYPE        "CHTYPE" // int

// размер блока на флэше под данные команды
#define EEPROM_TEAM_BLOCK_SIZE  "TBSIZE" // int

// минимальное напряжение батареи
#define EEPROM_BATTERY_LIMIT    "BATLIM" // float

// включить автоотчет о новых сканах
#define EEPROM_AUTOREPORT       "AREPORT" // bool

// включить авторизацию записи на карту
#define EEPROM_AUTH             "AUTH" // bool

// Ключ авторизации
#define EEPROM_AUTH_PWD         "AUTHPWD" // byte[4]

// Ключ авторизации
#define EEPROM_AUTH_PACK        "AUTHPACK" // byte[2]
