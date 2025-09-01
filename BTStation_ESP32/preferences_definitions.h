#pragma once

#define PREFERENCE_NAME			"BTStation" // ID Preferences
#define EEPROM_STATION_NUMBER   "ST#" // int: номер станции
#define EEPROM_STATION_NAME     "STNAME" // string: Bluetooth имя
#define EEPROM_STATION_MODE     "STMODE#" // int: номер режима работы станции
#define EEPROM_VOLTAGE_KOEFF    "VKOEF" // float: коэфф. пересчета значения ADC в вольты = 0.00587
#define EEPROM_GAIN             "GAIN" // int: усиление сигнала RFID
#define EEPROM_CHIP_TYPE        "CHTYPE" // int: тип чипа, с которым должна работать станция

constexpr uint16_t EEPROM_TEAM_BLOCK_SIZE_DEFAULT = 1024; // размер блока на флэше под данные команды по умолчанию
#define EEPROM_TEAM_BLOCK_SIZE  "TBSIZE" // int: размер блока на флэше под данные команды

#define EEPROM_BATTERY_LIMIT    "BATLIM" // float: минимальное напряжение батареи
#define EEPROM_AUTOREPORT       "AREPORT" // bool: включить автоотчет о новых сканах
#define EEPROM_AUTH             "AUTH" // bool: включить авторизацию записи на карту
#define EEPROM_AUTH_PWD         "AUTHPWD" // byte[4]: Ключ авторизации
#define EEPROM_AUTH_PACK        "AUTHPACK" // byte[2]: Ключ авторизацииs