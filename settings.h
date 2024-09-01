#pragma once
// версия прошивки, номер пишется в чипы
#define FW_VERSION			110

#define UART_SPEED			115200

#define RED_LED_PIN			16 // светодиод ошибки (красный)
#define GREEN_LED_PIN		17 // светодиод синий
#define BUZZER_PIN			14 // пищалка

#define RFID_RST_PIN		13 // рфид модуль reset
#define RFID_SS_PIN			SS // рфид модуль chip_select
//#define RFID_MOSI_PIN		MOSI // рфид модуль
//#define RFID_MISO_PIN		MISO // рфид модуль
//#define RFID_SCK_PIN		SCK // рфид модуль

#define USE_PN532			1
#define PN532_IRQ			32
#define PN532_RESET			33

#define BATTERY_PIN			04 // замер напряжения батареи

//#define RTC_ENABLE_PIN	5 // питание часов

// тайм-аут приема команды с момента начала
#define RECEIVE_TIMEOUT		1000

// периодичность поиска чипа
#define RFID_READ_PERIOD	1000

// размер буфера последних команд
#define LAST_TEAMS_LENGTH	30

// размер буфера последних ошибок
#define LAST_ERRORS_LENGTH	30

// максимальный размер пакета данных для передачи по UART/Bluetooth
#define MAX_PAKET_LENGTH	1200
