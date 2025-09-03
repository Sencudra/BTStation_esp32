#pragma once

#define FW_VERSION		        110     // версия прошивки, номер пишется в чипы
#define UART_SPEED		        115200  // скорость UART
#define BT_CONFIRM_PIN		    00      // кнопка подтверждения подключения к Bluetooth

#define RED_LED_PIN		        16	    // светодиод ошибки (красный)
#define GREEN_LED_PIN		    17	    // светодиод синий

#define BUZZER_PIN		        25	    // пищалка

#if defined(USE_PN532)
    #define PN532_IRQ		    33	    // прерывание PN532
    #define PN532_RESET		    32	    // сброс PN532
#else
    #define RFID_RST_PIN		13	    // рфид модуль reset
    #define RFID_SS_PIN		    SS	    // рфид модуль chip_select
    // #define RFID_MOSI_PIN	MOSI	// рфид модуль MOSI
    //#define RFID_MISO_PIN	    MISO	// рфид модуль MISO
    //#define RFID_SCK_PIN	    SCK	    // рфид модуль SCK
#endif

#define BATTERY_PIN		        27	// замер напряжения батареи
#define BATTERY_ALARM_COUNT	    100	// сколько подряд замеров ниже нормы приводят к срабатвыванию тревоги

// #define RTC_ENABLE_PIN	        5	// питание часов
#define RTC_ALARM_DELAY		    10000	// задержка между проверкой хода часов

#define RECEIVE_TIMEOUT		    1000 // тайм-аут приема команды с момента начала
#define RFID_READ_PERIOD		1000 // периодичность поиска чипа
#define LAST_TEAMS_LENGTH		30 // размер буфера последних команд
#define LAST_ERRORS_LENGTH		30 // размер буфера последних ошибок
#define MAX_PAKET_LENGTH		1200 // максимальный размер пакета данных для передачи по UART/Bluetooth
