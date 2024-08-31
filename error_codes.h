#pragma once
// коды ошибок в списке последних ошибок
//коды ошибок STARTUP
#define STARTUP_NUMBER          50 //STARTUP: incorrect station number in EEPROM
#define STARTUP_MODE            51 //STARTUP: incorrect station mode in EEPROM
#define STARTUP_VKOEFF          52 //STARTUP: incorrect Vkoeff in EEPROM
#define STARTUP_GAIN            53 //STARTUP: incorrect gain in EEPROM
#define STARTUP_CHIP_TYPE       54 //STARTUP: incorrect chip type in EEPROM
#define STARTUP_TEAM_SIZE       55 //STARTUP: incorrect team block size in EEPROM
#define STARTUP_ERASE_SIZE      56 //STARTUP: incorrect erase block size in EEPROM
#define STARTUP_BATTERY_LIMIT   57 //STARTUP: incorrect battery limit in EEPROM
#define STARTUP_AUTOREPORT      58 //STARTUP: incorrect autoreport mode in EEPROM
//коды ошибок UART
#define UART_TIMEOUT            60 //UART: receive timeout
#define UART_PACKET_LENGTH      61 //UART: incorrect packet length
#define UART_CRC                62 //UART: CRC incorrect
#define UART_UNEXPECTED_BYTE    63 //UART: unexpected byte
#define UART_WRONG_STATION      64 //UART: incorrect station number
//коды ошибок обработки чипа
#define PROCESS_READ_CHIP       70 //CARD PROCESSING: error reading chip
#define PROCESS_HW_CHIP_TYPE    71 //CARD PROCESSING: wrong hw chip type
#define PROCESS_SW_CHIP_TYPE    72 //CARD PROCESSING: wrong sw chip type
#define PROCESS_FW_VERSION      73 //CARD PROCESSING: wrong fw. version
#define PROCESS_INIT_TIME       74 //CARD PROCESSING: chip init time is due
#define PROCESS_CHIP_NUMBER     75 //CARD PROCESSING: wrong chip number
#define PROCESS_WRITE_CHIP      76 //CARD PROCESSING: error writing to chip
#define PROCESS_ALREADY_CHECKED 77 //CARD PROCESSING: chip already checked
#define PROCESS_FIND_FREE_PAGE  78 //CARD PROCESSING: error finding free page
#define PROCESS_SAVE_DUMP       79 //CARD PROCESSING: error saving dump
#define PROCESS_SEND_AUTOREPORT 80 //CARD PROCESSING: error sending autoreport
