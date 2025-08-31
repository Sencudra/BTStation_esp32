#pragma once

// коды ошибок в списке последних ошибок
#define CLOCK_ERROR				10 // clock chip failure
#define POWER_UNDERVOLTAGE		11 // battery low

//коды ошибок STARTUP
#define STARTUP_NUMBER          50 // incorrect station number in EEPROM
#define STARTUP_MODE            51 // incorrect station mode in EEPROM
#define STARTUP_VKOEFF          52 // incorrect Vkoeff in EEPROM
#define STARTUP_GAIN            53 // incorrect gain in EEPROM
#define STARTUP_CHIP_TYPE       54 // incorrect chip type in EEPROM
#define STARTUP_TEAM_SIZE       55 // incorrect team block size in EEPROM
#define STARTUP_ERASE_SIZE      56 // incorrect erase block size in EEPROM
#define STARTUP_BATTERY_LIMIT   57 // incorrect battery limit in EEPROM
#define STARTUP_AUTOREPORT      58 // incorrect autoreport mode in EEPROM
#define STARTUP_RFID			59 // RFID initialization failed
#define STARTUP_AUTH			90 // incorrect auth mode in EEPROM
#define STARTUP_FFAT			91 // can't init FFAT
#define STARTUP_SETTINGS		92 // can't init settings

// коды ошибок UART
#define UART_TIMEOUT            60 // receive timeout
#define UART_PACKET_LENGTH      61 // incorrect packet length
#define UART_CRC                62 // CRC incorrect
#define UART_UNEXPECTED_BYTE    63 // unexpected byte
#define UART_WRONG_STATION      64 // incorrect station number

// коды ошибок обработки чипа CARD PROCESSING
#define PROCESS_READ_CHIP       70 // error reading chip
#define PROCESS_HW_CHIP_TYPE    71 // wrong hw chip type
#define PROCESS_SW_CHIP_TYPE    72 // wrong sw chip type
#define PROCESS_FW_VERSION      73 // wrong fw. version
#define PROCESS_INIT_TIME       74 // chip init time is due
#define PROCESS_CHIP_NUMBER     75 // wrong chip number
#define PROCESS_WRITE_CHIP      76 // error writing to chip
#define PROCESS_ALREADY_CHECKED 77 // chip already checked
#define PROCESS_FIND_FREE_PAGE  78 // error finding free page
#define PROCESS_SAVE_DUMP       79 // error saving dump
#define PROCESS_SEND_AUTOREPORT 80 // error sending autoreport
