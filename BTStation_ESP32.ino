//#define DEBUG
//#define BENCHMARK
#include "FS.h"
#include "FFat.h"
#include <Wire.h>
#include "ds3231.h"
#include "BluetoothSerial.h"
#include "command_definitions.h"
#include "card_definitions.h"
#include "error_codes.h"
#include "settings.h"
#include "protocol_definitions.h"
#include "preferences_definitions.h"
#include <Preferences.h>

#if defined(USE_PN532)
#include "Adafruit_PN532.h"
#else
#include <SPI.h>
#include <MFRC522.h>
#endif

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif
BluetoothSerial SerialBT;

Preferences preferences;

#if defined(USE_PN532)
Adafruit_PN532 pn532(PN532_IRQ, PN532_RESET);
#else
MFRC522 mfrc522(RFID_SS_PIN, RFID_RST_PIN); // рфид-модуль
#endif

uint32_t rfidReadStartTime = 0;

uint8_t lastTeams[LAST_TEAMS_LENGTH * 2]; // последние отмеченные команды
uint8_t lastErrors[LAST_ERRORS_LENGTH]; // последние коды ошибок станции
uint8_t ntag_page[16]; // буфер для чтения из чипа через ntagRead4pages()
uint32_t lastTimeChecked = 0; // время последней отметки чипа
uint16_t totalChipsChecked = 0; // количество отмеченных чипов

uint8_t newTeamMask[8]; // новая маска для замены в чипе
uint16_t lastTeamFlag = 0; // номер последней отмеченной команды для отсечки двойной отметки

const String teamFilePrefix = "/team";

uint8_t stationNumber = 0; // номер станции по умолчанию
uint8_t stationMode = MODE_INIT; // режим станции по умолчанию
bool scanAutoreport = false; // автоматически отправлять данные сканирования в UART порт
uint8_t chipType = NTAG215_ID; // тип чипа
uint8_t ntagMark = NTAG215_MARK; // метка в страницу 3 чипа
uint8_t tagMaxPage = NTAG215_MAX_PAGE; // размер чипа в страницах
uint16_t teamFlashSize = 1024;
int maxTeamNumber = 10; // максимальное кол-во записей в логе = (flashSize - flashBlockSize) / teamFlashSize - 1;
const uint32_t maxTimeInit = 7UL * 24UL * 60UL * 60UL; // максимальный срок годности чипа [секунд] - дата инициализации одна неделя назад от текущего момента
float voltageCoeff = 0.00578; // коэфф. перевода значения АЦП в напряжение для делителя 10кОм/2.2кОм
float batteryLimit = 0; // минимальное напряжение батареи
uint8_t gainCoeff = 96; // коэфф. усиления антенны - работают только биты 4,5,6; значения [0, 16, 32, 48, 64, 80, 96, 112]
String BTName = "";
String BTPin = "";

uint8_t uartBuffer[MAX_PAKET_LENGTH]; // UART command buffer
uint16_t uartBufferPosition = 0;
bool uartReady = false; // получен пакет
bool receivingData = false; // идет прием пакета
uint32_t receiveStartTime = 0; // время начала получения пакета

uint16_t batteryLevel = 500; // текущий уровень напряжения (замер АЦП)
uint8_t batteryAlarmCount = 0; // счетчик нарушений границы допустимого напряжения

void RfidStart();
void RfidEnd();
bool RfidFindChip();
void processRfidCard();
bool readUart(Stream& SerialPort);
void executeCommand();
void setMode();
void setTime();
void resetStation();
void getStatus();
void initChip();
void getLastTeams();
void getTeamRecord();
void readCardPages();
void updateTeamMask();
void writeCardPage();
void readFlash();
void writeFlash();
void eraseTeamFlash();
void getConfig();
void setVCoeff();
void setGain();
void setChipType();
void setTeamFlashSize();
void setFlashBlockSize();
void setBtName();
void setBtPinCode();
void setBatteryLimit();
void scanTeams();
void getLastErrors();
void setAutoReport();
void saveNewMask();
void clearNewMask();
uint16_t getBatteryLevel();
void beep(uint8_t, uint16_t);
void errorBeepMs(uint8_t, uint16_t);
void errorBeep(uint8_t);
void init_package(uint8_t);
bool addData(uint8_t);
void sendData();
bool ntagWritePage(uint8_t*, uint8_t, bool verify);
bool ntagRead4pages(uint8_t);
bool writeCheckPointToCard(uint8_t, uint32_t);
int findNewPage();
bool checkTeamExists(uint16_t teamNumber);
bool writeDumpToFlash(uint16_t, uint32_t, uint32_t, uint16_t);
bool eraseTeamFromFlash(uint16_t);
bool readTeamFromFlash(uint16_t);
uint16_t refreshChipCounter();
void sendError(uint8_t, uint8_t);
void sendError(uint8_t);
void addLastTeam(uint16_t, bool);
void addLastError(uint8_t);
uint8_t crcCalc(uint8_t*, uint16_t, uint16_t);
void floatToByte(uint8_t*, float);
bool selectChipType(uint8_t);

void setup()
{
	Serial.begin(UART_SPEED);

	pinMode(GREEN_LED_PIN, OUTPUT);
	pinMode(RED_LED_PIN, OUTPUT);
	pinMode(BUZZER_PIN, OUTPUT);

	digitalWrite(GREEN_LED_PIN, LOW);
	digitalWrite(RED_LED_PIN, LOW);
	digitalWrite(BUZZER_PIN, LOW);

	Wire.begin();
	DS3231_init(DS3231_INTCN);

#if defined(USE_PN532)
	if (!pn532.begin())
	{
#ifdef DEBUG
		Serial.println(F("!!! PN532 failed"));
#endif
		errorBeepMs(4, 200);
		addLastError(STARTUP_RFID);
	}
#else
	SPI.begin();
	byte s = mfrc522.PCD_ReadRegister(MFRC522::PCD_Register::VersionReg);
	if (s == 0 || s == 0xff)
	{
#ifdef DEBUG
		Serial.println(F("!!! MFRC522 failed"));
#endif
		errorBeepMs(4, 200);
		addLastError(STARTUP_RFID);
	}
	SPI.end();
#endif

	if (!FFat.begin())
	{
#ifdef DEBUG
		Serial.println(F("!!! FFat failed"));
#endif
		errorBeepMs(4, 200);
		addLastError(PROCESS_SAVE_DUMP);
	}

	// read settings
	if (!preferences.begin("BTStation", false))
	{
#ifdef DEBUG
		Serial.println("Preferences mount failed");
		errorBeepMs(4, 200);
		addLastError(PROCESS_SAVE_DUMP);
#endif
	}

	//читаем номер станции из eeprom
	uint32_t c = preferences.getUInt(EEPROM_STATION_NUMBER, 0xff);
	if (c == 0xff)
	{
		stationNumber = 0;
#ifdef DEBUG
		Serial.println(F("!!! StationNumber"));
#endif
		errorBeepMs(4, 200);
		addLastError(STARTUP_NUMBER);
	}
	else stationNumber = c;

	//читаем номер режима из eeprom
	c = preferences.getUInt(EEPROM_STATION_MODE, 0xff);
	if (c == MODE_INIT)
		stationMode = MODE_INIT;
	else if (c == MODE_START_KP)
		stationMode = MODE_START_KP;
	else if (c == MODE_FINISH_KP)
		stationMode = MODE_FINISH_KP;
	else
	{
		stationMode = MODE_INIT;
#ifdef DEBUG
		Serial.println(F("!!! StationMode"));
#endif
		errorBeepMs(4, 200);
		addLastError(STARTUP_MODE);
	}

	//читаем коэфф. пересчета напряжения
	voltageCoeff = preferences.getFloat(EEPROM_VOLTAGE_KOEFF, 0);
	if (voltageCoeff == 0)
	{
		voltageCoeff = 0.1;
#ifdef DEBUG
		Serial.println(F("!!! VoltageCoeff"));
#endif
		errorBeepMs(4, 200);
		addLastError(STARTUP_GAIN); //STARTUP: incorrect gain in EEPROM
	}

	//читаем коэфф. усиления
	c = preferences.getUInt(EEPROM_GAIN, 0xff);
	if (c != 0xff)
		gainCoeff = c;
	else
	{
		gainCoeff = 96;
#ifdef DEBUG
		Serial.println(F("!!! AntennaGain"));
#endif
		errorBeepMs(4, 200);
		addLastError(STARTUP_GAIN); //STARTUP: incorrect gain in EEPROM
	}

	//читаем тип чипа
	c = preferences.getUInt(EEPROM_CHIP_TYPE, 0xff);
	if (c != 0xff)
		selectChipType(c);
	else
	{
		selectChipType(NTAG215_ID);
#ifdef DEBUG
		Serial.println(F("!!! ChipType"));
#endif
		errorBeepMs(4, 200);
		addLastError(STARTUP_CHIP_TYPE);
	}

	//читаем размер блока команды
	teamFlashSize = preferences.getUInt(EEPROM_TEAM_BLOCK_SIZE, 0xff);
	if (teamFlashSize <= 0)
	{
		teamFlashSize = 1024;
#ifdef DEBUG
		Serial.println(F("!!! TeamSize"));
#endif
		errorBeepMs(4, 200);
		addLastError(STARTUP_TEAM_SIZE);
	}

	//читаем минимальное напряжение батареи
	batteryLimit = preferences.getFloat(EEPROM_BATTERY_LIMIT, -1);
	if (batteryLimit <= 0)
	{
		batteryLimit = 0;
#ifdef DEBUG
		Serial.println(F("!!! BatteryLimit"));
#endif
		errorBeepMs(4, 200);
		addLastError(STARTUP_BATTERY_LIMIT);
	}

	//читаем режим автооповещения из памяти
	c = preferences.getUInt(EEPROM_AUTOREPORT, 0xff);
	if (c == AUTOREPORT_ON)
		scanAutoreport = true;
	else if (c == AUTOREPORT_OFF)
	{
		scanAutoreport = false;
	}
	else
	{
		scanAutoreport = false;
#ifdef DEBUG
		Serial.println(F("!!! AutoReport"));
#endif
		errorBeepMs(4, 200);
		addLastError(STARTUP_AUTOREPORT);
	}

	//читаем Bluetooth имя из памяти
	String btName = preferences.getString(EEPROM_STATION_NAME, String("BtStation-?"));
	if (!btName || btName.length() <= 0)
	{
#ifdef DEBUG
		Serial.println(F("!!! BT Name"));
#endif
		btName = String("BtStation-") + String(stationNumber);
	}
	SerialBT.begin(btName.c_str()); //Bluetooth device name

	//читаем Bluetooth пин из памяти
	btName = preferences.getString(EEPROM_STATION_PIN, String(""));
	if (btName && btName.length() > 0)
	{
#ifdef DEBUG
		Serial.println(F("Setting BT Pin: "));
		Serial.println(btName);
#endif
		SerialBT.setPin(btName.c_str());
	}
	else
	{
#ifdef DEBUG
		Serial.println(F("!!! BT Pin"));
#endif
	}

	const uint32_t flashSize = FFat.freeBytes();
	maxTeamNumber = (flashSize / teamFlashSize) - 1;
	totalChipsChecked = refreshChipCounter();
	batteryLevel = analogRead(BATTERY_PIN);
	beep(1, 800);
}

void loop()
{
	//если режим КП то отметить чип автоматом
	if (stationMode != MODE_INIT && millis() >= rfidReadStartTime)
	{
		processRfidCard();
		rfidReadStartTime = millis() + RFID_READ_PERIOD;
	}

	// check UART for data
	if (Serial.available())
	{
#ifdef DEBUG
		Serial.println(F("!!! UART available"));
#endif
		uartReady = readUart(Serial);
	}

	// check Bluetooth for data
	if (SerialBT.available())
	{
#ifdef DEBUG
		Serial.println(F("!!! UART available"));
#endif

		uartReady = readUart(SerialBT);
	}

	//обработать пришедшую команду
	if (uartReady)
	{
		uartReady = false;
		executeCommand();
	}

	// check receive timeout
	if (receivingData && millis() - receiveStartTime > RECEIVE_TIMEOUT)
	{
#ifdef DEBUG
		//Serial.println(F("!!!receive timeout"));
#endif
		uartBufferPosition = 0;
		uartReady = false;
		receivingData = false;
		//errorBeepMs(1, 50);
		addLastError(UART_TIMEOUT);
	}

	/*batteryLevel = (batteryLevel + getBatteryLevel()) / 2;
	if ((float)((float)batteryLevel * voltageCoeff) <= batteryLimit)
	{
		if (batteryAlarmCount > 100)
		{
			digitalWrite(RED_LED_PIN, HIGH);
			//tone(BUZZER_PIN, 50, 50);
			delay(50);
			digitalWrite(RED_LED_PIN, LOW);
		}
		else batteryAlarmCount++;
	}
	else batteryAlarmCount = 0;*/
}

void RfidStart()
{
#if defined(USE_PN532)
#else
	// включаем SPI ищем чип вблизи. Если не находим выходим из функции чтения чипов
	SPI.begin();      // Init SPI bus
	mfrc522.PCD_Init();    // Init MFRC522
	mfrc522.PCD_SetAntennaGain(gainCoeff);
	delay(1);
#endif
}

void RfidEnd()
{
#if defined(USE_PN532)
	pn532.powerDownMode();
#else
	mfrc522.PCD_AntennaOff();
	SPI.end();
#endif
}

bool RfidFindChip()
{
#if defined(USE_PN532)
	uint8_t uid[8] = { 0 };	// Buffer to store the returned UID
	uint8_t uidLength;		// Length of the UID (4 or 7 bytes depending on ISO14443A card type)
	bool result = pn532.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 1000);
#ifdef DEBUG
	if (result)
	{
		Serial.println(F("!!!chip found"));
		Serial.print(F("UIDlength: "));
		Serial.println(String(uidLength));
	}
	else
		Serial.println(F("!!!chip not found"));
#endif

	return result;
#else
	// Look for new cards
	if (!mfrc522.PICC_IsNewCardPresent())
	{
#ifdef DEBUG
		Serial.println(F("!!!chip not found"));
#endif
		return false;
	}
#ifdef DEBUG
	Serial.println(F("!!!chip found"));
#endif

	// Select one of the cards
	if (!mfrc522.PICC_ReadCardSerial())
	{
#ifdef DEBUG
		Serial.println(F("!!!fail to select chip"));
#endif
		return false;
	}
#ifdef DEBUG
	Serial.println(F("!!!chip selected"));
#endif
	return true;
#endif
}

// Обработка поднесенного чипа
void processRfidCard()
{
	if (stationNumber == 0 || stationNumber == 0xff)
		return;

#ifdef BENCHMARK
	const unsigned long startCheck = millis();
#endif

	struct ts checkTime;
	DS3231_get(&checkTime);

	// включаем SPI ищем чип вблизи. Если не находим выходим из функции чтения чипов
	RfidStart();
#ifdef DEBUG
	Serial.println(F("!!!search chip"));
#endif

	if (!RfidFindChip())
	{
		RfidEnd();
		lastTeamFlag = 0;
#ifdef DEBUG
		Serial.println(F("!!!fail to find chip"));
#endif
		return;
	}

	// читаем блок информации
	if (!ntagRead4pages(PAGE_CHIP_SYS))
	{
		RfidEnd();
#ifdef DEBUG
		Serial.println(F("!!!fail to read chip"));
#endif
		//errorBeep(1);
		addLastError(PROCESS_READ_CHIP);
		return;
	}

#ifdef DEBUG
	Serial.println(F("!!!chip page read"));
#endif

	//неправильный тип чипа
	if (ntag_page[2] != chipType)
	{
		RfidEnd();
#ifdef DEBUG
		Serial.println(F("!!!incorrect hw chip type"));
#endif
		errorBeep(1);
		addLastError(PROCESS_HW_CHIP_TYPE);
		return;
	}

#ifdef DEBUG
	Serial.println(F("!!!chip type correct"));
#endif

	/*
	Фильтруем
	1 - неправильный тип чипа
	2 - чип от другой прошивки
	3 - чип более недельной давности инициализации
	4 - чипы с командой №0 или >maxTeamNumber
	5 - чип, который совпадает с уже отмеченным (в lastTeams[])
	*/

	// неправильный тип чипа
	/*if (ntag_page[6] != NTAG_MARK)
	{
		RfidEnd();
#ifdef DEBUG
		Serial.println(F("!!!incorrect sw chip type"));
#endif
		errorBeep(2);
		addLastError(PROCESS_SW_CHIP_TYPE);
		return;
	}*/

	// чип от другой прошивки
	if (ntag_page[7] != FW_VERSION)
	{
		RfidEnd();
#ifdef DEBUG
		Serial.println(F("!!!incorrect fw ver."));
#endif
		errorBeep(4);
		addLastError(PROCESS_FW_VERSION);
		return;
	}

#ifdef DEBUG
	Serial.println(F("!!!chip fw version correct"));
#endif

	// Не слишком ли старый чип? Недельной давности и более
	uint32_t initTime = ntag_page[8];
	initTime = initTime << 8;
	initTime += ntag_page[9];
	initTime = initTime << 8;
	initTime += ntag_page[10];
	initTime = initTime << 8;
	initTime += ntag_page[11];
	if ((checkTime.unixtime - initTime) > maxTimeInit)
	{
		RfidEnd();
#ifdef DEBUG
		Serial.println(F("!!!outdated chip"));
#endif
		errorBeep(4);
		addLastError(PROCESS_INIT_TIME); //CARD PROCESSING: chip init time is due
		return;
	}

#ifdef DEBUG
	Serial.println(F("!!!chip time correct"));
#endif

	// Если номер чипа =0 или >maxTeamNumber
	uint16_t teamNumber = (ntag_page[4] << 8) + ntag_page[5];
	if (teamNumber < 1 || teamNumber > maxTeamNumber)
	{
		RfidEnd();
#ifdef DEBUG
		Serial.print(F("!!!incorrect chip #: "));
		Serial.println(String(teamNumber));
#endif
		errorBeep(4);
		addLastError(PROCESS_CHIP_NUMBER);
		return;
	}

#ifdef DEBUG
	Serial.println(F("!!!chip team correct"));
#endif

	// не надо ли обновить у чипа маску?
	// 0-1: номер команды
	// 2-5: время выдачи чипа
	// 6-7: маска участников
	if (newTeamMask[0] + newTeamMask[1] != 0
		&& ntag_page[4] == newTeamMask[0]
		&& ntag_page[5] == newTeamMask[1]
		&& ntag_page[8] == newTeamMask[2]
		&& ntag_page[9] == newTeamMask[3]
		&& ntag_page[10] == newTeamMask[4]
		&& ntag_page[11] == newTeamMask[5])
	{
#ifdef DEBUG
		Serial.print(F("!!!updating mask"));
#endif
		if (ntag_page[12] != newTeamMask[6] || ntag_page[13] != newTeamMask[7])
		{
			digitalWrite(GREEN_LED_PIN, HIGH);
			uint8_t dataBlock[4] = { newTeamMask[6], newTeamMask[7], ntag_page[14], ntag_page[15] };
			if (!ntagWritePage(dataBlock, PAGE_TEAM_MASK, false))
			{
				RfidEnd();
#ifdef DEBUG
				Serial.print(F("!!!failed to write chip"));
#endif
				digitalWrite(GREEN_LED_PIN, LOW);
				errorBeep(1);
				addLastError(PROCESS_WRITE_CHIP); //CARD PROCESSING: error writing to chip
				return;
			}
		}

		RfidEnd();
		clearNewMask();
		lastTeamFlag = teamNumber;
		digitalWrite(GREEN_LED_PIN, LOW);
#ifdef DEBUG
		Serial.print(F("!!!mask updated"));
#endif
		return;
	}

	uint16_t mask = (ntag_page[12] << 8) + ntag_page[13];

	// Если это повторная отметка
	if (teamNumber == lastTeamFlag)
	{
		RfidEnd();
#ifdef DEBUG
		Serial.print(F("!!!same chip attached"));
#endif
		return;
	}

	bool already_checked = false;
	// сравнить с буфером последних команд
	if (stationMode == MODE_START_KP)
	{
		for (uint8_t i = 0; i < LAST_TEAMS_LENGTH * 2; i = i + 2)
		{
			if (lastTeams[i] == ntag_page[0] && lastTeams[i + 1] == ntag_page[1])
			{
				already_checked = true;
#ifdef DEBUG
				Serial.println(F("!!!chip already checked"));
#endif
				break;
			}
		}
	}

	// Есть ли чип на флэше
	if (!already_checked && checkTeamExists(teamNumber))
	{
		already_checked = true;
#ifdef DEBUG
		Serial.println(F("!!!chip already checked"));
#endif
	}

	// если известный чип и стартовый КП
	if (already_checked && stationMode == MODE_START_KP)
	{
		RfidEnd();
#ifdef DEBUG
		Serial.println(F("!!!Can't read chip"));
#endif
		//digitalWrite(GREEN_LED_PIN, LOW);
		errorBeep(1);
		addLastError(PROCESS_ALREADY_CHECKED);
		lastTeamFlag = teamNumber;
		return;
	}

#ifdef DEBUG
	Serial.println(F("!!!searching free page"));
#endif

	// ищем свободную страницу на чипе
	int newPage = findNewPage();

	// ошибка чтения чипа
	if (newPage == 0)
	{
		RfidEnd();
		//digitalWrite(GREEN_LED_PIN, LOW);
		errorBeep(1);
		addLastError(PROCESS_READ_CHIP);
#ifdef DEBUG
		Serial.println(F("!!!Can't read chip"));
#endif
		return;
	}

	// больше/меньше нормы... Наверное, переполнен???
	if (newPage != -1 && (newPage < PAGE_DATA_START || newPage >= tagMaxPage))
	{
		RfidEnd();
		//digitalWrite(GREEN_LED_PIN, LOW);
		errorBeep(4);
		addLastError(PROCESS_FIND_FREE_PAGE);
#ifdef DEBUG
		Serial.print(F("!!!chip page# incorrect: "));
		Serial.println(String(newPage));
#endif
		return;
	}

	// chip was checked by another station with the same number
	if (newPage == -1)
	{
#ifdef DEBUG
		Serial.print(F("!!!chip marked by another station"));
#endif
		lastTeamFlag = teamNumber;
		//digitalWrite(GREEN_LED_PIN, LOW);
		return;
	}

#ifdef DEBUG
	Serial.println(F("!!!writing to chip"));
#endif
	// Пишем на чип отметку
	digitalWrite(GREEN_LED_PIN, HIGH);
	if (!writeCheckPointToCard(newPage, checkTime.unixtime))
	{
		RfidEnd();
		digitalWrite(GREEN_LED_PIN, LOW);
		errorBeep(1);
		addLastError(PROCESS_WRITE_CHIP); //CARD PROCESSING: error writing chip
#ifdef DEBUG
		Serial.print(F("!!!failed to write chip"));
#endif
		return;
	}
	// Пишем дамп чипа во флэш
	if (!writeDumpToFlash(teamNumber, checkTime.unixtime, initTime, mask))
	{
		RfidEnd();
		digitalWrite(GREEN_LED_PIN, LOW);
		errorBeep(2);
		addLastError(PROCESS_SAVE_DUMP); //CARD PROCESSING: error saving dump
#ifdef DEBUG
		Serial.print(F("!!!failed to write dump"));
#endif
		return;
	}

	RfidEnd();
	digitalWrite(GREEN_LED_PIN, LOW);
	beep(1, 200);

	// добавляем в буфер последних команд
	addLastTeam(teamNumber, already_checked);
	lastTimeChecked = checkTime.unixtime;
	lastTeamFlag = teamNumber;

#ifdef DEBUG
	Serial.print(F("!!!New record #"));
	Serial.println(String(teamNumber));
#endif

#ifdef BENCHMARK
	const unsigned long result = millis() - startCheck;
	Serial.print(F("check time="));
	Serial.println(String(result));
#endif

	if (scanAutoreport)
	{
		//autoreport new command
		if (/*digitalRead(BT_CONNECTED) || */Serial)
		{
#ifdef DEBUG
			Serial.print(F("!!!Autoreport team #"));
			Serial.println(String(teamNumber));
#endif

			if (readTeamFromFlash(teamNumber))
			{
				init_package(REPLY_GET_TEAM_RECORD);
				// 0: код ошибки
				if (!addData(OK)) return;

				// 1-2: номер команды
				// 3-6 : время инициализации
				// 7-8: маска команды
				// 9-12 : время последней отметки на станции
				// 13: счетчик сохраненных страниц
				for (uint8_t i = 0; i < 13; i++)
				{
					if (!addData(ntag_page[i])) return;
				}
				sendData();
			}
			else addLastError(PROCESS_SEND_AUTOREPORT); //CARD PROCESSING: error sending autoreport
		}
	}
}

// обработка входящих команд
bool readUart(Stream& SerialPort)
{
	while (SerialPort.available())
	{
		int c = SerialPort.read();

		if (c == -1) // can't read stream
		{
#ifdef DEBUG
			Serial.println(F("!!! UART read error"));
#endif
			uartBufferPosition = 0;
			receivingData = false;
			return false;
		}

		// 0 byte = FE
		if (uartBufferPosition == HEADER_BYTE && c == 0xfe)
		{
#ifdef DEBUG
			Serial.print(F("!!!byte0="));
			if (c < 0x10) Serial.print(F("0"));
			Serial.println(String(uint8_t(c), HEX));
#endif
			receivingData = true;
			uartBuffer[uartBufferPosition] = uint8_t(c);
			uartBufferPosition++;
			// refresh timeout
			receiveStartTime = millis();
		}
		// 1st byte = ID, Station number, Command, Length and Data
		else if (receivingData)
		{
			uartBuffer[uartBufferPosition] = uint8_t(c);
#ifdef DEBUG
			Serial.print(F("!!!byte"));
			Serial.print(String(uartBufferPosition));
			Serial.print(F("="));
			if (c < 0x10) Serial.print(F("0"));
			Serial.println(String(uint8_t(c), HEX));
#endif
			// incorrect length
			if (uartBufferPosition == DATA_LENGTH_LOW_BYTE && uint32_t(uint16_t(uartBuffer[DATA_LENGTH_HIGH_BYTE]) * uint16_t(256) + uint16_t(uartBuffer[DATA_LENGTH_LOW_BYTE])) > uint16_t(uint16_t(MAX_PAKET_LENGTH) - uint16_t(DATA_START_BYTE)))
			{
#ifdef DEBUG
				Serial.println(F("!!!incorrect length"));
#endif
				uartBufferPosition = 0;
				receivingData = false;
				errorBeepMs(3, 50);
				addLastError(UART_PACKET_LENGTH);
				sendError(PARSE_PACKET_LENGTH_ERROR, uartBuffer[COMMAND_BYTE] + 0x10);
				return false;
			}

			// packet is received
			if (uartBufferPosition > DATA_LENGTH_LOW_BYTE && uartBufferPosition >= uint32_t(uint32_t(DATA_START_BYTE) + uint32_t(uint32_t(uartBuffer[DATA_LENGTH_HIGH_BYTE]) * uint32_t(256) + uint32_t(uartBuffer[DATA_LENGTH_LOW_BYTE]))))
			{
				// crc matching
#ifdef DEBUG
				Serial.print(F("!!!received packet expected CRC="));
				Serial.println(String(crcCalc(uartBuffer, PACKET_ID_BYTE, uartBufferPosition - 1), HEX));
#endif
				if (uartBuffer[uartBufferPosition] == crcCalc(uartBuffer, PACKET_ID_BYTE, uartBufferPosition - 1))
				{
					// incorrect station number
					if (uartBuffer[STATION_NUMBER_BYTE] != stationNumber && uartBuffer[COMMAND_BYTE] != COMMAND_GET_STATUS && uartBuffer[COMMAND_BYTE] != COMMAND_GET_CONFIG)
					{
#ifdef DEBUG
						Serial.println(F("!!!incorrect station#"));
#endif
						uartBufferPosition = 0;
						receivingData = false;
						errorBeepMs(3, 50);
						addLastError(UART_WRONG_STATION);
						sendError(WRONG_STATION, uartBuffer[COMMAND_BYTE] + 0x10);
						return false;
					}
#ifdef DEBUG
					Serial.print(F("!!!Command received:"));
					for (uint8_t i = 0; i <= uartBufferPosition; i++)
					{
						Serial.print(F(" "));
						if (uartBuffer[i] < 0x10) Serial.print(F("0"));
						Serial.print(String(uartBuffer[i], HEX));
					}
					Serial.println();
#endif
					uartBufferPosition = 0;
					receivingData = false;
					return true;
				}
				else // CRC not correct
				{
#ifdef DEBUG
					Serial.println(F("!!!incorrect crc"));
#endif
					uartBufferPosition = 0;
					receivingData = false;
					errorBeepMs(3, 50);
					addLastError(UART_CRC);
					return false;
				}
			}
			uartBufferPosition++;
		}
		else
		{
#ifdef DEBUG
			Serial.println(F("!!!unexpected byte"));
#endif
			receivingData = false;
			uartBufferPosition = 0;
			addLastError(UART_UNEXPECTED_BYTE);
		}
	}
	return false;
}

// Commands processing

// поиск функции
void executeCommand()
{
#ifdef DEBUG
	Serial.print(F("!!!Command: "));
	Serial.println(String(uartBuffer[COMMAND_BYTE], HEX));
#endif
	bool errorLengthFlag = false;
	uint16_t data_length = uint16_t(uint16_t(uartBuffer[DATA_LENGTH_HIGH_BYTE]) * uint16_t(256) + uint16_t(uartBuffer[DATA_LENGTH_LOW_BYTE]));
	switch (uartBuffer[COMMAND_BYTE])
	{
	case COMMAND_SET_MODE:
		if (data_length == DATA_LENGTH_SET_MODE) setMode();
		else errorLengthFlag = true;
		break;
	case COMMAND_SET_TIME:
		if (data_length == DATA_LENGTH_SET_TIME) setTime();
		else errorLengthFlag = true;
		break;
	case COMMAND_RESET_STATION:
		if (data_length == DATA_LENGTH_RESET_STATION) resetStation();
		else errorLengthFlag = true;
		break;
	case COMMAND_GET_STATUS:
		if (data_length == DATA_LENGTH_GET_STATUS) getStatus();
		else errorLengthFlag = true;
		break;
	case COMMAND_INIT_CHIP:
		if (data_length == DATA_LENGTH_INIT_CHIP) initChip();
		else errorLengthFlag = true;
		break;
	case COMMAND_GET_LAST_TEAMS:
		if (data_length == DATA_LENGTH_GET_LAST_TEAMS) getLastTeams();
		else errorLengthFlag = true;
		break;
	case COMMAND_GET_TEAM_RECORD:
		if (data_length == DATA_LENGTH_GET_TEAM_RECORD) getTeamRecord();
		else errorLengthFlag = true;
		break;
	case COMMAND_READ_CARD_PAGE:
		if (data_length == DATA_LENGTH_READ_CARD_PAGE) readCardPages();
		else errorLengthFlag = true;
		break;
	case COMMAND_UPDATE_TEAM_MASK:
		if (data_length == DATA_LENGTH_UPDATE_TEAM_MASK) updateTeamMask();
		else errorLengthFlag = true;
		break;
	case COMMAND_WRITE_CARD_PAGE:
		if (data_length == DATA_LENGTH_WRITE_CARD_PAGE) writeCardPage();
		else errorLengthFlag = true;
		break;
	case COMMAND_READ_FLASH:
		if (data_length == DATA_LENGTH_READ_FLASH) readFlash();
		else errorLengthFlag = true;
		break;
	case COMMAND_WRITE_FLASH:
		if (data_length >= DATA_LENGTH_WRITE_FLASH) writeFlash();
		else errorLengthFlag = true;
		break;
	case COMMAND_ERASE_FLASH_SECTOR:
		if (data_length >= DATA_LENGTH_ERASE_FLASH_SECTOR) eraseTeamFlash();
		else errorLengthFlag = true;
		break;
	case COMMAND_GET_CONFIG:
		if (data_length == DATA_LENGTH_GET_CONFIG) getConfig();
		else errorLengthFlag = true;
		break;
	case COMMAND_SET_V_KOEFF:
		if (data_length == DATA_LENGTH_SET_V_KOEFF) setVCoeff();
		else errorLengthFlag = true;
		break;
	case COMMAND_SET_GAIN:
		if (data_length == DATA_LENGTH_SET_GAIN) setGain();
		else errorLengthFlag = true;
		break;
	case COMMAND_SET_CHIP_TYPE:
		if (data_length == DATA_LENGTH_SET_CHIP_TYPE) setChipType();
		else errorLengthFlag = true;
		break;
	case COMMAND_SET_TEAM_FLASH_SIZE:
		if (data_length == DATA_LENGTH_SET_TEAM_FLASH_SIZE) setTeamFlashSize();
		else errorLengthFlag = true;
		break;
	case COMMAND_SET_FLASH_BLOCK_SIZE:
		if (data_length == DATA_LENGTH_SET_FLASH_BLOCK_SIZE) setFlashBlockSize();
		else errorLengthFlag = true;
		break;
	case COMMAND_SET_BT_NAME:
		if (data_length >= DATA_LENGTH_SET_BT_NAME) setBtName();
		else errorLengthFlag = true;
		break;
	case COMMAND_SET_BT_PIN:
		if (data_length >= DATA_LENGTH_SET_BT_PIN) setBtPinCode();
		else errorLengthFlag = true;
		break;
	case COMMAND_SET_BATTERY_LIMIT:
		if (data_length == DATA_LENGTH_SET_BATTERY_LIMIT) setBatteryLimit();
		else errorLengthFlag = true;
		break;
	case COMMAND_SCAN_TEAMS:
		if (data_length == DATA_LENGTH_SCAN_TEAMS) scanTeams();
		else errorLengthFlag = true;
		break;
	case COMMAND_GET_LAST_ERRORS:
		if (data_length == DATA_LENGTH_GET_LAST_ERRORS) getLastErrors();
		else errorLengthFlag = true;
		break;
	case COMMAND_SET_AUTOREPORT:
		if (data_length == DATA_LENGTH_SET_AUTOREPORT) setAutoReport();
		else errorLengthFlag = true;
		break;

	default:
		sendError(WRONG_COMMAND, uartBuffer[COMMAND_BYTE] + 0x10);
		break;
	}

	uartBufferPosition = 0;
	if (errorLengthFlag)
	{
#ifdef DEBUG
		Serial.println(F("!!!Incorrect data length"));
#endif
		sendError(WRONG_PACKET_LENGTH, uartBuffer[COMMAND_BYTE] + 0x10);
	}
}

// установка режима
void setMode()
{
	// 0: новый номер режима
	stationMode = uartBuffer[DATA_START_BYTE];
	preferences.putUInt(EEPROM_STATION_MODE, stationMode);

	// формирование пакета данных.
	init_package(REPLY_SET_MODE);

	// 0: код ошибки
	if (!addData(OK)) return;
	sendData();
}

// обновление времени на станции
void setTime()
{
#ifdef DEBUG
	Serial.print(F("!!!Time: "));
	for (uint8_t i = 0; i < 6; i++) Serial.println(String(uartBuffer[DATA_START_BYTE + i]) + " ");
#endif

	// 0-3: дата и время в unixtime

	struct ts systemTime;

	systemTime.year = uartBuffer[DATA_START_BYTE] + 2000;
	systemTime.mon = uartBuffer[DATA_START_BYTE + 1];
	systemTime.mday = uartBuffer[DATA_START_BYTE + 2];
	systemTime.hour = uartBuffer[DATA_START_BYTE + 3];
	systemTime.min = uartBuffer[DATA_START_BYTE + 4];
	systemTime.sec = uartBuffer[DATA_START_BYTE + 5];
	DS3231_set(systemTime); // correct time
	delay(1);
	DS3231_get(&systemTime);

	init_package(REPLY_SET_TIME);

	// 0: код ошибки
	// 1-4: текущее время
	bool flag = true;
	flag &= addData(OK);
	flag &= addData((systemTime.unixtime & 0xFF000000) >> 24);
	flag &= addData((systemTime.unixtime & 0x00FF0000) >> 16);
	flag &= addData((systemTime.unixtime & 0x0000FF00) >> 8);
	flag &= addData(systemTime.unixtime & 0x000000FF);
	if (!flag) return;
	sendData();
}

// сброс настроек станции
void resetStation()
{
	// 0-1: кол-во отмеченных чипов (для сверки)
	// 2-5: время последней отметки(для сверки)
	// 6 : новый номер станции
	// проверить количество отметок
	uint16_t checkCardNumber = uartBuffer[DATA_START_BYTE];
	checkCardNumber <<= 8;
	checkCardNumber += uartBuffer[DATA_START_BYTE + 1];

	// проверить кол-во отметок (для безопасности)
	if (checkCardNumber != totalChipsChecked)
	{
		sendError(WRONG_DATA, REPLY_RESET_STATION);
		return;
	}

	// проверить время последней отметки
	uint32_t checkLastTime = uartBuffer[DATA_START_BYTE + 2];
	checkLastTime <<= 8;
	checkLastTime += uartBuffer[DATA_START_BYTE + 3];
	checkLastTime <<= 8;
	checkLastTime += uartBuffer[DATA_START_BYTE + 4];
	checkLastTime <<= 8;
	checkLastTime += uartBuffer[DATA_START_BYTE + 5];
	if (checkLastTime != lastTimeChecked)
	{
		sendError(WRONG_DATA, REPLY_RESET_STATION);
		return;
	}

	if (uartBuffer[DATA_START_BYTE + 6] == 0xff)
	{
		sendError(WRONG_STATION, REPLY_RESET_STATION);
		return;
	}
	stationNumber = uartBuffer[DATA_START_BYTE + 6];
	preferences.putUInt(EEPROM_STATION_NUMBER, stationNumber);

	stationMode = 0;
	preferences.putUInt(EEPROM_STATION_MODE, stationMode);

	lastTimeChecked = 0;
	totalChipsChecked = 0;

	if (!FFat.format())
	{
		String teamFile;
		teamFile.reserve(teamFilePrefix.length() + 5);
		for (int teamNumber = 0; teamNumber <= maxTeamNumber; teamNumber++)
		{
			teamFile = teamFilePrefix + String(teamNumber);
			if (FFat.exists(teamFile.c_str()))
			{
				if (!FFat.remove(teamFile.c_str()))
				{
					sendError(FLASH_WRITE_ERROR, REPLY_RESET_STATION);
					return;
				}
			}
		}
	}

	init_package(REPLY_RESET_STATION);

	// 0: код ошибки
	if (!addData(OK)) return;
	sendData();
	delay(100);
	ESP.restart();
}

// выдает статус: время на станции, номер станции, номер режима, число отметок, время последней страницы
void getStatus()
{
	struct ts systemTime;
	DS3231_get(&systemTime);

	// 0: код ошибки
	init_package(REPLY_GET_STATUS);

	bool flag = true;
	flag &= addData(OK);

	// 1 - 4: текущее время
	flag &= addData((systemTime.unixtime & 0xFF000000) >> 24);
	flag &= addData((systemTime.unixtime & 0x00FF0000) >> 16);
	flag &= addData((systemTime.unixtime & 0x0000FF00) >> 8);
	flag &= addData(systemTime.unixtime & 0x000000FF);

	// 5 - 6 : количество отметок на станции
	flag &= addData((totalChipsChecked & 0xFF00) >> 8);
	flag &= addData(totalChipsChecked & 0x00FF);

	// 7 - 10 : время последней отметки на станции
	flag &= addData((lastTimeChecked & 0xFF000000) >> 24);
	flag &= addData((lastTimeChecked & 0x00FF0000) >> 16);
	flag &= addData((lastTimeChecked & 0x0000FF00) >> 8);
	flag &= addData(lastTimeChecked & 0x000000FF);

	// 11 - 12 : напряжение батареи в условных единицах[0..1023] ~[0..1.1В]
	flag &= addData((batteryLevel & 0xFF00) >> 8);
	flag &= addData(batteryLevel & 0x00FF);

	// 13 - 14 : температура чипа DS3231(чуть выше окружающей среды)
	int temperature = int(DS3231_get_treg());
	flag &= addData((temperature & 0xFF00) >> 8);
	flag &= addData(temperature & 0x00FF);

	if (!flag) return;
	sendData();
}

// инициализация чипа
void initChip()
{
	struct ts systemTime;
	DS3231_get(&systemTime);

	digitalWrite(GREEN_LED_PIN, HIGH);

	RfidStart();
	// Look for new cards
	if (!RfidFindChip())
	{
		RfidEnd();
		digitalWrite(GREEN_LED_PIN, LOW);
		sendError(NO_CHIP, REPLY_INIT_CHIP);
		return;
	}

	// читаем блок информации
	if (!ntagRead4pages(PAGE_CHIP_SYS))
	{
		RfidEnd();
		digitalWrite(GREEN_LED_PIN, LOW);
		sendError(RFID_READ_ERROR, REPLY_INIT_CHIP);
		return;
	}

	// Фильтруем неправильный тип чипа
	if (ntag_page[2] != chipType)
	{
		RfidEnd();
		digitalWrite(GREEN_LED_PIN, LOW);
		sendError(WRONG_CHIP_TYPE, REPLY_INIT_CHIP);
		return;
	}

	// инициализация сработает только если время инициализации записанное уже на чипе превышает неделю до текущего времени
	uint32_t initTime = ntag_page[8];
	initTime <<= 8;
	initTime += ntag_page[9];
	initTime <<= 8;
	initTime += ntag_page[10];
	initTime <<= 8;
	initTime += ntag_page[11];
	if ((systemTime.unixtime - initTime) < maxTimeInit)
	{
		RfidEnd();
		digitalWrite(GREEN_LED_PIN, LOW);
		sendError(LOW_INIT_TIME, REPLY_INIT_CHIP);
		return;
	}

	// заполняем чип 0x00
	uint8_t dataBlock[4] = { 0,0,0,0 };
	for (uint8_t page = PAGE_CHIP_NUM; page < tagMaxPage; page++)
	{
		if (!ntagWritePage(dataBlock, page, false))
		{
			RfidEnd();
			digitalWrite(GREEN_LED_PIN, LOW);
			sendError(RFID_WRITE_ERROR, REPLY_INIT_CHIP);
			return;
		}
	}

	// 0-1: номер команды
	// 2-3 : маска участников

	// пишем данные на чип
	// номер команды, тип чипа, версия прошивки станции
	dataBlock[0] = uartBuffer[DATA_START_BYTE];
	dataBlock[1] = uartBuffer[DATA_START_BYTE + 1];
	dataBlock[2] = ntagMark;
	dataBlock[3] = FW_VERSION;
	if (!ntagWritePage(dataBlock, PAGE_CHIP_NUM, false))
	{
		RfidEnd();
		digitalWrite(GREEN_LED_PIN, LOW);
		sendError(RFID_WRITE_ERROR, REPLY_INIT_CHIP);
		return;
	}

	// пишем на чип текущее время
	dataBlock[0] = (systemTime.unixtime & 0xFF000000) >> 24;
	dataBlock[1] = (systemTime.unixtime & 0x00FF0000) >> 16;
	dataBlock[2] = (systemTime.unixtime & 0x0000FF00) >> 8;
	dataBlock[3] = systemTime.unixtime & 0x000000FF;
	if (!ntagWritePage(dataBlock, PAGE_INIT_TIME, false))
	{
		RfidEnd();
		digitalWrite(GREEN_LED_PIN, LOW);
		sendError(RFID_WRITE_ERROR, REPLY_INIT_CHIP);
		return;
	}

	// маска участников
	dataBlock[0] = uartBuffer[DATA_START_BYTE + 2];
	dataBlock[1] = uartBuffer[DATA_START_BYTE + 3];
	dataBlock[2] = 0;
	dataBlock[3] = 0;
	if (!ntagWritePage(dataBlock, PAGE_TEAM_MASK, false))
	{
		RfidEnd();
		digitalWrite(GREEN_LED_PIN, LOW);
		sendError(RFID_WRITE_ERROR, REPLY_INIT_CHIP);
		return;
	}

	// получаем UID чипа
	if (!ntagRead4pages(PAGE_UID))
	{
		RfidEnd();
		digitalWrite(GREEN_LED_PIN, LOW);
		sendError(RFID_READ_ERROR, REPLY_INIT_CHIP);
		return;
	}

	RfidEnd();
	digitalWrite(GREEN_LED_PIN, LOW);

	init_package(REPLY_INIT_CHIP);
	if (!addData(OK)) return;

	// добавляем в ответ время инициализации
	bool flag = true;
	flag &= addData((systemTime.unixtime & 0xFF000000) >> 24);
	flag &= addData((systemTime.unixtime & 0x00FF0000) >> 16);
	flag &= addData((systemTime.unixtime & 0x0000FF00) >> 8);
	flag &= addData(systemTime.unixtime & 0x000000FF);
	if (!flag) return;

	// добавляем в ответ UID
	for (uint8_t i = 0; i <= 7; i++)
	{
		if (!addData(ntag_page[i])) return;
	}
	sendData();
}

// получить последнюю отметившуюся команду
void getLastTeams()
{
	init_package(REPLY_GET_LAST_TEAMS);

	// 0: код ошибки
	if (!addData(OK)) return;

	// номера последних команд
	for (uint8_t i = 0; i < LAST_TEAMS_LENGTH * 2; i++)
	{
		//stop if command is empty
		// if (lastTeams[i] + lastTeams[i + 1] == 0) break;
		if (!addData(lastTeams[i])) return;
		i++;
		if (!addData(lastTeams[i])) return;
	}
	sendData();
	for (uint8_t i = 0; i < LAST_TEAMS_LENGTH * 2; i++) lastTeams[i] = 0;
}

// получаем запись о команде из флэша
void getTeamRecord()
{
	// 0-1: какую запись
	uint16_t recordNumber = uartBuffer[DATA_START_BYTE];
	recordNumber <<= 8;
	recordNumber += uartBuffer[DATA_START_BYTE + 1];

	if (recordNumber < 1 || recordNumber > maxTeamNumber)
	{
		sendError(WRONG_TEAM, REPLY_GET_TEAM_RECORD);
		return;
	}

	init_package(REPLY_GET_TEAM_RECORD);
	// 0: код ошибки
	if (!addData(OK)) return;

	// если ячейка лога пуста
	if (!readTeamFromFlash(recordNumber))
	{
		sendError(NO_DATA, REPLY_GET_TEAM_RECORD);
		return;
	}

	// 1-2: номер команды
	// 3-6 : время инициализации
	// 7-8: маска команды
	// 9-12 : время последней отметки на станции
	// 13: счетчик сохраненных страниц
	for (uint8_t i = 0; i < 13; i++)
	{
		if (!addData(ntag_page[i])) return;
	}
	sendData();
}

// читаем страницы с чипа
void readCardPages()
{
#ifdef BENCHMARK
	const unsigned long startCheck = millis();
#endif

	digitalWrite(GREEN_LED_PIN, HIGH);
	RfidStart();
	// Look for new cards
	if (!RfidFindChip())
	{
		RfidEnd();
		digitalWrite(GREEN_LED_PIN, LOW);
		sendError(NO_CHIP, REPLY_READ_CARD_PAGE);
		return;
	}

	uint16_t pageFrom = uartBuffer[DATA_START_BYTE];
	uint16_t pageTo = uartBuffer[DATA_START_BYTE + 1];

	init_package(REPLY_READ_CARD_PAGE);

	// 0: код ошибки
	// 1-8: UID чипа
	// 9-12: данные из страницы чипа(4 байта)
	if (!addData(OK))
	{
		RfidEnd();
		digitalWrite(GREEN_LED_PIN, LOW);
		return;
	}

	// читаем UID
	if (!ntagRead4pages(PAGE_UID))
	{
		RfidEnd();
		digitalWrite(GREEN_LED_PIN, LOW);
		sendError(RFID_READ_ERROR, REPLY_READ_CARD_PAGE);
		return;
	}

	// пишем UID в буфер ответа
	for (uint8_t i = 0; i <= 7; i++)
	{
		if (!addData(ntag_page[i]))
		{
			RfidEnd();
			digitalWrite(GREEN_LED_PIN, LOW);
			return;
		}
	}

	// начальная страница
	if (!addData(pageFrom))
	{
		RfidEnd();
		digitalWrite(GREEN_LED_PIN, LOW);
		return;
	}

	while (pageFrom <= pageTo)
	{
		if (!ntagRead4pages(pageFrom))
		{
			RfidEnd();
			digitalWrite(GREEN_LED_PIN, LOW);
			sendError(RFID_READ_ERROR, REPLY_READ_CARD_PAGE);
			return;
		}
		uint8_t n = (pageTo - pageFrom + 1);
		if (n > 4) n = 4;
		for (uint8_t i = 0; i < n; i++)
		{
			for (uint8_t j = 0; j < 4; j++)
			{
				if (!addData(ntag_page[i * 4 + j]))
				{
					RfidEnd();
					digitalWrite(GREEN_LED_PIN, LOW);
					return;
				}
			}
			pageFrom++;
		}
	}

	RfidEnd();
	digitalWrite(GREEN_LED_PIN, LOW);

#ifdef BENCHMARK
	const unsigned long result = millis() - startCheck;
	Serial.print(F("read time="));
	Serial.println(String(result));
#endif

	sendData();
}

// Обновить маску команды в буфере
void updateTeamMask()
{
	struct ts systemTime;
	DS3231_get(&systemTime);

	// 0-1: номер команды
	// 2-5: время выдачи чипа
	// 6-7: маска участников
	saveNewMask();

	init_package(REPLY_UPDATE_TEAM_MASK);
	// 0: код ошибки
	if (!addData(OK))
		return;

	sendData();

	if (stationMode != MODE_INIT)
		return;

	// включаем SPI ищем чип вблизи. Если не находим выходим из функции чтения чипов
	RfidStart();
	// Look for new cards
	if (!RfidFindChip())
	{
		RfidEnd();
		lastTeamFlag = 0;
		sendError(NO_CHIP, REPLY_UPDATE_TEAM_MASK);
		return;
	}

	// читаем блок информации
	if (!ntagRead4pages(PAGE_UID))
	{
		RfidEnd();
		sendError(RFID_READ_ERROR, REPLY_UPDATE_TEAM_MASK);
		return;
	}

	//неправильный тип чипа
	if (ntag_page[14] != chipType)
	{
		RfidEnd();
#ifdef DEBUG
		Serial.println(F("!!!incorrect chip"));
#endif
		sendError(WRONG_CHIP_TYPE, REPLY_UPDATE_TEAM_MASK);
		return;
	}

	/*
	Фильтруем
	1 - неправильный тип чипа
	2 - чип от другой прошивки
	3 - чип более недельной давности инициализации
	4 - чипы с командой №0 или >maxTeamNumber
	5 - чип, который совпадает с уже отмеченным (в lastTeams[])
	*/

	// читаем блок информации
	if (!ntagRead4pages(PAGE_CHIP_NUM))
	{
		RfidEnd();
		sendError(RFID_READ_ERROR, REPLY_UPDATE_TEAM_MASK);
		return;
	}

	// неправильный тип чипа
	/*if (ntag_page[2] != NTAG_MARK)
	{
		RfidEnd();
#ifdef DEBUG
			Serial.println(F("!!!incorrect chip"));
#endif
			sendError(WRONG_CHIP_TYPE, REPLY_UPDATE_TEAM_MASK);
			return;
		}*/

		// чип от другой прошивки
	if (ntag_page[3] != FW_VERSION)
	{
		RfidEnd();
#ifdef DEBUG
		Serial.println(F("!!!incorrect fw"));
#endif
		sendError(WRONG_FW_VERSION, REPLY_UPDATE_TEAM_MASK);
		return;
	}

	// Не слишком ли старый чип? Недельной давности и более
	uint32_t timeInit = ntag_page[4];
	timeInit = timeInit << 8;
	timeInit += ntag_page[5];
	timeInit = timeInit << 8;
	timeInit += ntag_page[6];
	timeInit = timeInit << 8;
	timeInit += ntag_page[7];
	if ((systemTime.unixtime - timeInit) > maxTimeInit)
	{
		RfidEnd();
#ifdef DEBUG
		Serial.println(F("!!!outdated chip"));
#endif
		sendError(LOW_INIT_TIME, REPLY_UPDATE_TEAM_MASK);
		return;
	}

	uint16_t chipNum = (ntag_page[0] << 8) + ntag_page[1];

	// Если номер чипа =0 или >maxTeamNumber
	if (chipNum < 1 || chipNum > maxTeamNumber)
	{
		RfidEnd();
#ifdef DEBUG
		Serial.print(F("!!!incorrect chip #"));
		Serial.println(String(chipNum));
#endif
		sendError(WRONG_TEAM, REPLY_UPDATE_TEAM_MASK);
		return;
	}

	// не надо ли обновить у чипа маску?
	// 0-1: номер команды
	// 2-5: время выдачи чипа
	// 6-7: маска участников
	if (newTeamMask[0] + newTeamMask[1] != 0
		&& ntag_page[0] == newTeamMask[0]
		&& ntag_page[1] == newTeamMask[1]
		&& ntag_page[4] == newTeamMask[2]
		&& ntag_page[5] == newTeamMask[3]
		&& ntag_page[6] == newTeamMask[4]
		&& ntag_page[7] == newTeamMask[5])
	{
		if (ntag_page[8] != newTeamMask[6] || ntag_page[9] != newTeamMask[7])
		{
#ifdef DEBUG
			Serial.print(F("!!!updating mask"));
#endif
			digitalWrite(GREEN_LED_PIN, HIGH);
			uint8_t dataBlock[4] = { newTeamMask[6], newTeamMask[7], ntag_page[10], ntag_page[11] };
			if (!ntagWritePage(dataBlock, PAGE_TEAM_MASK, false))
			{
				RfidEnd();
				digitalWrite(GREEN_LED_PIN, LOW);
				sendError(RFID_WRITE_ERROR, REPLY_UPDATE_TEAM_MASK);
				return;
			}
		}

		RfidEnd();
		digitalWrite(GREEN_LED_PIN, LOW);
		clearNewMask();
	}

	RfidEnd();
}

// пишем присланные с ББ 4 байта в указанную страницу
void writeCardPage()
{
	digitalWrite(GREEN_LED_PIN, HIGH);
	RfidStart();
	// Look for new cards
	if (!RfidFindChip())
	{
		RfidEnd();
		digitalWrite(GREEN_LED_PIN, LOW);
		sendError(NO_CHIP, REPLY_WRITE_CARD_PAGE);
		return;
	}

	// 0-7: UID чипа
	// 8: номер страницы
	// 9-12: данные для записи (4 байта)

	// проверить UID
	if (!ntagRead4pages(PAGE_UID))
	{
		RfidEnd();
		digitalWrite(GREEN_LED_PIN, LOW);
		sendError(RFID_READ_ERROR, REPLY_WRITE_CARD_PAGE);
		return;
	}
	bool flag = false;
	for (uint8_t i = 0; i <= 7; i++)
	{
		if (uartBuffer[DATA_START_BYTE + i] != 0xff && ntag_page[i] != uartBuffer[DATA_START_BYTE + i])
		{
			flag = true;
			break;
		}
	}
	if (flag)
	{
		RfidEnd();
		digitalWrite(GREEN_LED_PIN, LOW);
		sendError(WRONG_UID, REPLY_WRITE_CARD_PAGE);
		return;
	}

	// записать страницу
	uint8_t dataBlock[] =
	{
		uartBuffer[DATA_START_BYTE + 9],
		uartBuffer[DATA_START_BYTE + 10],
		uartBuffer[DATA_START_BYTE + 11],
		uartBuffer[DATA_START_BYTE + 12]
	};
	if (!ntagWritePage(dataBlock, uartBuffer[DATA_START_BYTE + 8], false))
	{
		RfidEnd();
		digitalWrite(GREEN_LED_PIN, LOW);
		sendError(RFID_WRITE_ERROR, REPLY_WRITE_CARD_PAGE);
		return;
	}

	RfidEnd();
	digitalWrite(GREEN_LED_PIN, LOW);
	init_package(REPLY_WRITE_CARD_PAGE);

	// 0: код ошибки
	if (!addData(OK)) return;
	sendData();
}

// читаем флэш
void readFlash()
{
	// 0-3: адрес начала чтения
	// 4: размер блока
	uint32_t startAddress = uartBuffer[DATA_START_BYTE];
	startAddress <<= 8;
	startAddress += uartBuffer[DATA_START_BYTE + 1];
	startAddress <<= 8;
	startAddress += uartBuffer[DATA_START_BYTE + 2];
	startAddress <<= 8;
	startAddress += uartBuffer[DATA_START_BYTE + 3];

	uint32_t length = uint32_t(uint32_t(uint32_t(uartBuffer[DATA_START_BYTE + 4]) * uint32_t(256))
		+ uint32_t(uartBuffer[DATA_START_BYTE + 5]));

	if (length > uint32_t(uint32_t(MAX_PAKET_LENGTH) - 7 - uint32_t(DATA_LENGTH_READ_FLASH) - 1))
		length = uint32_t(MAX_PAKET_LENGTH - 7 - DATA_LENGTH_READ_FLASH - 1);

#ifdef DEBUG
	Serial.print(F("!!!flash read="));
	Serial.print(String(startAddress));
	Serial.print(F("/"));
	Serial.println(String(length));
#endif

	init_package(REPLY_READ_FLASH);

	// 0: код ошибки
	// 0-3: адрес начала чтения
	// 4-n: данные из флэша
	if (!addData(OK)) return;
#ifdef DEBUG
	Serial.print(F("!!!OK "));
	Serial.println(String(uartBufferPosition));
#endif

	bool flag = true;
	flag &= addData((startAddress & 0xFF000000) >> 24);
	flag &= addData((startAddress & 0x00FF0000) >> 16);
	flag &= addData((startAddress & 0x0000FF00) >> 8);
	flag &= addData(startAddress & 0x000000FF);
	if (!flag)
	{
		sendError(FLASH_READ_ERROR, REPLY_READ_FLASH);
		return;
	}

	uint16_t teamNumber = (uint16_t)(startAddress / teamFlashSize) + 1;
	startAddress -= (uint32_t)(startAddress * teamFlashSize);
#ifdef DEBUG
	//Serial.print(F("!!!address "));
	//Serial.println(String(uartBufferPosition));
#endif
	const String teamFile = teamFilePrefix + String(teamNumber);
	File file = FFat.open(teamFile, FILE_READ);
	if (!file)
	{
		sendError(FLASH_READ_ERROR, REPLY_READ_FLASH);
		return;
	}

	file.seek(startAddress);
	for (; length > 0; length--)
	{
		if (!addData(file.read()))
		{
			sendError(FLASH_READ_ERROR, REPLY_READ_FLASH);
			return;
		}

#ifdef DEBUG
		/*Serial.print(String(i));
		Serial.print(F("="));
		if (b < 0x10) Serial.print(F("0"));
		Serial.println(String(b, HEX));*/
#endif
	}

	file.close();
	sendData();
}

// пишем в флэш
void writeFlash()
{
	// 0-3: адрес начала записи
	// 1: кол-во записанных байт (для проверки)
	uint32_t startAddress = uartBuffer[DATA_START_BYTE];
	startAddress <<= 8;
	startAddress += uartBuffer[DATA_START_BYTE + 1];
	startAddress <<= 8;
	startAddress += uartBuffer[DATA_START_BYTE + 2];
	startAddress <<= 8;
	startAddress += uartBuffer[DATA_START_BYTE + 3];

	uint16_t length = uint16_t(uint16_t(uartBuffer[DATA_LENGTH_HIGH_BYTE]) * uint16_t(256) + uint16_t(uartBuffer[DATA_LENGTH_LOW_BYTE])) - 4;

	init_package(REPLY_WRITE_FLASH);

	uint16_t teamNumber = (uint16_t)(startAddress / teamFlashSize) + 1;
	startAddress -= (uint32_t)(startAddress * teamFlashSize);
	const String teamFile = teamFilePrefix + String(teamNumber);
	File file = FFat.open(teamFile, FILE_WRITE);
	if (!file
		|| !file.seek(startAddress)
		|| length != file.write(reinterpret_cast<uint8_t*>(uartBuffer[DATA_START_BYTE + 4]), length))
	{
		sendError(FLASH_WRITE_ERROR, REPLY_WRITE_FLASH);
		return;
	}

	file.close();

	// 0: код ошибки
	// 1-2: кол-во записанных байт (для проверки)
	if (!addData(OK)) return;
	if (!addData(length >> 8)) return;
	if (!addData(length & 0x00ff)) return;

	sendData();
}

// стираем команду с флэша (4096 байт)
void eraseTeamFlash()
{
	uint32_t teamNumber = uartBuffer[DATA_START_BYTE];
	teamNumber <<= 8;
	teamNumber += uartBuffer[DATA_START_BYTE + 1];
#ifdef DEBUG
	Serial.print(F("!!!erasing "));
	Serial.println(String(teamNumber));
#endif

	if (!eraseTeamFromFlash(teamNumber))
	{
		sendError(ERASE_ERROR, REPLY_ERASE_FLASH_SECTOR);
		return;
	}

	init_package(REPLY_ERASE_FLASH_SECTOR);
	// 0: код ошибки
	if (!addData(OK)) return;
	sendData();
}

// выдает конфигурацию станции
void getConfig()
{
	// 0: код ошибки
	// 1: версия прошивки
	// 2: номер режима
	// 3: тип чипов (емкость разная, а распознать их программно можно только по ошибкам чтения "дальних" страниц)
	// 4-7: емкость флэш - памяти
	// 8-11: размер сектора флэш - памяти
	// 12-15: коэффициент пересчета напряжения(float, 4 bytes) - просто умножаешь коэффициент на полученное в статусе число и будет температура
	// 16: коэффициент усиления антенны RFID
	// 17-18: размер блока хранения команды
	init_package(REPLY_GET_CONFIG);

	bool flag = true;
	flag &= addData(OK);
	flag &= addData(FW_VERSION);
	flag &= addData(stationMode);
	flag &= addData(ntagMark);

	uint32_t n = FFat.totalBytes();
	flag &= addData((n & 0xFF000000) >> 24);
	flag &= addData((n & 0x00FF0000) >> 16);
	flag &= addData((n & 0x0000FF00) >> 8);
	flag &= addData(n & 0x000000FF);

	uint8_t v[4];
	floatToByte(v, voltageCoeff);
	flag &= addData(v[0]);
	flag &= addData(v[1]);
	flag &= addData(v[2]);
	flag &= addData(v[3]);

	flag &= addData(gainCoeff);

	flag &= addData(teamFlashSize >> 8);
	flag &= addData(teamFlashSize & 0x00FF);

	flag &= addData(0 >> 8);
	flag &= addData(0 & 0x00FF);

	floatToByte(v, batteryLimit);
	flag &= addData(v[0]);
	flag &= addData(v[1]);
	flag &= addData(v[2]);
	flag &= addData(v[3]);

	flag &= addData(MAX_PAKET_LENGTH >> 8);
	flag &= addData(MAX_PAKET_LENGTH & 0x00FF);

	flag &= addData(scanAutoreport);

	if (!flag) return;

	sendData();
}

// сохранить коэфф. пересчета ADC в напряжение для резисторного делителя 10кОм + 2.2кОм
void setVCoeff()
{
	// 0-3: коэфф.
	union Convert
	{
		float number;
		uint8_t byte[4];
	} p;

	for (uint8_t i = 0; i < 4; i++)
	{
		p.byte[i] = uartBuffer[DATA_START_BYTE + i];
	}

	voltageCoeff = p.number;
	preferences.putFloat(EEPROM_VOLTAGE_KOEFF, voltageCoeff);
	init_package(REPLY_SET_V_KOEFF);
	// 0: код ошибки
	// 1...: данные из флэша
	if (!addData(OK)) return;

	sendData();
}

// сохранить коэфф. усиления для RFID
void setGain()
{
	// 0: коэфф.
	gainCoeff = uartBuffer[DATA_START_BYTE] & 0x70;
	preferences.putUInt(EEPROM_GAIN, gainCoeff);
	init_package(REPLY_SET_GAIN);
	// 0: код ошибки
	if (!addData(OK)) return;

	sendData();
}

// сохранить тип чипа
void setChipType()
{
	// 0: тип чипа
	chipType = uartBuffer[DATA_START_BYTE];
	if (!selectChipType(chipType))
	{
		sendError(WRONG_CHIP_TYPE, REPLY_SET_CHIP_TYPE);
		return;
	}

	preferences.putUInt(EEPROM_CHIP_TYPE, chipType);
	init_package(REPLY_SET_CHIP_TYPE);
	// 0: код ошибки
	if (!addData(OK)) return;

	sendData();
}

// сохранить размер блока команды
void setTeamFlashSize()
{
	// 0-1: размер блока
	uint16_t n = uint16_t(uint16_t(uint16_t(uartBuffer[DATA_START_BYTE]) * 256) + uint16_t(uartBuffer[DATA_START_BYTE + 1]));
	if (n < 16)
	{
		sendError(WRONG_SIZE, REPLY_SET_TEAM_FLASH_SIZE);
		return;
	}

	teamFlashSize = n;
	preferences.putUInt(EEPROM_TEAM_BLOCK_SIZE, teamFlashSize);
	init_package(REPLY_SET_TEAM_FLASH_SIZE);
	// 0: код ошибки
	if (!addData(OK)) return;

	sendData();
}

// сохранить размер стираемого блока
// DEPRECATED!!!
void setFlashBlockSize()
{
	init_package(REPLY_SET_FLASH_BLOCK_SIZE);
	// 0: код ошибки
	if (!addData(OK)) return;

	sendData();
}

// поменять имя BT адаптера
void setBtName()
{
#ifdef DEBUG
	Serial.print(F("!!!Set new BT name"));
#endif
	uint16_t data_length = uint16_t(uint16_t(uartBuffer[DATA_LENGTH_HIGH_BYTE]) * uint16_t(256) + uint16_t(uartBuffer[DATA_LENGTH_LOW_BYTE]));
	if (data_length < 1 || data_length>32)
	{
		sendError(WRONG_DATA, REPLY_SET_BT_NAME);
		return;
	}

	String btCommand;
	btCommand.reserve(data_length + 1);
	for (uint16_t i = 0; i < data_length; i++)
		btCommand += String(static_cast<char>(uartBuffer[DATA_START_BYTE + i]));

	preferences.putString(EEPROM_STATION_NAME, btCommand);
	init_package(REPLY_SET_BT_NAME);
	// 0: код ошибки
	if (!addData(OK)) return;
	sendData();
}

// поменять пин-код BT адаптера
void setBtPinCode()
{
#ifdef DEBUG
	Serial.print(F("!!!Set new BT pin"));
#endif
	uint16_t data_length = uint16_t(uint16_t(uartBuffer[DATA_LENGTH_HIGH_BYTE]) * uint16_t(256) + uint16_t(uartBuffer[DATA_LENGTH_LOW_BYTE]));

	if (data_length < 1 || data_length>16)
	{
		sendError(WRONG_DATA, REPLY_SET_BT_PIN);
		return;
	}
	String btCommand;
	btCommand.reserve(data_length + 1);
	for (uint16_t i = 0; i < data_length; i++)
		btCommand += String(static_cast<char>(uartBuffer[DATA_START_BYTE + i]));

	preferences.putString(EEPROM_STATION_PIN, btCommand);
	init_package(REPLY_SET_BT_PIN);
	// 0: код ошибки
	if (!addData(OK)) return;

	sendData();
}

// установить лимит срабатывания сигнала о разряде батареи
void setBatteryLimit()
{
	// 0-3: коэфф.
	union Convert
	{
		float number;
		uint8_t byte[4];
	} p;

	for (uint8_t i = 0; i < 4; i++)
	{
		p.byte[i] = uartBuffer[DATA_START_BYTE + i];
	}

	batteryLimit = p.number;
	preferences.putFloat(EEPROM_BATTERY_LIMIT, batteryLimit);
	init_package(REPLY_SET_BATTERY_LIMIT);
	// 0: код ошибки
	// 1...: данные из флэша
	if (!addData(OK)) return;

	sendData();
}

// получить список команд на флэше
void scanTeams()
{
#ifdef DEBUG
	Serial.print(F("!!!Scan commands in flash"));
#endif

	// 0-1: номер команды
	uint16_t startNumber = uint16_t(uint16_t(uartBuffer[DATA_START_BYTE] * 256) + uartBuffer[DATA_START_BYTE + 1]);
#ifdef DEBUG
	Serial.print(F("!!!Start from command "));
	Serial.println(String(startNumber));
#endif

	if (startNumber < 1 || startNumber > maxTeamNumber)
	{
		sendError(WRONG_DATA, REPLY_SCAN_TEAMS);
		return;
	}

	init_package(REPLY_SCAN_TEAMS);
	// 0: код ошибки
	if (!addData(OK)) return;

	// 1...: список команд
	//uint8_t data[2];
	for (; startNumber <= maxTeamNumber; startNumber++)
	{
#ifdef DEBUG
		Serial.print(F("!!!Trying "));
		Serial.println(String(startNumber));
#endif

		if (checkTeamExists(startNumber))
		{
#ifdef DEBUG
			Serial.print(F("!!!Found "));
			Serial.println(String(startNumber));
#endif
			if (!addData((startNumber & 0xFF00) >> 8)) return;
			if (!addData(startNumber & 0x00FF)) return;
		}
		if (uartBufferPosition > MAX_PAKET_LENGTH - 10) break;
	}
	sendData();
}

// получить последние ошибки
void getLastErrors()
{
	init_package(REPLY_GET_LAST_ERRORS);

	// 0: код ошибки
	if (!addData(OK)) return;

	// номера последних ошибок
	uint8_t i = 0;
	while (lastErrors[i] != 0 && i < LAST_TEAMS_LENGTH)
	{
		if (!addData(lastErrors[i])) return;
		i++;
	}
	sendData();
	for (i = 0; i < LAST_TEAMS_LENGTH; i++) lastErrors[i] = 0;
}

// установка режима автоответа
void setAutoReport()
{
	// 0: новый режим
	scanAutoreport = uartBuffer[DATA_START_BYTE];
	preferences.putUInt(EEPROM_AUTOREPORT, scanAutoreport);
	init_package(REPLY_SET_AUTOREPORT);

	// 0: код ошибки
	if (!addData(OK)) return;
	sendData();
}

// Internal functions

// заполнить буфер смены маски
void saveNewMask()
{
#ifdef DEBUG
	Serial.print(F("!!!New mask set to: "));
#endif

	for (uint8_t i = 0; i < 8; i++)
	{
		newTeamMask[i] = uartBuffer[DATA_START_BYTE + i];
#ifdef DEBUG
		Serial.print(String(newTeamMask[i], HEX));
#endif
	}
#ifdef DEBUG
	Serial.println();
#endif
}

// очистить буфер смены маски
void clearNewMask()
{
	for (uint8_t i = 0; i < 8; i++)
	{
		newTeamMask[i] = 0;
	}
#ifdef DEBUG
	Serial.print(F("!!!Mask cleared: "));
#endif
}

// ToDo: переделать для ESP32
// чтение напряжения батареи
uint16_t getBatteryLevel()
{
	const uint8_t MeasurementsToAverage = 16;
	uint32_t AverageValue = analogRead(BATTERY_PIN);
	for (uint8_t i = 1; i < MeasurementsToAverage; ++i)
	{
		uint16_t val = analogRead(BATTERY_PIN);
		AverageValue = (AverageValue + val) / 2;
		delay(5);
	}
	return AverageValue;
}

// ToDo: переделать для ESP32
// сигнал станции, длительность сигнала и задержки в мс и число повторений
void beep(uint8_t n, uint16_t ms)
{
	for (uint8_t i = 0; i < n; i++)
	{
		digitalWrite(GREEN_LED_PIN, HIGH);
		//tone(BUZZER_PIN, 4000, ms);
		delay(ms);
		digitalWrite(GREEN_LED_PIN, LOW);
		if (n - i != 0)
		{
			delay(ms);
		}
	}
}

// ToDo: переделать для ESP32
// сигнал ошибки станции
void errorBeepMs(uint8_t n, uint16_t ms)
{
	for (uint8_t i = 0; i < n; i++)
	{
		digitalWrite(RED_LED_PIN, HIGH);
		//tone(BUZZER_PIN, 500, ms);
		delay(ms);
		digitalWrite(RED_LED_PIN, LOW);
		if (n - i > 0)
		{
			delay(ms);
		}
	}
}

// ToDo: переделать для ESP32
// сигнал ошибки станции
void errorBeep(uint8_t n)
{
	uint16_t ms = 200;
	for (uint8_t i = 0; i < n; i++)
	{
		digitalWrite(RED_LED_PIN, HIGH);
		//tone(BUZZER_PIN, 500, ms);
		delay(ms);
		digitalWrite(RED_LED_PIN, LOW);
		if (n - i > 0)
		{
			delay(ms);
		}
	}
}

// инициализация пакета данных
void init_package(uint8_t command)
{
	uartBuffer[HEADER_BYTE] = 0xFE;
	uartBuffer[STATION_NUMBER_BYTE] = stationNumber;
	uartBuffer[COMMAND_BYTE] = command;
	uartBufferPosition = DATA_START_BYTE;
}

// добавление данных в буфер
bool addData(uint8_t data)
{
#ifdef DEBUG
	Serial.print(F("!!!Adding to buffer: 0x"));
	Serial.println(String(data, HEX));
#endif

	if (uartBufferPosition >= uint16_t(MAX_PAKET_LENGTH - 1))
	{
#ifdef DEBUG
		Serial.println(F("!!!Adding to buffer fail"));
#endif

		sendError(BUFFER_OVERFLOW);
		return false;
	}
	uartBuffer[uartBufferPosition] = data;
	uartBufferPosition++;
	return true;
}

// передача пакета данных
void sendData()
{
	uartBuffer[DATA_LENGTH_HIGH_BYTE] = (uartBufferPosition - DATA_LENGTH_LOW_BYTE - 1) >> 8;
	uartBuffer[DATA_LENGTH_LOW_BYTE] = (uartBufferPosition - DATA_LENGTH_LOW_BYTE - 1) & 0x0FF;
	uartBuffer[uartBufferPosition] = crcCalc(uartBuffer, PACKET_ID_BYTE, uartBufferPosition - 1);
	uartBufferPosition++;
#ifdef DEBUG
	Serial.print(F("!!!Sending:"));
	for (uint16_t i = 0; i < uartBufferPosition; i++)
	{
		Serial.print(F(" "));
		if (uartBuffer[i] < 0x10) Serial.print(F("0"));
		Serial.print(String(uartBuffer[i], HEX));
	}
	Serial.println();
#endif
	Serial.write(uartBuffer, uartBufferPosition);
	if (SerialBT.connected())
		SerialBT.write(uartBuffer, uartBufferPosition);

	uartBufferPosition = 0;
}

// запись страницы (4 байта) в чип
bool ntagWritePage(uint8_t* dataBlock, uint8_t pageAdr, bool verify)
{
#if !defined(USE_PN532)
	const uint8_t sizePageNtag = 4;
#endif
	uint8_t n = 0;
	bool status = false;
	while (!status && n < 3)
	{
#if defined(USE_PN532)
		status = pn532.ntag2xx_WritePage(pageAdr, dataBlock);
#else
		status = (MFRC522::STATUS_OK == MFRC522::StatusCode(mfrc522.MIFARE_Ultralight_Write(pageAdr, dataBlock, sizePageNtag)));
#endif

		n++;
		if (!status)
		{
			RfidStart();
			RfidFindChip();
		}
	}

	if (!status)
	{
#ifdef DEBUG
		Serial.println(F("!!!chip write failed"));
#endif
		return false;
	}

	//if (verify)
	{
#ifdef DEBUG
		Serial.println(F("!!!chip write verification started"));
#endif
		n = 0;
		uint8_t buffer[18];
		status = false;
		while (!status && n < 3)
		{
#if defined(USE_PN532)
			status = pn532.ntag2xx_Read4Pages(pageAdr, buffer);
#else
			uint8_t size = sizeof(buffer);
			status = (MFRC522::STATUS_OK == MFRC522::StatusCode(mfrc522.MIFARE_Read(pageAdr, buffer, &size)));
#endif
			n++;
		}

		if (!status)
		{
#ifdef DEBUG
			Serial.println(F("!!!chip read failed"));
#endif
			return false;
		}

		for (uint8_t i = 0; i < 4; i++)
		{
			if (buffer[i] != dataBlock[i])
			{
#ifdef DEBUG
				Serial.println(F("!!!chip verify failed"));
#endif
				return false;
			}
		}
	}

	return true;
}

// чтение 4-х страниц (16 байт) из чипа
bool ntagRead4pages(uint8_t pageAdr)
{
	uint8_t size = 18;
	uint8_t buffer[size];
	uint8_t n = 0;
	bool status = false;
	while (!status && n < 3)
	{
#ifdef DEBUG
		Serial.println(F("!!!card reading"));
#endif
#if defined(USE_PN532)
		status = pn532.ntag2xx_Read4Pages(pageAdr, buffer);
#else
		status = (MFRC522::STATUS_OK == mfrc522.MIFARE_Read(pageAdr, buffer, &size));
#endif
		if (!status)
		{
#ifdef DEBUG
			Serial.println(F("!!!card read fail"));
#endif
			RfidStart();
			RfidFindChip();
#ifdef DEBUG
			Serial.println(F("!!!chip re-initialized"));
#endif
		}
		n++;
	}

	if (!status)
	{
#ifdef DEBUG
		Serial.println(F("!!!card read failed"));
#endif
		return false;
	}

	for (uint8_t i = 0; i < 16; i++)
	{
		ntag_page[i] = buffer[i];
	}
#ifdef DEBUG
	Serial.println(F("!!!card reading done"));
#endif

	return true;
}

// пишет на чип время и станцию отметки
bool writeCheckPointToCard(uint8_t newPage, uint32_t checkTime)
{
	uint8_t dataBlock[4];
	dataBlock[0] = stationNumber;
	dataBlock[1] = (checkTime & 0x00FF0000) >> 16;
	dataBlock[2] = (checkTime & 0x0000FF00) >> 8;
	dataBlock[3] = (checkTime & 0x000000FF);

	if (!ntagWritePage(dataBlock, newPage, false))
	{
		return false;
	}
	return true;
}

// Поиск пустой страницы на чипе.
// !!! разобраться в алгоритме Саши или сделать свой бинарный поиск
int findNewPage()
{
	uint8_t page = PAGE_DATA_START;
	while (page < tagMaxPage)
	{
		if (!ntagRead4pages(page))
		{
#ifdef DEBUG
			Serial.println(F("!!!Can't read chip"));
#endif
			// chip read error
			return 0;
		}
		for (uint8_t n = 0; n < 4; n++)
		{
			if (stationMode == MODE_START_KP && ntag_page[n * 4] == stationNumber)
			{
#ifdef DEBUG
				Serial.println(F("!!!Chip checked already"));
#endif
				// chip was checked by another station with the same number
				return -1;
			}
			if (ntag_page[n * 4] == 0 ||
				(stationMode == MODE_FINISH_KP && ntag_page[n * 4] == stationNumber))
			{
				// free page found
				return page;
			}
			page++;
		}
	}
	// чип заполнен
	return tagMaxPage;
}

bool checkTeamExists(uint16_t teamNumber)
{
	String teamFile = teamFilePrefix + String(teamNumber);
	File root = FFat.open(teamFile, FILE_READ);
	if (!root)
		return false;

	return true;
}

// пишем дамп чипа в лог
bool writeDumpToFlash(uint16_t teamNumber, uint32_t checkTime, uint32_t initTime, uint16_t mask)
{
	// адрес хранения в каталоге
	String teamFile = teamFilePrefix + String(teamNumber);
#ifdef DEBUG
	Serial.print(F("!!!Write to flash address: "));
	Serial.println(String(teamNumber));
#endif

	if (checkTeamExists(teamNumber))
	{
		// если режим финишной станции, то надо переписать содержимое
		if (stationMode == MODE_FINISH_KP)
		{
#ifdef DEBUG
			Serial.print(F("!!!erasing team #"));
			Serial.println(String(teamNumber));
#endif
			if (!eraseTeamFromFlash(teamNumber))
			{
#ifdef DEBUG
				Serial.println(F("!!!failed to erase"));
#endif
				return false;
			}
		}
		else
		{
#ifdef DEBUG
			Serial.print(F("!!!team already saved"));
#endif
			return false;
		}
	}

	// save basic parameters
	uint8_t basic_record[16];
	// 1-2: номер команды
	basic_record[0] = (teamNumber & 0xFF00) >> 8;
	basic_record[1] = teamNumber & 0x00FF;
	//3-6: время инициализации
	basic_record[2] = (initTime & 0xFF000000) >> 24;
	basic_record[3] = (initTime & 0x00FF0000) >> 16;
	basic_record[4] = (initTime & 0x0000FF00) >> 8;
	basic_record[5] = initTime & 0x000000FF;
	//7-8: маска команды
	basic_record[6] = (mask & 0xFF00) >> 8;
	basic_record[7] = mask & 0x00FF;
	//9-12: время последней отметки на станции
	basic_record[8] = (checkTime & 0xFF000000) >> 24;
	basic_record[9] = (checkTime & 0x00FF0000) >> 16;
	basic_record[10] = (checkTime & 0x0000FF00) >> 8;
	basic_record[11] = checkTime & 0x000000FF;
	//13: кол-во сохраненных страниц
	basic_record[12] = 0;
	//14-16: резерв
	basic_record[13] = 0;
	basic_record[14] = 0;
	basic_record[15] = 0;

	File file = FFat.open(teamFile, FILE_WRITE, true);
	if (!file)
	{
#ifdef DEBUG
		Serial.print(F("!!!fail open file: "));
		Serial.println(teamFile);
#endif
		return false;
	}

	if (file.write(basic_record, 16) != 16)
	{
		// append file to team_file_size
		file.close();
#ifdef DEBUG
		Serial.println(F("!!!fail write flash1"));
#endif
		return false;
	}

#ifdef DEBUG
	Serial.println(F("!!!basics written"));
#endif

	// copy card content to flash. все страницы не начинающиеся с 0
	uint8_t checkCount = 0;
	uint8_t block = 0;
	while (block < tagMaxPage)
	{
#ifdef DEBUG
		Serial.print(F("!!!reading 4 page from #"));
		Serial.println(String(block));
#endif
		if (!ntagRead4pages(block))
		{
			file.close();
#ifdef DEBUG
			Serial.println(F("!!!fail read chip"));
#endif
			return false;
		}

		//4 pages in one read block
		for (uint8_t i = 0; i < 4; i++)
		{
			if (block < 8 || ntag_page[i * 4]>0)
			{
#ifdef DEBUG
				Serial.print(F("!!!writing to flash: "));
				for (uint8_t k = 0; k < 16; k++)Serial.print(String(ntag_page[k], HEX) + " ");
				Serial.println();
#endif
				if (!file.write(&ntag_page[0 + i * 4], 4))
				{
					file.close();
					return false;
				}

				checkCount++;
			}
			else
			{
#ifdef DEBUG
				Serial.print(F("!!!chip last block: "));
				Serial.println(String(block + i));
#endif
				block = tagMaxPage;
				break;
			}
		}
		block += 4;
	}

	// add dump pages number
	if (!file.seek(12) || !file.write(checkCount))
	{
		file.close();
#ifdef DEBUG
		Serial.println(F("!!!fail write flash2"));
#endif
		return false;
	}

	file.close();

	return true;
}

// сохраняем весь блок, стираем весь блок и возвращаем назад все, кроме переписываемой команды
bool eraseTeamFromFlash(uint16_t teamNumber)
{
	const String teamFile = teamFilePrefix + String(teamNumber);
	return FFat.remove(teamFile);
}

// получаем сведения о команде из лога
bool readTeamFromFlash(uint16_t teamNumber)
{
	const String teamFile = teamFilePrefix + String(teamNumber);

	File file = FFat.open(teamFile, FILE_READ);
	if (!file)
		return false;

	if (file.readBytes(reinterpret_cast<char*>(ntag_page), 13) != 13)
		return false;

	file.close();

	return true;
}

// подсчет записанных в флэш отметок
uint16_t refreshChipCounter()
{
#ifdef DEBUG
	Serial.print(F("!!!team records found="));
#endif
	uint16_t chips = 0;
	char data[12];
	for (uint16_t i = 1; i <= maxTeamNumber; i++)
	{
		String teamFile = "/team" + String(i);
		File file = FFat.open(teamFile, FILE_READ);
		if (!file)
			continue;

		file.readBytes(data, 12);
		if (data[0] != 0xff && data[1] != 0xff)
		{
			chips++;
			uint32_t time = data[8];
			time <<= 8;
			time += data[9];
			time <<= 8;
			time += data[10];
			time <<= 8;
			time += data[11];
			if (time > lastTimeChecked)
			{
				lastTimeChecked = time;
				lastTeams[0] = data[0];
				lastTeams[1] = data[1];
			}

#ifdef DEBUG
			Serial.print(String(i));
			Serial.print(F(", "));
#endif
		}

		file.close();
	}

#ifdef DEBUG
	Serial.println();
	Serial.print(F("!!!checked chip counter="));
	Serial.println(String(chips));
#endif
	return chips;
}

// обработка ошибок. формирование пакета с сообщением о ошибке
void sendError(uint8_t errorCode, uint8_t commandCode)
{
	init_package(commandCode);
	uartBuffer[DATA_START_BYTE] = errorCode;
	uartBufferPosition = DATA_START_BYTE + 1;
	sendData();
}

void sendError(uint8_t errorCode)
{
	uartBuffer[DATA_START_BYTE] = errorCode;
	uartBufferPosition = DATA_START_BYTE + 1;
	sendData();
}

// сделать проверку дублей по всему массиву
// добавляем номер в буфер последних команд
void addLastTeam(uint16_t teamNumber, bool already_checked)
{
	// фильтровать дубли
	if (lastTeams[0] == uint8_t(teamNumber >> 8) && lastTeams[1] == uint8_t(teamNumber)) return;

	for (uint8_t i = LAST_TEAMS_LENGTH * 2 - 1; i > 1; i = i - 2)
	{
		lastTeams[i] = lastTeams[i - 2];
		lastTeams[i - 1] = lastTeams[i - 3];
	}
	lastTeams[0] = uint8_t(teamNumber >> 8);
	lastTeams[1] = uint8_t(teamNumber);
	if (!already_checked) totalChipsChecked++;
}

// добавляем код ошибки в буфер последних ошибок
void addLastError(uint8_t errorCode)
{
	for (uint8_t i = LAST_TEAMS_LENGTH - 1; i > 0; i--)
	{
		lastErrors[i] = lastErrors[i - 1];
	}
	lastErrors[0] = errorCode;
}

uint8_t crcCalc(uint8_t* dataArray, uint16_t dataStart, uint16_t dataEnd)
{
	uint8_t crc = 0x00;
	while (dataStart <= dataEnd)
	{
		uint8_t tmpByte = dataArray[dataStart];
		for (uint8_t tempI = 8; tempI; tempI--)
		{
			uint8_t sum = (crc ^ tmpByte) & 0x01;
			crc >>= 1;
			if (sum)
			{
				crc ^= 0x8C;
			}
			tmpByte >>= 1;
		}
		dataStart++;
	}
	return crc;
}

void floatToByte(uint8_t* bytes, float f)
{
	uint16_t length = sizeof(float);

	for (uint16_t i = 0; i < length; i++)
	{
		bytes[i] = ((uint8_t*)&f)[i];
	}
}

// check chip type consistence
bool selectChipType(uint8_t type)
{
	if (type == NTAG213_ID) //NTAG213
	{
		chipType = NTAG213_ID;
		ntagMark = NTAG213_MARK;
		tagMaxPage = NTAG213_MAX_PAGE;
	}
	else if (type == NTAG216_ID) //NTAG216
	{
		chipType = NTAG216_ID;
		ntagMark = NTAG216_MARK;
		tagMaxPage = NTAG216_MAX_PAGE;
	}
	else if (type == NTAG215_ID)//NTAG215
	{
		chipType = NTAG215_ID;
		ntagMark = NTAG215_MARK;
		tagMaxPage = NTAG215_MAX_PAGE;
	}
	else return false;
	return true;
}
