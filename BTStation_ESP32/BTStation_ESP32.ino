#include <Wire.h>
#include <FS.h>
#include <FFat.h>
#include <BluetoothSerial.h>
#include <Preferences.h>
#include <ds3231.h>
#include <esp_mac.h>

#include "BTStation_ESP32.h"
#include "command_definitions.h"
#include "card_definitions.h"
#include "error_codes.h"
#include "settings.h"
#include "protocol_definitions.h"
#include "preferences_definitions.h"
#include "basic_helpers.h"
#include "logger.h"
#include "notifier.h"

#if defined(USE_PN532)
	#include <Adafruit_PN532.h>
	Adafruit_PN532 pn532(PN532_IRQ, PN532_RESET);
#else
	#include <SPI.h>
	#include <MFRC522.h>
	MFRC522 mfrc522(RFID_SS_PIN, RFID_RST_PIN); // рфид-модуль
#endif

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;
Preferences preferences;

uint32_t rfidReadStartTime = 0;

uint8_t lastTeams[LAST_TEAMS_LENGTH * 2]; // последние отмеченные команды
uint8_t lastErrors[LAST_ERRORS_LENGTH]; // последние коды ошибок станции
uint8_t ntag_page[16]; // буфер для чтения из чипа через ntagRead4pages()
uint32_t lastTimeChecked = 0; // время последней отметки чипа
uint16_t totalChipsChecked = 0; // количество отмеченных чипов

uint8_t newTeamMask[8]; // новая маска для замены в чипе
uint16_t lastTeamFlag = 0; // номер последней отмеченной команды для отсечки двойной отметки

const String teamFilePrefix = "/team";

uint8_t stationNumber = 0;
uint8_t stationMode = MODE_INIT;
bool scanAutoreport = false; // автоматически отправлять данные сканирования в UART порт
uint8_t chipType = NTAG215_ID; // тип чипа
uint8_t tagMaxPage = NTAG215_MAX_PAGE; // размер чипа в страницах
uint16_t teamFlashSize = EEPROM_TEAM_BLOCK_SIZE_DEFAULT; // размер записи лога
int maxTeamNumber = 1; // максимальное кол-во записей в флэш-памяти = (flashSize - flashBlockSize) / teamFlashSize - 1;
const uint32_t maxTimeInit = 7UL * 24UL * 60UL * 60UL;    // максимальный срок годности чипа [секунд] - дата инициализации
//7 дней назад от текущего момента. Максимум 194 дня
float voltageCoeff = 0.0011; // коэфф. перевода значения АЦП в напряжение для делителя 10кОм/4.7кОм
float batteryLimit = 0; // минимальное напряжение батареи
uint8_t gainCoeff = 96; // коэфф. усиления антенны - работают только биты 4,5,6; значения [0, 16, 32, 48, 64, 80, 96, 112]
bool AuthEnabled = false; // авторизация RFID
uint8_t AuthPwd[4] = { 0xff,0xff,0xff,0xff }; // ключ авторизации RFID
uint8_t AuthPack[2] = { 0,0 }; // ответ авторизации RFID

uint8_t uartBuffer[MAX_PAKET_LENGTH]; // UART command buffer
uint16_t uartBufferPosition = 0;
bool uartReady = false; // получен пакет
bool receivingData = false; // идет прием пакета
uint32_t receiveStartTime = 0; // время начала получения пакета

uint16_t batteryLevel = 500; // текущий уровень напряжения (замер АЦП)
uint8_t batteryAlarmCount = 0; // счетчик нарушений границы допустимого напряжения

struct ts systemTime;

uint32_t nextClockCheck = 0;
uint32_t lastSystemClock = 0;
uint32_t lastExternalClock = 0;

Notifier g_notifier;

void setup() {
	Serial.begin(UART_SPEED);
	
	setupNotifier();
	
	Wire.begin();
	DS3231_init(DS3231_INTCN);
	
	setupRFID();

	if (!FFat.begin())
	{
		logError(F("FFat failed, formatting..."));
		bool result = FFat.format();
		if (!result || !FFat.begin()) {
			if (!result)
				logError(F("FFat format failed"));
			else
				logError(F("FFat failed after format"));
			g_notifier.notifyError();
			addLastError(STARTUP_FFAT);
		}
	}
	
	// read settings
	if (!preferences.begin(PREFERENCE_NAME, false))	{
		logError(F("Preferences mount failed"));
		g_notifier.notifyError();
		addLastError(STARTUP_SETTINGS);
	}
	setupVars();

	//читаем Bluetooth имя из памяти
	String btName = preferences.getString(EEPROM_STATION_NAME, String(""));
	if (!btName || btName.length() <= 0) {
		logError(F("Bluetooth name not set"));
		uint8_t mac[6];
		esp_read_mac(mac, ESP_MAC_BT);
		btName = String("SportStation-") + String(mac[4], HEX) + String(":") + String(mac[5], HEX);
	}

	SerialBT.enableSSP(true, true); // Must be called before begin
	SerialBT.onConfirmRequest(BTConfirmRequestCallback);
	SerialBT.onAuthComplete(BTAuthCompleteCallback);
	SerialBT.begin(btName.c_str()); //Bluetooth device name
}

void loop() {
	g_notifier.update();

	//если режим КП то отметить чип автоматом
	if (stationMode != MODE_INIT && millis() >= rfidReadStartTime) {
		processRfidCard();
		rfidReadStartTime = millis() + RFID_READ_PERIOD;
	}

	// check UART for data
	if (Serial.available()) {
		logDebug(F("UART data available"));
		uartReady = readUart(Serial);
	}

	// check Bluetooth for data
	if (SerialBT.available()) {
		logDebug(F("Bluetooth data available"));
		uartReady = readUart(SerialBT);
	}

	//обработать пришедшую команду
	if (uartReady) {
		uartReady = false;
		executeCommand();
	}

	// check receive timeout
	if (receivingData && millis() - receiveStartTime > RECEIVE_TIMEOUT)	{
		logError(F("Receive timeout"));
		uartBufferPosition = 0;
		uartReady = false;
		receivingData = false;
		g_notifier.notifyError();
		addLastError(UART_TIMEOUT);
	}

	checkBatteryLevel();
	checkClockIsRunning();
}

void setupNotifier() {
	pinMode(BUZZER_PIN, OUTPUT);
	Buzzer buzzer{BUZZER_PIN};

	pinMode(GREEN_LED_PIN, OUTPUT);
	LED green_led{GREEN_LED_PIN};

	pinMode(RED_LED_PIN, OUTPUT);
	LED red_led{RED_LED_PIN};

	g_notifier.init(buzzer, green_led, red_led);
}

void setupRFID() {
#if defined(USE_PN532)
	if (!pn532.begin())	{
		logError(F("Failed to initialize PN532!"));
		g_notifier.notifyError();
		addLastError(STARTUP_RFID);
	}
#else
	SPI.begin();
	mfrc522.PCD_Init();
	byte s = mfrc522.PCD_ReadRegister(MFRC522::PCD_Register::VersionReg);
	if (s == 0 || s == 0xff) {
		logError(F("MFRC522 initialization failed"));
		g_notifier.notifyError();
		addLastError(STARTUP_RFID);
	}
	SPI.end();
#endif
}

void setupVars() {
	//читаем номер станции из eeprom
	stationNumber = preferences.getUInt(EEPROM_STATION_NUMBER, 0);
	if (stationNumber == 0) {
		logError(F("Station number not set"));
		g_notifier.notifyError();
		addLastError(STARTUP_NUMBER);
	}

	//читаем номер режима из eeprom
	stationMode = preferences.getUInt(EEPROM_STATION_MODE, 0xff);
	if (stationMode == 0xff) {
		stationMode = MODE_INIT;
		logError(F("Station mode invalid"));
		g_notifier.notifyError();
		addLastError(STARTUP_MODE);
	}

	//читаем коэфф. пересчета напряжения
	voltageCoeff = preferences.getFloat(EEPROM_VOLTAGE_KOEFF, 0);
	if (voltageCoeff <= 0) {
		voltageCoeff = 0.0011;
		logError(F("Station voltage coefficient invalid"));
		g_notifier.notifyError();
		addLastError(STARTUP_GAIN); //STARTUP: incorrect gain in EEPROM
	}

	//читаем коэфф. усиления
	gainCoeff = preferences.getUInt(EEPROM_GAIN, 0xff);
	if (gainCoeff == 0xff) {
		gainCoeff = 96;
		logError(F("Station antenna gain invalid"));
		g_notifier.notifyError();
		addLastError(STARTUP_GAIN); //STARTUP: incorrect gain in EEPROM
	}

	//читаем тип чипа
	uint8_t chipType = preferences.getUInt(EEPROM_CHIP_TYPE, 0xff);
	if (chipType != 0xff) {
		selectChipType(chipType);
	}
	else {
		selectChipType(NTAG215_ID);
		logError(F("Station chip type invalid"));
		g_notifier.notifyError();
		addLastError(STARTUP_CHIP_TYPE);
	}

	//читаем размер блока команды
	teamFlashSize = preferences.getUInt(EEPROM_TEAM_BLOCK_SIZE, EEPROM_TEAM_BLOCK_SIZE_DEFAULT);
	if (teamFlashSize == 0) {
		teamFlashSize = EEPROM_TEAM_BLOCK_SIZE_DEFAULT;
		logError(F("Station team size invalid"));
		g_notifier.notifyError();
		addLastError(STARTUP_TEAM_SIZE);
	}

	//читаем минимальное напряжение батареи
	batteryLimit = preferences.getFloat(EEPROM_BATTERY_LIMIT, -1);
	if (batteryLimit < 0) {
		batteryLimit = 0;
		logError(F("Station battery limit invalid"));
		g_notifier.notifyError();
		addLastError(STARTUP_BATTERY_LIMIT);
	}

	//читаем режим автооповещения из памяти
	uint8_t scanAutoreportRaw = preferences.getUInt(EEPROM_AUTOREPORT, 0xff);
	if (scanAutoreportRaw == AUTOREPORT_ON) {
		scanAutoreport = true;
	}
	else if (scanAutoreportRaw == AUTOREPORT_OFF) {
		scanAutoreport = false;
	}
	else {
		scanAutoreport = false;
		logError(F("Station auto report setting invalid"));
		g_notifier.notifyError();
		addLastError(STARTUP_AUTOREPORT);
	}

	//читаем режим авторизации из памяти
	AuthEnabled = preferences.getBool(EEPROM_AUTH, false);

	//читаем ключ авторизации RFID из памяти
	uint32_t c = preferences.getBytes(EEPROM_AUTH_PWD, AuthPwd, 4);
	if (c != 4) {
		AuthEnabled = false;
		logError(F("Auth password invalid"));
	}

	//читаем ответ авторизации RFID из памяти
	c = preferences.getBytes(EEPROM_AUTH_PACK, AuthPack, 2);
	if (c != 2) {
		AuthEnabled = false;
		logError(F("Auth pack invalid"));
	}

	const uint32_t flashSize = FFat.totalBytes();
	maxTeamNumber = (flashSize / teamFlashSize) - 1;
	totalChipsChecked = refreshChipCounter();
	batteryLevel = getBatteryLevel();

	uint32_t currentMillis = millis();
	DS3231_get(&systemTime);

	lastSystemClock = currentMillis;
	lastExternalClock = systemTime.unixtime;
	nextClockCheck = currentMillis + 10000;

	lastTeamFlag = 0;
	clearNewMask();

	g_notifier.notify(1, 800);
}

bool RfidStart() {
	bool result = false;
#if defined(USE_PN532)
	uint8_t uid[8] = { 0 };    // Buffer to store the returned UID
	uint8_t uidLength;    	// Length of the UID (4 or 7 bytes depending on ISO14443A card type)
	result = pn532.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 1000);
	if (result) {
		logDebug(F("RFID chip is found. UID uidLength:"), uidLength);
	}
	else {
		logDebug(F("RFID chip not found"));
	}

#else
	// включаем SPI ищем чип вблизи. Если не находим выходим из функции чтения чипов
	SPI.begin();      // Init SPI bus
	mfrc522.PCD_Init();    // Init MFRC522
	mfrc522.PCD_SetAntennaGain(gainCoeff);
	delay(1);
	// Look for new cards
	result = mfrc522.PICC_IsNewCardPresent();
	if (!result)
	{
		logDebug(F("RFID chip not found"));
		return result;
	}
	logDebug(F("RFID chip found"));

	// Select one of the cards
	result = mfrc522.PICC_ReadCardSerial();
	if (!result)
	{
		logError(F("RFID fail to select chip"));
		return result;
	}
	else
	{
		logDebug(F("RFID chip selected"));
	}
#endif

	if (AuthEnabled)
	{
		result = ntagAuth(AuthPwd, AuthPack, false);
	}

	return result;
}

void RfidEnd()
{
#if defined(USE_PN532)
	pn532.powerDownMode();
#else
	mfrc522.PCD_AntennaOff();
	SPI.end();
	delay(5);
#endif
}

// Обработка поднесенного чипа
void processRfidCard() {
	if (stationNumber == 0 || stationNumber == 0xffffffff)
		return;

#ifdef BENCHMARK
	const unsigned long startCheck = millis();
#endif

	DS3231_get(&systemTime);

	// включаем SPI ищем чип вблизи. Если не находим выходим из функции чтения чипов
	logDebug(F("Searching for RFID chip"));

	if (!RfidStart()) {
		RfidEnd();
		lastTeamFlag = 0;
		return;
	}

	// читаем блок информации
	if (!ntagRead4pages(PAGE_CHIP_SYS2)) {
		RfidEnd();
		logError(F("Failed to read RFID chip"));
		g_notifier.notifyError();
		addLastError(PROCESS_READ_CHIP);
		return;
	}

	logDebug(F("Reading RFID chip"));

	//неправильный тип чипа
	if (ntag_page[2] != chipType) {
		RfidEnd();
		logError(F("Incorrect hardware RFID chip type"));
		g_notifier.notifyError();
		addLastError(PROCESS_HW_CHIP_TYPE);
		return;
	}

	/*
	Фильтруем
	1 - чип от другой прошивки
	2 - чип более недельной давности инициализации
	3 - чипы с командой №0 или >maxTeamNumber
	4 - чип, который совпадает с уже отмеченным (в lastTeams[])
	*/

	// чип от другой прошивки
	if (ntag_page[7] != FW_VERSION)	{
		RfidEnd();
		logError(F("Incorrect firmware version"));
		logDebug(F("Expected version: %d"), FW_VERSION);
		logDebug(F("Actual version: %d"), ntag_page[7]);
		g_notifier.notifyError();
		addLastError(PROCESS_FW_VERSION);
		return;
	}

	// Не слишком ли старый чип? Недельной давности и более
	uint32_t initTime = ntag_page[8];
	initTime = initTime << 8;
	initTime += ntag_page[9];
	initTime = initTime << 8;
	initTime += ntag_page[10];
	initTime = initTime << 8;
	initTime += ntag_page[11];
	if ((systemTime.unixtime - initTime) > maxTimeInit)	{
		RfidEnd();
		logError(F("Outdated RFID chip"));
		g_notifier.notifyError();
		addLastError(PROCESS_INIT_TIME); //CARD PROCESSING: chip init time is due
		return;
	}

	// Если номер чипа =0 или >maxTeamNumber
	uint16_t teamNumber = (ntag_page[4] << 8) + ntag_page[5];
	if (teamNumber < 1 || teamNumber > maxTeamNumber) {
		RfidEnd();
		logError(F("Incorrect chip number"));
		logDebug(F("Chip number: %d"), teamNumber);
		g_notifier.notifyError();
		addLastError(PROCESS_CHIP_NUMBER);
		return;
	}

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
		logDebug(F("Updating mask"));

		if (ntag_page[12] != newTeamMask[6] || ntag_page[13] != newTeamMask[7])
		{
			digitalWrite(GREEN_LED_PIN, HIGH);
			uint8_t dataBlock[4] = { newTeamMask[6], newTeamMask[7], ntag_page[14], ntag_page[15] };
			if (!ntagWritePage(dataBlock, PAGE_TEAM_MASK, true, false))
			{
				RfidEnd();
				logError(F("Failed to write mask"));
				digitalWrite(GREEN_LED_PIN, LOW);
				g_notifier.notifyError();
				addLastError(PROCESS_WRITE_CHIP); //CARD PROCESSING: error writing to chip
				return;
			}
		}

		RfidEnd();
		clearNewMask();
		lastTeamFlag = teamNumber;
		digitalWrite(GREEN_LED_PIN, LOW);
		logDebug(F("Mask is updated"));
		return;
	}

	uint16_t mask = (ntag_page[12] << 8) + ntag_page[13];

	// Если это повторная отметка
	if (teamNumber == lastTeamFlag)
	{
		logDebug(F("Same chip attached"));
		RfidEnd();
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
				logDebug(F("Chip already checked"));
				break;
			}
		}
	}

	// Есть ли чип на флэше
	if (!already_checked && checkTeamExists(teamNumber))
	{
		already_checked = true;
		logDebug(F("Chip already checked on flash"));
	}

	// если известный чип и стартовый КП
	if (already_checked && stationMode == MODE_START_KP)
	{
		logError(F("Chip already checked on start KP"));
		RfidEnd();
		//digitalWrite(GREEN_LED_PIN, LOW);
		g_notifier.notifyError();
		addLastError(PROCESS_ALREADY_CHECKED);
		lastTeamFlag = teamNumber;
		return;
	}

	// ищем свободную страницу на чипе
	int newPage = findNewPage();

	// ошибка чтения чипа
	if (newPage == 0) {
		RfidEnd();
		logError(F("Can't read chip"));
		g_notifier.notifyError();
		addLastError(PROCESS_READ_CHIP);
		return;
	}

	// больше/меньше нормы... Наверное, переполнен???
	if (newPage != -1 && (newPage < PAGE_DATA_START || newPage >= tagMaxPage)) {
		RfidEnd();
		logError(F("Incorrect chip page number"));
		logDebug(F("Chip page number: %d"), newPage);
		g_notifier.notifyError();
		addLastError(PROCESS_FIND_FREE_PAGE);
		return;
	}

	// chip was checked by another station with the same number
	if (newPage == -1) {
		logError(F("Chip marked by another station"));
		lastTeamFlag = teamNumber;
		return;
	}

	logDebug(F("Writing to chip"));

	// Пишем на чип отметку
	digitalWrite(GREEN_LED_PIN, HIGH);
	if (!writeCheckPointToCard(newPage, systemTime.unixtime)) {
		RfidEnd();
		logError(F("Failed to write chip"));
		g_notifier.notifyError();
		addLastError(PROCESS_WRITE_CHIP);
		return;
	}

	// Пишем дамп чипа во флэш
	if (!writeDumpToFlash(teamNumber, systemTime.unixtime, initTime, mask)) {
		RfidEnd();
		logError(F("Failed to write chip dump to flash"));
		g_notifier.notifyError();
		addLastError(PROCESS_SAVE_DUMP);
		return;
	}

	RfidEnd();
	digitalWrite(GREEN_LED_PIN, LOW);
	g_notifier.notify(1, 200);

	// добавляем в буфер последних команд
	addLastTeam(teamNumber, already_checked);
	lastTimeChecked = systemTime.unixtime;
	lastTeamFlag = teamNumber;

	logDebug(F("New record"));
	logDebug(F("Team number: %d"), teamNumber);

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
			logDebug(F("Autoreport team"));
			logDebug(F("Team number: %d"), teamNumber);

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
				for (uint8_t i = 0; i < 13; ++i) {
					if (!addData(ntag_page[i])) return;
				}
				sendData();
			}
			else
				addLastError(PROCESS_SEND_AUTOREPORT); //CARD PROCESSING: error sending autoreport
		}
	}
}

// обработка входящих команд
bool readUart(Stream& SerialPort) {
	while (SerialPort.available()) {
		int c = SerialPort.read();
		if (c == -1) { // can't read stream
			logError(F("UART read error"));
			uartBufferPosition = 0;
			receivingData = false;
			return false;
		}

		// 0 byte = FE
		if (uartBufferPosition == HEADER_BYTE && c == 0xfe) {
			logDebug(F("Recevied inital byte"));
			receivingData = true;
			uartBuffer[uartBufferPosition] = uint8_t(c);
			++uartBufferPosition;
			receiveStartTime = millis(); // refresh timeout
		}
		// 1st byte = ID, Station number, Command, Length and Data
		else if (receivingData) {
			uartBuffer[uartBufferPosition] = uint8_t(c);
			if (uartBufferPosition == DATA_LENGTH_LOW_BYTE) {
				uint16_t length = readUInt16(uartBuffer + DATA_LENGTH_HIGH_BYTE);

				// incorrect length
				if (length > MAX_PAKET_LENGTH - DATA_START_BYTE) {
					logError(F("Incorrect length"));

					uartBufferPosition = 0;
					receivingData = false;
					g_notifier.notifyError();
					addLastError(UART_PACKET_LENGTH);
					sendError(PARSE_PACKET_LENGTH_ERROR, uartBuffer[COMMAND_BYTE] + 0x10);
					return false;
				}
			}

			// packet is received
			if (uartBufferPosition >= DATA_START_BYTE + readUInt16(uartBuffer + DATA_LENGTH_HIGH_BYTE)) {
				logDebugHexArray(F("Packet received"), uartBuffer, uartBufferPosition + 1);

				// crc matching
				logDebugHex(F("Expected CRC"), crcCalc(uartBuffer, PACKET_ID_BYTE, uartBufferPosition - 1));
				if (uartBuffer[uartBufferPosition] == crcCalc(uartBuffer, PACKET_ID_BYTE, uartBufferPosition - 1))
				{
					// incorrect station number
					if (uartBuffer[STATION_NUMBER_BYTE] != stationNumber
						&& uartBuffer[COMMAND_BYTE] != COMMAND_GET_STATUS
						&& uartBuffer[COMMAND_BYTE] != COMMAND_GET_CONFIG)
					{
						logError(F("Incorrect station number"));
						uartBufferPosition = 0;
						receivingData = false;
						g_notifier.notifyError();
						addLastError(UART_WRONG_STATION);
						sendError(WRONG_STATION, uartBuffer[COMMAND_BYTE] + 0x10);
						return false;
					}

					uartBufferPosition = 0;
					receivingData = false;
					return true;
				}
				else { // CRC not correct
					logError(F("Incorrect CRC"));
					uartBufferPosition = 0;
					receivingData = false;
					g_notifier.notifyError();
					addLastError(UART_CRC);
					return false;
				}
			}
			++uartBufferPosition;
		}
		else {
			logError(F("Unexpected byte"));
			receivingData = false;
			uartBufferPosition = 0;
			addLastError(UART_UNEXPECTED_BYTE);
		}

		yield();
	}
	return false;
}

#pragma region Commands processing

// поиск функции
void executeCommand() {
	logDebugHex(F("Command"), uartBuffer[COMMAND_BYTE]);
	bool errorLengthFlag = false;
	uint16_t data_length = readUInt16(uartBuffer + DATA_LENGTH_HIGH_BYTE);
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
	case COMMAND_SET_BT_NAME:
		if (data_length >= DATA_LENGTH_SET_BT_NAME) setBtName();
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
	case COMMAND_SET_AUTH:
		if (data_length == DATA_LENGTH_SET_AUTH) setAuth();
		else errorLengthFlag = true;
		break;
	case COMMAND_SET_PWD:
		if (data_length == DATA_LENGTH_SET_PWD) setAuthPwd();
		else errorLengthFlag = true;
		break;
	case COMMAND_SET_PACK:
		if (data_length == DATA_LENGTH_SET_PACK) setAuthPack();
		else errorLengthFlag = true;
		break;
	case COMMAND_UNLOCK_CHIP:
		if (data_length == DATA_LENGTH_UNLOCK_CHIP) unlockChip();
		else errorLengthFlag = true;
		break;
	case COMMAND_GET_AUTH:
		if (data_length == DATA_LENGTH_GET_AUTH) getAuth();
		else errorLengthFlag = true;
		break;
	case COMMAND_GET_BTNAME:
		if (data_length == DATA_LENGTH_GET_BTNAME) getBtName();
		else errorLengthFlag = true;
		break;
	default:
		sendError(WRONG_COMMAND, uartBuffer[COMMAND_BYTE] + 0x10);
		break;
	}

	uartBufferPosition = 0;
	if (errorLengthFlag)
	{
		logError(F("Incorrect data length"));
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
	if (!addData(OK))
		return;

	sendData();
}

// обновление времени на станции
void setTime()
{
	logDebugDateTime(uartBuffer + DATA_START_BYTE);

	// 0-3: дата и время в unixtime

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
void resetStation() {
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

	teamFlashSize = EEPROM_TEAM_BLOCK_SIZE_DEFAULT;
	preferences.putUInt(EEPROM_TEAM_BLOCK_SIZE, teamFlashSize);

	lastTimeChecked = 0;
	totalChipsChecked = 0;

	if (!FFat.format())
	{
		String teamFile;
		teamFile.reserve(teamFilePrefix.length() + 5);
		for (int teamNumber = 0; teamNumber <= maxTeamNumber; ++teamNumber)
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

			yield();
		}
	}

	init_package(REPLY_RESET_STATION);

	// 0: код ошибки
	if (!addData(OK)) return;
	sendData();
	delay(100);
	setupVars();
}

// выдает статус: время на станции, номер станции, номер режима, число отметок, время последней страницы
void getStatus()
{
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
	DS3231_get(&systemTime);

	digitalWrite(GREEN_LED_PIN, HIGH);

	// Look for new cards
	if (!RfidStart())
	{
		RfidEnd();
		digitalWrite(GREEN_LED_PIN, LOW);
		sendError(NO_CHIP, REPLY_INIT_CHIP);
		return;
	}

	// читаем блок информации
	if (!ntagRead4pages(PAGE_CHIP_SYS2))
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

	if (AuthEnabled)
	{
		if (!ntagSetPassword(AuthPwd, AuthPack, true, false, 0, 0))
		{
			RfidEnd();
			digitalWrite(GREEN_LED_PIN, LOW);
			sendError(CHIP_SETPASS_ERROR, REPLY_INIT_CHIP);
			return;
		}
	}

	// заполняем чип 0x00
	uint8_t dataBlock[4] = { 0,0,0,0 };
	for (uint8_t page = PAGE_CHIP_NUM; page < tagMaxPage; ++page)
	{
		if (!ntagWritePage(dataBlock, page, true, false))
		{
			RfidEnd();
			digitalWrite(GREEN_LED_PIN, LOW);
			sendError(RFID_WRITE_ERROR, REPLY_INIT_CHIP);

			return;
		}

		yield();
	}

	// 0-1: номер команды
	// 2-3 : маска участников

	// пишем данные на чип
	// номер команды, тип чипа, версия прошивки станции
	dataBlock[0] = uartBuffer[DATA_START_BYTE];
	dataBlock[1] = uartBuffer[DATA_START_BYTE + 1];
	dataBlock[2] = chipType;
	dataBlock[3] = FW_VERSION;
	if (!ntagWritePage(dataBlock, PAGE_CHIP_NUM, true, false))
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
	if (!ntagWritePage(dataBlock, PAGE_INIT_TIME, true, false))
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
	if (!ntagWritePage(dataBlock, PAGE_TEAM_MASK, true, false))
	{
		RfidEnd();
		digitalWrite(GREEN_LED_PIN, LOW);
		sendError(RFID_WRITE_ERROR, REPLY_INIT_CHIP);

		return;
	}

	// получаем UID чипа
	if (!ntagRead4pages(PAGE_UID1))
	{
		RfidEnd();
		digitalWrite(GREEN_LED_PIN, LOW);
		sendError(RFID_READ_ERROR, REPLY_INIT_CHIP);
		return;
	}

	RfidEnd();
	digitalWrite(GREEN_LED_PIN, LOW);

	init_package(REPLY_INIT_CHIP);
	if (!addData(OK))
		return;

	// добавляем в ответ время инициализации
	bool flag = true;
	flag &= addData((systemTime.unixtime & 0xFF000000) >> 24);
	flag &= addData((systemTime.unixtime & 0x00FF0000) >> 16);
	flag &= addData((systemTime.unixtime & 0x0000FF00) >> 8);
	flag &= addData(systemTime.unixtime & 0x000000FF);
	if (!flag) return;

	// добавляем в ответ UID
	for (uint8_t i = 0; i <= 7; ++i)
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
	for (uint8_t i = 0; i < LAST_TEAMS_LENGTH * 2; ++i)
	{
		//stop if command is empty
		// if (lastTeams[i] + lastTeams[i + 1] == 0) break;
		if (!addData(lastTeams[i])) return;
		++i;
		if (!addData(lastTeams[i])) return;
	}
	sendData();
	for (uint8_t i = 0; i < LAST_TEAMS_LENGTH * 2; ++i) lastTeams[i] = 0;
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
	for (uint8_t i = 0; i < 13; ++i)
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

	// Look for new cards
	if (!RfidStart())
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
	if (!ntagRead4pages(PAGE_UID1))
	{
		RfidEnd();
		digitalWrite(GREEN_LED_PIN, LOW);
		sendError(RFID_READ_ERROR, REPLY_READ_CARD_PAGE);
		return;
	}

	// пишем UID в буфер ответа
	for (uint8_t i = 0; i <= 7; ++i)
	{
		if (!addData(ntag_page[i]))
		{
			RfidEnd();
			digitalWrite(GREEN_LED_PIN, LOW);
			return;
		}
		yield();
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
		for (uint8_t i = 0; i < n; ++i)
		{
			for (uint8_t j = 0; j < 4; ++j)
			{
				if (!addData(ntag_page[i * 4 + j]))
				{
					RfidEnd();
					digitalWrite(GREEN_LED_PIN, LOW);
					return;
				}

				yield();
			}

			++pageFrom;
			yield();
		}

		yield();
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
	// Look for new cards
	if (!RfidStart())
	{
		RfidEnd();
		lastTeamFlag = 0;
		sendError(NO_CHIP, REPLY_UPDATE_TEAM_MASK);
		return;
	}

	// читаем блок информации
	if (!ntagRead4pages(PAGE_UID1))
	{
		RfidEnd();
		sendError(RFID_READ_ERROR, REPLY_UPDATE_TEAM_MASK);
		return;
	}

	//неправильный тип чипа
	if (ntag_page[14] != chipType)
	{
		RfidEnd();
		logError(F("Incorrect chip type"));
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
		logError(F("Incorrect chip"));
		sendError(WRONG_CHIP_TYPE, REPLY_UPDATE_TEAM_MASK);
		return;
	}*/

	// чип от другой прошивки
	if (ntag_page[3] != FW_VERSION)
	{
		RfidEnd();
		logError(F("Incorrect chip firmware"));
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
		logError(F("Outdated chip"));
		sendError(LOW_INIT_TIME, REPLY_UPDATE_TEAM_MASK);
		return;
	}

	uint16_t chipNum = (ntag_page[0] << 8) + ntag_page[1];

	// Если номер чипа =0 или >maxTeamNumber
	if (chipNum < 1 || chipNum > maxTeamNumber)
	{
		RfidEnd();
		logError(F("Incorrect chip number"));
		logDebug(F("Chip number"), chipNum);
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
			logDebug(F("Updating mask"));
			digitalWrite(GREEN_LED_PIN, HIGH);
			uint8_t dataBlock[4] = { newTeamMask[6], newTeamMask[7], ntag_page[10], ntag_page[11] };
			if (!ntagWritePage(dataBlock, PAGE_TEAM_MASK, true, false))
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
	// Look for new cards
	if (!RfidStart())
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
	if (!ntagRead4pages(PAGE_UID1))
	{
		RfidEnd();
		digitalWrite(GREEN_LED_PIN, LOW);
		sendError(RFID_READ_ERROR, REPLY_WRITE_CARD_PAGE);

		return;
	}
	bool flag = false;
	for (uint8_t i = 0; i <= 7; ++i)
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

	if (!ntagWritePage(dataBlock, uartBuffer[DATA_START_BYTE + 8], true, false))
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
	if (!addData(OK))
		return;

	sendData();
}

void readFlash() {
	// 0-3: start address for reading
	uint32_t startAddress = readUInt32(uartBuffer + DATA_START_BYTE);

	// 4-5: Number of bytes to read
	uint16_t length = readUInt16(uartBuffer + DATA_START_BYTE + 4);

	// SM is hardcoded to maximum of 60 * 4 = 240 bytes
	// getTeamRecord provides up to 255 records * 4 = 1020 bytes
	if (length > static_cast<uint16_t>(MAX_PAKET_LENGTH)) {
		sendError(FLASH_READ_ERROR, REPLY_READ_FLASH);
		return;
	}

	logDebug(F("Flash read"), startAddress, F("/"), length);

	init_package(REPLY_READ_FLASH);

	// 0: код ошибки
	// 1-4: адрес начала чтения
	// 5-n: данные из флэша
	bool flag = true;
	flag &= addData(OK);
	flag &= addData((startAddress & 0xFF000000) >> 24);
	flag &= addData((startAddress & 0x00FF0000) >> 16);
	flag &= addData((startAddress & 0x0000FF00) >> 8);
	flag &= addData(startAddress & 0x000000FF);
	if (!flag) {
		sendError(FLASH_READ_ERROR, REPLY_READ_FLASH);
		return;
	}

	uint16_t teamNumber = startAddress / teamFlashSize;
	uint32_t teamStartAddress = startAddress % teamFlashSize;

	const String teamFile = teamFilePrefix + String(teamNumber);
	logDebug(F("Team number"), teamNumber);
	logDebug(F("Start address"), teamStartAddress);
	logDebug(F("Team file"), teamFile);

	if (!FFat.exists(teamFile.c_str())) {
		logError(F("Team file does not exist"), teamFile);
		sendError(FLASH_READ_ERROR, REPLY_READ_FLASH);
		return;
	}

	File file = FFat.open(teamFile, FILE_READ);
	if (!file) {
		logError(F("Failed to open file"));
		sendError(FLASH_READ_ERROR, REPLY_READ_FLASH);
		return;
	}

	file.seek(teamStartAddress);
	while (length > 0 && file.available() > 0) {
		if (!addData(file.read())) {
			logError(F("Failed to add data"));
			sendError(FLASH_READ_ERROR, REPLY_READ_FLASH);
			return;
		}

		--length;
		yield();
	}

	file.close();
	sendData();
}

// пишем в флэш
void writeFlash() {
	uint16_t length = readUInt16(uartBuffer + DATA_LENGTH_HIGH_BYTE);

	// 0-3: адрес начала записи
	// 4-n: данные
	uint32_t startAddress = readUInt32(uartBuffer + DATA_START_BYTE);

	init_package(REPLY_WRITE_FLASH);

	uint16_t teamNumber = startAddress / teamFlashSize;
	uint32_t teamFlashOffset = startAddress % teamFlashSize;

	const String teamFile = teamFilePrefix + String(teamNumber);
	File file = FFat.open(teamFile, FILE_WRITE);
	if (!file || !file.seek(teamFlashOffset) || length != file.write(reinterpret_cast<uint8_t*>(uartBuffer[DATA_START_BYTE + 4]), length)) {
		file.close();
		logError(F("Failed to write to flash"));
		sendError(FLASH_WRITE_ERROR, REPLY_WRITE_FLASH);
		return;
	}

	file.close();

	// 0: код ошибки
	// 1-2: кол-во записанных байт (для проверки)
	bool flag = true;
	flag &= addData(OK);
	flag &= addData(length >> 8);
	flag &= addData(length & 0x00ff);
	if (!flag) {
		logError(F("Failed to add data"));
		sendError(FLASH_READ_ERROR, REPLY_READ_FLASH);
		return;
	}

	sendData();
}

// стираем команду с флэша (4096 байт)
void eraseTeamFlash()
{
	uint32_t teamNumber = uartBuffer[DATA_START_BYTE];
	teamNumber <<= 8;
	teamNumber += uartBuffer[DATA_START_BYTE + 1];

	logDebug(F("Erasing team"), teamNumber);

	if (!eraseTeamFromFlash(teamNumber))
	{
		sendError(ERASE_ERROR, REPLY_ERASE_FLASH_SECTOR);
		return;
	}

	init_package(REPLY_ERASE_FLASH_SECTOR);
	// 0: код ошибки
	if (!addData(OK))
		return;

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
	// 12-15: коэффициент пересчета напряжения(float, 4 bytes) - умножить коэффициент на полученное в статусе число и будет температура
	// 16: коэффициент усиления антенны RFID
	// 17-18: размер блока хранения команды
	init_package(REPLY_GET_CONFIG);

	bool flag = true;
	flag &= addData(OK);
	flag &= addData(FW_VERSION);
	flag &= addData(stationMode);
	flag &= addData(chipType);

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

	for (uint8_t i = 0; i < 4; ++i)
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
void setChipType() {
	// 0: тип чипа
	chipType = uartBuffer[DATA_START_BYTE];
	if (!selectChipType(chipType)) {
		logError(F("Wrong chip type"), chipType);
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
void setTeamFlashSize() {
	// 0-1: размер блока
	uint16_t n = readUInt16(uartBuffer + DATA_START_BYTE);
	if (n < 16) {
		logError(F("Wrong team flash size"), n);
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

// обработать запрос на подтверждение BT подключения
void BTConfirmRequestCallback(uint32_t numVal) {
	logDebug(F("The PIN is"), numVal);

	bool confirmedByButton = !digitalRead(0);
	SerialBT.confirmReply(confirmedByButton);

	logDebug(F("Confirmation"), confirmedByButton);
}

// поменять имя BT адаптера
void setBtName() {
	logDebug(F("Set new BT name"));
	uint16_t name_length = readUInt16(uartBuffer + DATA_LENGTH_HIGH_BYTE);
	if (name_length < 1 || name_length > 32) {
		sendError(WRONG_DATA, REPLY_SET_BT_NAME);
		return;
	}

	String btCommand;
	btCommand.reserve(name_length + 1);
	for (uint16_t i = 0; i < name_length; ++i) {
		btCommand += String(static_cast<char>(uartBuffer[DATA_START_BYTE + i]));
	}

	preferences.putString(EEPROM_STATION_NAME, btCommand);
	init_package(REPLY_SET_BT_NAME);
	// 0: код ошибки
	if (!addData(OK)) return;
	sendData();
}

void BTAuthCompleteCallback(boolean success) {
	logDebug(F("Pairing is"), success);
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

	for (uint8_t i = 0; i < 4; ++i)
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
	logDebug(F("Scan teams in flash"));

	// 0-1: номер команды
	uint16_t startNumber = readUInt16(uartBuffer + DATA_START_BYTE);

	logDebug(F("Start from team"), startNumber);

	if (startNumber < 1 || startNumber > maxTeamNumber)
	{
		logError(F("Wrong team number"), startNumber);
		sendError(WRONG_DATA, REPLY_SCAN_TEAMS);
		return;
	}

	init_package(REPLY_SCAN_TEAMS);
	// 0: код ошибки
	if (!addData(OK)) return;

	// 1...: список команд
	//uint8_t data[2];
	for (; startNumber <= maxTeamNumber; ++startNumber)
	{
		logDebug(F("Trying team"), startNumber);

		if (checkTeamExists(startNumber))
		{
			logDebug(F("Found team"), startNumber);

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
	if (!addData(OK))
		return;

	// номера последних ошибок
	uint8_t i = 0;
	while (lastErrors[i] != 0 && i < LAST_ERRORS_LENGTH)
	{
		if (!addData(lastErrors[i]))
			return;

		++i;
		yield();
	}

	sendData();
	for (i = 0; i < LAST_ERRORS_LENGTH; ++i)
		lastErrors[i] = 0;
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

// установка режима авторизации
void setAuth()
{
	// 0: новый режим
	AuthEnabled = uartBuffer[DATA_START_BYTE];
	preferences.putBool(EEPROM_AUTH, AuthEnabled);
	init_package(REPLY_SET_AUTH);

	// 0: код ошибки
	if (!addData(OK)) return;
	sendData();
}

// включение/выключение авторизации
void setAuthPwd()
{
	// 0: новый режим
	AuthPwd[0] = uartBuffer[DATA_START_BYTE];
	AuthPwd[1] = uartBuffer[DATA_START_BYTE + 1];
	AuthPwd[2] = uartBuffer[DATA_START_BYTE + 2];
	AuthPwd[3] = uartBuffer[DATA_START_BYTE + 3];
	preferences.putBytes(EEPROM_AUTH_PWD, AuthPwd, 4);
	init_package(REPLY_SET_PWD);

	// 0: код ошибки
	if (!addData(OK)) return;
	sendData();
}

// установка ключа авторизации
void setAuthPack()
{
	// 0: новый режим
	AuthPack[0] = uartBuffer[DATA_START_BYTE];
	AuthPack[1] = uartBuffer[DATA_START_BYTE + 1];
	preferences.putBytes(EEPROM_AUTH_PACK, AuthPack, 2);
	init_package(REPLY_SET_PACK);

	// 0: код ошибки
	if (!addData(OK)) return;
	sendData();
}

// разблокировка чипа
void unlockChip()
{
	init_package(REPLY_UNLOCK_CHIP);

	uint8_t defaultPwd[4] = { 0xff,0xff,0xff,0xff }; // ключ авторизации RFID
	uint8_t defaultPack[2] = { 0,0 }; // ответ авторизации RFID

	RfidStart();
	// Пытаемся авторизоваться с текущим и стандартным ключами
	bool result = ntagAuth(AuthPwd, AuthPack, true);
	if (!result)
		result = ntagAuth(defaultPwd, defaultPack, true);

	if (!ntagRemovePassword(defaultPwd, defaultPack, true, false, 0, 0xff))
	{
		sendError(CHIP_SETPASS_ERROR, REPLY_UNLOCK_CHIP);

		return;
	}

	// 0: код ошибки
	if (!addData(OK)) return;
	sendData();
}

// получить состояние авторизации
void getAuth()
{
	init_package(REPLY_GET_AUTH);

	// 0: код ошибки
	if (!addData(OK)) return;

	// 1: режим авторизации
	if (!addData(AuthEnabled)) return;

	// 2-5: пароль авторизации
	if (!addData(AuthPwd[0])) return;
	if (!addData(AuthPwd[1])) return;
	if (!addData(AuthPwd[2])) return;
	if (!addData(AuthPwd[3])) return;

	// 6-7: ответ авторизации
	if (!addData(AuthPack[0])) return;
	if (!addData(AuthPack[1])) return;

	sendData();
}

// получить имя Bluetooth адаптера
void getBtName()
{
	init_package(REPLY_GET_BTNAME);

	String btName = preferences.getString(EEPROM_STATION_NAME, String(""));
	// 0: код ошибки
	if (!addData(OK)) return;

	// BluetoothName
	for (int i = 0; i < btName.length(); ++i)
		if (!addData(btName[i])) return;

	sendData();
}
#pragma endregion

#pragma region Internal functions
// заполнить буфер смены маски
void saveNewMask()
{
	for (uint8_t i = 0; i < 8; ++i)
	{
		newTeamMask[i] = uartBuffer[DATA_START_BYTE + i];
	}
	logDebugHexArray(F("New mask"), newTeamMask, 8);
}

// очистить буфер смены маски
void clearNewMask()
{
	for (uint8_t i = 0; i < 8; ++i)
		newTeamMask[i] = 0;

	logDebug(F("Mask cleared"));
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
		yield();
	}
	return AverageValue;
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
	logDebugHex(F("Adding to buffer"), data);

	if (uartBufferPosition >= uint16_t(MAX_PAKET_LENGTH - 1))
	{
		logError(F("Adding to buffer failed"));

		sendError(BUFFER_OVERFLOW);
		return false;
	}

	uartBuffer[uartBufferPosition] = data;
	++uartBufferPosition;

	return true;
}

// передача пакета данных
void sendData()
{
	uartBuffer[DATA_LENGTH_HIGH_BYTE] = (uartBufferPosition - DATA_LENGTH_LOW_BYTE - 1) >> 8;
	uartBuffer[DATA_LENGTH_LOW_BYTE] = (uartBufferPosition - DATA_LENGTH_LOW_BYTE - 1) & 0x0FF;
	uartBuffer[uartBufferPosition] = crcCalc(uartBuffer, PACKET_ID_BYTE, uartBufferPosition - 1);
	++uartBufferPosition;

	logDebug(F("Sending data"));
	logDebugHexArray(F("Data"), uartBuffer, uartBufferPosition);

	Serial.write(uartBuffer, uartBufferPosition);
	if (SerialBT.connected())
		SerialBT.write(uartBuffer, uartBufferPosition);

	uartBufferPosition = 0;
}

bool ntagAuth(uint8_t* pass, uint8_t* pack, bool ignorePack)
{
	logDebug(F("Chip authentication"));
	uint8_t n = 0;
	bool status = 0;
	uint8_t p_Ack[2] = { 0,0 };
	while (!status && n < 3)
	{
		logDebugHexArray(F("PWD"), pass, 4);
		logDebugHexArray(F("PACK"), pack, 2);

#if defined(USE_PN532)
		status = (1 == pn532.ntag2xx_Auth(pass, p_Ack));
#else
		status = (MFRC522::STATUS_OK == MFRC522::StatusCode(mfrc522.PCD_NTAG216_AUTH(pass, p_Ack)));
#endif

		logDebug(F("Status"), status);

		if (status)
		{
			if (ignorePack)
				status = true;
			else if (pack[0] != p_Ack[0] || pack[1] != p_Ack[1])
				status = false;
		}

		++n;
		if (!status)
		{
			RfidStart();
		}

		yield();
	}

	if (!status)
	{
		logError(F("Failed to authenticate chip"));
		return false;
	}

	return true;
}

// запись страницы (4 байта) в чип
bool ntagWritePage(uint8_t* data, uint8_t pageAdr, bool verify, bool forceNoAuth)
{
#if !defined(USE_PN532)
	const uint8_t sizePageNtag = 4;
#endif

	uint8_t n = 0;
	bool status = false;
	while (!status && n < 3)
	{
		logDebug(F("Writing RFID page"), pageAdr);
		logDebugHexArray(F("Data"), data, 4);

#if defined(USE_PN532)
		status = pn532.ntag2xx_WritePage(pageAdr, data);
#else
		status = (MFRC522::STATUS_OK == MFRC522::StatusCode(mfrc522.MIFARE_Ultralight_Write(pageAdr, data, sizePageNtag)));
#endif

		logDebug(F("Status"), status);

		++n;
		if (!status)
		{
			RfidStart();
		}

		yield();
	}

	if (!status)
	{
		logError(F("Failed to write RFID page with status:"), status);
		return false;
	}

	if (verify)
	{
		logDebug(F("Chip write verification started"));
		n = 0;
		uint8_t const buffer_size = 18;
		uint8_t buffer[buffer_size];
		status = false;
		while (!status && n < 3) {
			#if defined(USE_PN532)
				status = pn532.ntag2xx_Read4Pages(pageAdr, buffer);
			#else
				uint8_t size = buffer_size;
				status = (MFRC522::STATUS_OK == MFRC522::StatusCode(mfrc522.MIFARE_Read(pageAdr, buffer, &size)));
			#endif

			++n;
			yield();
		}

		if (!status)
		{
			logError(F("Chip read failed"));
			return false;
		}


		for (uint8_t i = 0; i < 4; ++i)
		{
			logDebug(F("Buffer"), buffer[i], F("?="), data[i]);
			if (buffer[i] != data[i])
			{
				logError(F("Failed to verify chip"));
				return false;
			}

			yield();
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
		logDebug(F("Card reading"));

#if defined(USE_PN532)
		status = pn532.ntag2xx_Read4Pages(pageAdr, buffer);
#else
		status = (MFRC522::STATUS_OK == mfrc522.MIFARE_Read(pageAdr, buffer, &size));
#endif
		if (!status)
		{
			logError(F("Card read failed"));
			RfidStart();
			logDebug(F("Chip re-initialized"));
		}

		++n;
		yield();
	}

	if (!status)
	{
		logError(F("Card read failed"));
		return false;
	}

	for (uint8_t i = 0; i < 16; ++i)
	{
		ntag_page[i] = buffer[i];
	}
	logDebug(F("Card reading done"));

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

	if (!ntagWritePage(dataBlock, newPage, true, false))
	{
		return false;
	}
	return true;
}

// desired password (default value is 0xFF FF FF FF)
// desired password acknowledge (default value is 0x00 00)
// try to authenticate first
// false = PWD_AUTH for write only, true = PWD_AUTH for read and write
// value between 0 and 7
// first page to be protected, set to a value between 0 and 37 for NTAG212
bool ntagSetPassword(uint8_t* pass, uint8_t* pack, bool noAuth, bool readAndWrite, uint8_t authlim, uint8_t startPage)
{
	RfidStart();

	if (!ntagRead4pages(0))
		return false;

	if (ntag_page[14] != chipType)
		return false;

	//Set PWD (page 133) to your desired password (default value is 0xFF FF FF FF).
	if (!ntagWritePage(pass, tagMaxPage + PAGE_PWD, false, noAuth))
		return false;

	//Set PACK (page 140, bytes 0-1) to your desired password acknowledge (default value is 0x00 00).
	uint8_t pwd[4] = { pack[0],pack[1],0,0 };
	if (!ntagWritePage(pwd, tagMaxPage + PAGE_PACK, false, noAuth))
		return false;

	//Set AUTHLIM (page 132, byte 0, bits 2-0) to the maximum number of failed password verification attempts
	//(setting this value to 0 will permit an unlimited number of PWD_AUTH attempts).
	//Set PROT (page 132, byte 0, bit 7) to your desired value:
	//(0 = PWD_AUTH in needed only for write access, 1 = PWD_AUTH is necessary for read and write access).
	if (!ntagRead4pages(tagMaxPage + PAGE_CFG1))
		return false;

	//var readAndWrite = false;  // false = PWD_AUTH for write only, true = PWD_AUTH for read and write
	//int authlim = 0; // value between 0 and 7
	//keep old value for bytes 1-3, you could also simply set them to 0 as they are currently RFU and must always be written as 0
	//(response[1], response[2], response[3] will contain 0 too as they contain the read RFU value)
	uint8_t cfg1[4] = {
		(byte)((ntag_page[0] & 0x78) | (readAndWrite ? 0x080 : 0x00) | (authlim & 0x07)),
		ntag_page[1],
		ntag_page[2],
		ntag_page[3]
	};

	if (!ntagWritePage(cfg1, tagMaxPage + PAGE_CFG1, true, noAuth))
		return false;

	//Set AUTH0 (page 131, byte 3) to the first page that should require password authentication.
	if (!ntagRead4pages(tagMaxPage + PAGE_CFG0))
		return false;

	// keep old value for byte 0,1,2
	uint8_t cfg0[4] = { ntag_page[0], ntag_page[1], ntag_page[2], (byte)(startPage & 0x0ff) };
	if (!ntagWritePage(cfg0, tagMaxPage + PAGE_CFG0, true, noAuth))
		return false;

	return true;
}

bool ntagRemovePassword(uint8_t* pass, uint8_t* pack, bool noAuth, bool readAndWrite, uint8_t authlim, uint8_t startPage)
{
	RfidStart();

	if (!ntagRead4pages(0))
		return false;

	if (ntag_page[14] != chipType)
		return false;

	//Set AUTH0 (page 131, byte 3) to the first page that should require password authentication.
	if (!ntagRead4pages(tagMaxPage + PAGE_CFG0))
		return false;

	// keep old value for byte 0,1,2
	uint8_t cfg0[4] = { ntag_page[0], ntag_page[1], ntag_page[2], (byte)(startPage & 0x0ff) };
	if (!ntagWritePage(cfg0, tagMaxPage + PAGE_CFG0, true, noAuth))
		return false;

	//Set PWD (page 133) to your desired password (default value is 0xFF FF FF FF).
	if (!ntagWritePage(pass, tagMaxPage + PAGE_PWD, false, noAuth))
		return false;

	//Set PACK (page 140, bytes 0-1) to your desired password acknowledge (default value is 0x00 00).
	uint8_t pwd[4] = { pack[0],pack[1],0,0 };
	if (!ntagWritePage(pwd, tagMaxPage + PAGE_PACK, false, noAuth))
		return false;

	//Set AUTHLIM (page 132, byte 0, bits 2-0) to the maximum number of failed password verification attempts
	//(setting this value to 0 will permit an unlimited number of PWD_AUTH attempts).
	//Set PROT (page 132, byte 0, bit 7) to your desired value
	//(0 = PWD_AUTH in needed only for write access, 1 = PWD_AUTH is necessary for read and write access).
	if (!ntagRead4pages(tagMaxPage + PAGE_CFG1))
		return false;

	//var readAndWrite = false;  // false = PWD_AUTH for write only, true = PWD_AUTH for read and write
	//int authlim = 0; // value between 0 and 7
	//keep old value for bytes 1-3, you could also simply set them to 0 as they are currently RFU and must always be written as 0
	//(response[1], response[2], response[3] will contain 0 too as they contain the read RFU value)
	uint8_t cfg1[4] = {
		(byte)((ntag_page[0] & 0x78) | (readAndWrite ? 0x080 : 0x00) | (authlim & 0x07)),
		ntag_page[1],
		ntag_page[2],
		ntag_page[3]
	};

	if (!ntagWritePage(cfg1, tagMaxPage + PAGE_CFG1, true, noAuth))
		return false;

	return true;
}

// Поиск пустой страницы на чипе.
int findNewPage()
{
	uint8_t page = PAGE_DATA_START;
	while (page < tagMaxPage)
	{
		if (!ntagRead4pages(page))
		{
			logError(F("Can't read chip"));
			// chip read error
			return 0;
		}
		for (uint8_t n = 0; n < 4; ++n)
		{
			// chip was checked by another station with the same number
			if (stationMode == MODE_START_KP && ntag_page[n * 4] == stationNumber)
			{
				logError(F("Chip checked already"));
				return -1;
			}

			// free page found
			if (ntag_page[n * 4] == 0 || (stationMode == MODE_FINISH_KP && ntag_page[n * 4] == stationNumber))
			{
				return page;
			}

			++page;
		}

		yield();
	}

	// чип заполнен
	return tagMaxPage;
}

bool checkTeamExists(uint16_t teamNumber)
{
	String teamFile = teamFilePrefix + String(teamNumber);
	return FFat.exists(teamFile);
}

// пишем дамп чипа во флэш
bool writeDumpToFlash(uint16_t teamNumber, uint32_t checkTime, uint32_t initTime, uint16_t mask)
{
	// адрес хранения в каталоге
	String teamFile = teamFilePrefix + String(teamNumber);

	logDebug(F("Write to flash address"), teamNumber);

	if (checkTeamExists(teamNumber))
	{
		// если режим финишной станции, то надо переписать содержимое
		if (stationMode == MODE_FINISH_KP)
		{
			logDebug(F("Erasing team"), teamNumber);

			if (!eraseTeamFromFlash(teamNumber))
			{
				logError(F("Failed to erase team"));
				return false;
			}
		}
		else
		{
			logError(F("Team already saved"));
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
		logError(F("Failed to open file"));
		logDebug(F("Team file"), teamFile);
		return false;
	}

	if (file.write(basic_record, 16) != 16)
	{
		// append file to team_file_size
		file.close();
		logError(F("Failed to write basics to file"));
		return false;
	}

	logDebug(F("Basics written"));

	// copy card content to flash. все страницы не начинающиеся с 0
	uint8_t checkCount = 0;
	uint8_t block = 0;
	bool endOrRecords = false;
	while (block < tagMaxPage && !endOrRecords) {
		logDebug(F("Reading 4 pages from chip"), block);

		if (!ntagRead4pages(block)) {
			file.close();
			logError(F("Failed to read 4 pages from chip"));
			return false;
		}

		// 4 pages in one read block
		for (uint8_t i = 0; i < 4; ++i) {
			if (block < PAGE_DATA_START || ntag_page[i * 4] > 0) {
				logDebugHexArray(F("Writing to flash"), ntag_page, 16);
				if (!file.write(&ntag_page[0 + i * 4], 4)) {
					file.close();
					return false;
				}

				++checkCount;
			}
			else {
				logDebug(F("Chip last block"), block + i);
				block = tagMaxPage;
				endOrRecords = true;
				break;
			}
		}

		block += 4;
		yield();
	}

	// add dump pages number
	if (!file.seek(12) || !file.write(checkCount)) {
		file.close();
		logError(F("Failed to write check count"));
		return false;
	}

	file.close();

	return true;
}

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
	logDebug(F("Refreshing chip counter"));
	uint16_t chips = 0;
	char data[12];
	for (uint16_t i = 1; i <= maxTeamNumber; ++i)
	{
		String teamFile = "/team" + String(i);
		File file = FFat.open(teamFile, FILE_READ);
		if (!file)
			continue;

		file.readBytes(data, 12);
		if (data[0] != 0xff && data[1] != 0xff)
		{
			++chips;
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

			logDebug(F("Checked chip"), i);
		}

		file.close();
		yield();
	}

	logDebug(F("Checked chip counter"), chips);
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
	if (lastTeams[0] == uint8_t(teamNumber >> 8) && lastTeams[1] == uint8_t(teamNumber))
		return;

	for (uint8_t i = LAST_TEAMS_LENGTH * 2 - 1; i > 1; i = i - 2)
	{
		lastTeams[i] = lastTeams[i - 2];
		lastTeams[i - 1] = lastTeams[i - 3];
	}

	lastTeams[0] = uint8_t(teamNumber >> 8);
	lastTeams[1] = uint8_t(teamNumber);

	if (!already_checked)
		++totalChipsChecked;
}

// добавляем код ошибки в буфер последних ошибок
void addLastError(uint8_t errorCode)
{
	for (uint8_t i = LAST_ERRORS_LENGTH - 1; i > 0; i--)
		lastErrors[i] = lastErrors[i - 1];

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

		++dataStart;
		yield();
	}

	return crc;
}

void floatToByte(uint8_t* bytes, float f)
{
	uint16_t length = sizeof(float);
	for (uint16_t i = 0; i < length; ++i)
		bytes[i] = ((uint8_t*)&f)[i];
}

// check chip type consistence
bool selectChipType(uint8_t type)
{
	if (type == NTAG213_ID) //NTAG213
	{
		chipType = NTAG213_ID;
		tagMaxPage = NTAG213_MAX_PAGE;
	}
	else if (type == NTAG216_ID) //NTAG216
	{
		chipType = NTAG216_ID;
		tagMaxPage = NTAG216_MAX_PAGE;
	}
	else if (type == NTAG215_ID)//NTAG215
	{
		chipType = NTAG215_ID;
		tagMaxPage = NTAG215_MAX_PAGE;
	}
	else
		return false;

	return true;
}

void checkBatteryLevel() {
	if (batteryLimit == 0) return;

	batteryLevel = (batteryLevel + getBatteryLevel()) / 2;
	if (static_cast<float>(batteryLevel) * voltageCoeff <= batteryLimit) {
		if (batteryAlarmCount > BATTERY_ALARM_COUNT) {
			logError(F("Battery is low!"));
			batteryAlarmCount = 0;
			g_notifier.notifyError();
			addLastError(POWER_UNDERVOLTAGE);
			return;
		}

		++batteryAlarmCount;
		return;
	}

	batteryAlarmCount = 0;
}

void checkClockIsRunning() {
	if (millis() > nextClockCheck) {
		uint32_t currentMillis = millis();
		uint32_t diffSystemClock = (currentMillis - lastSystemClock) / 1000;

		struct ts systemTime;
		DS3231_get(&systemTime);
		uint32_t diffExternalClock = systemTime.unixtime - lastExternalClock;

		if (abs(long(diffSystemClock - diffExternalClock)) > 2) {
			logError(F("External clock is not working."));
			g_notifier.notifyError();
			addLastError(CLOCK_ERROR);
		}

		lastSystemClock = currentMillis;
		lastExternalClock = systemTime.unixtime;
		nextClockCheck = currentMillis + RTC_ALARM_DELAY;
	}
}

#ifdef DEBUG
void listDir(const char* dirname, uint8_t levels)
{
	Serial.printf("Listing directory: %s\r\n", dirname);

	File root = FFat.open(dirname);
	if (!root)
	{
		Serial.println("- failed to open directory");
		return;
	}

	if (!root.isDirectory()) {
		Serial.println(" - not a directory");
		return;
	}

	File file = root.openNextFile();
	while (file)
	{
		if (file.isDirectory())
		{
			Serial.print("  DIR : ");
			Serial.println(file.name());
			if (levels)
			{
				listDir(file.path(), levels - 1);
			}
		}
		else {
			Serial.print("  FILE: ");
			Serial.print(file.name());
			Serial.print("\tSIZE: ");
			Serial.println(file.size());
		}
		file = root.openNextFile();
	}
}
#endif
#pragma endregion
