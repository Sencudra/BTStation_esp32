#pragma once

// Commands

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
void setBtName();
void setBatteryLimit();
void scanTeams();
void getLastErrors();
void setAutoReport();
void setAuth();
void setAuthPwd();
void setAuthPack();
void unlockChip();

// Internal

bool RfidStart();
void RfidEnd();
void processRfidCard();
bool readUart(Stream& SerialPort);
void executeCommand();

void BTConfirmRequestCallback(uint32_t numVal);
void BTAuthCompleteCallback(boolean success);

void saveNewMask();
void clearNewMask();

uint16_t getBatteryLevel();

void init_package(uint8_t);
bool addData(uint8_t);
void sendData();
bool ntagAuth(uint8_t* pass, uint8_t* pack, bool ignorePack);
bool ntagWritePage(uint8_t*, uint8_t, bool verify, bool forceNoAuth);
bool ntagRead4pages(uint8_t);
bool ntagSetPassword(uint8_t* pass, uint8_t* pack, bool noAuth, bool readAndWrite, uint8_t authlim, uint8_t startPage);
bool ntagRemovePassword(uint8_t* pass, uint8_t* pack, bool noAuth, bool readAndWrite, uint8_t authlim, uint8_t startPage);
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
void checkBatteryLevel();
void checkClockIsRunning();

#ifdef DEBUG
void listDir(const char* dirname, uint8_t levels);
#endif
