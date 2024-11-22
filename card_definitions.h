#pragma once
// страницы в чипе. 0-7 служебные, 8-... для отметок
#define PAGE_UID1         0
#define PAGE_UID2         1
#define PAGE_CHIP_SYS1    2
#define PAGE_CHIP_SYS2    3 // system[2] + тип_чипа[1] + system[1]
#define PAGE_CHIP_NUM     4 // номер_чипа[2] + тип_чипа[1] + версия_прошивки[1]
#define PAGE_INIT_TIME    5 // время инициализации[4]
#define PAGE_TEAM_MASK    6 // маска команды[2] + resserved[2]
#define PAGE_RESERVED     7 // reserved for future use[4]
#define PAGE_DATA_START   8 // 1st data page: номер КП[1] + время посещения КП[3]

#define PAGE_DYNAMIC_LOCK 0 // dynamic lock bytes, RFUI
#define PAGE_CFG0         1 // CFG 0
#define PAGE_CFG1         2 // CFG 1
#define PAGE_PWD          3 // PWD
#define PAGE_PACK         4 // PACK, RFUI1

#define NTAG213_ID       0x12
#define NTAG213_MARK     213
#define NTAG213_MAX_PAGE 40

#define NTAG215_ID       0x3e
#define NTAG215_MARK     215
#define NTAG215_MAX_PAGE 130

#define NTAG216_ID       0x6d
#define NTAG216_MARK     216
#define NTAG216_MAX_PAGE 226
