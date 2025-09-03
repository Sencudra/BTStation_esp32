# Общее описание

RFID-enabled device for marking the NTAG21x chips for the orienteering competitions.
Refactored to run on the original ESP32 (incompatible with ESP32-C3/-S2/-S3/C6 and others) using DS3231 for the RTC clock module and MFRC522 for the RFID interface.
Based on a https://github.com/jekyll2014/BTStation

Bluetooth PIN is not available out-of-the-box for ESP32 but there are some hints how to enable it:
https://github.com/espressif/arduino-esp32/issues/4566
https://github.com/espressif/arduino-esp32/issues/9599
https://github.com/espressif/arduino-esp32/tree/master/libraries/BluetoothSerial#legacy-pairing-idf-component
https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/esp-idf_component.html


# Подготовка окружения для разработки

Подготовку окружения и сборку проекта рекомендуется осуществлять через скрипты из папки bin. Для запуска скриптов необходимо установить официальную CLI утилиту под названием [arduino-cli](https://github.com/arduino/arduino-cli). На Windows для запуска скриптов следует использовать любой unix-подобный терминал (Git Bash, Cygwin).

## 1. Установка Arduino CLI

1. Скачайте последнюю версию Arduino CLI:  
   [https://arduino.github.io/arduino-cli/installation/](https://arduino.github.io/arduino-cli/installation/)
2. Если скачивание выполнено с помощью скрипта или вручную, то исполняемый файл необходимо положить в директорию, которая указана в PATH. На linux это может быть /usr/local/bin. На Windows можно выбрать любую или создать свою в настройках PATH в [переменных среды](https://remontka.pro/environment-variables-windows/).
3. Перезагрузите терминал, если он был открыт.
3. Проверьте установку можно командой:  
   ```bash
   arduino-cli version
   ```

## 2. Подготовка окружения

Перед сборкой проекта необходимо выкачать основные компоненты для ESP32 и установить библиотеки.

```bash
./bin/install_requirements_local.sh
```

## 3. Сборка проекта

```bash
./bin/build_firmware_local.sh
```

Артефакты сборки появятся в папке BTStation_ESP32/build

# Установка прошивки без сборки проекта

Установку прошивки на плату можно выполнить с помощью скрипта. Для этого в теле скрипта необходимо указать PORT и проверить путь к папке с прошивкой.

```bash
./bin/upload_local.sh
```
или с помощью программы [Flash Download Tool](https://docs.espressif.com/projects/esp-test-tools/en/latest/esp32/production_stage/tools/flash_download_tool.html).