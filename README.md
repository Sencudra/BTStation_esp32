RFID-enabled device for marking the NTAG21x chips for the orienteering competitions.
Refactored to run on the original ESP32 (incompatible with ESP32-C3/-S2/-S3/C6 and others) using DS3231 for the RTC clock module and MFRC522 for the RFID interface.
Based on a https://github.com/jekyll2014/BTStation

Bluetooth PIN is not available out-of-the-box for ESP32 but there are some hints how to enable it:
https://github.com/espressif/arduino-esp32/issues/4566
https://github.com/espressif/arduino-esp32/issues/9599
https://github.com/espressif/arduino-esp32/tree/master/libraries/BluetoothSerial#legacy-pairing-idf-component
https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/esp-idf_component.html

ToDo:
1) Optimize the workflow considering the available memory amount.
2) Consider the energy consumption optimization.


## Notes for CI

### Arduino CLI

1. Update core index from configuration file arduino_cli.yaml
    arduino-cli core update-index --config-file arduino_cli.yaml
2. arduino-cli core install esp32:esp32
3.  arduino-cli compile -b esp32:esp32:esp32
4. arduino-cli lib install --zip-path libs/ds3231.zip --config-file arduino_cli.yaml
5. arduino-cli lib install --zip-path libs/ds3231.zip --config-file arduino_cli.yaml
6. arduino-cli lib install "Adafruit BusIO@1.17.2"
7. For USE_PN532 two more libs should be installed from zip: SPI and MFRC522