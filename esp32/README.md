

# Usage

```sh
$ esptool.py --port /dev/ttyUSB0 read_flash 0x0 0x400000 fulldump.bin
esptool.py v3.0
Serial port /dev/ttyUSB0
Connecting........_____....._
Detecting chip type... ESP32
Chip is unknown ESP32 (revision 1)
Features: WiFi, BT, Single Core, 240MHz, VRef calibration in efuse, Coding Scheme 3/4
Crystal is 40MHz
MAC: 40:31:AA:BB:CC:DD
Uploading stub...
Running stub...
Stub running...
4194304 (100 %)
4194304 (100 %)
Read 4194304 bytes at 0x0 in 377.5 seconds (88.9 kbit/s)...
Hard resetting via RTS pin...
```
