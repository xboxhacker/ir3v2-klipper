# IR3V2 Klipper Eddy Current Probe

This project is based on the [BDSensor](https://github.com/markniu/Bed_Distance_sensor/issues), a tiny eddy current bed porbe sensor. 
Do the very small size of the probe I elected to use this for my IR3V2.

The probe is mounted in the fan duct. And wired to an STM32F103 dev board for breaking out the I2C from the BDsensor board to USB for klipper. I used a FK103M2 from Amazon. There may have been GPIO pins on the IR3V2 main board, or maybe on the tool head board. But this was the fastest way to get it running and tested.
