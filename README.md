# Physical computing wireless tutorial - CO2 and PM 2.5 monitor with ESP32 and Adafruit IO

Easy Arduino TX. Monitor CO2 and PM 2.5 levels, display on 7-segment displays, and transmit to Adafruit IO

### Setup

You can click the image to play the associated YouTube video

[![Alt text](Assets/13d%20result.jpg)](https://www.youtube.com/watch?v=eIEShsB4q_s)

### Schematic

![](Assets/13d%20schematic.png)

### BOM

<pre>
€  13,00 Adafruit Metro Mini 328 5V 16MHz microcontroller
€  14,00 Adafruit AirLift FeatherWing - ESP32 WiFi
€  53,00 SENSIRION SCD30 CO2 and RH/T Sensor
€  45,00 SENSIRION Particulate Matter Sensor SPS30
€  11,00 Adafruit 0.56" 4-Digit 7-Seg. Disp. w/I2C Backp. - White
€  11,00 Adafruit 0.56" 4-Digit 7-Seg. Disp. w/I2C Backp. - Red
€   1,00 JST ZHR5 power and data cable for SPS30
€   8,00 2 Half-size transparent breadboards
€   6,00 3 Breadboards mini modular black
€   1,00 Jumper cables
€   1,00 2,1mm DC barrel-jack
€   1,00 100 µF 10V el. cap
€  13,00 MEANWELL GS15E-11P1J PSU
€ 178,00
</pre>  

### Useful links

μc https://www.adafruit.com/product/2590  
WiFi module https://www.adafruit.com/product/4264  
Adafruit WiFiNINA library https://github.com/adafruit/WiFiNINA  
Adafruit MQTT library https://github.com/adafruit/Adafruit_MQTT_Library  
Sensor CO2 https://www.sparkfun.com/products/15112  
Sensor CO2 library https://github.com/sparkfun/SparkFun_SCD30_Arduino_Library  
Sensor PM https://www.sparkfun.com/products/15103  
Sensor PM library https://github.com/Sensirion/arduino-sps  
Display https://www.adafruit.com/category/103  
Display library https://github.com/adafruit/Adafruit_LED_Backpack  
