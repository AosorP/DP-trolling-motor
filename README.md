# DP-trolling-motor
Dynamic positioning on sea with GPS based electrical trolling motor
Android phone app (MIT app inventor) is your remote control
Bluetooth conection with Android phone APP: press "Connect BT" and pick the BT link of your BLE conected to MEGA
Before starting you must send PWM maximum to Mega, you do that by adjusting the slider in app (0-255 PWM) and press "Send Motor Speed PWM". With slider you can adjust the power of motor depending on wind speed, sea stream, batery voltage. It limits maximum PWM outpoot. Without that you will not move since initial walue is zero.
You can get curent GPS position from GPS module on MEGA and store it in your Android APP and Arduino MEGA: press "1) Get GPS data" (geting coordinates from Arduino GPS and store it to data base, only single point), then press "2) Send GPS data" (it sends it back to Mega), press "3) Set way" and "Go to Waypoint" to start. 
You can send last GPS position single point stored to Android APP to Mega by pressing "2) Send GPS data", press "3) Set way" and "Go to Waypoint" to start. 
You can pick the point on the Open source map in Android APP and send it to MEGA: pick the spot on the map by long pressing on the map and long press "2) Send GPS data", press "3) Set way" and "Go to Waypoint" to start. 
You can enter Latitude and Longitude as you wish and send it to Mega: clear all, hit any spot on the map, you will get LAT and LONG, you manually correct coordinates to desired and long press "2) Send GPS data". Again after that you must press "3) Set Way" and "GO to Waypoint".
You can use it as position holding on sea like electric ancorage "Spot Lock" like Minn Kota, or similar: With GPS data send to Mega, you press "3) set Waypoint" and  "Go to waypoint"
you can learn the track and fallow that track in looops: press "Learn track" (all waypoints must be delited before by long pressing CLEAR ALL), when you finished learning track long press "STOP", and press "Start_track". 
You can go in circle by entering the radius of circle: enter radius of circle (min. 10 m) in blank field on left sid of "Send_R" button, press "Send_R" and arduino get the desired circle radius that trolling motor will fallow by going thro several GPS waypoints. Now you can single press "Circle(Spiral)" to go in circle.
You can go in spiral: long press "Circle(Spiral)", similar like circle mode but each circle the diameter will increment
You can use JOG function in combination with Android phone properly aligned with the boat that can sense the orientation of the boat: press Jog and then arrows will appear. Single press desired arow to move 5m in desired direction. By long pressing any arrow, you go back out of Jog mode. For Jog mode Android phone compass must be activ and calibratet and mobile phone must be fixed on the stand to bi aligned with the boat heading. 
You also can activate troling motor manually by single press "Left" (rotate left for few degreas), "Right" (rotate right for few degreas) "Forward" (activate trolling motor, you must send motor speed PWM), long press "STOP"; with long press "Left" or "Right" the gearbox rotate untill you press "STOP" in that direction
The hardware i used:
GPS sensor: NEO-6 or higher; conected on Serial 2 (RX2,TX2)
Digital compass with accelerometeer for tilt compensation: LSM303DHLC or LSM303AGR; conected to I2C (SCL, SDA) 
Bluetooth module: HC-05 (HC-06); conected to Serial 1 (RX1, TX1) 
STEPPER MOTOR (drives the gearbox thet rotate trolling motor left right): NEMA17 17HS19-2004S1 (fit the 3D printed case); conected to stepper motor driver DRV8825
STEPPER MOTOR DRIVER: DRV8825 or equivalent; conected to digital pins: D2 = DIRECTION, D3 = STEP; D4 = ENABLE
Arduino mega 2560 
Fyearfly 3D Printer Heater Controller mosfet driver 30A with PWM imput from Mega; conected to digital pin D7 for PWM signal
External GPS antena for better signal
Electrical trolling motor up to 30A (like Minn Kota endura C30 (30 lbs; 30A), for higher you will need stronger motor driver
Gear box that will roatate the troling motor in desired direction (3D printed for 30 mm shaft)
![image](https://github.com/AosorP/DP-trolling-motor/assets/175119812/9fe04b67-ab1d-419d-b58a-dfe3dc6a3c02)

