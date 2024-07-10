# DP-trolling motor explained
<br>

- Dynamic positioning on sea with GPS based electrical trolling motor
- Android phone app (MIT app inventor) is your remote control "DP_MAPA_2S_V121123_TEST.apk"
- Bluetooth connection with Android phone APP: press "Connect BT" and pick the BT link of your BLE connected to MEGA
- Before starting you must send PWM maximum to Mega, you do that by adjusting the slider in app (0-255 PWM) and press "Send Motor Speed PWM". With slider you can adjust the power of motor depending on wind speed, sea stream, battery voltage. It limits maximum PWM output. Without that you will not move since initial value is zero.
- You can get current GPS position from GPS module on MEGA and store it in your Android APP and Arduino MEGA: press "1) Get GPS data" (getting coordinates from Arduino GPS and store it to data base, only single point), then press "2) Send GPS data" (it sends it back to Mega), press "3) Set way" and "Go to Waypoint" to start. 
- You can send last GPS position single point stored to Android APP to Mega by pressing "2) Send GPS data", press "3) Set way" and "Go to Waypoint" to start. 
- You can pick the point on the Open-source map in Android APP and send it to MEGA: pick the spot on the map by long pressing on the map and long press "2) Send GPS data", press "3) Set way" and "Go to Waypoint" to start. 
- You can enter Latitude and Longitude as you wish and send it to Mega: clear all, hit any spot on the map, you will get LAT and LONG, you manually correct coordinates to desired and long press "2) Send GPS data". Again, after that you must press "3) Set Way" and "GO to Waypoint".
- You can use it as position holding on sea like electric anchorage "Spot Lock" like Minn Kota, or similar: With GPS data send to Mega, you press "3) set Waypoint" and  "Go to waypoint"
you can learn the track and fallow that track in loops: press "Learn track" (all waypoints must be deleted before by long pressing CLEAR ALL), when you finished learning track long press "STOP", and press "Start_track". 
- You can go in circle by entering the radius of circle: enter radius of circle (min. 10 m) in blank field on left side of "Send_R" button, press "Send_R" and Arduino get the desired circle radius that trolling motor will fallow by going thru several GPS waypoints. Now you can single press "Circle(Spiral)" to go in circle.
- You can go in spiral: long press "Circle(Spiral)", similar like circle mode but each circle the diameter will increment
- You can use JOG function in combination with Android phone properly aligned with the boat that can sense the orientation of the boat: press Jog and then arrows will appear. Single press desired arow to move 5m in desired direction. By long pressing any arrow, you go back out of Jog mode. For Jog mode Android phone compass must be active and calibrated and mobile phone must be fixed on the stand to be aligned with the boat heading. 
- You also can activate trolling motor manually by single press "Left" (rotate left for few degrees), "Right" (rotate right for few degrees) "Forward" (activate trolling motor, you must send motor speed PWM), long press "STOP"; with long press "Left" or "Right" the gearbox rotate until you press "STOP" in that direction

 
# The hardware I used
<br>
- GPS sensor: NEO-6 or higher; connected on Serial 2 (RX2,TX2)
- Digital compass with accelerometer for tilt compensation: LSM303DHLC or LSM303AGR; connected to I2C (SCL, SDA) 
- Bluetooth module: HC-05 (HC-06); connected to Serial 1 (RX1, TX1) 
- STEPPER MOTOR (drives the gearbox that rotate trolling motor left/right): NEMA17 17HS19-2004S1 (fit the 3D printed case); connected to stepper motor driver DRV8825
- STEPPER MOTOR DRIVER: DRV8825 or equivalent; connected to digital pins: D2 = DIRECTION, D3 = STEP; D4 = ENABLE
- Arduino mega 2560 
- FYSETC Ext. MOS Module 1.0 (3D Printer Heater Controller mosfet driver 30A with PWM input from Mega; connected to digital pin D7 for PWM signal; powers the troling motor)
- External GPS antenna for better signal
- Electrical trolling motor up to 30A (like Minn Kota Endura C2 30 (30 lbs; 30A), or YAMAHA M12 (its MINN KOTA); for higher you will need stronger motor driver and freewheeling diode)
- 2x DIODE MUR1560 as freewheeling diode (two conected in paralel for frotection of MOSFET motor driver
- Gear box hausing with gears (1:18 ratio) that will acomodate stepper motor that rotates the trolling motor in desired direction (3D printed for 30 mm shaft)


