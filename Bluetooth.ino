//**************************************************************************************************************************************************
// This procedure reads the serial port - Serial1 - for bluetooth commands being sent from the Android device

void bluetooth()
{
  while (Serial1.available())                                    // Read bluetooth commands over Serial1 // Warning: If an error with Serial1 occurs, make sure Arduino Mega 2560 is Selected
  {
    {
      str = Serial1.readStringUntil('\n');                      // str is the temporary variable for storing the last sring sent over bluetooth from Android device
      //Serial.println(str);                                      // for testing purposes
    }
    //--------------------------------------------------- GPS
    if (str.startsWith("GPS")) {
      String lati = str.substring(3, 12);
      bluelati = lati.toDouble();
      Serial.print("APP latitude: ");
      Serial.println(bluelati, 6);
      Serial1.print("LA ");
      Serial1.print(bluelati, 6);
      Serial1.print(";LO ");
    }

    if (str.startsWith("GPS")) {
      String longi = str.substring(12, 21);
      bluelongi = longi.toDouble();
      Serial.print("APP longitude: ");
      Serial.println(bluelongi, 6);
      Serial1.println(bluelongi, 6);
    }
    //********************************************************* Compass android orientation sensor azimuth walue (boat heading)
    if (str.startsWith("COMP")) {
      String heading = str.substring(4, 7);
      boat_heading = heading.toInt();
      Serial.print("Boat heading = ");
      Serial.print(boat_heading);
      Serial.println("°");
    }
    //*********************************************************
    blueToothVal = (str.toInt());                               //  convert the string 'str' into an integer and assign it to blueToothVal
    Serial.print("BlueTooth Value ");
    Serial.println(blueToothVal);

    // **************************************************************************************************************************************************

    switch (blueToothVal)
    {
      case 1:
        Forward();
        Serial1.println(F("Forward"));
        break;

      case 2:
        Learning_track();
        Serial1.println(F("Learning "));
        delay(500);
        break;

      case 3:
        LeftTurn();
         Serial1.println(F("Left"));
        break;

      case 4:
        RightTurn();
        Serial1.println(F("Right"));
        break;

      case 5:
        StopCar();
         Serial1.println(F("STOP"));
        break;

      case 6:
        setWaypoint();  //Postavlja GPS toćku kao waypoint "Set Way"
        delay(50);
        ac = 0;
        Serial1.println(F("Home Spot set!"));
        delay(800);
        break;

      case 7:
        goWaypoint();   //Pokreće "Go to Waypoint"
        break;

      case 8:
        go_CircleMode();
        break;

      case 9:
        go_SpiralMode();
        break;

      case 10:
        Serial1.println(F("Fallow track ON!"));
        delay(1000);
        go_Track();
        break;

      case 11:
        gpsInfo();
        break;

      case 12:
        Serial1.println(F("Jog Froward 3 m"));
        delay(800);
        JogForward();
        break;

      case 13:
        Serial1.println(F("Jog Revers 3 m"));
        delay(800);
        JogReverse();
        break;

      case 14:
        Serial1.println(F("Jog Left 3 m"));
        delay(800);
        JogLeft();
        break;

      case 15:
        Serial1.println(F("Jog Right 3 m"));
        delay(800);
        JogRight();
        break;

      case 16:
        StopCar();
        clearWaypoints();
        delay(500);
        clearTrack();
        beta = 0.0;
        C = 0;
        ar = 0;
        ax = 1;
        break;
      /*
      case 17:                    // finish with waypoints
        ac = 0;
        Serial1.println(F("Waypoints Complete"));
        break;
      */
      case 18:
        LongLeftTurn();
        Serial1.println(F("Long Left"));
        //StopCar();
        stepAcc = 0;
        break;

      case 19:
        LongRightTurn();
        Serial1.println(F("Long Right"));
        //StopCar();
        stepAcc = 0;
        break;

      case 20:
        storeWaypoints();    //Šalje aktualne NEO GPS-kordinate iz Arduina u MitApp aplikaciju "Get GPS data"
        break;

    } // end of switch case

    // **************************************************************************************************************************************************
    // Slider Value for Speed

    if (blueToothVal)
    {
      Serial.println(blueToothVal);
      if (blueToothVal >= 1000)
      {
        Serial1.print(F("Speed set To:  "));
        Serial1.println(blueToothVal - 1000);
        run_Speed = (blueToothVal - 1000);
        Serial.println();
        Serial.print(F("Run_PWM_Speed "));
        Serial.println(run_Speed);
      }
    }
    // **************************************************************************************************************************************************
    //  Value for Circle radius

    if (blueToothVal)
    {
      Serial.println(blueToothVal);
      if ((blueToothVal >= 110) && (blueToothVal <= 200))
      {
        Serial1.print(F("Circle R.=  "));
        Serial1.println(blueToothVal - 100);
        radius = (blueToothVal - 100);
        Serial.println();
        Serial.print(F("CircleRadius= "));
        Serial.println(radius);
      }
      if (((blueToothVal >= 100) && (blueToothVal < 110)) || ((blueToothVal > 200) && (blueToothVal < 1000)))
      {
        Serial1.println(F("Invalid RADIUS! (10-100m)"));
      }
    }
  }                                                              // end of while loop Serial1 read
  // **************************************************************************************************************************************************
  // if no data from Bluetooth
  if (Serial1.available() < 0)                                 // if an error occurs, confirm that the arduino mega board is selected in the Tools Menu
  {
    Serial1.println(F("No Bluetooth Data "));
  }

} // end of bluetooth procedure

// **************************************************************************************************************************************************

void Forward()
{
  while (true)
  {
    bluetooth();                                                     // Run the Bluetooth procedure to see if there is any data being sent via BT
    if ((blueToothVal == 5) || (blueToothVal == 16)) {
      break;
    }

    unsigned long currentMillis = millis();
    if (((unsigned long)(currentMillis - previousMillis) >= interval_a) && (pwm < run_Speed))  {
      pwm = pwm + a;                     // akceleracija
      analogWrite(runPin, pwm);         // Go Forward "Pali troling motor"
      Serial.print("PWM=");
      Serial.println(pwm);
      previousMillis = currentMillis;
    }
    else if (((unsigned long)(currentMillis - previousMillis) >= interval_b) && (run_Speed < pwm)) {
      pwm = pwm - a;                    //deakceleracija
      analogWrite(runPin, pwm);
      Serial.println(pwm);
      previousMillis = currentMillis;
    }

  }

}

// **************************************************************************************************************************************************
void StopCar()
{
  while (pwm > 0) {
    unsigned long currentMillis = millis();
    if (((unsigned long)(currentMillis - previousMillis) >= interval_b)) {
      pwm = pwm - a;                    //deakceleracija
      analogWrite(runPin, pwm);
      Serial.print("PWM=");
      Serial.println(pwm);
      previousMillis = currentMillis;
    }
  }
  digitalWrite(runPin, LOW);
  noTone(stepPin);
  digitalWrite(enablePin, HIGH);//deaktivira stepper driver A4988

  pwm = 0;
  count = 0;
  PID_p = 0;
  PID_i = 0;
  PID_d = 0;
  lastdelta = 0;
  previousTime = 0;

}

// **************************************************************************************************************************************************
void RightTurn()
{
  digitalWrite(enablePin, LOW); //aktivira stepper driver A4988
  delay(5);
  for (int stepCount = 0 ; stepCount <= 800; stepCount += 1) {  //step count 25*16 za 5°; 50*16 za 10°; 100*16 za 20° getriba 11:100;1:16 microstepping
    digitalWrite(dirPin, LOW);// Stepper motor se okreće u smijeru kazaljke na satu LOW
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(250);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(250);
  }
  digitalWrite(enablePin, HIGH); //deaktivira stepper driver A4988
}

// **************************************************************************************************************************************************
void LongRightTurn()
{
  while (true)
  {
    bluetooth();                                                     // Run the Bluetooth procedure to see if there is any data being sent via BT
    if ((blueToothVal == 5) || (blueToothVal == 16)) {
      break;
    }
    digitalWrite(enablePin, LOW); //aktivira stepper driver A4988
    delay(5);
    digitalWrite(runPin, LOW); // Zaustavi troling motor
    digitalWrite(dirPin, LOW);// Stepper motor se okreće u smijeru kazaljke na satu LOW
    //-----------------------------------------------
    unsigned long currentMillis = millis();
    if (((unsigned long)(currentMillis - previousMillis) >= M) && (stepAcc < Puls))  {
      stepAcc = stepAcc + 250;
      previousMillis = currentMillis;
    }
    //-----------------------------------------------
    tone(stepPin, stepAcc);
    //Serial.println(stepAcc);
  }
}

// **************************************************************************************************************************************************
void LeftTurn()
{
  digitalWrite(enablePin, LOW); //aktivira stepper driver A4988
  delay(5);
  for (int stepCount = 0 ; stepCount <= 800; stepCount += 1) {  //step count 25*16 za 5°; 50*16 za 10°; 100*16 za 20° getriba 11:100;1:16 microstepping
    digitalWrite(dirPin, HIGH);// Stepper motor se okreće suprotno od smijera kazaljke na satu HIGH
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(250);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(250);
  }
  digitalWrite(enablePin, HIGH); //deaktivira stepper driver A4988
}

// **************************************************************************************************************************************************
void LongLeftTurn()
{
  while (true)
  {
    bluetooth();                                                     // Run the Bluetooth procedure to see if there is any data being sent via BT
    if ((blueToothVal == 5) || (blueToothVal == 16)) {
      break;
    }
    digitalWrite(enablePin, LOW); //aktivira stepper driver A4988
    delay(5);
    digitalWrite(runPin, LOW); // Zaustavi troling motor
    digitalWrite(dirPin, HIGH);// Stepper motor se okreće suprotno od smijera kazaljke na satu HIGH
    //-----------------------------------------------
    unsigned long currentMillis = millis();
    if (((unsigned long)(currentMillis - previousMillis) >= M) && (stepAcc < Puls))  {
      stepAcc = stepAcc + 250;
      previousMillis = currentMillis;
    }
    //-----------------------------------------------
    tone(stepPin, stepAcc);
    //Serial.println(stepAcc);
  }

}

//----------------------------------------------------------------------------------------------------------------
void storeWaypoints()
{
  if ((gpsCount >= 0) && (gpsCount < 1)) {
    getGPS();                                                 // get the latest GPS coordinates
    Home_LAT[ab] = gps.location.lat();
    Home_LON[ab] = gps.location.lng();

    Serial1.print("#");  //Begining of line for app           // send serial string to arduino "#,45.123456,13.123456 "
    Serial1.print(",");  //separator for app
    Serial1.print(Home_LAT[0], 6); //Latitude for app
    Serial1.print(","); //separator for app
    Serial1.println(Home_LON[0], 6); //Longitude for app


    Serial.print("#");  //Begining of line for app
    Serial.print(",");  //separator for app
    Serial.print(Home_LAT[0], 6); //Latitude for app
    Serial.print(","); //separator for app
    Serial.print(Home_LON[0], 6); //Longitude for app
    Serial.println();  // end of line for app

    delay(50);
    gpsCount++;
    Serial.print("gpsCount:");
    Serial.print(gpsCount);
    ab++;
    Serial.print("  ab:");
    Serial.println(ab);
  }
  else {
    Serial1.print("Waypoints Full");
    Serial.println();
    Serial.println("Waypoints Full");
  }
}

//************************************************************** Jog the boat 2 m Forward
void JogForward() {
  Circle_LATcorrection = 2 * cos(boat_heading * PI / 180) / 111000;
  Circle_LONcorrection = 2 * sin(boat_heading * PI / 180) / 111000;

  bluelati = Home_LATarray[ac] + Circle_LATcorrection;
  bluelongi = Home_LONarray[ac] + Circle_LONcorrection;

  clearWaypoints();
  delay(50);
  setWaypoint();  //Postavlja GPS toćku kao waypoint "Set Way"
  delay(50);
  ac = 0;
  goWaypoint();   //Pokreće "Go to Waypoint"
}

//************************************************************** Jog the boat 2 m Reverse
void JogReverse() {
  Circle_LATcorrection = 2 * cos((boat_heading + 180) * PI / 180) / 111000;
  Circle_LONcorrection = 2 * sin((boat_heading + 180) * PI / 180) / 111000;

  bluelati = Home_LATarray[ac] + Circle_LATcorrection;
  bluelongi = Home_LONarray[ac] + Circle_LONcorrection;

  clearWaypoints();
  delay(50);
  setWaypoint();  //Postavlja GPS toćku kao waypoint "Set Way"
  delay(50);
  ac = 0;
  goWaypoint();   //Pokreće "Go to Waypoint"

}

//************************************************************** Jog the boat 2 m Left
void JogLeft() {
  Circle_LATcorrection = 2 * cos((boat_heading + 270) * PI / 180) / 111000;
  Circle_LONcorrection = 2 * sin((boat_heading + 270) * PI / 180) / 111000;

  bluelati = Home_LATarray[ac] + Circle_LATcorrection;
  bluelongi = Home_LONarray[ac] + Circle_LONcorrection;

  clearWaypoints();
  delay(50);
  setWaypoint();  //Postavlja GPS toćku kao waypoint "Set Way"
  delay(50);
  ac = 0;
  goWaypoint();   //Pokreće "Go to Waypoint"

}

//************************************************************** Jog the boat 2 m Left
void JogRight() {

  Circle_LATcorrection = 2 * cos((boat_heading + 90) * PI / 180) / 111000;
  Circle_LONcorrection = 2 * sin((boat_heading + 90) * PI / 180) / 111000;

  bluelati = Home_LATarray[ac] + Circle_LATcorrection;
  bluelongi = Home_LONarray[ac] + Circle_LONcorrection;

  clearWaypoints();
  delay(50);
  setWaypoint();  //Postavlja GPS toćku kao waypoint "Set Way"
  delay(50);
  ac = 0;
  goWaypoint();   //Pokreće "Go to Waypoint"

}
