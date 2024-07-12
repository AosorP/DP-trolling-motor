void go_Track()
{
  Serial1.println(F("Track mode starts!"));
  delay(1000);
  ar = ar - 1;
  loop_dir = -1;
  Serial1.print(F("You have "));
  Serial1.print(ar);
  Serial1.println(F(" track points! "));
  //Serial1.print(F(" ax= "));
  //Serial1.println(ax);
  delay(1500);

  while (true)                                                       // Start of Track mode procedure
  { 
    bluetooth();                                                    // Run the Bluetooth procedure to see if there is any data being sent via BT
    if ((blueToothVal == 5) || (blueToothVal == 16)) {              // If a 'Stop' Bluetooth command is received then break from the Loop
      break;
    }

    lsm_procedure();
    getGPS();                                                        // Tiny GPS function that retrieves GPS data - update GPS location

    if (millis() > 5000 && gps.charsProcessed() < 10)                // If no Data from GPS within 5 seconds then send error
      Serial1.println(F("No GPS data: check wiring"));

    Distance_To_TrackPoint = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), Previouse_LATarray[ar], Previouse_LONarray[ar]); //Query Tiny GPS for Distance to Destination
    GPS_Course = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), Previouse_LATarray[ar], Previouse_LONarray[ar]);           //Query Tiny GPS for Course to Destination

    //******************************************************************************************************
    headingct = (atan2(Yh, Xh)) * 180 / PI;
    if (True_North == true) headingct += Declination; // calculate True North
    if (headingct < 0) headingct += 360.0;
    if (headingct > 360) headingct -= 360.0;

    //******************************************************************************************************
    kut = GPS_Course - headingct;
    error1 = (540 + kut) % 360 - 180;               // If GPS Course and the Compass Heading are within certain degrees of each other then go Forward

    //******************************************************************************************************
    long sum = 0;
    for (int i = 0; i < 50; i++) {
      sum = sum + error1;
    }
    error = sum / 50;
    //*****************************************************************************************************
    if (Distance_To_TrackPoint > 200)                 // If the Vehicle is 200 m of the next track point, Stop
    {
      digitalWrite(runPin, LOW);
      Serial1.println(F("Far away from track!"));       // Print to Bluetooth device - "Far away from track!"
      digitalWrite(enablePin, HIGH);                  // Deactivation of stepper motor driver A4988
      noTone(stepPin);
      pwm = 0;
      stepAcc = 0;
      count = 0;
      break;                                          // Break from Go_Home procedure and send control back to the Void Loop, go to next waypoint
    }
    //********************************************************************************************************
    if (Distance_To_TrackPoint > (setPoint + 3) && Distance_To_TrackPoint <= 200) {     // If the boat is between 3 and 200 m from track point, run trolling motor

      //turning ON/OFF trolling motor proppeler
      // +- 70°

      if (abs(error) <= 70) {    //  kut 70°
        unsigned long cTrMillis = millis();
        if (((unsigned long)(cTrMillis - pTrMillis) >= interval_a) && (pwm < (run_Speed)))  {
          pwm = pwm + b;
          analogWrite(runPin, pwm);         // turn on trolling motor and accelerate to pwm walue
          pTrMillis = cTrMillis;
          //Serial.println(pwm);
        }
        else if (((unsigned long)(cTrMillis - pTrMillis) >= interval_b) && ((run_Speed) < pwm)) {
          pwm = pwm - b;
          analogWrite(runPin, pwm);         // turn on trolling motor and decelerate to pwm walue
          pTrMillis = cTrMillis;
          //Serial.println(pwm);
        }
      }
      //*****************************************************************************
      // +- 70-90°
      else if (abs(error) > 70 && abs(error) <= 90) {     //
        unsigned long cTrMillis = millis();
        if (((unsigned long)(cTrMillis - pTrMillis) >= interval_a) && (pwm < (run_Speed * 0.8)))  {
          pwm = pwm + b;
          analogWrite(runPin, pwm);         // turn on trolling motor and accelerate to pwm walue
          pTrMillis = cTrMillis;
          //Serial.println(pwm);
        }
        else if (((unsigned long)(cTrMillis - pTrMillis) >= interval_b) && ((run_Speed * 0.8) < pwm ))  {
          pwm = pwm - b;
          analogWrite(runPin, pwm);         // turn on trolling motor and decelerate to pwm walue
          pTrMillis = cTrMillis;
          //Serial.println(pwm);
        }
      }
      //********************************************************
      // +- 90-180°
      else {
        unsigned long cTrMillis = millis();
        if (((unsigned long)(cTrMillis - pTrMillis) >= interval_a) && (pwm < (run_Speed * 0.4)))  {
          pwm = pwm + b;
          analogWrite(runPin, pwm);         // turn on trolling motor and accelerate to pwm walue
          pTrMillis = cTrMillis;
          //Serial.println(pwm);
        }
        else if (((unsigned long)(cTrMillis - pTrMillis) >= interval_b) && ((run_Speed * 0.4) < pwm ))  {
          pwm = pwm - b;
          analogWrite(runPin, pwm);         // turn on trolling motor and decelerate to pwm walue
          pTrMillis = cTrMillis;
          //Serial.println(pwm);
        }
      }

      //********************************************************
      //stop rotation of trolling motor shaft

      if (Distance_To_TrackPoint <= (setPoint + 0.0))                // If the distance of the vheicle is less or equal than 0.0 m from set point
      {
        unsigned long cMillis = millis();
        if ((unsigned long)(cMillis - pMillis) >= 50) {
          Serial1.println(F("You are on spot!"));                  // Print to Bluetooth device - "You have arrived"
          pMillis = cMillis;
        }
        digitalWrite(enablePin, HIGH);                        // deactivation of stepper motor driver A4988, stop pivoting trolling motor
        noTone(stepPin);
        stepAcc = 0;
      }
      //*******************************************************
      //enable rotatio of trolling motor shaft

      if (Distance_To_TrackPoint > (setPoint + 0.0))                 // If the distance of the vheicle is higher than 0.5 m from set point
      {
        //***************************************************
        //rotating trolling motor shaft CCW

        if (error < 0 && abs(error) > Tx) {                 // Tx = maximum allowed angle between compass and gps heading (set to 10 degree)
          digitalWrite(enablePin, LOW);                       // enable A4988 stepper motor driver who is rotating compass and trolling motor bar
          delay(5);
          digitalWrite(dirPin, HIGH);                         // Stepper motor rotate CCW; LOW/HIGH (check on hardware!!!)

          unsigned long cStRMillis = millis();
          if (((unsigned long)(cStRMillis - pStRMillis) >= M) && (stepAcc < Puls))  {
            stepAcc = stepAcc + 300;
            pStRMillis = cStRMillis;
          }
          tone(stepPin, stepAcc);
        }
        //***************************************
        //rotating trolling motor shaft CW

        if (error > 0 && abs(error) > Tx) {                  // Tx = maximum allowed angle diference between compass and gps heading (set to 10 degree)
          digitalWrite(enablePin, LOW);                        // enable A4988 stepper motor driver who is rotating compassand trolling motor bar
          delay(5);
          digitalWrite(dirPin, LOW);                           // Stepper motor rotate CW; LOW/HIGH (check on hardware!!!)

          unsigned long cStLMillis = millis();
          if (((unsigned long)(cStLMillis - pStLMillis) >= M) && (stepAcc < Puls))  {
            stepAcc = stepAcc + 300;
            pStLMillis = cStLMillis;
          }
          tone(stepPin, stepAcc);
        }
        //**********************************************************
        //histerezis of rotation trolling motor shaft

        if (abs(error) <= Tx) {
          digitalWrite(enablePin, HIGH);                      //deactivation of stepper motor driver A4988
          noTone(stepPin);
          stepAcc = 0;
        }
      }                                                       //end of enable rotatio of trolling motor shaft procedure

      unsigned long cMillis = millis();
      if ((unsigned long)(cMillis - pMillis) >= 500) {

        if (Distance_To_TrackPoint > (setPoint + 2.0)) {
          Serial1.print(F("Er:"));
          Serial1.print(error);
          Serial1.print(F("° D="));
          Serial1.print(Distance_To_TrackPoint,1);
          Serial1.print(F("m "));
          Serial1.print(F("P:"));
          Serial1.print(pwm);
          Serial1.print(F(" TP."));
          Serial1.println(ar);

          Serial.print(F("Er:"));
          Serial.print(error);
          Serial.print(F("° D="));
          Serial.print(Distance_To_TrackPoint);
          Serial.print(F("m "));
          Serial.print(F("P:"));
          Serial.print(pwm);
          Serial.println("");
        }
        pMillis = cMillis;
      }

    }    // End of Distance_To_TrackPoint >1.0 <200 m Loop

    if ((Distance_To_TrackPoint <= (setPoint + 3.0)) && (loop_dir == -1)) {    // If the boat is between 0 and 3m from CirclePoint, go next point
      ar--;
      Serial.print(F("ar="));
      Serial.println(ar);
      
      Serial1.print(F("you are on "));
      Serial1.print(ar);
      Serial1.println(F(" way point"));
    }

    if ((Distance_To_TrackPoint <= (setPoint + 3.0)) && (loop_dir == 1)) {    // If the boat is between 0 and 3m from CirclePoint, go next point
      ar++;
      Serial.print(F("ar="));
      Serial.println(ar);
      
      Serial1.print(F("you are on "));
      Serial1.print(ar);
      Serial1.println(F(" way point"));
      //Serial1.print("; ");
      //Serial1.print(F("ar="));
      //Serial1.println(ar);
    }

    if (ar == 0) {
      loop_dir = 1;
    }

    if (ar == ax - 2) {
      loop_dir = -1;
    }

  }  // End of While(true) Loop

}    // End of Tracking mode procedure
