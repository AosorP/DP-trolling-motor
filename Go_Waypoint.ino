void goWaypoint()
{
  Serial1.println(F("Go Home Spot ON!"));
  delay(1000);
  //Serial1.print(F("ac= "));
 // Serial1.println(ac);
 // delay(500);

  while (true) {                                                      // Start of Go_Home procedure
    bluetooth();                                                     // Run the Bluetooth procedure to see if there is any data being sent via BT
    if (blueToothVal == 5) {                // If a 'Stop' Bluetooth command is received then break from the Loop
      break;
    }

    lsm_procedure();
    getGPS();                                                        // Tiny GPS function that retrieves GPS data - update GPS location

    if (millis() > 5000 && gps.charsProcessed() < 10)                // If no Data from GPS within 5 seconds then send error
      Serial1.println(F("No GPS data: check wiring"));
    Distance_To_Home = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), Home_LATarray[ac], Home_LONarray[ac]); //Query Tiny GPS for Distance to Destination
    GPS_Course = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), Home_LATarray[ac], Home_LONarray[ac]);           //Query Tiny GPS for Course to Destination

    //-------------------------------------------------
    headingct = (atan2(Yh, Xh)) * 180 / PI;
    if (True_North == true) headingct += Declination; // calculate True North
    if (headingct < 0) headingct += 360.0;
    if (headingct > 360) headingct -= 360.0;

    //-------------------------------------------------
    kut = GPS_Course - headingct;
    error1 = (540 + kut) % 360 - 180;               //calculation of direction course error "kut"
    //---------------------------------------------------
    long sum = 0;                                   //geting average error from 50 measurements
    for (int i = 0; i < 50; i++) {
      sum = sum + error1;
    }
    error = sum / 50;
    //-----------------------------------------------------

    if (Home_LATarray[ac] == 0) {
      Serial1.println(F("End of Waypoints"));
      digitalWrite(runPin, LOW);
      break;
    }

    if (Distance_To_Home > 50000)                             // If the Vehicle is 50 km of the point stop
    {
      digitalWrite(runPin, LOW);                           // Stop the robot after each waypoint is reached
      Serial1.println(F("Far away from spot!"));              // Print to Bluetooth device - "Far away from spot!"
      digitalWrite(enablePin, HIGH);                       // deactivation of stepper motor driver A4988
      noTone(stepPin);
      pwm = 0;
      stepAcc = 0;
      count = 0;
      break;                                                // Break from Go_Home procedure and send control back to the Void Loop
    }                                                     // go to next waypoint


    if (Distance_To_Home > (setPoint + 0) && Distance_To_Home <= 50000) {     // If the boat is between 0 and 50 km  from home spot, run trolling motor

      /////////////////////////////////////////////////////////////////////
      //PID control of PWM of trolling motor

      unsigned long currentTime = millis();                           //get current time
      if ((unsigned long)(currentTime - previousTime >= 500)) {
        getGPS();
        delta = (TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), Home_LATarray[ac], Home_LONarray[ac])) - setPoint; // determine delta for PID_i & PID_d
        count++;
        //-----------------------PID_p

        if (delta > (setPoint + 4.0)) {
          PID_p = 255 / kp;
        }
        else if (delta <= (setPoint + 4.0) && delta >= (setPoint + 1.0)) {
          PID_p = delta;
        }
        else {
          PID_p = 0;
        }
        //-----------------------PID_i


        if (delta <= (setPoint + 4.0) && delta >= (setPoint + 1.0)) {        //ako je delta između 1,0 i 4.0 m počne se računati se PID_i
          if (count > 1) {
            PID_i += delta;                    // compute integral gain
          }
          if (PID_i > 200) {                     // limitiranje PID_i na 200 = 100 output
            PID_i = 200;
          }
        }
        else {
          PID_i = 0;
        }
        //-----------------------PID_d

        if (delta <= (setPoint + 6.0) && delta >= (setPoint + 0.5)) {        //ako je delta između 1,0 i 4.0 m počne se računati se PID_d
          if (count > 1) {
            PID_d = (delta - lastdelta) / 0.5;   // compute derivative
            lastdelta = delta;
          }
          else {
            PID_d = 0;
          }
        }

        timer = (unsigned long)(currentTime - previousTime); // služi za provjeru trajanja ciklusa 0,5 sek
        previousTime = currentTime;

        //-----------------------PID
        output = (PID_p * kp) + (PID_i * ki) + (PID_d * kd);      //PID output

        //-----------------------PID output normalization to PWM
        if (output > 255) {
          output = 255;                           //ako je izlaz veći od 255, izlaz se svodi na max pwm = 255
        }
        else if (output < 0) {
          output = 0;                            //ako je izlaz manj od 0, izlaz se svodi na min pwm = 0
        }

        //-----------------------
        Foutput = output / 255;                 //faktor multiplikator 0-1 za moduliranje PWM trolling motora

        /*
          Serial.print(Distance_To_Home);
          Serial.print(";");
        */
        /*
          Serial.print(delta, 4);
          Serial.print("; ");
          Serial.print(PID_p, 4);
          Serial.print("; ");
          Serial.print(PID_i);
          Serial.print("; ");
          Serial.print(PID_d, 4);
          Serial.print("; ");
          Serial.print(output);
          Serial.println("");
        */
        /*
          Serial.print(Foutput);
          Serial.print(";");

          Serial.print(pwm);
          Serial.print(";");
          Serial.print(timer, 3);
          Serial.println("");
        */
      }
      //**********************************************************
      //serial monitor print GPS NEMA data for evaluation pourpouse
      /*
        uint8_t GPSchar;
        while (1)
         {
           bluetooth();                                                     // Run the Bluetooth procedure to see if there is any data being sent via BT
           if (blueToothVal == 5){                                           // If a 'Stop' Bluetooth command is received then break from the Loop
           break;
           }
           if (Serial2.available() > 0)
         {
           GPSchar = Serial2.read();
           gps.encode(GPSchar);
           Serial.write(GPSchar);
         }
         }
      */
      ////////////////////////////////////////////////////////////////////
      //******************************************************************
      //turning ON/OFF trolling motor proppeler
      // +- 30°

      if (abs(error) <= 30) {    //  kut 30°
        unsigned long cTrMillis = millis();
        if (((unsigned long)(cTrMillis - pTrMillis) >= interval_a) && (pwm < (run_Speed * Foutput)))  {
          pwm = pwm + b;
          analogWrite(runPin, pwm);         // turn on trolling motor and accelerate to pwm walue
          pTrMillis = cTrMillis;
          //Serial.println(pwm);
        }
        else if (((unsigned long)(cTrMillis - pTrMillis) >= interval_b) && ((run_Speed * Foutput) < pwm)) {
          pwm = pwm - b;
          analogWrite(runPin, pwm);         // turn on trolling motor and decelerate to pwm walue
          pTrMillis = cTrMillis;
          //Serial.println(pwm);
        }
      }
      //*****************************************************************************
      // +- 30-50°
      else if (abs(error) > 30 && abs(error) <= 50) {     //
        unsigned long cTrMillis = millis();
        if (((unsigned long)(cTrMillis - pTrMillis) >= interval_a) && (pwm < (run_Speed * Foutput)))  {
          pwm = pwm + b;
          analogWrite(runPin, pwm);         // turn on trolling motor and accelerate to pwm walue
          pTrMillis = cTrMillis;
          //Serial.println(pwm);
        }
        else if (((unsigned long)(cTrMillis - pTrMillis) >= interval_b) && ((run_Speed * Foutput) < pwm ))  {
          pwm = pwm - b;
          analogWrite(runPin, pwm);         // turn on trolling motor and decelerate to pwm walue
          pTrMillis = cTrMillis;
          //Serial.println(pwm);
        }
      }
      //*********************************************************
      // +- 50-90°
      else if (abs(error) > 50 && abs(error) <= 90) {     //
        unsigned long cTrMillis = millis();
        if (((unsigned long)(cTrMillis - pTrMillis) >= interval_a) && (pwm < (run_Speed * 0.8 * Foutput)))  {
          pwm = pwm + b;
          analogWrite(runPin, pwm);         // turn on trolling motor and accelerate to pwm walue
          pTrMillis = cTrMillis;
          //Serial.println(pwm);
        }
        else if (((unsigned long)(cTrMillis - pTrMillis) >= interval_b) && ((run_Speed * 0.8 * Foutput) < pwm ))  {
          pwm = pwm - b;
          analogWrite(runPin, pwm);         // turn on trolling motor and decelerate to pwm walue
          pTrMillis = cTrMillis;
          //Serial.println(pwm);
        }
      }
      //********************************************************
      // +- 90-180°
      else {
        digitalWrite(runPin, LOW);
        pwm = 0;
      }

      //********************************************************
      //stop rotation of trolling motor shaft

      if (Distance_To_Home <= (setPoint + 0.5))                // If the distance of the vheicle is less or equal than 0.5 m from set point
      {
        unsigned long cMillis = millis();
        if ((unsigned long)(cMillis - pMillis) >= 50) {
          Serial1.println(F("You are on spot!"));                  // Print to Bluetooth device - "You have arrived"
          pMillis = cMillis;
        }
        digitalWrite(enablePin, HIGH);                        // deactivation of stepper motor driver A4988, stop pivoting trolling motor
        noTone(stepPin);
        stepAcc = 0;
        //ac++;                                               // Increment counter for next waypoint
        //break;                                              // Break from Go_Home procedure and send control back to the Void Loop go to next waypoint
      }
      //*******************************************************
      //enable rotatio of trolling motor shaft

      if (Distance_To_Home > (setPoint + 0.5))                 // If the distance of the vheicle is higher than 0.5 m from set point
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
      }                                                            //end of enable rotatio of trolling motor shaft procedure
      /*
        Serial.print("kut=");
        Serial.print(error);
        Serial.print("  D=");
        Serial.print(Distance_To_Home);
        Serial.print("m   heading=");
        Serial.print(headingct);
        Serial.print("    GPS_Course=");
        Serial.print(GPS_Course);
        Serial.println("");
      */
      /*
        Serial.print(Distance_To_Home);
        Serial.print(";");
        //Serial.print("  Delta=");
        Serial.print(delta);
        Serial.print(";");
        //Serial.print("  Cum_delta=");
        Serial.print(cumdelta);
        Serial.print(";");
        //Serial.print("  Rate_delta=");
        Serial.print(ratedelta, 6);
        Serial.print(";");
        //Serial.print(" F_Outpot=");
        Serial.print(Foutput);
        Serial.print(";");
        //Serial.print(" ElapsedTime=");
        Serial.print(elapsedTime);
        Serial.print(";");
        //Serial.print(" PWM=");
        Serial.print(pwm);
        Serial.print(";");
        Serial.println("");
      */


      unsigned long cMillis = millis();
      if ((unsigned long)(cMillis - pMillis) >= 500) {

        if (delta > (setPoint + 0.5)) {
          Serial1.print("Er:");
          Serial1.print(error);
          Serial1.print("° D=");
          Serial1.print(Distance_To_Home);
          Serial1.print("m ");
          Serial1.print("P:");
          Serial1.println(pwm);

          /*
            Serial.print(F("Er:"));
            Serial.print(error);
            Serial.print(F("° D="));
            Serial.print(Distance_To_Home);
            Serial.print(F("m "));
            Serial.print(F("P:"));
            Serial.print(pwm);
            Serial.println();
          */
        }
        pMillis = cMillis;
      }

    }                                                      // End of Distance_To_Home >2.0m Loop

  }                                                        // End of While(true) Loop

}                                                          // End of Go_Home procedure
