void go_CircleMode()
{
  Serial1.println(F("Circle mode ON!"));
  delay(1000);
  float Distance_To_CirclePoint;                                     // variable for distance to CirclePoint


  while (true)
  { // Start of CircleMode procedure
    bluetooth();                                                     // Run the Bluetooth procedure to see if there is any data being sent via BT
    if ((blueToothVal == 5) || (blueToothVal == 16)) {               // If a 'Stop' Bluetooth command is received then break from the Loop
      break;
    }

    lsm_procedure();
    getGPS();                                                        // Tiny GPS function that retrieves GPS data - update GPS location

    if (millis() > 5000 && gps.charsProcessed() < 10)                // If no Data from GPS within 5 seconds then send error
      Serial1.println(F("No GPS data: check wiring"));

    //********************************************************************************************
    //Circle koordinate calculation

    int n_circle_points = 12;
    int theta = 360 / n_circle_points;
    //Serial.print(F("Theta"));
    //Serial.println(theta);

    Circle_LATcorrection = radius * cos(beta * PI / 180) / 111000;
    Circle_LONcorrection = radius * sin(beta * PI / 180) / 111000;

    Circle_LATpoint = Home_LATarray[ac] + Circle_LATcorrection;
    Circle_LONpoint = Home_LONarray[ac] + Circle_LONcorrection;

    Distance_To_CirclePoint = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), Circle_LATpoint, Circle_LONpoint); //Query Tiny GPS for Distance to circle point
    GPS_Course = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), Circle_LATpoint, Circle_LONpoint); //Query Tiny GPS for Course to circle point
    //-------------------------------------------------
    headingct = (atan2(Yh, Xh)) * 180 / PI;
    if (True_North == true) headingct += Declination; // calculate True North
    if (headingct < 0) headingct += 360.0;
    if (headingct > 360) headingct -= 360.0;

    //-------------------------------------------------
    kut = GPS_Course - headingct;
    error1 = (540 + kut) % 360 - 180;               // If GPS Course and the Compass Heading are within T degrees of each other then go Forward
    //---------------------------------------------------
    long sum = 0;
    for (int i = 0; i < 50; i++) {
      sum = sum + error1;
    }
    error = sum / 50;
    //-----------------------------------------------------

    if (Home_LATarray[ac] == 0 && Home_LONarray[ac] == 0) {
      Serial1.println(F("End of Waypoints"));
      digitalWrite(runPin, LOW);
      break;
    }

    //------------------------------------------------------
    if (Distance_To_CirclePoint > 200)                   // If the boat is 200m from the point, stop!
    {
      digitalWrite(runPin, LOW);                           // motor off
      Serial1.println(F("Far away from nextpoint!"));           // Print to Bluetooth device - "Far away from spot!"
      digitalWrite(enablePin, HIGH);                       // deactivation of stepper motor driver A4988
      noTone(stepPin);
      pwm = 0;
      stepAcc = 0;
      count = 0;
      break;                                                // Break from go_CircleMode procedure and send control back to the Void Loop
    }

    if (Distance_To_CirclePoint > (setPoint + 1.0) && Distance_To_CirclePoint <= 200) {     // If the boat is between 1 and 150 m from circle point, run trolling motor

      //****************************************************************
      //PID control of PWM of trolling motor

      unsigned long currentTime = millis();                           //get current time
      if ((unsigned long)(currentTime - previousTime >= 500)) {
        getGPS();
        delta = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), Circle_LATpoint, Circle_LONpoint) - setPoint; // determine delta for PID_i & PID_d
        count++;
        //-----------------------PID_p
        if (delta > (setPoint + 4.0)) {
          PID_p = 255 / kp;
        }
        else if (delta <= (setPoint + 4.0) && delta >= (setPoint + 0.0)) {
          PID_p = delta;
        }
        else {
          PID_p = 0;
        }
        //-----------------------PID_i
        if (delta <= (setPoint + 4.0) && delta >= (setPoint + 2.0)) {        //ako je delta između 2,0 i 4.0 m počne se računati se PID_i
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

        if (delta <= (setPoint + 6.0) && delta >= (setPoint + 0.0)) {        //ako je delta između 0,0 i 6.0 m počne se računati se PID_d
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


      }
      //******************************************************************
      //turning ON/OFF trolling motor proppeler
      // +- 70°

      if (abs(error) <= 70) {    //  kut 30°
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

      //*********************************************************
      // +- 70-90°
      else if (abs(error) > 70 && abs(error) <= 90) {     //
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
        unsigned long cTrMillis = millis();
        if (((unsigned long)(cTrMillis - pTrMillis) >= interval_a) && (pwm < (run_Speed * 0.4 * Foutput)))  {
          pwm = pwm + b;
          analogWrite(runPin, pwm);         // turn on trolling motor and accelerate to pwm walue
          pTrMillis = cTrMillis;
          //Serial.println(pwm);
        }
        else if (((unsigned long)(cTrMillis - pTrMillis) >= interval_b) && ((run_Speed * 0.4 * Foutput) < pwm ))  {
          pwm = pwm - b;
          analogWrite(runPin, pwm);         // turn on trolling motor and decelerate to pwm walue
          pTrMillis = cTrMillis;
          //Serial.println(pwm);
        }
      }

      //*******************************************************
      //enable rotatio of trolling motor shaft

      if (Distance_To_CirclePoint > (setPoint + 0.0))                 // If the distance of the vheicle is higher than 0.0 m from set point
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
      }                                                      //end of enable rotatio of trolling motor shaft procedure


      unsigned long cMillis = millis();
      if ((unsigned long)(cMillis - pMillis) >= 500) {

        if (delta > (setPoint + (radius * 0.2))) {
          Serial1.print(F("Er:"));
          Serial1.print(error);
          Serial1.print(F("° D="));
          Serial1.print(Distance_To_CirclePoint);
          Serial1.print(F("m ;"));
          Serial1.print(F(" O="));
          Serial1.print(beta, 1);
          Serial1.println(F("°"));
        }
        pMillis = cMillis;
      }

    }                                                      // End of Distance_To_CirclePoint >1.0m Loop

    if (Distance_To_CirclePoint <= (setPoint + 3)) {    // If the boat is between 0-3 m from CirclePoint, increment angle beta by theta
      beta += theta;
      C++;
    }

    if (beta >= 360.0) {
      beta = 0.0;
      C = 0;
    }

  }                                                          // End of While(true) Loop

}                                                              // End of Go_Home procedure
