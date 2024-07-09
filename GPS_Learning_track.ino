void Learning_track() {

  getGPS();
  Previouse_LAT = gps.location.lat();
  Previouse_LON = gps.location.lng();

  //******************************************************************************************
  while (true) {
    bluetooth();                                                     // Run the Bluetooth procedure to see if there is any data being sent via BT
    if ((blueToothVal == 5) || (blueToothVal == 16)) {               // If a 'Stop' Bluetooth command is received then break from the Loop
      break;
    }
    getGPS();
    Curent_LAT = gps.location.lat();                   // store lat waypoint from GPS
    Curent_LON = gps.location.lng();                  //  store lon waypoint from GPS
    Distance = TinyGPSPlus::distanceBetween(Curent_LAT, Curent_LON, Previouse_LAT, Previouse_LON);

    if ((Distance > 10.0) && (Distance < 100)) {
      Previouse_LAT = Curent_LAT;
      Previouse_LON = Curent_LON;
      record_track();
    }
    unsigned long currentMillis = millis();
    if (((unsigned long)(currentMillis - previousMillis) >= 500)) {
      Serial1.print(F("Distance="));
      Serial1.print(Distance);
      Serial1.print(F(" TP. "));
      Serial1.print(tpCount + 1);
      Serial1.println(F(" Set!"));
      previousMillis = currentMillis;
    }
  }
}
//***************************************************************************
void record_track() {
  if ((tpCount >= 0) && (tpCount < 299))
  {
    Serial.print(F("Distance="));
    Serial.println(Distance, 3);
    
    getGPS();
    Previouse_LATarray[ar] = gps.location.lat();                   // store lat waypoint from GPS
    Previouse_LONarray[ar] = gps.location.lng();                  //  store lon waypoint from GPS

    Serial.print(F("Trackpoint #1: "));
    Serial.print(Previouse_LATarray[0], 6);
    Serial.print(F(" "));
    Serial.println(Previouse_LONarray[0], 6);
    Serial.print(F("Trackpoint #2: "));
    Serial.print(Previouse_LATarray[1], 6);
    Serial.print(F(" "));
    Serial.println(Previouse_LONarray[1], 6);
    Serial.print(F("Trackpoint #3: "));
    Serial.print(Previouse_LATarray[2], 6);
    Serial.print(F(" "));
    Serial.println(Previouse_LONarray[2], 6);
    Serial.print(F("Trackpoint #4: "));
    Serial.print(Previouse_LATarray[3], 6);
    Serial.print(F(" "));
    Serial.println(Previouse_LONarray[3], 6);
    Serial.print(F("Trackpoint #5: "));
    Serial.print(Previouse_LATarray[4], 6);
    Serial.print(F(" "));
    Serial.println(Previouse_LONarray[4], 6);

    tpCount++;                                                  // increment track point counter
    ar++;                                                       // increment array counter
    Serial.print("ar=");
    Serial.println(ar);
    ax++;
  }

  else {
    Serial1.println(F("Trackpoints Full"));
    delay(800);
    Serial1.println(F("Press STOP to exit Learning"));
  }

}

// *************************************************************************************************************************************************
void clearTrack()
{
  memset(Previouse_LATarray, 0, sizeof(Previouse_LATarray));             // clear the array
  memset(Previouse_LONarray, 0, sizeof(Previouse_LONarray));             // clear the array
  Serial.print(F("Trackpoint #1: "));
  Serial.print(Previouse_LATarray[0], 6);
  Serial.print(F(" "));
  Serial.println(Previouse_LONarray[0], 6);
  Serial.print(F("Trackpoint #2: "));
  Serial.print(Previouse_LATarray[1], 6);
  Serial.print(F(" "));
  Serial.println(Previouse_LONarray[1], 6);
  Serial.print(F("Trackpoint #3: "));
  Serial.print(Previouse_LATarray[2], 6);
  Serial.print(F(" "));
  Serial.println(Previouse_LONarray[2], 6);
  Serial.print(F("Trackpoint #4: "));
  Serial.print(Previouse_LATarray[3], 6);
  Serial.print(F(" "));
  Serial.println(Previouse_LONarray[3], 6);
  Serial.print(F("Trackpoint #5: "));
  Serial.print(Previouse_LATarray[4], 6);
  Serial.print(F(" "));
  Serial.println(Previouse_LONarray[4], 6);
  tpCount = 0;                                                 // reset increment track point counter to 0
  ar = 0;                                                      // reset GPS track array counter to 0
  ax = 1;
  Serial1.println(F("GPS Track Cleared"));                      // display waypoints cleared
}

// *************************************************************************************************************************************************
