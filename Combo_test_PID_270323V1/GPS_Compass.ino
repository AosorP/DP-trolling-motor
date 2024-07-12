void getGPS()                                                 // Get Latest GPS coordinates
{
  while (Serial2.available() > 0)
    gps.encode(Serial2.read());
}

// *************************************************************************************************************************************************
void setWaypoint()                                            // Set up to 5 GPS waypoints
{

  if ((wpCount >= 0) && (wpCount < 1))
    //if (wpCount >= 0)
  {
    getGPS();                                                 // get the latest GPS coordinates


    Home_LATarray[ac] = bluelati;                   // store waypoint in an array indirect Arduino>MitAPP>Arduino
    Home_LONarray[ac] = bluelongi;                   // store waypoint in an array indirect Arduino>MitAPP>Arduino


    Serial.print("Waypoint #1: ");
    Serial.print(Home_LATarray[0], 6);
    Serial.print(" ");
    Serial.println(Home_LONarray[0], 6);
    Serial.print("Waypoint #2: ");
    Serial.print(Home_LATarray[1], 6);
    Serial.print(" ");
    Serial.println(Home_LONarray[1], 6);
    Serial.print("Waypoint #3: ");
    Serial.print(Home_LATarray[2], 6);
    Serial.print(" ");
    Serial.println(Home_LONarray[2], 6);
    Serial.print("Waypoint #4: ");
    Serial.print(Home_LATarray[3], 6);
    Serial.print(" ");
    Serial.println(Home_LONarray[3], 6);
    Serial.print("Waypoint #5: ");
    Serial.print(Home_LATarray[4], 6);
    Serial.print(" ");
    Serial.println(Home_LONarray[4], 6);

    wpCount++;                                                  // increment waypoint counter
    ac++;                                                       // increment array counter

  }
  else {
    Serial1.println("Waypoints Full");

  }
}
// *************************************************************************************************************************************************
void clearWaypoints()
{
  memset(Home_LATarray, 0, sizeof(Home_LATarray));             // clear the array
  memset(Home_LONarray, 0, sizeof(Home_LONarray));             // clear the array
  Serial.print(F("Waypoint #1: "));
  Serial.print(Home_LATarray[0], 6);
  Serial.print(F(" "));
  Serial.println(Home_LONarray[0], 6);
  Serial.print(F("Waypoint #2: "));
  Serial.print(Home_LATarray[1], 6);
  Serial.print(F(" "));
  Serial.println(Home_LONarray[1], 6);
  Serial.print(F("Waypoint #3: "));
  Serial.print(Home_LATarray[2], 6);
  Serial.print(F(" "));
  Serial.println(Home_LONarray[2], 6);
  Serial.print(F("Waypoint #4: "));
  Serial.print(Home_LATarray[3], 6);
  Serial.print(F(" "));
  Serial.println(Home_LONarray[3], 6);
  Serial.print(F("Waypoint #5: "));
  Serial.print(Home_LATarray[4], 6);
  Serial.print(F(" "));
  Serial.println(Home_LONarray[4], 6);

  wpCount = 0;                                                 // reset increment counter to 0
  gpsCount = 0;
  ac = 0;
  ab = 0;                                                        // GPS array counter

  Serial1.println(F("GPS Waypoints Cleared!"));                      // display waypoints cleared
}

// *************************************************************************************************************************************************
void gpsInfo()                                                  // displays Satellite data to user
{
  Number_of_SATS = (int)(gps.satellites.value());         //Query Tiny GPS for the number of Satellites Acquired
  Distance_To_Home = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), Home_LATarray[ac], Home_LONarray[ac]); //Query Tiny GPS for Distance to Destination
  GPS_Course = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), Home_LATarray[ac], Home_LONarray[ac]); //Query Tiny GPS for Course to Destination

  // *************************************************************************************************************************************************
  lsm_procedure();

  headingct = (atan2(Yh, Xh)) * 180 / PI;
  if (True_North == true) headingct += Declination; // calculate True North
  if (headingct < 0) headingct += 360.0;
  if (headingct > 360) headingct -= 360.0;

  // ----- send the results to the Serial Monitor

  Serial.print(F("Heading:"));
  Serial.print(headingct);
  Serial.print(F(" Lat:"));
  Serial.print(gps.location.lat(), 6);
  Serial.print(F(" Lon:"));
  Serial.print(gps.location.lng(), 6);
  Serial.print(F("; Distance to Home "));
  Serial.println(Distance_To_Home);

  Serial1.print(F("H:"));
  Serial1.print(headingct,0);
  Serial1.print(F("°/"));
  Serial1.print(F("D:"));
  Serial1.print(Distance_To_Home,1);
  Serial1.print(F("m/"));
  Serial1.print(F("S:"));
  Serial1.print(Number_of_SATS);
  Serial1.print(F("/K:"));
  Serial1.print(GPS_Course,1);
  Serial1.println(F("°"));

}

// *************************************************************************************************************************************************
 
