void Startup()
{

  Serial.println(F("Pause for Startup... "));

  for (int i = 2; i >= 1; i--)                     // Count down for X seconds
  {
    Serial1.println(F("Pause for Startup... "));
    //Serial1.print(i);
    delay(1000);                                   // Delay for X seconds
  }

  Serial1.println(F("Searching for Satellites "));

  
  Serial.println(F("Searching for Satellites "));

  while (Number_of_SATS < 4)                         // Wait until 4 number of satellites are acquired before starting main loop
  {
    getGPS();                                         // Update gps data
    Number_of_SATS = (int)(gps.satellites.value());   // Query Tiny GPS for the number of Satellites Acquired
    bluetooth();                                      // Check to see if there are any bluetooth commands being received
  }


  //setWaypoint();                                      // set intial waypoint to current location
  wpCount = 0;                                        // zero waypoint counter
  ac = 0;                                             // zero array counter
  ab = 0;
  Serial1.print(Number_of_SATS);
  Serial1.println(" SatsUsed");
  
  Serial.print(Number_of_SATS);
  Serial.println(" Satellites Acquired"); 
  delay(1000);    
}
