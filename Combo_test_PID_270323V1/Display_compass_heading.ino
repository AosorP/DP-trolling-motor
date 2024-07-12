void display_compass_heading_on_serial_monitor()
{

  // ----- display the heading
  headingct = (atan2(Yh, Xh)) * 180 / PI;
  if (True_North == true) headingct += Declination; // calculate True North
  if (headingct < 0) headingct += 360.0;
  if (headingct > 360) headingct -= 360.0;
  Serial.print(F("    Heading:"));
  Serial.println(headingct);

}
