void lsm_procedure()
{
  lsm.read();

  float values_from_magnetometer[3];
  float values_from_accelerometer[3];

  getRawValuesMag();

  values_from_magnetometer[0] = Mx;
  values_from_magnetometer[1] = My;
  values_from_magnetometer[2] = Mz;

  transformation_mag(values_from_magnetometer);

  Mxc = calibrated_mag_values[0];
  Myc = calibrated_mag_values[1];
  Mzc = calibrated_mag_values[2];

  getRawValuesAcc();

  values_from_accelerometer[0] = Ax;
  values_from_accelerometer[1] = Ay;
  values_from_accelerometer[2] = Az;

  transformation_acc(values_from_accelerometer);

  Axc = calibrated_acc_values[0];
  Ayc = calibrated_acc_values[1];
  Azc = calibrated_acc_values[2];

  //  Normalizing
  norm_m = sqrt(sq(Mxc) + sq(Myc) + sq(Mzc)); //original code did not appear to normalize, and this seems to help
  Mxcn = Mxc / norm_m;
  Mycn = Myc / norm_m;
  Mzcn = Mzc / norm_m;

  norm_a = sqrt(sq(Axc) + sq(Ayc) + sq(Azc)); //original code did not appear to normalize, and this seems to help
  Axcn = Axc / norm_a;
  Aycn = Ayc / norm_a;
  Azcn = Azc / norm_a;


  // Low-Pass filter magnetometer
  Mxcnf = Mxcn * alpha + (Mxcnf * (1.0 - alpha));
  Mycnf = Mycn * alpha + (Mycnf * (1.0 - alpha));
  Mzcnf = Mzcn * alpha + (Mzcnf * (1.0 - alpha));

  // Low-Pass filter accelerometer
  Axcnf = Axcn * alpha + (Axcnf * (1.0 - alpha));
  Aycnf = Aycn * alpha + (Aycnf * (1.0 - alpha));
  Azcnf = Azcn * alpha + (Azcnf * (1.0 - alpha));


  // Calculating pitch and roll angles following Application Note
  pitch = (double) asin((double) - Axcnf);
  pitch_print = pitch * 180 / PI;
  roll = (double) asin((double) Aycnf / cos((double) pitch));
  roll_print = roll * 180 / PI;


  //  Calculating heading with raw measurements not tilt compensated
  float heading = (double)atan2((double)My, (double)Mx) * 180 / PI;
  if (heading < 0) {
    heading = 360 + heading;
  }

  //  Calculating heading with calibrated measurements not tilt compensated
  float headingc = (double)atan2((double)Myc, (double)Mxc) * 180 / PI;
  if (headingc < 0) {
    headingc = 360 + headingc;
  }

  //  Calculating tilt compensated heading
  Xh = Mxcnf * cos((double)pitch) + Mzcnf * sin((double)pitch);
  Yh = Mxcnf * sin((double)roll) * sin((double)pitch) + Mycnf * cos((double)roll) - Mzcnf * sin((double)roll) * cos((double)pitch);
  headingct = (atan2(Yh, Xh)) * 180 / PI;
  if (True_North == true) headingct += Declination; // calculate True North
  if (headingct < 0) headingct += 360.0;
  if (headingct > 360) headingct -= 360.0;



  //Calculating Tilt angle in degrees
  tiltcnf = (double)atan2((double)fabs(Azcnf), (double)Axcnf) * 180 / PI;

  //Serial.flush();

  /* // Display raw magnetometer values
    Serial.println("Magnetometer raw measurements");
    Serial.print("Mx: "); Serial.print(Mx); Serial.print("; ");
    Serial.print("My: "); Serial.print(My); Serial.print("; ");
    Serial.print("Mz: "); Serial.println(Mz);

    // Display calibrated magnetometer values
    Serial.println("Magnetometer calibrated measurements");
    Serial.print("Mxc: "); Serial.print(Mxc); Serial.print("; ");
    Serial.print("Myc: "); Serial.print(Myc); Serial.print("; ");
    Serial.print("Mzc: "); Serial.println(Mzc);
    Serial.println("--------------------------------------");

    // Display raw accelerometer measurements in milliG
    Serial.println("Accelerometer raw measurements");
    Serial.print("Ax: "); Serial.print(Ax); Serial.print("; ");
    Serial.print("Ay: "); Serial.print(Ay); Serial.print("; ");
    Serial.print("Az: "); Serial.println(Az);

    // Display calibrated accelerometer measurements in milliG
    Serial.println("Accelerometer calibrated measurements");
    Serial.print("Axc: "); Serial.print(Axc); Serial.print("; ");
    Serial.print("Ayc: "); Serial.print(Ayc); Serial.print("; ");
    Serial.print("Azc: "); Serial.println(Azc);
    Serial.println("--------------------------------------");
  */
  // Display Heading in degrees North = 0Â°--> 360Â° turning clockwise
  /* Serial.print ("Heading raw: "); Serial.println (heading);
    Serial.print ("Heading calibrated: "); Serial.println (headingc);
    Serial.print ("Heading tilt compensated: "); Serial.println (headingct);
    Serial.println("--------------------------------------");
  */
  /*
    // Display Tilt angle in degrees
    Serial.print("Tilt raw: "); Serial.println((double)atan2((double)fabs(Az), (double)Ax) * 180 / PI);
    Serial.print("Tilt calibrated: "); Serial.println(tiltcnf);

    // Display Pitch and Roll angles in degrees
    Serial.print ("Pitch: "); Serial.println (pitch_print);
    Serial.print ("Roll: "); Serial.println (roll_print);
    Serial.println("--------------------------------------");
    Serial.println();
    Serial.println();

  */
  //delay(50);


}

// Read the raw measurements
void getRawValuesMag()
{
  Mx = (float)lsm.magData.x;
  My = (float)lsm.magData.y;
  Mz = (float)lsm.magData.z;
}

void getRawValuesAcc()
{
  Ax = (int)lsm.accelData.x;
  Ay = (int)lsm.accelData.y;
  Az = (int)lsm.accelData.z;
}


//transformation(float uncalibrated_values[3]) is the function of the magnetometer data correction
//uncalibrated_values[3] is the array of the non calibrated magnetometer data
//uncalibrated_values[3]: [0]=Xnc, [1]=Ync, [2]=Znc

void transformation_mag(float uncalibrated_values[3])
{
  //calibration_matrix[3][3] is the transformation matrix
  //replace M11, M12,..,M33 with your transformation matrix data
  double calibration_matrix[3][3] =
  {
    {1.101, 0.033, 0.023},
    { -0.006, 1.09, 0.019},
    {0.068, -0.022, 1.217}
    /*
        {1.025, 0.026, 0.004},
        {0.003, 1.024, 0.002},
        {0.018, -0.002, 1.251}
    */
  };
  //bias[3] is the bias
  //replace Bx, By, Bz with your bias data
  double bias[3] =
  {
    67.668,
    -125.549,
    -79.725
    /*
      -6.278,
      -101.76,
      -179.707
    */
  };
  //calculation
  for (int i = 0; i < 3; ++i) uncalibrated_values[i] = uncalibrated_values[i] - bias[i];
  float result[3] = {0, 0, 0};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      result[i] += calibration_matrix[i][j] * uncalibrated_values[j];
  for (int i = 0; i < 3; ++i) calibrated_mag_values[i] = result[i];
}


//transformation(float uncalibrated_values[3]) is the function of the accelerometer data correction
//uncalibrated_values[3] is the array of the non calibrated magnetometer data
//uncalibrated_values[3]: [0]=Xnc, [1]=Ync, [2]=Znc

void transformation_acc(float uncalibrated_values[3])
{
  //calibration_matrix[3][3] is the transformation matrix
  //replace ACC11(M11), ACC12(M12),..,ACC33(M33) with your transformation matrix data
  double calibration_matrix[3][3] =
  {
    {1.033, 0.000, -0.018},
    { -0.004, 0.994, 0.029},
    {0.022, -0.027, 0.976}
    /*
      {0.98, 0.002, 0.001},
      { -0.005, 1.003, 0.007},
      {0.011, -0.005, 1.019}
    */
  };
  //zero-g[3] is the zero-g offset
  //replace ACC10(Bx), ACC20(By), ACC30(Bz) with your zero-g offset data
  double zero_g[3] =
  {
    -18.618,
    -0.803,
    -2.705
    /*
      -30.988,
      10.754,
      -8.697
    */
  };
  //calculation
  for (int i = 0; i < 3; ++i) uncalibrated_values[i] = uncalibrated_values[i] - zero_g[i];
  float result[3] = {0, 0, 0};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      result[i] += calibration_matrix[i][j] * uncalibrated_values[j];
  for (int i = 0; i < 3; ++i) calibrated_acc_values[i] = result[i];
}
