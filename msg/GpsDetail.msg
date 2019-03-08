  Header header
  float32 hdop      # /*!< horizontal dilution of precision */
  float32 pdop      # /*!< position dilution of precision */
  float32 fix       # /*!< the state of GPS fix */
  float32 gnssStatus #/*!< vertical position accuracy (mm) */
  float32 hacc       #/*!< horizontal position accuracy (mm) */
  float32 sacc      # /*!< the speed accuracy (cm/s) */
  uint32 usedGPS    #/*!< the number of GPS satellites used for pos fix */
  uint32  usedGLN   #/*!< the number of GLONASS satellites used for pos fix */
  uint16  NSV     #/*!< the total number of satellites used for pos fix */
  uint16  GPScounter #/*!< the accumulated times of sending GPS data  */

  # Doesn't work in simulation
