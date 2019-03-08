  Header header
  int16  current            #  /*!< ESC current, unit: mA */
  int16  speed              # /*!< ESC speed, unit: rpm */
  uint16 voltage             # /*!< Input power from battery to ESC, unit: mV */
  int16  temperature         # /*!< ESC temperature, unit: degree C */
  uint16 stall                # /*!< Motor is stall */
  uint16 empty                # /*!< Motor has no load */
  uint16 unbalanced           # /*!< Motor speed is unbalanced */
  uint16 escDisconnected       #/*!< ESC is disconnected */
  uint16 temperatureHigh       #/*!< Temperature is high */
