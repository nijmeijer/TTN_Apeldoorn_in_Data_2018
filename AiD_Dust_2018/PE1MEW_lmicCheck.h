
/******* Setup LoRaWAN stack LMIC 1.5 ********/

// Verifications on LMIC configuration
#ifdef DISABLE_JOIN
  #error "DISABLE_JOIN defined in LMIC, remove define!"
#endif 

//#ifdef LMIC_DEBUG_LEVEL
  // #error "LMIC_DEBUG_LEVEL defined in LMIC, remove define!"
//#endif 

#ifndef DISABLE_PING
  #error "DISABLE_PING not defined in LMIC, add define!"
#endif 

#ifndef DISABLE_BEACONS
  #error "DISABLE_BEACONS not defined in LMIC, add define!"
#endif 

#ifndef DISABLE_MCMD_PING_SET
  #error "DISABLE_MCMD_PING_SET not defined in LMIC, add define!"
#endif 

#ifndef DISABLE_MCMD_BCNI_ANS
  #error "DISABLE_MCMD_BCNI_ANS not defined in LMIC, add define!"
#endif 

#ifdef DISABLE_INVERT_IQ_ON_RX
  #error "DISABLE_INVERT_IQ_ON_RX defined in LMIC, remove define!"
#endif 

