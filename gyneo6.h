

#ifndef GYNEO6_H_
#define GYNEO6_H_

#include <time.h>
#include <nmea/nmea.h>
// CONFIGURATION PARAMETERS
#define GYNEO6_UART 0

struct gyneo6_info_t
{
  struct 
  tm time;
  nmea_position longitude;
  nmea_position latitude;
  int altitude;
  char altitude_unit;
  float heading;
  float speed_km;
  float maxSpeed_km;
  float speed_knots;

};

typedef struct gyneo6_info_t gyneo_info;

extern void gyneo6_init();
extern gyneo_info* gyneo6_info();
extern void gyneo6_currentTimeFix(char* buffer, size_t len);
extern void gyneo6_currentLocation(char *buffer, size_t len);
extern void gyneo6_currentSpeed(char *buffer, size_t len);
extern void gyneo6_maxSpeed(char *buffer, size_t len);

#endif // GYNEO6_H_