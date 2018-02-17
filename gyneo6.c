#include "gyneo6.h"

#include <esp/uart.h>
#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include <string.h>

#include <parser/gpgga.h>
#include <parser/gpgll.h>
#include <parser/gprmc.h>
#include <parser/gpvtg.h>
#include <parser/gpgsv.h>

#undef CALLBACK_DEBUG
// #define CALLBACK_DEBUG
#ifdef CALLBACK_DEBUG
#define debug(s, ...) printf(__FILE__ ", %d : DEBUG : " s "\r\n", __LINE__, ##__VA_ARGS__)
#else
#define debug(s, ...)
#endif

#define CALLBACK_INFO
#ifdef CALLBACK_INFO
#define info(s, ...) printf("INFO: " s "\r\n", ##__VA_ARGS__)
#else
#define info(s, ...)
#endif

#define MAX_SIZE_OF
#define SPEED_STRING_LENGTH 40

// uart buffer of 80 chars that can be used
struct gyneo6_buffer_t
{
  char uartBuffer[81];    // 2 full gps messages
  int currentPosition;    // the current position we are writing at
  nmea_s *currentMessage; // a pointer
  bool hasMessage;
};
typedef struct gyneo6_buffer_t gyneo6_buffer;

gyneo_info gyneo_info_buffer;

const TickType_t xDelay = 200 / portTICK_PERIOD_MS;

// A buffer that will alow, at least 1 message
gyneo6_buffer aBuffer;

void clearMessageBuffer(gyneo6_buffer *buffer)
{
  memset(buffer->uartBuffer, 0, buffer->currentPosition);
  buffer->currentPosition = 0;
}

bool gyneo6_hasMessage()
{
  return aBuffer.hasMessage;
}

static void updateStatusFromGPGGA(nmea_gpgga_s *message)
{
  gyneo_info_buffer.time = message->time;
  gyneo_info_buffer.longitude = message->longitude;
  gyneo_info_buffer.latitude = message->latitude;
  gyneo_info_buffer.altitude = message->altitude;
  gyneo_info_buffer.altitude_unit = message->altitude_unit;

  info("new data: lon %.4f, lat %.4f, alt %i",
       gyneo_info_buffer.longitude.degrees + gyneo_info_buffer.longitude.minutes / 60.0,
       gyneo_info_buffer.latitude.degrees + gyneo_info_buffer.latitude.minutes / 60.0,
       gyneo_info_buffer.altitude);
}

static void updateStatusFromGPGLL(nmea_gpgll_s *message)
{
  gyneo_info_buffer.time = message->time;
  gyneo_info_buffer.longitude = message->longitude;
  gyneo_info_buffer.latitude = message->latitude;
  info("new data: lon %.4f, lat %.4f",
       gyneo_info_buffer.longitude.degrees + gyneo_info_buffer.longitude.minutes / 60.0,
       gyneo_info_buffer.latitude.degrees + gyneo_info_buffer.latitude.minutes / 60.0);
}

static void updateStatusFromGPVTG(nmea_gpvtg_s *message)
{
  gyneo_info_buffer.heading = message->heading;
  gyneo_info_buffer.speed_km = message->speed_km;
  if (gyneo_info_buffer.maxSpeed_km < gyneo_info_buffer.speed_km)
  {
    gyneo_info_buffer.maxSpeed_km = gyneo_info_buffer.speed_km;
  }
  gyneo_info_buffer.speed_knots = message->speed_knots;

  info("new data: heading %.4f, speed_km %.4f, maxspeed_km %.4f",
       gyneo_info_buffer.heading, gyneo_info_buffer.speed_km,
       gyneo_info_buffer.maxSpeed_km);
}

static void updateStatusFromGPGSV(nmea_gpgsv_s *message)
{
  gyneo_info_buffer.number_of_satelites = message->numberOfSatelietesInView;
}

void updateStatus(gyneo6_buffer *buffer)
{
  nmea_s *currentMessage = buffer->currentMessage;
  debug("update status for type %i", currentMessage->type);
  switch (currentMessage->type)
  {
  case (NMEA_GPGGA):
  {
    // They provide almost identical info
    updateStatusFromGPGGA((nmea_gpgga_s *)currentMessage);
    break;
  }
  case (NMEA_GPGLL):
  {
    updateStatusFromGPGLL((nmea_gpgll_s *)currentMessage);
    break;
  }
  case (NMEA_GPRMC):
  {
    updateStatusFromGPGLL((nmea_gpgll_s *)currentMessage);
    break;
  }
  case (NMEA_GPVTG):
  {
    updateStatusFromGPVTG((nmea_gpvtg_s *)currentMessage);
    break;
  }
  case (NMEA_GPGSV):
  {
    updateStatusFromGPGSV((nmea_gpgsv_s *)currentMessage);
  }
  default:
  {
    debug("Unknown type %i! Cannot update message", currentMessage->type);
  }
  }
}

void gyneo6_currentLocation(char *buffer, size_t len)
{
  memset(buffer, 0, len);
  sprintf(buffer, "lon %.4f, lat %.4f",
          gyneo_info_buffer.longitude.degrees + gyneo_info_buffer.longitude.minutes / 60.0,
          gyneo_info_buffer.latitude.degrees + gyneo_info_buffer.latitude.minutes / 60.0);
}

void gyneo6_currentTimeFix(char *buffer, size_t len)
{
  strftime(buffer, len, "%a %H:%M:%S", &gyneo_info_buffer.time);
}

void gyneo6_currentSpeed(char *buffer, size_t len)
{
  memset(buffer, 0, len);
  sprintf(buffer, "%.4f %s", gyneo_info_buffer.speed_km, "km/h");
}

void gyneo6_maxSpeed(char *buffer, size_t len)
{
  memset(buffer, 0, len);
  sprintf(buffer, "%.4f %s", gyneo_info_buffer.maxSpeed_km, "km/h");
}

static void injectEndOfMessage(gyneo6_buffer *buffer)
{
  buffer->uartBuffer[buffer->currentPosition] = NMEA_END_CHAR_1;
  ++buffer->currentPosition;
  buffer->uartBuffer[buffer->currentPosition] = NMEA_END_CHAR_2;
  ++buffer->currentPosition;
}

void gyneo6_task(void *taskPointer)
{
  int currentValue = 0;
  uint16_t i;
  gyneo6_buffer *buffer = (gyneo6_buffer *)taskPointer;

  while (true)
  {
    i = 0;
    while (true)
    {
      currentValue = uart_getc_nowait(GYNEO6_UART);
      ++i;

      if (currentValue == -1)
      {
        break;
      }

      buffer->uartBuffer[buffer->currentPosition] = currentValue;
      ++buffer->currentPosition;

      if (buffer->currentPosition > NMEA_PREFIX_LENGTH)
      {
        if (NMEA_END_CHAR_2 == currentValue || i > NMEA_MAX_LENGTH)
        {
          debug("have a nice well formed message, or too long");
          break;
        }
        else if (NMEA_START_CHAR == currentValue && NMEA_END_CHAR_2 != buffer->uartBuffer[buffer->currentPosition - 1])
        {
          info("Droped message: %s", buffer->uartBuffer);
          clearMessageBuffer(buffer);
          buffer->uartBuffer[buffer->currentPosition] = currentValue;
          ++buffer->currentPosition;
        }
      }
    }

    if (nmea_validate(buffer->uartBuffer, buffer->currentPosition, 0) == 0)
    {
      if (buffer->currentMessage != NULL)
      {
        nmea_free(buffer->currentMessage);
      }
      // nmea message can be accepted
      buffer->currentMessage = nmea_parse(buffer->uartBuffer, buffer->currentPosition, 0);
      clearMessageBuffer(buffer);
      buffer->hasMessage = buffer->currentMessage != NULL;
      if (buffer->hasMessage)
      {
        updateStatus(buffer);
      }
    }
    else if (strcmp(buffer->uartBuffer + buffer->currentPosition - 5, "clear") == 0)
    {
      debug("Drop package via clear: %s", buffer->uartBuffer);
      buffer->hasMessage = false;
      if (buffer->currentMessage != NULL)
      {
        nmea_free(buffer->currentMessage);
      }
      clearMessageBuffer(buffer);
      uart_flush_rxfifo(GYNEO6_UART);
    }
    else if (strcmp(buffer->uartBuffer + buffer->currentPosition - 5, "speed") == 0)
    {
      debug("Request speed: %s", buffer->uartBuffer);
      char speedString[SPEED_STRING_LENGTH];
      gyneo6_currentSpeed(speedString, SPEED_STRING_LENGTH);
      printf(speedString);
      gyneo6_maxSpeed(speedString, SPEED_STRING_LENGTH);
      printf(speedString);
      buffer->hasMessage = false;
      if (buffer->currentMessage != NULL)
      {
        nmea_free(buffer->currentMessage);
      }
      clearMessageBuffer(buffer);
      uart_flush_rxfifo(GYNEO6_UART);
    }

    else if (NMEA_MAX_LENGTH < buffer->currentPosition)
    {
      debug("Drop package: %s", buffer->uartBuffer);
      clearMessageBuffer(buffer);
    }
    vTaskDelay(xDelay);
  }
}

gyneo_info *gyneo6_info()
{
  return &gyneo_info_buffer;
}

// do init stuff
void gyneo6_init()
{
  // initialisation of the buffer
  aBuffer.currentPosition = 0;
  aBuffer.hasMessage = false;
  aBuffer.currentMessage = NULL;

  // set the uart
  uart_set_baud(GYNEO6_UART, 9600);

  // create a task that will read the uart fifo incoming and stores it in our buffer
  xTaskCreate(gyneo6_task, "gyneo6_task", 1024, &aBuffer, 3, NULL);
}