#ifndef ElegantOTA_h
#define ElegantOTA_h

#include "Arduino.h"
#include "stdlib_noniso.h"
#include "WiFi.h"
#include "WiFiClient.h"
#include "WebServer.h"
#include "Update.h"
#include "Adafruit_AVRProg.h"

class ElegantOtaClass
{
public:
  ElegantOtaClass();
  void setID(const char *id);
  void begin(WebServer *server, const char *username = "", const char *password = "");

  void updateController(uint8_t * version);
  void flashImage();

private:

  WebServer *_server;

  String uploadProg();
  void setupProg();

  char _username[64];
  char _password[64];
  char _id[128];

  const static uint8_t versionLen = 3;
  uint8_t controllerVersion[versionLen] = {1,4,1};
};
extern String response;
extern ElegantOtaClass ElegantOTA;
#endif
