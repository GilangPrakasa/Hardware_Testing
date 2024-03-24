#ifndef LED_STATUS_H_
#define LED_STATUS_H_

// #include "feeding_event.h"

#ifdef __cplusplus
extern "C" {
#endif

enum CONNECTIVITY_MODE {
    EFEEDER_STANDALONE,         // 0
    EFEEDER_MODE_CONNECTED_TO_AP
};

void initLedStatus(bool en);
void enableLedStatus(bool en);
void idleBehaviourBasedOnMode();
void ledsuccessbooting();
void leddrivererror();
void ledrunmotor(uint8_t blinkNumber);
void ledrunstate(uint8_t blinkNumber);
void ledrunerrorstate(uint8_t blinkNumber);

#ifdef __cplusplus
}
#endif
#endif // LED_STATUS_H_