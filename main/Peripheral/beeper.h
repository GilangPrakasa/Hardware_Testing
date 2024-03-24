#ifndef BEEPER_H_
#define BEEPER_H_

// #include "feeding_event.h"

#ifdef __cplusplus
extern "C" {
#endif
void beeperStartup();
void initBeeper(bool en);
void enableBeeper(bool en);
void turnOffBeeper();
void beepBeeper();
void beepdrivererror();
void statebeep(uint8_t loop);
void beep(uint32_t periodMs);

#ifdef __cplusplus
}
#endif
#endif // BEEPER_H_