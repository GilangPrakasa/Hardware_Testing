#include "button.h"
#include <esp_log.h>
#include "runbtn.h"
#include "beeper.h"

#define cmdbutton 27
#define bootbtn 0


static int counter = 0;

static button_t btn;
static button_t btn2;

void cbrunbtn(button_t *btn, button_state_t state)
{
    if (state == BUTTON_CLICKED) 
    {
        beep(50);
        counter++;
        if (counter > 1)
        {
            counter = 0;
        }
    }
    // ESP_LOGE("BUTTON", "BTN OKE: %d", counter);
}

void cbbootbtn(button_t *btn, button_state_t state)
{
    if (state == BUTTON_CLICKED) 
    {
        beep(50);
        counter--;
    }
    // ESP_LOGE("BUTTON", "BTN OKE: %d", counter);
}

void initrunbtn()
{
    btn.gpio = (gpio_num_t) cmdbutton;
    btn.pressed_level = 0;
    btn.internal_pull = true;
    btn.autorepeat = false;
    btn.callback = cbrunbtn;

    ESP_ERROR_CHECK(button_init(&btn));
}

void initbootbtn()
{
    {
    btn2.gpio = (gpio_num_t) bootbtn;
    btn2.pressed_level = 0;
    btn2.internal_pull = true;
    btn2.autorepeat = false;
    btn2.callback = cbbootbtn;

    ESP_ERROR_CHECK(button_init(&btn2));
}
}

int getcounter()
{
    return counter;
} 