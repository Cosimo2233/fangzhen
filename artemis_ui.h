#ifndef ARTEMIS_UI_H
#define ARTEMIS_UI_H

#include <stdbool.h>
#include <stdint.h>

#include "artemis_controller.h"

typedef enum {
    ARTEMIS_UI_STATE_MENU,
    ARTEMIS_UI_STATE_STARTING,
    ARTEMIS_UI_STATE_RUNNING,
    ARTEMIS_UI_STATE_DONE,
    ARTEMIS_UI_STATE_ERROR
} artemis_ui_state_t;

typedef struct {
    bool key0_pressed;
    bool key1_pressed;
} artemis_ui_event_t;

void artemis_ui_init(void);
artemis_ui_event_t artemis_ui_poll(uint32_t now_ms);
void artemis_ui_show(artemis_task_id_t task_id, artemis_ui_state_t state);

#endif /* ARTEMIS_UI_H */
