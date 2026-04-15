#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/// Initialise the button GPIO and start the button-polling task.
void button_service_init(void);

#ifdef __cplusplus
}
#endif
