#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* Select one output owner. The bench image never starts a competition task. */
#ifndef APP_MOTOR1_TEST_ENABLED
#define APP_MOTOR1_TEST_ENABLED 1
#endif

#ifndef APP_OLED_ENABLED
#define APP_OLED_ENABLED 0
#endif

/* Read-only bench telemetry, independent from the full button/menu UI. */
#ifndef APP_MOTOR1_STATUS_OLED_ENABLED
#define APP_MOTOR1_STATUS_OLED_ENABLED 1
#endif

#define APP_MOTOR1_OLED_PERIOD_MS 100U

#if APP_MOTOR1_TEST_ENABLED && APP_OLED_ENABLED
#error "Motor 1 bench testing owns the buttons; disable the OLED UI."
#endif

#endif
