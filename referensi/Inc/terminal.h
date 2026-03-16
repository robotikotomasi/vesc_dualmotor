/**
  ******************************************************************************
  * @file    terminal.h
  * @brief   Terminal command interface (VESC-style)
  *          Register and process text commands via UART
  ******************************************************************************
  */

#ifndef TERMINAL_H
#define TERMINAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* Callback for terminal command */
typedef void (*terminal_callback_t)(int argc, const char **argv);

/* Callback for sending response text */
typedef void (*terminal_write_t)(const char *str);

/* Maximum number of registered commands */
#define TERMINAL_MAX_COMMANDS  16

/*============================================================================
 * Terminal API
 *============================================================================*/

/* Initialize terminal */
void terminal_init(void);

/* Set the write callback for output */
void terminal_set_write_callback(terminal_write_t write_fn);

/* Process a command string */
void terminal_process_string(const char *str);

/* Register a custom command */
bool terminal_register_command(const char *name,
                               const char *help,
                               terminal_callback_t callback);

/* Unregister a custom command */
void terminal_unregister_command(const char *name);

#ifdef __cplusplus
}
#endif

#endif /* TERMINAL_H */
