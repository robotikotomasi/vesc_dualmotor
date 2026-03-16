/**
  ******************************************************************************
  * @file    terminal.c
  * @brief   Terminal command interface (VESC-style)
  *          Text-based command processing for diagnostics and configuration
  *
  * Built-in commands:
  *  help          - List all commands
  *  status        - Show motor status
  *  faults        - Show fault codes
  *  detect_motor  - Run motor detection
  *  reset_faults  - Clear fault codes
  *  save_config   - Save configuration to flash
  *  load_config   - Load configuration from flash
  *  set_defaults  - Reset configuration to defaults
  *  reboot        - Reboot the system
  ******************************************************************************
  */

#include "terminal.h"
#include "mc_tasks.h"
#include "flash_helper.h"
#include "timeout.h"
#include <string.h>
#include <stdio.h>

/*============================================================================
 * Private Data
 *============================================================================*/
typedef struct {
    const char *name;
    const char *help;
    terminal_callback_t callback;
} TerminalCommand_t;

static TerminalCommand_t customCommands[TERMINAL_MAX_COMMANDS];
static int numCustomCommands = 0;
static terminal_write_t writeFunc = NULL;
static char outputBuf[256];

/*============================================================================
 * Helper: write output
 *============================================================================*/
static void term_write(const char *str)
{
    if (writeFunc) {
        writeFunc(str);
    }
}

/*============================================================================
 * Helper: parse command string into argc/argv
 *============================================================================*/
static int parse_args(char *str, const char **argv, int max_args)
{
    int argc = 0;
    char *p = str;
    while (*p && argc < max_args) {
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '\0') break;
        argv[argc++] = p;
        while (*p && *p != ' ' && *p != '\t') p++;
        if (*p) *p++ = '\0';
    }
    return argc;
}

/*============================================================================
 * Built-in Commands
 *============================================================================*/
static void cmd_help(void)
{
    term_write("== Built-in Commands ==\r\n");
    term_write("  help          - Show this help\r\n");
    term_write("  status        - Show motor status\r\n");
    term_write("  faults        - Show fault codes\r\n");
    term_write("  detect_motor  - Run motor R/L/flux detection\r\n");
    term_write("  detect_hall   - Run Hall sensor detection\r\n");
    term_write("  reset_faults  - Clear fault codes\r\n");
    term_write("  save_config   - Save configuration to flash\r\n");
    term_write("  load_config   - Load configuration from flash\r\n");
    term_write("  set_defaults  - Reset configuration to defaults\r\n");
    term_write("  reboot        - Reboot the system\r\n");

    if (numCustomCommands > 0) {
        term_write("== Custom Commands ==\r\n");
        for (int i = 0; i < numCustomCommands; i++) {
            snprintf(outputBuf, sizeof(outputBuf), "  %-14s - %s\r\n",
                     customCommands[i].name,
                     customCommands[i].help ? customCommands[i].help : "");
            term_write(outputBuf);
        }
    }
}

static void cmd_status(void)
{
    for (uint8_t m = 0; m < NBR_OF_MOTORS; m++) {
        const char *name = (m == M_RIGHT) ? "RIGHT" : "LEFT";
        snprintf(outputBuf, sizeof(outputBuf),
                 "Motor %s: state=%d mode=%d en=%d fault=0x%04X\r\n",
                 name,
                 (int)motor[m].state,
                 (int)motor[m].ctrlMode,
                 (int)motor[m].enable,
                 motor[m].faultCode);
        term_write(outputBuf);

        snprintf(outputBuf, sizeof(outputBuf),
                 "  Speed=%.1f ERPM  Duty=%.3f  Iq=%.2f A  Id=%.2f A\r\n",
                 motor[m].speedNow,
                 motor[m].dutyNow,
                 motor[m].iqNow,
                 motor[m].idNow);
        term_write(outputBuf);

        snprintf(outputBuf, sizeof(outputBuf),
                 "  Tacho=%ld  TachoAbs=%ld\r\n",
                 (long)motor[m].hall.tachometer,
                 (long)motor[m].hall.tachometerAbs);
        term_write(outputBuf);

        snprintf(outputBuf, sizeof(outputBuf),
                 "  Ah=%.4f  AhChg=%.4f  Wh=%.4f  WhChg=%.4f\r\n",
                 motor[m].ampHours,
                 motor[m].ampHoursCharged,
                 motor[m].wattHours,
                 motor[m].wattHoursCharged);
        term_write(outputBuf);

        snprintf(outputBuf, sizeof(outputBuf),
                 "  TempScale=%.2f  BatScale=%.2f\r\n",
                 motor[m].tempCurrentScale,
                 motor[m].batCurrentScale);
        term_write(outputBuf);
    }

    snprintf(outputBuf, sizeof(outputBuf),
             "Vbat=%.1f V  Temp=%.1f C\r\n",
             batVoltage, boardTemp);
    term_write(outputBuf);
}

static void cmd_faults(void)
{
    for (uint8_t m = 0; m < NBR_OF_MOTORS; m++) {
        const char *name = (m == M_RIGHT) ? "RIGHT" : "LEFT";
        uint16_t fc = motor[m].faultCode;
        if (fc == MC_NO_FAULTS) {
            snprintf(outputBuf, sizeof(outputBuf), "Motor %s: No faults\r\n", name);
            term_write(outputBuf);
        } else {
            snprintf(outputBuf, sizeof(outputBuf), "Motor %s: Fault 0x%04X", name, fc);
            term_write(outputBuf);
            if (fc & MC_OVER_VOLT)  term_write(" OVER_VOLT");
            if (fc & MC_UNDER_VOLT) term_write(" UNDER_VOLT");
            if (fc & MC_OVER_TEMP)  term_write(" OVER_TEMP");
            if (fc & MC_OVER_CURR)  term_write(" OVER_CURR");
            term_write("\r\n");
        }
    }
}

static void cmd_detect_motor(void)
{
    term_write("Starting motor R/L/flux detection on motor RIGHT...\r\n");
    MC_DetectMotorRL(M_RIGHT);
    term_write("Detection started. Use 'status' to check results.\r\n");
}

static void cmd_detect_hall(void)
{
    term_write("Starting Hall sensor detection on motor RIGHT...\r\n");
    MC_DetectHallFOC(M_RIGHT);
    term_write("Detection started. Use 'status' to check results.\r\n");
}

static void cmd_reset_faults(void)
{
    MC_FaultAck(M_RIGHT);
    MC_FaultAck(M_LEFT);
    term_write("Faults cleared.\r\n");
}

static void cmd_save_config(void)
{
    mc_configuration *conf = flash_helper_get_config();
    if (flash_helper_save_config(conf)) {
        term_write("Configuration saved to flash.\r\n");
    } else {
        term_write("ERROR: Failed to save configuration!\r\n");
    }
}

static void cmd_load_config(void)
{
    mc_configuration conf;
    if (flash_helper_load_config(&conf)) {
        term_write("Configuration loaded from flash.\r\n");
    } else {
        term_write("ERROR: No valid configuration in flash. Using defaults.\r\n");
        flash_helper_set_defaults(&conf);
    }
}

static void cmd_set_defaults(void)
{
    mc_configuration *conf = flash_helper_get_config();
    flash_helper_set_defaults(conf);
    term_write("Configuration reset to defaults (not saved to flash).\r\n");
}

static void cmd_reboot(void)
{
    term_write("Rebooting...\r\n");
    MC_StopMotor(M_RIGHT);
    MC_StopMotor(M_LEFT);
    NVIC_SystemReset();
}

/*============================================================================
 * Public API
 *============================================================================*/

void terminal_init(void)
{
    numCustomCommands = 0;
    writeFunc = NULL;
    memset(customCommands, 0, sizeof(customCommands));
}

void terminal_set_write_callback(terminal_write_t write_fn)
{
    writeFunc = write_fn;
}

void terminal_process_string(const char *str)
{
    char cmdBuf[128];
    strncpy(cmdBuf, str, sizeof(cmdBuf) - 1);
    cmdBuf[sizeof(cmdBuf) - 1] = '\0';

    const char *argv[8];
    int argc = parse_args(cmdBuf, argv, 8);
    if (argc == 0) return;

    const char *cmd = argv[0];

    /* Built-in commands */
    if (strcmp(cmd, "help") == 0) { cmd_help(); return; }
    if (strcmp(cmd, "status") == 0) { cmd_status(); return; }
    if (strcmp(cmd, "faults") == 0) { cmd_faults(); return; }
    if (strcmp(cmd, "detect_motor") == 0) { cmd_detect_motor(); return; }
    if (strcmp(cmd, "detect_hall") == 0) { cmd_detect_hall(); return; }
    if (strcmp(cmd, "reset_faults") == 0) { cmd_reset_faults(); return; }
    if (strcmp(cmd, "save_config") == 0) { cmd_save_config(); return; }
    if (strcmp(cmd, "load_config") == 0) { cmd_load_config(); return; }
    if (strcmp(cmd, "set_defaults") == 0) { cmd_set_defaults(); return; }
    if (strcmp(cmd, "reboot") == 0) { cmd_reboot(); return; }

    /* Custom commands */
    for (int i = 0; i < numCustomCommands; i++) {
        if (strcmp(cmd, customCommands[i].name) == 0) {
            if (customCommands[i].callback) {
                customCommands[i].callback(argc, argv);
            }
            return;
        }
    }

    snprintf(outputBuf, sizeof(outputBuf), "Unknown command: '%s'. Type 'help' for list.\r\n", cmd);
    term_write(outputBuf);
}

bool terminal_register_command(const char *name, const char *help,
                                terminal_callback_t callback)
{
    if (numCustomCommands >= TERMINAL_MAX_COMMANDS) return false;
    customCommands[numCustomCommands].name = name;
    customCommands[numCustomCommands].help = help;
    customCommands[numCustomCommands].callback = callback;
    numCustomCommands++;
    return true;
}

void terminal_unregister_command(const char *name)
{
    for (int i = 0; i < numCustomCommands; i++) {
        if (strcmp(name, customCommands[i].name) == 0) {
            /* Shift remaining commands */
            for (int j = i; j < numCustomCommands - 1; j++) {
                customCommands[j] = customCommands[j + 1];
            }
            numCustomCommands--;
            return;
        }
    }
}
