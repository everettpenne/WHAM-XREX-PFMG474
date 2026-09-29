/*
 * command_table.c -- the product's SCPI command table and the dispatch
 * entry point for a received line. Matching rules: scpi_parser.c.
 *
 * To add a command
 * -----------------
 *  1. Implement the handler in the matching src/app/commands/cmd_*.c
 *     (file map: commands.h)
 *  2. Declare it in commands.h
 *  3. Add a { "PATTern:MNEMonic?", handler } row to command_table[]
 *     below. Nothing else changes -- the table is flat (no separate
 *     tree structure to build), so adding a new leaf or a whole new
 *     subsystem is always just one more row.
 */
#include "commands.h"
#include "cmd_common.h"
#include "scpi_parser.h"
#include "boot_jump.h"
#include "qspi_test.h"
#include "pfm_input.h"

static const scpi_command_t command_table[] = {
    /* Common commands (IEEE 488.2) */
    { "*IDN?",              cmd_idn          },

    /* Serial-bootloader entry -- excluded entirely (not just an
       unreachable row) when BOOT_JUMP_FEATURE_ENABLED is 0, so BOOT
       falls through to "ERR 1 Unknown command" like any other
       unrecognized mnemonic. See boot_jump.h. */
#if (BOOT_JUMP_FEATURE_ENABLED != 0)
    { "BOOT",                cmd_boot         },
#endif

    /* PFM table upload -- see cmd_table.c's own header comment on
       these four handlers. */
    { "TABle:BEGin",         cmd_table_begin  },
    { "TABle:STEP",          cmd_table_step   },
    { "TABle:END",           cmd_table_end    },
    { "TABle?",              cmd_table_query  },

    /* Runtime-configurable PFM_MAX_CARRIER_FREQ_HZ -- see cmd_table.c's
       own header comment on cmd_config_max_carrier_hz(). */
    { "CONFig:MaxCarrierHz",   cmd_config_max_carrier_hz       },
    { "CONFig:MaxCarrierHz?",  cmd_config_max_carrier_hz_query },

    /* Begins PWM output -- see cmd_table.c's own header comment on
       cmd_fire(). */
    { "FIRE",                cmd_fire         },

    /* PFM table-advance ISR timing diagnostic -- see cmd_table.c's own
       header comment on cmd_pfm_diag(). */
    { "PFM:DIAG?",           cmd_pfm_diag     },

    /* TEMPORARY, see commands.h -- raw per-call gap log behind
       PFM:DIAG?'s aggregate max-gap number. */
    { "PFM:GAPLOG?",         cmd_pfm_gaplog   },

    /* Reports the compile-time HRTIM channel count -- see cmd_config.c's
       own header comment on cmd_config_channels(). */
    { "CONFig:CHANnels?",    cmd_config_channels },

    /* Runtime-configurable PID_LOOP_RATE_HZ -- see cmd_config.c's own
       header comment on cmd_config_pid_rate(). */
    { "CONFig:PIDRate",      cmd_config_pid_rate       },
    { "CONFig:PIDRate?",     cmd_config_pid_rate_query },

    /* Runtime-configurable PFM_TURNON_FREQ_HZ/PFM_MAX_FREQ_HZ/
       PFM_MAX_CURRENT_A_PER_CHANNEL -- see cmd_config.c's own header
       comments on cmd_config_turnon_hz()/cmd_config_max_freq_hz()/
       cmd_config_max_current(). */
    { "CONFig:TURNONHz",     cmd_config_turnon_hz        },
    { "CONFig:TURNONHz?",    cmd_config_turnon_hz_query  },
    { "CONFig:MAXFREQHz",    cmd_config_max_freq_hz       },
    { "CONFig:MAXFREQHz?",   cmd_config_max_freq_hz_query },
    { "CONFig:MAXCURRent",   cmd_config_max_current       },
    { "CONFig:MAXCURRent?",  cmd_config_max_current_query },

    /* Runtime-configurable PID_OUTPUT_MAX_SLEW_HZ_PER_TICK/
       FAULT_RAMP_DOWN_TIME_S -- see cmd_config.c's own header comments
       on cmd_config_slew_rate()/cmd_config_fault_ramp_time(). */
    { "CONFig:SLEWRate",        cmd_config_slew_rate             },
    { "CONFig:SLEWRate?",       cmd_config_slew_rate_query       },
    { "CONFig:FaultRampTime",   cmd_config_fault_ramp_time       },
    { "CONFig:FaultRampTime?",  cmd_config_fault_ramp_time_query },

    /* Runtime-configurable fault-pin polarities -- see cmd_config.c's
       own header comment on ConfigFaultPolaritySet(). */
    { "CONFig:FaultPolarity:WATER",    cmd_config_fault_polarity_water          },
    { "CONFig:FaultPolarity:WATER?",   cmd_config_fault_polarity_water_query    },
    { "CONFig:FaultPolarity:TEMP",     cmd_config_fault_polarity_temp           },
    { "CONFig:FaultPolarity:TEMP?",    cmd_config_fault_polarity_temp_query     },
    { "CONFig:FaultPolarity:ENERPRO",  cmd_config_fault_polarity_enerpro        },
    { "CONFig:FaultPolarity:ENERPRO?", cmd_config_fault_polarity_enerpro_query  },
    { "CONFig:FaultPolarity:OCP",      cmd_config_fault_polarity_ocp            },
    { "CONFig:FaultPolarity:OCP?",     cmd_config_fault_polarity_ocp_query      },

    /* PC10/HRTIM1_FLT6 hardware fault status/clear -- see cmd_state.c's
       own header comment on cmd_fault_query()/cmd_fault_clear(). */
    { "FAULT?",              cmd_fault_query  },
    { "FAULT:CLEar",         cmd_fault_clear  },

    /* Top-level operating-state machine (IDLE/ARMED/FIRING/FAULT) --
       see cmd_state.c's own header comment on cmd_arm()/cmd_disarm()/
       cmd_state_query() and state_machine.h for the full design. */
    { "ARM",                 cmd_arm          },
    { "DISARM",              cmd_disarm       },
    { "STATE?",              cmd_state_query  },

    /* Telemetry contract (docs/telemetry.md, Phase 1) -- see cmd_system.c's
       own header comment on cmd_sys_*(). */
    { "SYS:TIME?",           cmd_sys_time         },
    { "SYS:TELEM?",          cmd_sys_telem        },
    { "SYS:EVENT",           cmd_sys_event        },
    { "SYS:EVENT?",          cmd_sys_event_query  },
    { "SYS:EVLOG?",          cmd_sys_evlog        },

    /* In-application dual-bank firmware update over this serial link,
       added 2026-09-24 -- see cmd_fwupdate.c (protocol) and flash_bank.h (mechanism). */
    { "FWUPdate:BEGin",      cmd_fwup_begin       },
    { "FWUPdate:DATA",       cmd_fwup_data        },
    { "FWUPdate:END",        cmd_fwup_end         },
    { "FWUPdate:SWAP",       cmd_fwup_swap        },
    { "FWUPdate:ROLLback",   cmd_fwup_rollback    },
    { "FWUPdate:ABORt",      cmd_fwup_abort       },
    { "FWUPdate:STATus?",    cmd_fwup_status      },

    /* Bench-only debug override, added 2026-09-21 -- see cmd_state.c's
       own header comment on cmd_debug_fault_bypass() and
       state_machine.h's SM_SetFaultBypassEnabled() for the full
       reasoning/safety warning. NOT BUILD_TARGET-guarded -- available
       on both controller and simulator builds. */
    { "DEBUG:FAULT:BYPASS",  cmd_debug_fault_bypass       },
    { "DEBUG:FAULT:BYPASS?", cmd_debug_fault_bypass_query },

    /* External-enable interlock (PF13, MOVED 2026-09-17 from PF15) --
       see commands.h's own comment on cmd_ext_enable(). Added
       2026-09-16. Two-level namespace (mandatory "EXT"/"ENA"/"INP",
       matching this project's existing SOURce:ENAble-style
       abbreviation convention) rather than one compound word -- a
       single "EXTEnable" token would only let "EXTE" (an
       unrecognizable fragment) be typed as its short form, since
       scpi_token_match() only recognizes a LEADING uppercase run, not
       caps resuming mid-word. */
    { "EXTernal:ENAble",     cmd_ext_enable        },
    { "EXTernal:ENAble?",    cmd_ext_enable_query  },
    { "EXTernal:INPut?",     cmd_ext_enable_input_query },

    /* External trigger (rising edge on PF15 fires a shot while ARMED,
       and ONLY from ARMED) -- see commands.h's own comment on
       cmd_ext_trigger(). Added 2026-09-16; RESTRUCTURED 2026-09-17 --
       PF15 now backs trigger only, no longer coupled to
       EXTernal:ENAble (moved to PF13, above). */
    { "EXTernal:TRIGger",       cmd_ext_trigger             },
    { "EXTernal:TRIGger?",      cmd_ext_trigger_query       },
    { "EXTernal:TRIGger:INPut?", cmd_ext_trigger_input_query },

    /* Emergency-stop commands REMOVED 2026-09-21 (PG10 was NRST, not a
       usable GPIO -- see state_machine.h / docs/changelog.txt). */

    /* Generic diagnostic output (PD1, "GPOut_12") -- see commands.h's
       own comment on cmd_diag_gpout12(). Added 2026-09-16. */
    { "DIAGnostic:GPOut12",  cmd_diag_gpout12       },
    { "DIAGnostic:GPOut12?", cmd_diag_gpout12_query },

    /* Second, independent generic diagnostic output (PD0, "GPOut_11")
       -- see commands.h's own comment on cmd_diag_gpout11(). Added
       2026-09-17 after PD1 was found double-used for PF15 AND PG10
       testing. */
    { "DIAGnostic:GPOut11",  cmd_diag_gpout11       },
    { "DIAGnostic:GPOut11?", cmd_diag_gpout11_query },

    /* Third/fourth generic diagnostic outputs (PG8/PG9, "GPOut_09"/
       "GPOut_10") -- see commands.h's own comment on cmd_diag_gpout09()/
       cmd_diag_gpout10(). Added 2026-09-18, closing the Transrex
       simulator's OCP fiber-transmitter gap for XR1/XR2. */
    { "DIAGnostic:GPOut09",  cmd_diag_gpout09       },
    { "DIAGnostic:GPOut09?", cmd_diag_gpout09_query },
    { "DIAGnostic:GPOut10",  cmd_diag_gpout10       },
    { "DIAGnostic:GPOut10?", cmd_diag_gpout10_query },

    /* TEMPORARY diagnostic -- reads FLASH_OPTR to check whether PB8
       (BOOT0) is free for GPIO reuse, see cmd_system.c's own header
       comment on cmd_diag_optbytes_query(). */
    { "DIAGnostic:OPTBytes?", cmd_diag_optbytes_query },

    /* TEMPORARY diagnostic -- reads/clears the real RCC->CSR reset-cause
       flags to test whether DIAGnostic:GPOut11 1's garbled response is
       a genuine MCU reset, see cmd_system.c's own header comment on
       cmd_diag_rstcause_query(). */
    { "DIAGnostic:RSTCause?",      cmd_diag_rstcause_query },
    { "DIAGnostic:RSTCause:CLEar", cmd_diag_rstcause_clear },

    /* TEMPORARY debug/verification command -- software OCP fault
       injection, see cmd_state.c's own header comment on
       cmd_ocp_test_fault(). */
    { "OCP:TEST:FAULT",      cmd_ocp_test_fault },

    /* TEMPORARY debug/verification command -- software General Fault
       injection, see cmd_state.c's own header comment on
       cmd_general_test_fault(). */
    { "GENERAL:TEST:FAULT",  cmd_general_test_fault },

    /* Raw GateDriverStatus_01..12 (PE0..PE11) diagnostic readback --
       see cmd_io.c's own header comment on cmd_gds_query(). */
    { "GDS?",                cmd_gds_query    },

    /* Per-Transrex-channel fault-pin readback (Water/Temp/Enerpro/OCP)
       -- see cmd_io.c's own header comment on
       cmd_xrex_channel_status() and xrex_io.h for the full design. */
    { "XREX:CHANnel:STATus?", cmd_xrex_channel_status },

    /* Per-Transrex-channel ENA_OUT/CONTACT_OUT fiber outputs -- see
       cmd_io.c's own header comment on cmd_xrex_ena_out() and
       state_machine.h's SM_FAULT_ENABLE_OUTPUT/enable-output sections
       for the full design. Added 2026-09-17. */
    { "XREX:CHANnel:ENAOut",        cmd_xrex_ena_out          },
    { "XREX:CHANnel:ENAOut?",       cmd_xrex_ena_out_query    },
    { "XREX:CHANnel:CONTactOut",    cmd_xrex_contact_out      },
    { "XREX:CHANnel:CONTactOut?",   cmd_xrex_contact_out_query },

    /* PC13 ("GPOut_Enable_Pin"), default HIGH -- see commands.h's own
       comment on cmd_gpout_enable(). Added 2026-09-17. */
    { "GPOut:ENAble",   cmd_gpout_enable       },
    { "GPOut:ENAble?",  cmd_gpout_enable_query },

    /* PC15 ("PWM_Alt_Enable"), default HIGH -- see commands.h's own
       comment on cmd_pwmalt_enable(). Added 2026-09-17. */
    { "PWMAlt:ENAble",  cmd_pwmalt_enable       },
    { "PWMAlt:ENAble?", cmd_pwmalt_enable_query },

    /* QUADSPI connectivity test (W25Q128JVS) -- excluded entirely when
       QSPI_TEST_FEATURE_ENABLED is 0, same removability pattern as
       BOOT above. See cmd_system.c's own header comment on
       cmd_qspi_id(). */
#if (QSPI_TEST_FEATURE_ENABLED != 0)
    { "QSPI:ID?",            cmd_qspi_id      },
#endif

    /* PFM_Input period/duty capture -- excluded entirely when
       PFM_INPUT_FEATURE_ENABLED is 0, same removability pattern as
       BOOT/QSPI:ID? above. See cmd_pfmin.c's own header comment on
       cmd_pfmin_capture()/cmd_pfmin_status()/cmd_pfmin_data(). */
#if (PFM_INPUT_FEATURE_ENABLED != 0)
    { "PFMIN:CAPTURE",       cmd_pfmin_capture },
    { "PFMIN:STATus?",       cmd_pfmin_status  },
    { "PFMIN:DMASTAT?",      cmd_pfmin_dmastat },  /* TEMPORARY, see commands.h */
    { "PFMIN:DEBUG:RAW?",    cmd_pfmin_debug_raw }, /* TEMPORARY, see commands.h */
    { "PFMIN:DEBUG:REG?",    cmd_pfmin_debug_reg }, /* TEMPORARY, see commands.h */
    { "PFMIN:DATA?",         cmd_pfmin_data    },
#endif

    /* Closed-loop PID control + the demand-output / shot-profile / log /
       channel namespaces, this project's whole point -- see
       cmd_control.c's and cmd_shot.c's own header comments on the
       cmd_*() handlers and pid.h for the architecture. Not gated on a feature-enable flag, unlike the
       modules above.

       RENAMED 2026-09-22: the output/demand commands moved OUT of PID:
       into SOURce:/SHOT:/LOG:/CHANnel:, leaving PID: to mean only the
       actual PID-loop parameters (PID:GAINS/PID:LOOPMODE below). Clean
       cut -- the old PID:START/PID:SETPOINT/... mnemonics are gone, not
       aliased. */
    { "SOURce:RUN",           cmd_source_run     },
    { "SOURce:STOP",          cmd_source_stop    },
    { "SOURce:SETpoint",      cmd_source_setpoint },
    { "PID:GAINS",            cmd_pid_gains     },
    { "PID:GAINS?",           cmd_pid_gains_query },
    { "SOURce:STATus?",       cmd_source_status  },
    { "LOG:ARM",              cmd_log_arm       },
    { "LOG:DATA?",            cmd_log_data      },
    { "SOURce:RAMP",          cmd_source_ramp    },

    /* Production shot profile + open/closed-loop mode, added
       2026-09-10 -- see commands.h's own header comment on these
       handlers and pid.h's "DEMAND PROFILE"/"OPEN-LOOP MODE" doc
       sections for the full design. */
    { "PID:LOOPMODE",         cmd_pid_loopmode        },
    { "PID:LOOPMODE?",        cmd_pid_loopmode_query  },
    { "SOURce:ENAble",        cmd_source_enable       },
    { "SOURce:ENAble?",       cmd_source_enable_query },
    { "CHANnel:NICKname",     cmd_chan_nickname       },
    { "CHANnel:NICKname?",    cmd_chan_nickname_query },
    { "SHOT:TIMing",          cmd_shot_timing         },
    { "SHOT:TIMing?",         cmd_shot_timing_query   },
    { "SHOT:CURRent",         cmd_shot_current        },
    { "SHOT:CURRent?",        cmd_shot_current_query  },
    { "SHOT:STARt",           cmd_shot_start          },

    /* SIM: namespace -- sim_transrex.h backed, SIMULATOR-ONLY, added
       2026-09-18. See cmd_sim.c's own header comment on
       cmd_sim_fault_watertemp() for the full reasoning -- these rows
       (and the handlers they point to) do not exist at all in a
       controller build. */
#if defined(BUILD_TARGET_SIMULATOR)
    { "SIM:FAULT:WATERTEMP",  cmd_sim_fault_watertemp        },
    { "SIM:FAULT:WATERTEMP?", cmd_sim_fault_watertemp_query  },
    { "SIM:FAULT:ENERPRO",    cmd_sim_fault_enerpro          },
    { "SIM:FAULT:ENERPRO?",   cmd_sim_fault_enerpro_query    },
    { "SIM:FAULT:OCP",        cmd_sim_fault_ocp              },
    { "SIM:FAULT:OCP?",       cmd_sim_fault_ocp_query        },
    { "SIM:MODEL:TAU",        cmd_sim_model_tau              },
    { "SIM:MODEL:TAU?",       cmd_sim_model_tau_query        },
    { "SIM:DIAGnostic:IDLETONE",  cmd_sim_diag_idletone          },
    { "SIM:DIAGnostic:IDLETONE?", cmd_sim_diag_idletone_query    },
    { "SIM:CHANnel:STATus?",  cmd_sim_channel_status         },
    { "SIM:LOG",              cmd_sim_log                    },
    { "SIM:LOGDATA?",         cmd_sim_logdata                },
#endif
};

#define NUM_COMMANDS  (sizeof(command_table) / sizeof(command_table[0]))

void Commands_Dispatch(uart_instance_t *inst, char *line)
{
    if (scpi_dispatch(command_table, NUM_COMMANDS, inst, line) == SCPI_UNKNOWN)
    {
        SendErr(inst, ERR_UNKNOWN_COMMAND, "Unknown command");
    }
}
