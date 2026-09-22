#ifndef TELEMETRY_H_
#define TELEMETRY_H_

#include <stdint.h>
#include "uart.h"
#include "state_machine.h"   /* SM_State_t / SM_FaultType_t for the push API */

/*
 * telemetry.h / telemetry.c
 *
 * Telemetry event stream -- Phase 1 of docs/telemetry.md. Owns a small ring
 * buffer that carries timestamped state/fault events from wherever they
 * happen (the state machine -- including ISR context) to the main loop, which
 * formats and transmits them as unsolicited `!EVT ...` lines.
 *
 * Isolation guarantees (docs/telemetry.md Section 6):
 *   - Push is a few stores inside a NESTABLE critical section
 *     (__get_PRIMASK()/__disable_irq()/__set_PRIMASK()), so it is safe from
 *     thread or ISR context and adds only a fixed, tiny cost to the fault
 *     path -- including when the fault path itself already holds an outer
 *     __disable_irq() critical section (EnterFault etc.).
 *   - Emit (Telemetry_PollEmit) runs only in the main loop, transmits at most
 *     TELEM_EVENT_EMIT_PER_CALL events per call, and never masks IRQs -- so
 *     it cannot disturb the 1 kHz control loop or any fault ISR.
 *   - The `!EVT` prefix can never collide with an `OK`/`ERR` reply line.
 *
 * Wire grammar (see docs/telemetry.md Section 5):
 *     !EVT <tick_ms> STATE <IDLE|ARMED|FIRING|FAULT>
 *     !EVT <tick_ms> FAULT <GENERAL|OVERCURRENT|EXTERNAL_ENABLE|ENABLE_OUTPUT|ENERPRO> [ch]
 *     !EVT <tick_ms> FAULT CLEAR
 *     !EVT <tick_ms> TRIGGER FIRING   (external trigger on PF15 fired a shot)
 * where <ch> is 1-based and only present for per-channel fault types.
 *
 * `SYS:EVENT <0|1>` gates EMISSION only (default 1): while disabled,
 * Telemetry_PollEmit still drains the transmit queue but transmits nothing.
 *
 * FLIGHT RECORDER (Phase 4, docs/telemetry.md): every pushed event is ALSO
 * retained in a fixed-size ring (overwrite-oldest) for post-mortem. It is
 * queried via SYS:EVLOG? (Telemetry_ReplayFlightLog), which returns a header
 * line "OK <n>" followed by n one-line `!EVT` packets -- the same text format
 * as the live stream, oldest first. RAM-only (cleared on reset); warm-reset
 * retention via .noinit is a later step. Unlike the gated live stream, the
 * flight recorder always records, so SYS:EVENT 0 silences the stream without
 * losing the history.
 */

#define TELEMETRY_SCHEMA_VERSION  (4U)

void Telemetry_Init(void);
void Telemetry_SetEventEnabled(uint8_t enabled);
uint8_t Telemetry_GetEventEnabled(void);

/* Push wrappers -- safe from ISR or thread context. `channel` is 0-based and
 * meaningful only for per-channel fault types (OVERCURRENT/ENABLE_OUTPUT/
 * ENERPRO); pass 0xFF otherwise. */
void Telemetry_PushState(SM_State_t state);
void Telemetry_PushFault(SM_FaultType_t type, uint8_t channel);
void Telemetry_PushFaultClear(void);
void Telemetry_PushTrigger(void);   /* external trigger (PF15) fired a shot --
                                       emits "!EVT <tick> TRIGGER FIRING" */

/* Called from the main loop every iteration. Emits up to
 * TELEM_EVENT_EMIT_PER_CALL pending events as `!EVT` lines. */
void Telemetry_PollEmit(uart_instance_t *inst);

/* SYS:EVLOG? -- replay the retained flight-recorder history (oldest first) as
 * "OK <n>" then n one-line !EVT packets. Called from the command layer
 * (commands.c); safe to call any time, including mid-fault. */
void Telemetry_ReplayFlightLog(uart_instance_t *inst);

#endif /* TELEMETRY_H_ */
