/*
 * telemetry.c
 *
 * See telemetry.h for the design. Implementation notes that belong here, not
 * the header:
 *
 * Concurrency model: two rings, both written by a NESTABLE critical section
 * (PRIMASK save/restore) because producers run from both thread and ISR
 * context, and because the fault path already holds an outer __disable_irq()
 * critical section when it reaches EnterFault() -- a plain __enable_irq() at
 * the end of PushEvent would break that outer section. PRIMASK save/restore is
 * safe to nest.
 *
 *   1. The transmit queue (g_ring) feeds the live `!EVT` stream: single
 *      consumer (Telemetry_PollEmit, main loop only) over a head/tail pair;
 *      if it fills, the NEWEST event is dropped (bounded, non-blocking).
 *   2. The flight recorder (g_flight) is RETAINED history for SYS:EVLOG?: a
 *      fixed-size ring that always records and overwrites the OLDEST entry
 *      when full. It is read back by Telemetry_ReplayFlightLog() from the main
 *      loop; that read snapshots the count + write index and tolerates a
 *      (cosmetic) torn entry if a fault fires mid-dump -- acceptable for a
 *      post-mortem diagnostic, and it can never index out of bounds.
 *
 * RAM-only by design (cleared on reset). Warm-reset retention via a .noinit
 * section is a later step (docs/telemetry.md Phase 4).
 */

#include "telemetry.h"
#include <stdio.h>

#define TELEM_EVENT_RING_SIZE      (64U)     /* transmit queue depth (live !EVT) */
#define TELEM_EVENT_EMIT_PER_CALL  (4U)      /* bound on !EVT emitted per main-loop call */
#define TELEM_FLIGHT_LOG_SIZE      (128U)    /* retained history depth (SYS:EVLOG?) */

typedef enum
{
    TELEM_EVT_STATE = 0,
    TELEM_EVT_FAULT,
    TELEM_EVT_FAULT_CLEAR,
    TELEM_EVT_TRIGGER,   /* external trigger (PF15) fired a shot */
} TelemetryEventKind_t;

typedef struct
{
    uint32_t tick;     /* HAL_GetTick() ms at the moment the event happened */
    uint8_t  kind;     /* TelemetryEventKind_t */
    uint8_t  type;     /* SM_State_t (STATE) / SM_FaultType_t (FAULT); 0 for CLEAR */
    uint8_t  channel;  /* 0-based channel for per-channel faults; 0xFF = N/A */
} TelemetryEvent_t;

/* Transmit queue (Phase 1) -- consumed by Telemetry_PollEmit to emit !EVT. */
static TelemetryEvent_t g_ring[TELEM_EVENT_RING_SIZE];
static volatile uint32_t g_head = 0U;   /* consumer index (main loop only) */
static volatile uint32_t g_tail = 0U;   /* producer index (written under critical section) */

/* Flight recorder (Phase 4) -- retained, overwrite-oldest. */
static TelemetryEvent_t g_flight[TELEM_FLIGHT_LOG_SIZE];
static volatile uint32_t g_flightWrite = 0U;   /* next slot to write (mod SIZE) */
static volatile uint32_t g_flightCount = 0U;   /* valid entries, 0..SIZE */

static uint8_t g_eventEnabled = 1U;

static const char *StateName(SM_State_t s)
{
    switch (s)
    {
        case SM_STATE_IDLE:   return "IDLE";
        case SM_STATE_ARMED:  return "ARMED";
        case SM_STATE_FIRING: return "FIRING";
        case SM_STATE_FAULT:  return "FAULT";
        default:              return "UNKNOWN";
    }
}

static const char *FaultName(SM_FaultType_t t)
{
    switch (t)
    {
        case SM_FAULT_GENERAL:         return "GENERAL";
        case SM_FAULT_OVERCURRENT:     return "OVERCURRENT";
        case SM_FAULT_EXTERNAL_ENABLE: return "EXTERNAL_ENABLE";
        case SM_FAULT_ENABLE_OUTPUT:   return "ENABLE_OUTPUT";
        case SM_FAULT_ENERPRO:         return "ENERPRO";
        default:                       return "UNKNOWN";
    }
}

static uint8_t FaultIsPerChannel(SM_FaultType_t t)
{
    return (uint8_t)((t == SM_FAULT_OVERCURRENT) ||
                     (t == SM_FAULT_ENABLE_OUTPUT) ||
                     (t == SM_FAULT_ENERPRO));
}

/* Format + send ONE event as a `!EVT` line -- shared by Telemetry_PollEmit
 * (live stream) and Telemetry_ReplayFlightLog (SYS:EVLOG? replay) so both
 * render identically. */
static void EmitEventLine(uart_instance_t *inst, uint32_t tick, uint8_t kind,
                          uint8_t type, uint8_t channel)
{
    char buf[48];

    switch (kind)
    {
        case TELEM_EVT_STATE:
            snprintf(buf, sizeof(buf), "!EVT %lu STATE %s\r\n",
                     (unsigned long)tick, StateName((SM_State_t)type));
            break;
        case TELEM_EVT_FAULT:
            if (FaultIsPerChannel((SM_FaultType_t)type) != 0U)
            {
                snprintf(buf, sizeof(buf), "!EVT %lu FAULT %s %u\r\n",
                         (unsigned long)tick, FaultName((SM_FaultType_t)type),
                         (unsigned)(channel + 1U));
            }
            else
            {
                snprintf(buf, sizeof(buf), "!EVT %lu FAULT %s\r\n",
                         (unsigned long)tick, FaultName((SM_FaultType_t)type));
            }
            break;
        case TELEM_EVT_TRIGGER:
            snprintf(buf, sizeof(buf), "!EVT %lu TRIGGER FIRING\r\n",
                     (unsigned long)tick);
            break;
        case TELEM_EVT_FAULT_CLEAR:
        default:
            snprintf(buf, sizeof(buf), "!EVT %lu FAULT CLEAR\r\n",
                     (unsigned long)tick);
            break;
    }

    uart_send(inst, buf);
}

static void PushEvent(uint8_t kind, uint8_t type, uint8_t channel)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    {
        uint32_t tick = HAL_GetTick();

        /* Flight recorder -- always retained, overwrite-oldest. */
        TelemetryEvent_t *f = &g_flight[g_flightWrite];
        f->tick    = tick;
        f->kind    = kind;
        f->type    = type;
        f->channel = channel;
        g_flightWrite = (g_flightWrite + 1U) % (uint32_t)TELEM_FLIGHT_LOG_SIZE;
        if (g_flightCount < (uint32_t)TELEM_FLIGHT_LOG_SIZE)
        {
            g_flightCount++;
        }

        /* Transmit queue -- enqueue for the live !EVT stream. */
        uint32_t head = g_head;
        uint32_t tail = g_tail;
        if ((tail - head) < (uint32_t)TELEM_EVENT_RING_SIZE)
        {
            TelemetryEvent_t *ev = &g_ring[tail % (uint32_t)TELEM_EVENT_RING_SIZE];
            ev->tick    = tick;
            ev->kind    = kind;
            ev->type    = type;
            ev->channel = channel;
            g_tail      = tail + 1U;
        }
        /* else: transmit queue full -- drop newest for the live stream; the
           flight recorder above still has it for SYS:EVLOG?. */
    }

    __set_PRIMASK(primask);
}

void Telemetry_Init(void)
{
    g_head         = 0U;
    g_tail         = 0U;
    g_flightWrite  = 0U;
    g_flightCount  = 0U;
    g_eventEnabled = 1U;
}

void Telemetry_SetEventEnabled(uint8_t enabled)
{
    g_eventEnabled = (enabled != 0U) ? 1U : 0U;
}

uint8_t Telemetry_GetEventEnabled(void)
{
    return g_eventEnabled;
}

void Telemetry_PushState(SM_State_t state)
{
    PushEvent(TELEM_EVT_STATE, (uint8_t)state, 0xFFU);
}

void Telemetry_PushFault(SM_FaultType_t type, uint8_t channel)
{
    PushEvent(TELEM_EVT_FAULT, (uint8_t)type, channel);
}

void Telemetry_PushFaultClear(void)
{
    PushEvent(TELEM_EVT_FAULT_CLEAR, 0U, 0xFFU);
}

void Telemetry_PushTrigger(void)
{
    PushEvent(TELEM_EVT_TRIGGER, 0U, 0xFFU);
}

void Telemetry_PollEmit(uart_instance_t *inst)
{
    uint32_t tail    = g_tail;   /* snapshot: atomic volatile read */
    uint32_t emitted = 0U;

    while ((g_head != tail) && (emitted < (uint32_t)TELEM_EVENT_EMIT_PER_CALL))
    {
        TelemetryEvent_t ev = g_ring[g_head % (uint32_t)TELEM_EVENT_RING_SIZE];
        g_head++;   /* only the consumer advances g_head */

        if (g_eventEnabled != 0U)
        {
            EmitEventLine(inst, ev.tick, ev.kind, ev.type, ev.channel);
        }
        emitted++;
    }
}

void Telemetry_ReplayFlightLog(uart_instance_t *inst)
{
    uint32_t n     = g_flightCount;   /* snapshot */
    uint32_t write = g_flightWrite;   /* snapshot */
    char buf[24];

    snprintf(buf, sizeof(buf), "OK %lu\r\n", (unsigned long)n);
    uart_send(inst, buf);

    for (uint32_t i = 0U; i < n; i++)
    {
        /* oldest = (write - count + i) mod SIZE, walking oldest -> newest;
           +SIZE keeps the index positive without relying on uint wrap. */
        uint32_t idx = (write + (uint32_t)TELEM_FLIGHT_LOG_SIZE - n + i)
                        % (uint32_t)TELEM_FLIGHT_LOG_SIZE;
        TelemetryEvent_t *ev = &g_flight[idx];
        EmitEventLine(inst, ev->tick, ev->kind, ev->type, ev->channel);
    }
}
