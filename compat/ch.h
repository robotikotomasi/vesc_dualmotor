/*
 * ChibiOS ch.h compatibility shim → FreeRTOS / CMSIS-RTOS v2
 * Provides ChibiOS kernel API mapped to FreeRTOS equivalents.
 */

#ifndef CH_H_COMPAT
#define CH_H_COMPAT

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
#include "cmsis_os2.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Types
 * ======================================================================== */

typedef struct ch_thread {
    osThreadId_t id;
    int motor_selected;
    const char *name;
    EventGroupHandle_t events;
} thread_t;

typedef struct {
    SemaphoreHandle_t sem;
    StaticSemaphore_t buf;
} mutex_t;

typedef struct {
    SemaphoreHandle_t sem;
    StaticSemaphore_t buf;
} semaphore_t;

typedef semaphore_t binary_semaphore_t;

typedef uint32_t eventmask_t;
typedef uint32_t eventflags_t;
typedef uint32_t systime_t;
typedef uint32_t sysinterval_t;

typedef struct {
    void *dummy;
} event_listener_t;

typedef struct {
    void *dummy;
} event_source_t;

typedef struct {
    osTimerId_t id;
    void (*func)(void *);
    void *par;
} virtual_timer_t;

typedef struct {
    int dummy;
} mailbox_t;

typedef struct {
    int dummy;
} memory_pool_t;

/* Message queue type */
typedef struct {
    int dummy;
} mqueue_t;

/* Base stream types */
typedef struct {
    int dummy;
} BaseSequentialStream;

typedef BaseSequentialStream BaseChannel;

/* ========================================================================
 * Priority levels
 * ======================================================================== */

#ifndef NORMALPRIO
#define NORMALPRIO      osPriorityNormal
#endif
#ifndef LOWPRIO
#define LOWPRIO         osPriorityLow
#endif
#ifndef HIGHPRIO
#define HIGHPRIO        osPriorityHigh
#endif
#ifndef IDLEPRIO
#define IDLEPRIO        osPriorityIdle
#endif

/* ========================================================================
 * Thread macros
 * ======================================================================== */

#define THD_WORKING_AREA(name, size)  uint8_t name[(size) + 256]
#define THD_FUNCTION(name, arg)       void name(void *arg)

/* ========================================================================
 * Time conversion macros
 * ======================================================================== */

#define MS2ST(msec)        ((systime_t)(msec))
#define ST2MS(st)          ((uint32_t)(st))
#define TIME_MS2I(ms)      ((sysinterval_t)(ms))
#define TIME_US2I(us)      ((sysinterval_t)(((us) + 999UL) / 1000UL))
#define TIME_I2MS(i)       ((uint32_t)(i))
#define TIME_I2US(i)       ((uint32_t)((i) * 1000UL))

#ifndef TIME_INFINITE
#define TIME_INFINITE      osWaitForever
#endif
#ifndef TIME_IMMEDIATE
#define TIME_IMMEDIATE     0
#endif

#define ALL_EVENTS         ((eventmask_t)0xFFFFFFFFUL)

/* ========================================================================
 * System functions (inline stubs)
 * ======================================================================== */

static inline void chSysInit(void) {
    /* Replaced by osKernelInitialize() + osKernelStart() */
}

static inline void chSysLock(void) {
    vPortEnterCritical();
}

static inline void chSysUnlock(void) {
    vPortExitCritical();
}

static inline void chSysLockFromISR(void) {
    /* Simplified - use taskENTER_CRITICAL_FROM_ISR in real ISR context */
}

static inline void chSysUnlockFromISR(void) {
    /* Simplified */
}

static inline void chSysDisable(void) {
    taskDISABLE_INTERRUPTS();
}

/* ========================================================================
 * Thread functions
 * ======================================================================== */

static inline thread_t *chThdCreateStatic(void *wa, size_t size, int prio,
                                           void (*func)(void *), void *arg) {
    (void)wa; (void)size;
    osThreadAttr_t attr = {0};
    attr.stack_mem = wa;
    attr.stack_size = size;
    attr.priority = (osPriority_t)prio;
    osThreadNew((osThreadFunc_t)func, arg, &attr);
    return NULL;
}

static inline thread_t *chThdCreateFromHeap(void *heap, size_t size,
                                             const char *name, int prio,
                                             void (*func)(void *), void *arg) {
    (void)heap; (void)name;
    return chThdCreateStatic(NULL, size, prio, func, arg);
}

static inline void chThdSleepMilliseconds(uint32_t ms) {
    osDelay(ms);
}

static inline void chThdSleepMicroseconds(uint32_t us) {
    uint32_t ms = (us + 999U) / 1000U;
    if (ms == 0) ms = 1;
    osDelay(ms);
}

static inline void chThdSleep(sysinterval_t interval) {
    osDelay(interval);
}

static inline void chThdSleepUntil(systime_t time) {
    (void)time;
    osDelay(1);
}

static inline thread_t *chThdGetSelfX(void) {
    return NULL;
}

static inline void chThdTerminate(thread_t *tp) {
    (void)tp;
}

static inline bool chThdTerminatedX(thread_t *tp) {
    (void)tp;
    return false;
}

static inline bool chThdShouldTerminateX(void) {
    return false;
}

static inline void chThdExit(int code) {
    (void)code;
    osThreadExit();
}

static inline void chThdSetPriority(int prio) {
    (void)prio;
}

static inline int chThdWait(thread_t *tp) {
    (void)tp;
    return 0;
}

/* ========================================================================
 * Registry functions
 * ======================================================================== */

static inline void chRegSetThreadName(const char *name) {
    (void)name;
}

static inline thread_t *chRegFirstThread(void) {
    return NULL;
}

static inline thread_t *chRegNextThread(thread_t *tp) {
    (void)tp;
    return NULL;
}

/* ========================================================================
 * Mutex functions
 * ======================================================================== */

static inline void chMtxObjectInit(mutex_t *mp) {
    mp->sem = xSemaphoreCreateMutexStatic(&mp->buf);
}

static inline void chMtxLock(mutex_t *mp) {
    if (mp && mp->sem) {
        xSemaphoreTake(mp->sem, portMAX_DELAY);
    }
}

static inline void chMtxUnlock(mutex_t *mp) {
    if (mp && mp->sem) {
        xSemaphoreGive(mp->sem);
    }
}

static inline bool chMtxTryLock(mutex_t *mp) {
    if (mp && mp->sem) {
        return xSemaphoreTake(mp->sem, 0) == pdTRUE;
    }
    return false;
}

/* ========================================================================
 * Semaphore functions
 * ======================================================================== */

static inline void chSemObjectInit(semaphore_t *sp, int n) {
    sp->sem = xSemaphoreCreateCountingStatic(0xFFFF, n, &sp->buf);
}

static inline int chSemWait(semaphore_t *sp) {
    if (sp && sp->sem) {
        return (xSemaphoreTake(sp->sem, portMAX_DELAY) == pdTRUE) ? 0 : -1;
    }
    return -1;
}

static inline int chSemWaitTimeout(semaphore_t *sp, sysinterval_t timeout) {
    if (sp && sp->sem) {
        return (xSemaphoreTake(sp->sem, timeout) == pdTRUE) ? 0 : -1;
    }
    return -1;
}

static inline void chSemSignal(semaphore_t *sp) {
    if (sp && sp->sem) {
        xSemaphoreGive(sp->sem);
    }
}

static inline void chSemSignalI(semaphore_t *sp) {
    if (sp && sp->sem) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(sp->sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

static inline void chSemReset(semaphore_t *sp, int n) {
    (void)sp; (void)n;
}

/* ========================================================================
 * Event functions (stubs)
 * ======================================================================== */

static inline void chEvtSignal(thread_t *tp, eventmask_t events) {
    (void)tp; (void)events;
}

static inline void chEvtSignalI(thread_t *tp, eventmask_t events) {
    (void)tp; (void)events;
}

static inline eventmask_t chEvtWaitAny(eventmask_t events) {
    (void)events;
    osDelay(1);
    return 0;
}

static inline eventmask_t chEvtWaitAnyTimeout(eventmask_t events, sysinterval_t timeout) {
    (void)events;
    osDelay(timeout > 0 ? timeout : 1);
    return 0;
}

static inline eventmask_t chEvtGetAndClearEvents(eventmask_t events) {
    (void)events;
    return 0;
}

static inline void chEvtRegister(event_source_t *esp, event_listener_t *elp, int event) {
    (void)esp; (void)elp; (void)event;
}

static inline void chEvtRegisterMaskWithFlags(event_source_t *esp, event_listener_t *elp,
                                               eventmask_t events, eventflags_t flags) {
    (void)esp; (void)elp; (void)events; (void)flags;
}

static inline void chEvtUnregister(event_source_t *esp, event_listener_t *elp) {
    (void)esp; (void)elp;
}

static inline void chEvtObjectInit(event_source_t *esp) {
    (void)esp;
}

static inline void chEvtBroadcast(event_source_t *esp) {
    (void)esp;
}

static inline void chEvtBroadcastFlags(event_source_t *esp, eventflags_t flags) {
    (void)esp; (void)flags;
}

static inline void chEvtBroadcastFlagsI(event_source_t *esp, eventflags_t flags) {
    (void)esp; (void)flags;
}

static inline eventflags_t chEvtGetAndClearFlags(event_listener_t *elp) {
    (void)elp;
    return 0;
}

/* ========================================================================
 * Virtual Timer functions
 * ======================================================================== */

static inline void chVTObjectInit(virtual_timer_t *vtp) {
    if (vtp) memset(vtp, 0, sizeof(virtual_timer_t));
}

static inline systime_t chVTGetSystemTime(void) {
    return (systime_t)xTaskGetTickCount();
}

static inline systime_t chVTGetSystemTimeX(void) {
    return (systime_t)xTaskGetTickCount();
}

static inline sysinterval_t chVTTimeElapsedSinceX(systime_t start) {
    return (sysinterval_t)(chVTGetSystemTimeX() - start);
}

static inline void chVTSet(virtual_timer_t *vtp, sysinterval_t delay,
                            void (*func)(void *), void *par) {
    (void)vtp; (void)delay; (void)func; (void)par;
}

static inline void chVTSetI(virtual_timer_t *vtp, sysinterval_t delay,
                              void (*func)(void *), void *par) {
    (void)vtp; (void)delay; (void)func; (void)par;
}

static inline void chVTReset(virtual_timer_t *vtp) {
    (void)vtp;
}

static inline void chVTResetI(virtual_timer_t *vtp) {
    (void)vtp;
}

static inline bool chVTIsArmedI(virtual_timer_t *vtp) {
    (void)vtp;
    return false;
}

/* ========================================================================
 * Core/Heap status
 * ======================================================================== */

static inline size_t chCoreGetStatusX(void) {
    return xPortGetFreeHeapSize();
}

static inline size_t chHeapStatus(void *heap, size_t *total, size_t *largest) {
    (void)heap;
    size_t free_heap = xPortGetFreeHeapSize();
    if (total)   *total = free_heap;
    if (largest) *largest = free_heap;
    return 0;
}

/* ========================================================================
 * Mailbox functions (stubs)
 * ======================================================================== */

static inline void chMBObjectInit(mailbox_t *mbp, void *buf, size_t n) {
    (void)mbp; (void)buf; (void)n;
}

static inline int chMBPost(mailbox_t *mbp, uint32_t msg, sysinterval_t timeout) {
    (void)mbp; (void)msg; (void)timeout;
    return 0;
}

static inline int chMBPostI(mailbox_t *mbp, uint32_t msg) {
    (void)mbp; (void)msg;
    return 0;
}

static inline int chMBFetch(mailbox_t *mbp, uint32_t *msgp, sysinterval_t timeout) {
    (void)mbp; (void)msgp; (void)timeout;
    return -1;
}

static inline int chMBFetchTimeout(mailbox_t *mbp, uint32_t *msgp, sysinterval_t timeout) {
    (void)mbp; (void)msgp; (void)timeout;
    return -1;
}

/* ========================================================================
 * Memory Pool functions (stubs)
 * ======================================================================== */

static inline void chPoolObjectInit(memory_pool_t *mp, size_t size, void *provider) {
    (void)mp; (void)size; (void)provider;
}

static inline void *chPoolAlloc(memory_pool_t *mp) {
    (void)mp;
    return NULL;
}

static inline void chPoolFree(memory_pool_t *mp, void *objp) {
    (void)mp; (void)objp;
}

/* ========================================================================
 * Sequential stream functions (stubs)
 * ======================================================================== */

static inline int chSequentialStreamRead(BaseSequentialStream *ssp, void *bp, size_t n) {
    (void)ssp; (void)bp; (void)n;
    return 0;
}

static inline int chSequentialStreamWrite(BaseSequentialStream *ssp, const void *bp, size_t n) {
    (void)ssp; (void)bp; (void)n;
    return 0;
}

/* ========================================================================
 * Message functions (stubs)
 * ======================================================================== */

static inline void chMsgSend(thread_t *tp, int msg) {
    (void)tp; (void)msg;
}

static inline int chMsgGet(thread_t *tp) {
    (void)tp;
    return 0;
}

static inline void chMsgRelease(thread_t *tp, int msg) {
    (void)tp; (void)msg;
}

/* ========================================================================
 * IRQ handler macros
 * ======================================================================== */

#define CH_IRQ_HANDLER(id)    void id(void)
#define CH_IRQ_PROLOGUE()     do {} while(0)
#define CH_IRQ_EPILOGUE()     do {} while(0)
#define OSAL_IRQ_HANDLER(id)  void id(void)
#define OSAL_IRQ_PROLOGUE()   do {} while(0)
#define OSAL_IRQ_EPILOGUE()   do {} while(0)

/* ========================================================================
 * Misc ChibiOS defines
 * ======================================================================== */

#ifndef MSG_OK
#define MSG_OK          0
#endif
#ifndef MSG_TIMEOUT
#define MSG_TIMEOUT     -1
#endif
#ifndef MSG_RESET
#define MSG_RESET       -2
#endif

#ifndef CH_DBG_SYSTEM_STATE_CHECK
#define CH_DBG_SYSTEM_STATE_CHECK   0
#endif

/* chprintf support - map to empty */
#define chprintf(stream, fmt, ...)   do {} while(0)
#define chsnprintf                   snprintf

#ifdef __cplusplus
}
#endif

#endif /* CH_H_COMPAT */
