#ifndef HALT_IF_DEBUGGING_H
#define HALT_IF_DEBUGGING_H
// https://interrupt.memfault.com/blog/cortex-m-hardfault-debug#relevant-status-registers
#define HALT_IF_DEBUGGING()                                 \
    do                                                      \
    {                                                       \
        if ((*(volatile uint32_t *) 0xE000EDF0) & (1 << 0)) \
        {                                                   \
            __asm("bkpt 1");                                \
        }                                                   \
    } while (0)
#endif // HALT_IF_DEBUGGING_H