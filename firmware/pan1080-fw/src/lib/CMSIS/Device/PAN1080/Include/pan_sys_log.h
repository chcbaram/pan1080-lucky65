#ifndef __PAN_SYS_LOG_H__
#define __PAN_SYS_LOG_H__

#include <stdio.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DBG_ON     0
#define INFO_ON    1
#define WARN_ON    1
#define ERR_ON     1
#define TEST_ON    1
#define ASSERT_ON  1

#define SYS_ABORT()    do { } while (1) //sys_abort()

#define BIT(x)   (0x1UL << (x))

#define LOG(flags, ...)    \
    do {                                \
        if (flags)                      \
            printf(__VA_ARGS__);    \
    } while (0)

#define SYS_DBG(fmt, ...)   LOG(DBG_ON, "[SYS DBG] %s:%s():%d, "   fmt, __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define SYS_INFO(fmt, ...)  LOG(INFO_ON, "[SYS INFO] %s:%s():%d, " fmt, __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define SYS_WARN(fmt, ...)  LOG(WARN_ON, "[SYS WARN] %s:%s():%d, " fmt, __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define SYS_TEST(...)  LOG(TEST_ON, __VA_ARGS__)

#define SYS_ERR(fmt, ...)                               \
    do {                                                    \
        LOG(ERR_ON, "[SYS ERR] %s:%s():%d, "fmt, __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__);   \
        if (ERR_ON)                                                         \
            SYS_ABORT();                                                        \
    } while (0)

    
#ifdef ASSERT_ON
#define SYS_ASSERT(expr)    ((expr) ? (void)0u : SYS_AssertError((uint8_t*)__FILE__, __LINE__))
extern void SYS_AssertError(uint8_t* file, uint32_t line);
#else
#define SYS_ASSERT(expr)    ((void)0u)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __PAN_SYS_LOG_H__ */
