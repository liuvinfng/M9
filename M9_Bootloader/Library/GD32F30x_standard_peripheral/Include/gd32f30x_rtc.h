/*!
    \file  gd32f30x_rtc.h
    \brief definitions for the RTC
*/

/*
    Copyright (C) 2017 GigaDevice

    2017-02-10, V1.0.0, firmware for GD32F30x
*/

#ifndef GD32F30X_RTC_H
#define GD32F30X_RTC_H

#include "gd32f30x.h"

/* RTC definitions */
#define RTC                          RTC_BASE

/* registers definitions */
#define RTC_INTEN                    REG32(RTC + 0x00U)      /*!< interrupt enable register */
#define RTC_CTL                      REG32(RTC + 0x04U)      /*!< control register */
#define RTC_PSCH                     REG32(RTC + 0x08U)      /*!< prescaler high register */
#define RTC_PSCL                     REG32(RTC + 0x0CU)      /*!< prescaler low register */
#define RTC_DIVH                     REG32(RTC + 0x10U)      /*!< divider high register */
#define RTC_DIVL                     REG32(RTC + 0x14U)      /*!< divider low register */
#define RTC_CNTH                     REG32(RTC + 0x18U)      /*!< counter high register */
#define RTC_CNTL                     REG32(RTC + 0x1CU)      /*!< counter low register */
#define RTC_ALRMH                    REG32(RTC + 0x20U)      /*!< alarm high register */
#define RTC_ALRML                    REG32(RTC + 0x24U)      /*!< alarm low register */

/* bits definitions */
/* RTC_INTEN */
#define RTC_INTEN_SCIE               BIT(0)                   /*!< second interrupt enable */
#define RTC_INTEN_ALRMIE             BIT(1)                   /*!< alarm interrupt enable */
#define RTC_INTEN_OVIE               BIT(2)                   /*!< overflow interrupt enable */

/* RTC_CTL */
#define RTC_CTL_SCIF                 BIT(0)                   /*!< second interrupt flag */
#define RTC_CTL_ALRMIF               BIT(1)                   /*!< alarm interrupt flag */
#define RTC_CTL_OVIF                 BIT(2)                   /*!< overflow interrupt flag */
#define RTC_CTL_RSYNF                BIT(3)                   /*!< registers synchronized flag */
#define RTC_CTL_CMF                  BIT(4)                   /*!< configuration mode flag */
#define RTC_CTL_LWOFF                BIT(5)                   /*!< last write operation finished flag */

/* constants definitions */
#define RTC_INT_SECOND               RTC_INTEN_SCIE           /*!< second interrupt enable */
#define RTC_INT_ALARM                RTC_INTEN_ALRMIE         /*!< alarm interrupt enable */
#define RTC_INT_OVERFLOW             RTC_INTEN_OVIE           /*!< overflow interrupt enable */

#define RTC_FLAG_SECOND              RTC_CTL_SCIF             /*!< second interrupt flag */
#define RTC_FLAG_ALARM               RTC_CTL_ALRMIF           /*!< alarm interrupt flag */
#define RTC_FLAG_OVERFLOW            RTC_CTL_OVIF             /*!< overflow interrupt flag */
#define RTC_FLAG_RSYN                RTC_CTL_RSYNF            /*!< registers synchronized flag */
#define RTC_FLAG_LWOF                RTC_CTL_LWOFF            /*!< last write operation finished flag */

/* function declarations */
/* enable RTC interrupt */
void rtc_interrupt_enable(uint32_t rtc_int);
/* disable RTC interrupt */
void rtc_interrupt_disable(uint32_t rtc_int);

/* enter RTC configuration mode */
void rtc_configuration_mode_enter(void);
/* exit RTC configuration mode */
void rtc_configuration_mode_exit(void);

/* wait RTC last write operation finished flag set */
void rtc_lwoff_wait(void);
/* wait RTC registers synchronized flag set */
void rtc_rsynf_wait(void);

/* get RTC counter value */
uint32_t rtc_counter_get(void);
/* set RTC counter value */
void rtc_counter_set(uint32_t cnt);

/* set RTC prescaler value */
void rtc_prescaler_set(uint32_t psc);
/* set RTC alarm value */
void rtc_alarm_set(uint32_t alarm);
/* get RTC divider value */
uint32_t rtc_divider_get(void);

/* get RTC flag status */
FlagStatus rtc_flag_get(uint32_t rtc_flag);
/* clear RTC flag status */
void rtc_flag_clear(uint32_t rtc_flag);

#endif /* GD32F30X_RTC_H */