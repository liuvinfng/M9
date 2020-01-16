/*!
    \file  gd32f30x_fmc.h
    \brief definitions for the FMC
*/

/*
    Copyright (C) 2017 GigaDevice

    2017-02-10, V1.0.0, firmware for GD32F30x
*/

#ifndef GD32F30X_FMC_H
#define GD32F30X_FMC_H

#include "gd32f30x.h"

/* FMC and option byte definition */
#define FMC                        FMC_BASE                       /*!< FMC register base address */
#define OB                         OB_BASE                        /*!< option bytes base address */

/* registers definitions */
#define FMC_WS                     REG32((FMC) + 0x00U)           /*!< FMC wait state register */
#define FMC_KEY0                   REG32((FMC) + 0x04U)           /*!< FMC unlock key register 0 */
#define FMC_OBKEY                  REG32((FMC) + 0x08U)           /*!< FMC option bytes unlock key register */
#define FMC_STAT0                  REG32((FMC) + 0x0CU)           /*!< FMC status register 0 */
#define FMC_CTL0                   REG32((FMC) + 0x10U)           /*!< FMC control register 0 */
#define FMC_ADDR0                  REG32((FMC) + 0x14U)           /*!< FMC address register 0 */
#define FMC_OBCTL                  REG32((FMC) + 0x1CU)           /*!< FMC option bytes control register */
#define FMC_WP                     REG32((FMC) + 0x20U)           /*!< FMC erase/program protection register */
#define FMC_KEY1                   REG32((FMC) + 0x44U)           /*!< FMC unlock key register 1 */
#define FMC_STAT1                  REG32((FMC) + 0x4CU)           /*!< FMC status register 1 */
#define FMC_CTL1                   REG32((FMC) + 0x50U)           /*!< FMC control register 1 */
#define FMC_ADDR1                  REG32((FMC) + 0x54U)           /*!< FMC address register 1 */
#define FMC_WSEN                   REG32((FMC) + 0xFCU)           /*!< FMC wait state enable register */
#define FMC_PID                    REG32((FMC) + 0x100U)          /*!< FMC product ID register */

#define OB_SPC                     REG16((OB) + 0x00U)            /*!< option byte security protection value */
#define OB_USER                    REG16((OB) + 0x02U)            /*!< option byte user value*/
#define OB_WP0                     REG16((OB) + 0x08U)            /*!< option byte write protection 0 */
#define OB_WP1                     REG16((OB) + 0x0AU)            /*!< option byte write protection 1 */
#define OB_WP2                     REG16((OB) + 0x0CU)            /*!< option byte write protection 2 */
#define OB_WP3                     REG16((OB) + 0x0EU)            /*!< option byte write protection 3 */

/* bits definitions */
/* FMC_WS */
#define FMC_WC_WSCNT               BITS(0,2)                      /*!< wait state counter */

/* FMC_KEY0 */
#define FMC_KEY0_KEY               BITS(0,31)                     /*!< FMC_CTL10 register unlock key bits */

/* FMC_OBKEY */
#define FMC_OBKEY_OBKEY            BITS(0,31)                     /*!< option bytes key bits */

/* FMC_STAT0 */
#define FMC_STAT0_BUSY             BIT(0)                         /*!< flash busy flag bit */
#define FMC_STAT0_PGERR            BIT(2)                         /*!< flash program error flag bit */
#define FMC_STAT0_WPERR            BIT(4)                         /*!< erase/program protection error flag bit */
#define FMC_STAT0_END              BIT(5)                         /*!< end of operation flag bit */

/* FMC_CTL0 */
#define FMC_CTL0_PG                BIT(0)                         /*!< main flash program for bank0 command bit */
#define FMC_CTL0_PER               BIT(1)                         /*!< main flash page erase for bank0 command bit */
#define FMC_CTL0_MER               BIT(2)                         /*!< main flash mass erase for bank0 command bit */
#define FMC_CTL0_OBPG              BIT(4)                         /*!< option bytes program command bit */
#define FMC_CTL0_OBER              BIT(5)                         /*!< option bytes erase command bit */
#define FMC_CTL0_START             BIT(6)                         /*!< send erase command to FMC bit */
#define FMC_CTL0_LK                BIT(7)                         /*!< FMC_CTL0 lock bit */
#define FMC_CTL0_OBWPEN            BIT(9)                         /*!< option bytes erase/program enable bit */
#define FMC_CTL0_ERIEN             BIT(10)                        /*!< error interrupt enable bit */
#define FMC_CTL0_ENDIEN            BIT(12)                        /*!< end of operation interrupt enable bit */

/* FMC_ADDR0 */
#define FMC_ADDR0_ADDR             BITS(0,31)                     /*!< Flash erase/program command address bits */

/* FMC_OBCTL */
#define FMC_OBCTL_REER             BIT(0)                         /*!< option bytes read error bit. */
#define FMC_OBCTL_SPC              BIT(1)                         /*!< option bytes security protection code */
#define FMC_OBCTL_USER             BITS(2,9)                      /*!< store USER of option bytes block after system reset */
#define FMC_OBCTL_DATA             BITS(10,25)                    /*!< store DATA of option bytes block after system reset. */

/* FMC_WP */
#define FMC_WP_WP                  BITS(0,31)                     /*!< store WP of option bytes block after system reset */

/* FMC_KEY1 */
#define FMC_KEY1_KEY               BITS(0,31)                     /*!< FMC_CTL1 register unlock key bits */

/* FMC_STAT1 */
#define FMC_STAT1_BUSY             BIT(0)                         /*!< flash busy flag bit */
#define FMC_STAT1_PGERR            BIT(2)                         /*!< flash program error flag bit */
#define FMC_STAT1_WPERR            BIT(4)                         /*!< erase/program protection error flag bit */
#define FMC_STAT1_END              BIT(5)                         /*!< end of operation flag bit */

/* FMC_CTL1 */
#define FMC_CTL1_PG                BIT(0)                         /*!< main flash program for bank01 command bit */
#define FMC_CTL1_PER               BIT(1)                         /*!< main flash page erase for bank1 command bit */
#define FMC_CTL1_MER               BIT(2)                         /*!< main flash mass erase for bank1 command bit */
#define FMC_CTL1_START             BIT(6)                         /*!< send erase command to FMC bit */
#define FMC_CTL1_LK                BIT(7)                         /*!< FMC_CTL1 lock bit */
#define FMC_CTL1_ERIE              BIT(10)                        /*!< error interrupt enable bit */
#define FMC_CTL1_ENDIE             BIT(12)                        /*!< end of operation interrupt enable bit */

/* FMC_ADDR1 */
#define FMC_ADDR1_ADDR             BITS(0,31)                     /*!< Flash erase/program command address bits */

/* FMC_WSEN */
#define FMC_WSEN_WSEN              BIT(0)                         /*!< FMC wait state enable bit */

/* FMC_PID */
#define FMC_PID_PID                BITS(0,31)                     /*!< product ID bits */

/* constants definitions */
/* fmc state */
typedef enum
{
    FMC_READY,                                                    /*!< the operation has been completed */
    FMC_BUSY,                                                     /*!< the operation is in progress */
    FMC_PGERR,                                                    /*!< program error */
    FMC_WPERR,                                                    /*!< erase/program protection error */
    FMC_TIMEOUT_ERR                                               /*!< timeout error */
}fmc_state_enum;

/* unlock key */
#define UNLOCK_KEY0                ((uint32_t)0x45670123U)        /*!< unlock key 0 */
#define UNLOCK_KEY1                ((uint32_t)0xCDEF89ABU)        /*!< unlock key 1 */

/* FMC wait state counter */
#define WC_WSCNT(regval)           (BITS(0,2) & ((uint32_t)(regval)))
#define WS_WSCNT_0                 WC_WSCNT(0)                    /*!< FMC 0 wait */
#define WS_WSCNT_1                 WC_WSCNT(1)                    /*!< FMC 1 wait */
#define WS_WSCNT_2                 WC_WSCNT(2)                    /*!< FMC 2 wait */

/* option bytes software/hardware free watch dog timer */  
#define OB_FWDGT_SW                ((uint8_t)0x01U)               /*!< software free watchdog */
#define OB_FWDGT_HW                ((uint8_t)0x00U)               /*!< hardware free watchdog */

/* option bytes reset or not entering deep sleep mode */
#define OB_DEEPSLEEP_NRST          ((uint8_t)0x02U)               /*!< no reset when entering deepsleep mode */
#define OB_DEEPSLEEP_RST           ((uint8_t)0x00U)               /*!< generate a reset instead of entering deepsleep mode */

/* option bytes reset or not entering standby mode */
#define OB_STDBY_NRST              ((uint8_t)0x04U)               /*!< no reset when entering deepsleep mode */
#define OB_STDBY_RST               ((uint8_t)0x00U)               /*!< generate a reset instead of entering standby mode */

/* option bytes boot bank value */
#define OB_BOOT_B0                 ((uint8_t)0x08U)               /*!< boot from bank0 */
#define OB_BOOT_B1                 ((uint8_t)0x00U)               /*!< boot from bank1 */
#define OB_BOOT_MASK               ((uint16_t)0x0008U)            /*!< MASK value */

/* read protect configure */
#define FMC_NSPC                   ((uint8_t)0xA5U)               /*!< no security protection */
#define FMC_USPC                   ((uint8_t)0xABU)               /*!< under security protection */

/* OB_SPC */
#define  OB_SPC_SPC                ((uint32_t)0x000000FFU)        /*!< option byte security protection value */
#define  OB_SPC_nSPC               ((uint32_t)0x0000FF00U)        /*!< option byte security protection complement value */

/* OB_USER */
#define  OB_USER_USER              ((uint32_t)0x00FF0000U)        /*!< user option value */
#define  OB_USER_nUSER             ((uint32_t)0xFF000000U)        /*!< user option complement value */

/* OB_WP0 */
#define  OB_WP0_WP0                ((uint32_t)0x000000FFU)        /*!< FMC write protection option value */
#define  OB_WP0_nWP0               ((uint32_t)0x0000FF00U)        /*!< FMC write protection option complement value */

/* OB_WP1 */
#define  OB_WP1_WP1                ((uint32_t)0x00FF0000U)        /*!< FMC write protection option value */
#define  OB_WP1_nWP1               ((uint32_t)0xFF000000U)        /*!< FMC write protection option complement value */

/* OB_WP2 */
#define  OB_WP2_WP2                ((uint32_t)0x000000FFU)        /*!< FMC write protection option value */
#define  OB_WP2_nWP2               ((uint32_t)0x0000FF00U)        /*!< FMC write protection option complement value */

/* OB_WP3 */
#define  OB_WP3_WP3                ((uint32_t)0x00FF0000U)        /*!< FMC write protection option value */
#define  OB_WP3_nWP3               ((uint32_t)0xFF000000U)        /*!< FMC write protection option complement value */

/* option bytes write protection */
#define OB_WP_0                    ((uint32_t)0x00000001U)        /*!< erase/program protection of sector 0  */
#define OB_WP_1                    ((uint32_t)0x00000002U)        /*!< erase/program protection of sector 1  */
#define OB_WP_2                    ((uint32_t)0x00000004U)        /*!< erase/program protection of sector 2  */
#define OB_WP_3                    ((uint32_t)0x00000008U)        /*!< erase/program protection of sector 3  */
#define OB_WP_4                    ((uint32_t)0x00000010U)        /*!< erase/program protection of sector 4  */
#define OB_WP_5                    ((uint32_t)0x00000020U)        /*!< erase/program protection of sector 5  */
#define OB_WP_6                    ((uint32_t)0x00000040U)        /*!< erase/program protection of sector 6  */
#define OB_WP_7                    ((uint32_t)0x00000080U)        /*!< erase/program protection of sector 7  */
#define OB_WP_8                    ((uint32_t)0x00000100U)        /*!< erase/program protection of sector 8  */
#define OB_WP_9                    ((uint32_t)0x00000200U)        /*!< erase/program protection of sector 9  */
#define OB_WP_10                   ((uint32_t)0x00000400U)        /*!< erase/program protection of sector 10 */
#define OB_WP_11                   ((uint32_t)0x00000800U)        /*!< erase/program protection of sector 11 */
#define OB_WP_12                   ((uint32_t)0x00001000U)        /*!< erase/program protection of sector 12 */
#define OB_WP_13                   ((uint32_t)0x00002000U)        /*!< erase/program protection of sector 13 */
#define OB_WP_14                   ((uint32_t)0x00004000U)        /*!< erase/program protection of sector 14 */
#define OB_WP_15                   ((uint32_t)0x00008000U)        /*!< erase/program protection of sector 15 */
#define OB_WP_16                   ((uint32_t)0x00010000U)        /*!< erase/program protection of sector 16 */
#define OB_WP_17                   ((uint32_t)0x00020000U)        /*!< erase/program protection of sector 17 */
#define OB_WP_18                   ((uint32_t)0x00040000U)        /*!< erase/program protection of sector 18 */
#define OB_WP_19                   ((uint32_t)0x00080000U)        /*!< erase/program protection of sector 19 */
#define OB_WP_20                   ((uint32_t)0x00100000U)        /*!< erase/program protection of sector 20 */
#define OB_WP_21                   ((uint32_t)0x00200000U)        /*!< erase/program protection of sector 21 */
#define OB_WP_22                   ((uint32_t)0x00400000U)        /*!< erase/program protection of sector 22 */
#define OB_WP_23                   ((uint32_t)0x00800000U)        /*!< erase/program protection of sector 23 */
#define OB_WP_24                   ((uint32_t)0x01000000U)        /*!< erase/program protection of sector 24 */
#define OB_WP_25                   ((uint32_t)0x02000000U)        /*!< erase/program protection of sector 25 */
#define OB_WP_26                   ((uint32_t)0x04000000U)        /*!< erase/program protection of sector 26 */
#define OB_WP_27                   ((uint32_t)0x08000000U)        /*!< erase/program protection of sector 27 */
#define OB_WP_28                   ((uint32_t)0x10000000U)        /*!< erase/program protection of sector 28 */
#define OB_WP_29                   ((uint32_t)0x20000000U)        /*!< erase/program protection of sector 29 */
#define OB_WP_30                   ((uint32_t)0x40000000U)        /*!< erase/program protection of sector 30 */
#define OB_WP_31                   ((uint32_t)0x80000000U)        /*!< erase/program protection of sector 31 */
#define OB_WP_ALL                  ((uint32_t)0xFFFFFFFFU)        /*!< erase/program protection of all sectors */

/* FMC interrupt enable */
#define FMC_INTEN_END              ((uint32_t)0x00001000U)        /*!< enable FMC end of program interrupt */
#define FMC_INTEN_ERR              ((uint32_t)0x00000400U)        /*!< enable FMC error interrupt */
#define FMC_INTEN_BANK1_END        ((uint32_t)0x80000400U)        /*!< enable FMC bank1 end of program interrupt */
#define FMC_INTEN_BANK1_ERR        ((uint32_t)0x80001000U)        /*!< enable FMC bank1 error interrupt  */

/* FMC flags */
#define FMC_FLAG_BANK0_BUSY        ((uint32_t)0x00000001U)        /*!< FMC bank0 busy flag */ 
#define FMC_FLAG_BANK0_PGERR       ((uint32_t)0x00000004U)        /*!< FMC bank0 operation error flag bit */
#define FMC_FLAG_BANK0_WPERR       ((uint32_t)0x00000010U)        /*!< FMC bank0 erase/program protection error flag bit */
#define FMC_FLAG_BANK0_END         ((uint32_t)0x00000020U)        /*!< FMC bank0 end of operation flag bit */
#define FMC_FLAG_OPRERR            ((uint32_t)0x00000001U)        /*!< FMC option bytes read error flag */
#define FMC_FLAG_BANK1_BUSY        ((uint32_t)0x00000001U)        /*!< FMC bank1 busy flag */ 
#define FMC_FLAG_BANK1_PGERR       ((uint32_t)0x00000004U)        /*!< FMC bank1 operation error flag bit */
#define FMC_FLAG_BANK1_WPERR       ((uint32_t)0x00000010U)        /*!< FMC bank1 erase/program protection error flag bit */
#define FMC_FLAG_BANK1_END         ((uint32_t)0x00000020U)        /*!< FMC bank1 end of operation flag bit */

/* FMC timeout */
#define FMC_TIMEOUT_COUNT          ((uint32_t)0x000F0000U)        /*!< FMC timeout count value */

/* FMC BANK address */
#define FMC_BANK0_END_ADDRESS      ((uint32_t)0x0807FFFFU)        /*!< FMC bank0 end address */
#define FMC_BANK0_SIZE             ((uint32_t)0x000000200U)       /*!< FMC bank0 size */
#define FMC_SIZE                   (*(uint32_t *)0x1ffff7e0U)     /*!< FMC size */

/* function declarations */
/* FMC main memory programming functions */
/* set the FMC wait state counter */
void fmc_wscnt_set(uint32_t wscnt);
/* unlock the main FMC operation */
void fmc_unlock(void);
/* unlock the FMC bank0 operation */
void fmc_bank0_unlock(void);
/* unlock the FMC bank1 operation */
void fmc_bank1_unlock(void);
/* lock the main FMC operation */
void fmc_lock(void);
/* lock the bank0 FMC operation */
void fmc_bank0_lock(void);
/* lock the bank1 FMC operation */
void fmc_bank1_lock(void);
/* FMC erase page */
fmc_state_enum fmc_page_erase(uint32_t page_address);
/* FMC erase whole chip */
fmc_state_enum fmc_mass_erase(void);
/* FMC erase whole bank0 */
fmc_state_enum fmc_bank0_erase(void);
/* FMC erase whole bank1 */
fmc_state_enum fmc_bank1_erase(void);
/* FMC program a word at the corresponding address */
fmc_state_enum fmc_word_program(uint32_t address, uint32_t data);
/* FMC program a half word at the corresponding address */
fmc_state_enum fmc_halfword_program(uint32_t address, uint16_t data);

/* FMC option bytes programming functions */
/* unlock the option byte operation */
void ob_unlock(void);
/* lock the option byte operation */
void ob_lock(void);
/* erase the option byte */
fmc_state_enum ob_erase(void);
/* enable write protect */
fmc_state_enum ob_write_protection_enable(uint32_t ob_wp);
/* enable the read out protection */
fmc_state_enum ob_read_out_enable(void);
/* disable the read out protection */
fmc_state_enum ob_read_out_disable(void);
/* set the option byte security protection level  */
fmc_state_enum ob_security_protection_config(uint8_t ob_spc);
/* write the FMC option byte user */
fmc_state_enum ob_user_write(uint32_t ob_fwdgt, uint32_t ob_deepsleep, uint32_t ob_stdby);
/* configure the boot mode */
fmc_state_enum ob_boot_mode_config(uint32_t boot_mode);
/* program option bytes data */
fmc_state_enum ob_program_data(uint32_t address, uint8_t data);
/* get the FMC option byte user */
uint8_t ob_user_get(void);
/* get the FMC option byte write protection */
uint32_t ob_write_protection_get(void);
/* get option byte security protection code value */
FlagStatus ob_spc_get(void);

/* FMC interrupts and flags management functions */
/* enable FMC interrupt */
void fmc_int_enable(uint32_t fmc_int);
/* disable FMC interrupt */
void fmc_int_disable(uint32_t fmc_int);
/* get flag set or reset */
FlagStatus fmc_flag_get(uint32_t flag);
/* clear the FMC pending flag */
void fmc_flag_clear(uint32_t flag);
/* return the FMC state */
fmc_state_enum fmc_state_get(void);
/* return the  FMC bank0 state */
fmc_state_enum fmc_bank0_state_get(void);
/* return the  FMC bank1 state */
fmc_state_enum fmc_bank1_state_get(void);
/* check FMC ready or not */
fmc_state_enum fmc_ready_wait(uint32_t timeout);
/* check FMC bank0 ready or not */
fmc_state_enum fmc_bank0_ready_wait(uint32_t timeout);
/* check FMC bank1 ready or not */
fmc_state_enum fmc_bank1_ready_wait(uint32_t timeout);

#endif /* GD32F30X_FMC_H */
