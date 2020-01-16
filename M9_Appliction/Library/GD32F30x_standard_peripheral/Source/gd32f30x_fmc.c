/*!
    \file  gd32f30x_fmc.c
    \brief FMC driver
*/

/*
    Copyright (C) 2017 GigaDevice

    2017-02-10, V1.0.0, firmware for GD32F30x
*/

#include "gd32f30x_fmc.h"

/*!
    \brief      set the wait state counter value
    \param[in]  wscnt£ºwait state counter value
      \arg        WS_WSCNT_0: FMC 0 wait
      \arg        WS_WSCNT_1: FMC 1 wait
      \arg        WS_WSCNT_2: FMC 2 wait
    \param[out] none
    \retval     none
*/
void fmc_wscnt_set(uint32_t wscnt)
{
    uint32_t reg;
    
    reg = FMC_WS;
    /* set the wait state counter value */
    reg &= ~FMC_WC_WSCNT;
    FMC_WS = (reg | wscnt);
}

/*!
    \brief      unlock the main FMC operation
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_unlock(void)
{
    if((RESET != (FMC_CTL0 & FMC_CTL0_LK))){
        /* write the FMC key */
        FMC_KEY0 = UNLOCK_KEY0;
        FMC_KEY0 = UNLOCK_KEY1;
    }
    if(FMC_BANK0_SIZE < FMC_SIZE){
        /* write the FMC key */
        if(RESET != (FMC_CTL1 & FMC_CTL1_LK)){
            FMC_KEY1 = UNLOCK_KEY0;
            FMC_KEY1 = UNLOCK_KEY1;
        }
    }
}

/*!
    \brief      unlock the FMC bank0 operation 
                this function can be used for all GD32F30x devices.
                - for GD32F30X_XD and GD32F30X_CL devices this function unlocks bank0.
                - for all other devices it unlocks bank0 and it is equivalent to fmc_unlock function.
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_bank0_unlock(void)
{
    if((RESET != (FMC_CTL0 & FMC_CTL0_LK))){
        /* write the FMC key */
        FMC_KEY0 = UNLOCK_KEY0;
        FMC_KEY0 = UNLOCK_KEY1;
    }
}

/*!
    \brief      unlock the FMC bank1 operation 
                this function can be used for GD32F30X_XD and GD32F30X_CL density devices.
    \param[in]  none
    \param[out] none
    \retval     none
*/
#if defined GD32F30X_XD || defined GD32F30X_CL
void fmc_bank1_unlock(void)
{
    if((RESET != (FMC_CTL1 & FMC_CTL1_LK))){
        /* write the FMC key */
        FMC_KEY1 = UNLOCK_KEY0;
        FMC_KEY1 = UNLOCK_KEY1;
    }
}
#endif /* GD32F30X_XD and GD32F30X_CL */

/*!
    \brief      lock the main FMC operation
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_lock(void)
{
    /* set the LK bit */
    FMC_CTL0 |= FMC_CTL0_LK;
    
    if(FMC_BANK0_SIZE < FMC_SIZE){
        /* set the LK bit */
        FMC_CTL1 |= FMC_CTL1_LK;
    }
}

/*!
    \brief      lock the FMC bank0 operation
                this function can be used for all GD32F30X devices.
                - for GD32F30X_XD and GD32F30X_CL devices this function locks bank0.
                - for all other devices it locks bank0 and it is equivalent to fmc_lock function.
    \param[in]  none
    \param[out] none
    \retval     none
*/
void fmc_bank0_lock(void)
{
    /* set the LK bit*/
    FMC_CTL0 |= FMC_CTL0_LK;
}

/*!
    \brief      lock the FMC bank1 operation
                this function can be used for GD32F30X_XD and GD32F30X_CL density devices.
    \param[in]  none
    \param[out] none
    \retval     none
*/
#if defined GD32F30X_XD || defined GD32F30X_CL
void fmc_bank1_lock(void)
{
    /* set the LK bit*/
    FMC_CTL1 |= FMC_CTL1_LK;
}
#endif /* GD32F30X_XD and GD32F30X_CL */

/*!
    \brief      erase page
    \param[in]  page_address: the page address to be erased.
    \param[out] none
    \retval     fmc_state_enum
*/
fmc_state_enum fmc_page_erase(uint32_t page_address)
{
    fmc_state_enum fmc_state = FMC_READY;
    if(FMC_BANK0_SIZE < FMC_SIZE){
        if(FMC_BANK0_END_ADDRESS > page_address){
            /* if the last operation is completed, start page erase */
            if( FMC_READY == fmc_state){
                FMC_CTL0 |= FMC_CTL0_PER;
                FMC_ADDR0 = page_address;
                FMC_CTL0 |= FMC_CTL0_START;
    
                /* wait for the FMC ready */
                fmc_state = fmc_bank0_ready_wait(FMC_TIMEOUT_COUNT);
    
                /* reset the PER bit */
                FMC_CTL0 &= ~FMC_CTL0_PER;
            }
            /* return the FMC state */
            return fmc_state;
        }else{
            /* wait for the FMC ready */
           fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);
            /* if the last operation is completed, start page erase */
            if(FMC_READY== fmc_state){
                FMC_CTL1 |= FMC_CTL1_PER;
                FMC_ADDR1 = page_address;
                if(FMC_OBCTL & FMC_OBCTL_SPC){
                    FMC_ADDR0 = page_address;
                }
                FMC_CTL1 |= FMC_CTL1_PER;
    
                /* wait for the FMC ready */
                fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);
    
                /* reset the PER bit */
                FMC_CTL1 &= ~FMC_CTL1_PER;
            }
            /* return the FMC state */
            return fmc_state;
        }

    }else{
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
        /* if the last operation is completed, start page erase */
        if(FMC_READY== fmc_state){
            FMC_CTL0 |= FMC_CTL0_PER;
            FMC_ADDR0  = page_address;
            FMC_CTL0 |= FMC_CTL0_START;
    
            /* wait for the FMC ready */
            fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
    
            /* reset the PER bit */
            FMC_CTL0 &= ~FMC_CTL0_PER;
        }
        /* return the FMC state */
        return fmc_state;
    }
}

/*!
    \brief      erase whole chip
    \param[in]  none
    \param[out] none
    \retval     fmc_state_enum
*/
fmc_state_enum fmc_mass_erase(void)
{
    fmc_state_enum fmc_state = FMC_READY;
    if(FMC_BANK0_SIZE < FMC_SIZE){
        /* wait for the FMC ready */
        fmc_state = fmc_bank0_ready_wait(FMC_TIMEOUT_COUNT);
        if(FMC_READY== fmc_state){
            /* start whole chip erase */
            FMC_CTL0 |= FMC_CTL0_MER;
            FMC_CTL0 |= FMC_CTL0_START;
    
            /* wait for the FMC ready */
            fmc_state = fmc_bank0_ready_wait(FMC_TIMEOUT_COUNT);

            /* reset the MER bit */
            FMC_CTL0 &= ~FMC_CTL0_MER;
        }
        fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);
        if(FMC_READY== fmc_state){
            /* start whole chip erase */
            FMC_CTL1 |= FMC_CTL1_MER;
            FMC_CTL1 |= FMC_CTL1_START;
    
            /* wait for the FMC ready */
            fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

            /* reset the MER bit */
            FMC_CTL1 &= ~FMC_CTL1_MER;
        }
        /* return the FMC state  */
        return fmc_state;
    }else{
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
  
        if(FMC_READY== fmc_state){
            /* start whole chip erase */
            FMC_CTL0 |= FMC_CTL0_MER;
            FMC_CTL0 |= FMC_CTL0_START;    
    
            /* wait for the FMC ready */
            fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);

            /* reset the MER bit */
            FMC_CTL0 &= ~FMC_CTL0_MER;
        }
        /* return the FMC state  */
        return fmc_state;
    }
}

/*!
    \brief      erase all FMC sectors in bank0
    \param[in]  none
    \param[out] none
    \retval     fmc_state_enum
*/
fmc_state_enum fmc_bank0_erase(void)
{
    fmc_state_enum fmc_state = FMC_READY;
    /* wait for the FMC ready */
    fmc_state = fmc_bank0_ready_wait(FMC_TIMEOUT_COUNT);

    if(FMC_READY== fmc_state){
        /* start FMC bank0 erase */
        FMC_CTL0 |= FMC_CTL0_MER;
        FMC_CTL0 |= FMC_CTL0_START;
    
        /* wait for the FMC ready */
        fmc_state = fmc_bank0_ready_wait(FMC_TIMEOUT_COUNT);

        /* reset the MER bit */
        FMC_CTL0 &= (~FMC_CTL0_MER);
    }

    /* return the fmc state */
    return fmc_state;
}

/*!
    \brief      erase all FMC sectors in bank1
    \param[in]  none
    \param[out] none
    \retval     fmc_state_enum
*/
fmc_state_enum fmc_bank1_erase(void)
{
    fmc_state_enum fmc_state = FMC_READY;
    /* wait for the FMC ready */
    fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);
  
   if(FMC_READY== fmc_state){
        /* start FMC bank1 erase */
        FMC_CTL1 |= FMC_CTL1_MER;
        FMC_CTL1 |= FMC_CTL1_START;
    
        /* wait for the FMC ready */
        fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);

        /* reset the MER bit */
        FMC_CTL1 &= (~FMC_CTL1_MER);
    }

    /* return the fmc state */
    return fmc_state;
}

/*!
    \brief      program a word at the corresponding address
    \param[in]  address: address to program
    \param[in]  data: word to program
    \param[out] none
    \retval     fmc_state_enum
*/
fmc_state_enum fmc_word_program(uint32_t address, uint32_t data)
{
    fmc_state_enum fmc_state = FMC_READY;
    if(FMC_BANK0_SIZE < FMC_SIZE){
        if(FMC_BANK0_END_ADDRESS > address){
            fmc_state = fmc_bank0_ready_wait(FMC_TIMEOUT_COUNT); 
  
            if(FMC_READY== fmc_state){
                /* set the PG bit to start program */
                FMC_CTL0 |= FMC_CTL0_PG;
  
                *(__IO uint32_t*)address = data;

                /* wait for the FMC ready */
                fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
    
                /* reset the PG bit */
                FMC_CTL0 &= ~FMC_CTL0_PG;
            } 
            /* return the fmc state */
            return fmc_state;
        }else{
            fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT); 
  
            if(FMC_READY== fmc_state){
                /* set the PG bit to start program */
                FMC_CTL1 |= FMC_CTL1_PG;
  
                *(__IO uint32_t*)address = data;

                /* wait for the FMC ready */
                fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);
    
                /* reset the PG bit */
                FMC_CTL1 &= ~FMC_CTL1_PG;
            }
            /* return the FMC state */
            return fmc_state;
        }

    }else{
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
  
        if(FMC_READY== fmc_state){
            /* set the PG bit to start program */
            FMC_CTL0 |= FMC_CTL0_PG;
  
            *(__IO uint32_t*)address = data;

            /* wait for the FMC ready */
            fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
    
            /* reset the PG bit */
            FMC_CTL0 &= ~FMC_CTL0_PG;
        } 
        /* return the FMC state */
        return fmc_state;
    }
}

/*!
    \brief      program a half word at the corresponding address
    \param[in]  address: address to program
    \param[in]  data: halfword to program
    \param[out] none
    \retval     fmc_state_enum
*/
fmc_state_enum fmc_halfword_program(uint32_t address, uint16_t data)
{
    fmc_state_enum fmc_state = FMC_READY;
    if(FMC_BANK0_SIZE > FMC_SIZE){
        fmc_state = fmc_bank0_ready_wait(FMC_TIMEOUT_COUNT); 
        if(FMC_BANK0_END_ADDRESS > address){
             fmc_state = fmc_bank0_ready_wait(FMC_TIMEOUT_COUNT); 
  
            if(FMC_READY== fmc_state){
                /* set the PG bit to start program */
                FMC_CTL0 |= FMC_CTL0_PG;
  
                *(__IO uint32_t*)address = data;

                /* wait for the FMC ready */
                fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
    
                /* reset the PG bit */
                FMC_CTL0 &= ~FMC_CTL0_PG;
            }
        }else{
            fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT); 
  
            if(FMC_READY== fmc_state){
                /* set the PG bit to start program */
                FMC_CTL1 |= FMC_CTL1_PG;
  
                *(__IO uint32_t*)address = data;

                /* wait for the FMC ready */
                fmc_state = fmc_bank1_ready_wait(FMC_TIMEOUT_COUNT);
    
                /* reset the PG bit */
                FMC_CTL1 &= ~FMC_CTL1_PG;
            }
        }
        /* return the FMC state */
        return fmc_state;
    }else{
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
  
        if(FMC_READY== fmc_state){
            /* set the PG bit to start program */
            FMC_CTL0 |= FMC_CTL0_PG;
  
            *(__IO uint32_t*)address = data;

            /* wait for the FMC ready */
            fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
    
            /* reset the PG bit */
            FMC_CTL0 &= ~FMC_CTL0_PG;
        } 

        /* return the FMC state */
        return fmc_state;

    }
}

/*!
    \brief      unlock the option byte operation
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ob_unlock(void)
{
    if(RESET != (FMC_CTL0 & FMC_CTL0_OBWPEN)){
        /* write the FMC key */
        FMC_OBKEY = UNLOCK_KEY0;
        FMC_OBKEY = UNLOCK_KEY1;
    }
}

/*!
    \brief      lock the option byte operation
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ob_lock(void)
{
    /* reset the OB_LK bit */
    FMC_CTL0 &= ~FMC_CTL0_OBWPEN;
}

/*!
    \brief      erase the FMC option byte
    \param[in]  none
    \param[out] none
    \retval     fmc_state_enum
*/
fmc_state_enum ob_erase(void)
{
    uint16_t temp_rdp = FMC_NSPC;

    fmc_state_enum fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);

    /* check the option byte security protection value */
    if(RESET != ob_spc_get()){
        temp_rdp = 0x00U;  
    }

    if(FMC_READY == fmc_state){
        /* write the FMC key */
        FMC_OBKEY = UNLOCK_KEY0;
        FMC_OBKEY = UNLOCK_KEY1;
        /* start erase the option byte */
        FMC_CTL0 |= FMC_CTL0_OBER;
        FMC_CTL0 |= FMC_CTL0_START;

        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
    
        if(FMC_READY == fmc_state){
            /* reset the OBER bit */
            FMC_CTL0 &= ~FMC_CTL0_OBER;
       
            /* set the OBPG bit */
            FMC_CTL0 |= FMC_CTL0_OBPG;

            /* no security protection */
            OB_SPC = (uint16_t)temp_rdp; 

            /* wait for the FMC ready */
            fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
 
            if(FMC_TIMEOUT_ERR != fmc_state){
                /* reset the OBPG bit */
                FMC_CTL0 &= ~FMC_CTL0_OBPG;
            }
        }else{
            if (FMC_TIMEOUT_ERR != fmc_state){
                /* reset the OBPG bit */
                FMC_CTL0 &= ~FMC_CTL0_OBPG;
            }
        }
    }
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      enable write protection
    \param[in]  ob_wp: specify sector to be write protected
      \arg        OB_WPx(x=0..31): write protect specify sector
      \arg        OB_WP_ALL: write protect all sector
    \param[out] none
    \retval     fmc_state_enum
*/
fmc_state_enum ob_write_protection_enable(uint32_t ob_wp)
{
    uint16_t temp_WP0, temp_WP1, temp_WP2, temp_WP3;

    fmc_state_enum fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);

    ob_wp = (uint32_t)(~ob_wp);
    temp_WP0 = (uint16_t)(ob_wp & OB_WP0_WP0);
    temp_WP1 = (uint16_t)((ob_wp & OB_WP0_nWP0) >> 8);
    temp_WP2 = (uint16_t)((ob_wp & OB_WP1_WP1) >> 16);
    temp_WP3 = (uint16_t)((ob_wp & OB_WP1_nWP1) >> 24);

    if(FMC_READY == fmc_state){
        FMC_OBKEY = UNLOCK_KEY0;
        FMC_OBKEY = UNLOCK_KEY1;
        /* set the OBPG bit*/
        FMC_CTL0 |= FMC_CTL0_OBPG;

        if(0xFFU != temp_WP0){
            OB_WP0 = temp_WP0;
      
            /* wait for the FMC ready */
            fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
        }
        if((FMC_READY == fmc_state) && (0xFFU != temp_WP1)){
            OB_WP1 = temp_WP1;
      
            /* wait for the FMC ready */
            fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
        }
        if((FMC_READY == fmc_state) && (0xFFU != temp_WP2)){
            OB_WP2 = temp_WP2;
      
            /* wait for the FMC ready */
            fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
        }
        if((FMC_READY == fmc_state) && (0xFFU != temp_WP3)){
            OB_WP2 = temp_WP3;
      
            /* wait for the FMC ready */
            fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
        }
        if(FMC_TIMEOUT_ERR != fmc_state){
            /* reset the OBPG bit */
            FMC_CTL0 &= ~FMC_CTL0_OBPG;
        }
    } 
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      enable the read out protection
    \param[in]  none
    \param[out] none
    \retval     fmc_state_enum
*/
fmc_state_enum ob_read_out_enable(void)
{
    fmc_state_enum fmc_state = FMC_READY;
    /* wait for the FMC ready */
    fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
    if(FMC_READY == fmc_state){
        /* unlock option byte */
        FMC_OBKEY = UNLOCK_KEY0;
        FMC_OBKEY = UNLOCK_KEY1;
        while((FMC_CTL0 & FMC_CTL0_OBWPEN) != FMC_CTL0_OBWPEN){
        }
        FMC_CTL0 |= FMC_CTL0_OBWPEN;
        FMC_CTL0 |= FMC_CTL0_START;
        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
        if(FMC_READY == fmc_state){
            /* disable the OBER bit */
            FMC_CTL0 &=~ FMC_CTL0_OBER ;
            /* enable the OBPG bit */
            FMC_CTL0 |= FMC_CTL0_OBPG; 
            /* enable the read out protection */
            OB_SPC = 0x00U;
            /* wait for the FMC ready */
            fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
    
            if(FMC_TIMEOUT_ERR != fmc_state){
                /* enable the OBPG Bit */
                FMC_CTL0 &=~ FMC_CTL0_OBPG ;
            }
        }else{
            if(FMC_TIMEOUT_ERR != fmc_state){
                /* disable the OBER bit */
                FMC_CTL0 &= ~FMC_CTL0_OBER ;
            }
        }
    }
    /* return the FMC state */
    return fmc_state;  
}

/*!
    \brief      disable the read out protection
    \param[in]  none
    \param[out] none
    \retval     fmc_state_enum
*/
fmc_state_enum ob_read_out_disable(void)
{
    fmc_state_enum fmc_state = FMC_READY;
    /* wait for the FMC ready */
    fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
    if(FMC_READY == fmc_state){
        /* unlock option byte */
        FMC_OBKEY = UNLOCK_KEY0;
        FMC_OBKEY = UNLOCK_KEY1;
        while((FMC_CTL0 & FMC_CTL0_OBWPEN) != FMC_CTL0_OBWPEN){
        }
        FMC_CTL0 |= FMC_CTL0_OBWPEN;
        FMC_CTL0 |= FMC_CTL0_START;
        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
        if(FMC_READY == fmc_state){
            /* disable the OBER bit */
            FMC_CTL0 &=~ FMC_CTL0_OBER ;
            /* enable the OBPG bit */
            FMC_CTL0 |= FMC_CTL0_OBPG; 
            /* disable the read out protection */
            OB_SPC = FMC_NSPC;  
            /* wait for the FMC ready */
            fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
    
            if(FMC_TIMEOUT_ERR != fmc_state){
                /* enable the OBPG Bit */
                FMC_CTL0 &=~ FMC_CTL0_OBPG ;
            }
        }else{
            if(FMC_TIMEOUT_ERR != fmc_state){
                /* disable the OBER bit */
                FMC_CTL0 &=~ FMC_CTL0_OBER ;
            }
        }
    }
    /* return the FMC state */
    return fmc_state;  
}

/*!
    \brief      configure security protection level
    \param[in]  ob_spc: specify security protection level
      \arg        FMC_NSPC: no security protection
      \arg        FMC_USPC: under security protection
    \param[out] none
    \retval     fmc_state_enum
*/
fmc_state_enum ob_security_protection_config(uint8_t ob_spc)
{
    fmc_state_enum fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);

    if(FMC_READY == fmc_state){
        FMC_CTL0 |= FMC_CTL0_OBER;
        FMC_CTL0 |= FMC_CTL0_START;
    
        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
    
        if(fmc_state == FMC_READY){
            /* reset the OBER bit */
            FMC_CTL0 &= ~FMC_CTL0_OBER;
      
            /* start the option byte program */
            FMC_CTL0 |= FMC_CTL0_OBPG;
       
            OB_SPC = ob_spc;

            /* wait for the FMC ready */
            fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT); 
    
            if(FMC_TIMEOUT_ERR != fmc_state){
                /* reset the OBPG bit */
                FMC_CTL0 &= ~FMC_CTL0_OBPG;
            }
        }else{
            if(FMC_TIMEOUT_ERR != fmc_state){
                /* reset the OBER bit */
                FMC_CTL0 &= ~ FMC_CTL0_OBER;
            }
        }
    }
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      program the FMC user option byte 
    \param[in]  ob_fwdgt: option byte watchdog value
      \arg        OB_FWDGT_SW: software free watchdog
      \arg        OB_FWDGT_HW: hardware free watchdog
    \param[in]  ob_deepsleep: option byte deepsleep reset value
      \arg        OB_DEEPSLEEP_NRST: no reset when entering deepsleep mode
      \arg        OB_DEEPSLEEP_RST: generate a reset instead of entering deepsleep mode 
    \param[in]  ob_stdby:option byte standby reset value
      \arg        OB_STDBY_NRST: no reset when entering standby mode
      \arg        OB_STDBY_RST: generate a reset instead of entering standby mode 
    \param[out] none
    \retval     fmc_state_enum
*/
fmc_state_enum ob_user_write(uint32_t ob_fwdgt, uint32_t ob_deepsleep, uint32_t ob_stdby)
{
    fmc_state_enum fmc_state = FMC_READY;

    /* wait for the FMC ready */
    fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
  
    if(FMC_READY == fmc_state){
        /* set the OBPG bit*/
        FMC_CTL0 |= FMC_CTL0_OBPG; 

        OB_USER = (uint16_t)((uint16_t)(ob_fwdgt | ob_deepsleep) | (uint16_t)(ob_stdby | 0xF8U));
    
        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);

        if(FMC_TIMEOUT_ERR != fmc_state){
            /* reset the OBPG bit */
            FMC_CTL0 &= ~FMC_CTL0_OBPG;
        }
    }
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      configure the option byte boot bank value
    \param[in]  boot_mode: specifies the option byte boot bank value
      \arg        OB_BOOT_B0: boot from bank0
      \arg        OB_BOOT_B1: boot from bank1 or bank0 if bank1 is void
    \param[out] none
    \retval     fmc_state_enum
*/
fmc_state_enum ob_boot_mode_config(uint32_t boot_mode)
{
    fmc_state_enum fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
    FMC_OBKEY = UNLOCK_KEY0;
    FMC_OBKEY = UNLOCK_KEY1;

    if(FMC_READY == fmc_state){
        /* set the OBPG bit*/
        FMC_CTL0 |= FMC_CTL0_OBPG;

        if(OB_BOOT_B0 == boot_mode){
            OB_USER |= OB_BOOT_MASK;
        }else{
            OB_USER &= (uint16_t)(~(uint16_t)(OB_BOOT_MASK));
        }

        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);

        if(FMC_TIMEOUT_ERR != fmc_state){
            /* reset the OBPG bit */
            FMC_CTL0 &= ~FMC_CTL0_OBPG;
        }
    }
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      program option bytes data
    \param[in]  address: the option bytes address to be programmed
    \param[in]  data: the byte to be programmed
    \param[out] none
    \retval     fmc_state_enum
*/
fmc_state_enum ob_program_data(uint32_t address, uint8_t data)
{
    fmc_state_enum fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);

    if(FMC_READY == fmc_state){
        /* set the OBPG bit */
        FMC_CTL0 |= FMC_CTL0_OBPG; 
        *(__IO uint16_t*)address = data;
    
        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
    
        if(FMC_TIMEOUT_ERR != fmc_state){
            /* reset the OBPG bit */
            FMC_CTL0 &= ~FMC_CTL0_OBPG;
        }
    }
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      get the FMC user option byte
    \param[in]  none
    \param[out] none
    \retval     the FMC user option byte values
*/
uint8_t ob_user_get(void)
{
    /* return the FMC user option byte value */
    return (uint8_t)(FMC_OBCTL >> 2);
}

/*!
    \brief      get the FMC option byte write protection
    \param[in]  none
    \param[out] none
    \retval     the FMC write protection option byte value
*/
uint32_t ob_write_protection_get(void)
{
    /* return the FMC write protection option byte value */
    return (uint32_t)(FMC_WP);
}

/*!
    \brief      get the FMC option byte security protection
    \param[in]  none
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus ob_spc_get(void)
{
    FlagStatus spc_state = RESET;

    if(RESET != (uint8_t)(FMC_OBCTL & ( FMC_OBCTL_SPC))){
        spc_state = SET;
    }
    else{
        spc_state = RESET;
    }
    return spc_state;
}

/*!
    \brief      enable FMC interrupt
    \param[in]  fmc_int: the FMC interrupt source
      \arg        FMC_INTEN_END: enable FMC end of program interrupt
      \arg        FMC_INTEN_ERR: enable FMC error interrupt
      \arg        FMC_INTEN_BANK1_END: enable FMC bank1 end of program interrupt
      \arg        FMC_INTEN_BANK1_ERR: enable FMC bank1 error interrupt
    \param[out] none
    \retval     none
*/
void fmc_int_enable(uint32_t fmc_int)
{
    if(FMC_SIZE > FMC_BANK0_SIZE){
        if( 0x0U != (fmc_int & 0x80000000U)){
            /* enable the interrupt sources */
            FMC_CTL1 |= (fmc_int & 0x7fffffffU);
        }else{
            /* enable the interrupt sources */
            FMC_CTL0 |= fmc_int;
        }
    }else{
        /* enable the interrupt sources */
        FMC_CTL0 |= fmc_int;}
}

/*!
    \brief      disable FMC interrupt
    \param[in]  fmc_int: the FMC interrupt source
      \arg        FMC_INTEN_END: enable FMC end of program interrupt
      \arg        FMC_INTEN_ERR: enable FMC error interrupt
      \arg        FMC_INTEN_BANK1_END: enable FMC bank1 end of program interrupt
      \arg        FMC_INTEN_BANK1_ERR: enable FMC bank1 error interrupt
    \param[out] none
    \retval     none
*/
void fmc_int_disable(uint32_t fmc_int)
{
    if(FMC_SIZE > FMC_BANK0_SIZE){
        if( 0x0U != (fmc_int & 0x80000000U)){
            /* disable the interrupt sources */
            FMC_CTL1 &= ~(uint32_t)(fmc_int&0x7fffffffU);
        }else{
            /* disable the interrupt sources */
            FMC_CTL0 &= ~(uint32_t)fmc_int;
            }
    }else{
        /* disable the interrupt sources */
        FMC_CTL0 &= ~(uint32_t)fmc_int;
        }
    }

/*!
    \brief      get flag set or reset
    \param[in]  flag: check FMC flag
      \arg        FMC_FLAG_BANK0_BUSY: FMC bank0 busy flag bit
      \arg        FMC_FLAG_BANK0_PGERR: FMC bank0 operation error flag bit
      \arg        FMC_FLAG_BANK0_WPERR: FMC bank0 erase/program protection error flag bit
      \arg        FMC_FLAG_BANK0_END: FMC bank0 end of operation flag bit
      \arg        FMC_FLAG_OPRERR: FMC option bytes read error flag bit
      \arg        FMC_FLAG_BANK1_BUSY: FMC bank1 busy flag bit
      \arg        FMC_FLAG_BANK1_PGERR: FMC bank1 operation error flag bit
      \arg        FMC_FLAG_BANK1_WPERR: FMC bank1 erase/program protection error flag bit
      \arg        FMC_FLAG_BANK1_END: FMC bank1 end of operation flag bit
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus fmc_flag_get(uint32_t flag)
{
    if(FMC_BANK0_SIZE < FMC_SIZE){
        if( FMC_FLAG_OPRERR == flag){
            if((uint32_t)RESET != (FMC_OBCTL & FMC_FLAG_OPRERR)){
                return SET;
            }else{
                return RESET;
            }
        }else{
            if(0x0U != (flag & 0x80000000U)){
                if((uint32_t)RESET != (FMC_STAT1 & flag)){
                    return SET;
                }else{
                    return RESET;
                }
            }else{
                if((uint32_t)RESET != (FMC_STAT0 & flag)){
                    return SET;
                }else{
                    return RESET;
                }
            }
        }
    }else{
        if(FMC_FLAG_OPRERR == flag){
            if((uint32_t)RESET != (FMC_OBCTL & FMC_FLAG_OPRERR)){
                return SET;
            }else{
                return RESET;
            }
        }else{
            if((uint32_t)RESET != (FMC_STAT0 & flag)){
                return  SET;
            }else{
                return RESET;
            }
        }
    }
}

/*!
    \brief      clear the FMC pending flag
    \param[in]  flag: clear FMC flag
      \arg        FMC_FLAG_BANK0_BUSY: FMC bank0 busy flag bit
      \arg        FMC_FLAG_BANK0_PGERR: FMC bank0 operation error flag bit
      \arg        FMC_FLAG_BANK0_WPERR: FMC bank0 erase/program protection error flag bit
      \arg        FMC_FLAG_BANK0_END: FMC bank0 end of operation flag bit
      \arg        FMC_FLAG_OPRERR: FMC option bytes read error flag bit
      \arg        FMC_FLAG_BANK1_BUSY: FMC bank1 busy flag bit
      \arg        FMC_FLAG_BANK1_PGERR: FMC bank1 operation error flag bit
      \arg        FMC_FLAG_BANK1_WPERR: FMC bank1 erase/program protection error flag bit
      \arg        FMC_FLAG_BANK1_END: FMC bank1 end of operation flag bit
    \param[out] none
    \retval     none
*/
void fmc_flag_clear(uint32_t flag)
{
    if(FMC_BANK0_SIZE < FMC_SIZE){
        if(0x0U != (flag & 0x80000000U)){
            /* clear the flags */
            FMC_STAT1 = flag;
        }else{
            /* clear the flags */
            FMC_STAT0 = flag;
        }
    }else{
        FMC_STAT0 = flag;
    }
}

/*!
    \brief      get the FMC state
    \param[in]  none
    \param[out] none
    \retval     fmc_state_enum
*/
fmc_state_enum fmc_state_get(void)
{
    fmc_state_enum fmc_state = FMC_READY;
  
    if(FMC_STAT0_BUSY == (FMC_STAT0 & FMC_STAT0_BUSY)){
        fmc_state = FMC_BUSY;
    }else{
        if((uint32_t)0x00 != (FMC_STAT0 & (uint32_t)FMC_STAT0_WPERR)){
            fmc_state = FMC_WPERR;
        }else{
            if((uint32_t)0x00 != (FMC_STAT0 & (uint32_t)(FMC_STAT0_PGERR))){
                fmc_state = FMC_PGERR; 
            }else{
                fmc_state = FMC_READY;
            }
        }
    }
    /* Return the FMC state */
    return fmc_state;
}

/*!
    \brief      get the FMC bank0 state
    \param[in]  none
    \param[out] none
    \retval     fmc_state_enum
*/
fmc_state_enum fmc_bank0_state_get(void)
{
    fmc_state_enum fmc_state = FMC_READY;
  
    if(FMC_STAT0_BUSY == (FMC_STAT0 & FMC_STAT0_BUSY)){
        fmc_state = FMC_BUSY;
    }else{
        if((uint32_t)0x00 != (FMC_STAT0 & (uint32_t)FMC_STAT0_WPERR)){
            fmc_state = FMC_WPERR;
        }else{
            if((uint32_t)0x00 != (FMC_STAT0 & (uint32_t)(FMC_STAT0_PGERR))){
                fmc_state = FMC_PGERR; 
            }else{
                fmc_state = FMC_READY;
            }
        }
    }
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      get the FMC bank1 state
    \param[in]  none
    \param[out] none
    \retval     fmc_state_enum
*/
fmc_state_enum fmc_bank1_state_get(void)
{
    fmc_state_enum fmc_state = FMC_READY;

    if((FMC_STAT1 & FMC_STAT1_BUSY&0x7fffffffU) == (FMC_STAT1_BUSY&0x7fffffffU)){
        fmc_state = FMC_BUSY;
    }else{
        if((uint32_t)0x00 != (FMC_STAT1 & FMC_STAT1_WPERR&0x7fffffffU)){
            fmc_state = FMC_WPERR;
        }else{
            if((uint32_t)0x00 != (FMC_STAT1 & FMC_STAT1_PGERR&0x7fffffffU)){
                fmc_state = FMC_PGERR; 
            }else{
                fmc_state = FMC_READY;
            }
        }
    }

    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      check whether FMC is ready or not
    \param[in]  timeout:count of loop
    \param[out] none
    \retval     fmc_state_enum
*/
fmc_state_enum fmc_ready_wait(uint32_t timeout)
{
    fmc_state_enum fmc_state = FMC_BUSY;
  
    /* wait for FMC ready */
    do{
        /* get FMC state */
        fmc_state = fmc_state_get();
        timeout--;
    }while((FMC_BUSY == fmc_state) && (0x00U != timeout));
    if(FMC_BUSY == fmc_state){
        fmc_state = FMC_TIMEOUT_ERR;
    }
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      check whether FMC bank0 is ready or not
    \param[in]  timeout:count of loop
    \param[out] none
    \retval     fmc_state_enum
*/
fmc_state_enum fmc_bank0_ready_wait(uint32_t timeout)
{
    fmc_state_enum fmc_state = FMC_BUSY;
  
    /* wait for FMC ready */
    do{
        /* get FMC state */
        fmc_state = fmc_bank0_state_get();
        timeout--;
    }while((FMC_BUSY == fmc_state) && (0x00U != timeout));
    if(FMC_BUSY == fmc_state){
        fmc_state = FMC_TIMEOUT_ERR;
    }
    /* return the FMC state */
    return fmc_state;
}

/*!
    \brief      check whether FMC bank1 is ready or not
    \param[in]  FMC_TIMEOUT_COUNT:count of loop
    \param[out] none
    \retval     fmc_state_enum
*/
fmc_state_enum fmc_bank1_ready_wait(uint32_t timeout)
{
    fmc_state_enum fmc_state = FMC_BUSY;
  
    /* wait for FMC ready */
    do{
        /* get FMC state */
        fmc_state = fmc_bank1_state_get();
        timeout--;
    }while((0x00U != timeout)&& ((FMC_BUSY & 0x7FFFFFFFU) == fmc_state));
    if(FMC_BUSY == fmc_state){
        fmc_state = FMC_TIMEOUT_ERR;
    }
    /* return the FMC state */
    return fmc_state;
}
