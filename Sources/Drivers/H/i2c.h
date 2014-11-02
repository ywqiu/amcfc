#ifndef I2C_H_
#define I2C_H_


        #include "common.h"   


        #define i2c0_Start()    I2C0_C1 |= I2C_C1_TX_MASK;\
                                I2C0_C1 |= I2C_C1_MST_MASK
        
        #define i2c0_Stop()     I2C0_C1 &= ~I2C_C1_MST_MASK;\
                                I2C0_C1 &= ~I2C_C1_TX_MASK
        
        #define i2c0_Wait()     while((I2C0_S & I2C_S_IICIF_MASK)==0) {} \
                                I2C0_S |= I2C_S_IICIF_MASK;

        
        void i2c_init(void);
                                                                     
        void send_signal(uint8 Signal);
        
        uint8 i2c_wait(uint8 x);

        void pause(void);
        

#endif