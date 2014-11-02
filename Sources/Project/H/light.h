#ifndef LIGHT_H_
#define LIGHT_H_

  
        #include "common.h"
        #include "gpio.h"
            
        #define RUNLIGHT 0
        #define WARNLIGHT 1

        #define LIGHT_ON 0
        #define LIGHT_OFF 1
        
        void light_init(void);
        void light_flash(void);    
        void light_ctrl(uint8 light,uint8 cmd);
        void light_runtime(uint8 cmd);
        void light_warning(uint8 cmd);

#endif 
