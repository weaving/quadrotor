SET PATH=F:\Develop Tools\keil\ARM\ARMCC\bin\;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Program Files (x86)\NVIDIA Corporation\PhysX\Common;F:\usual_software\sam-ba_2.12\drv\;F:\usual_software\sam-ba_2.12
SET CPU_TYPE=STM32F407VE
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=Template
SET CPU_CLOCK=0x017D7840
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\main.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\stm32f4xx_it.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\system_stm32f4xx.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\led.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\24l01.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\flash_eeprom.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\hcsr04.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\mpu6050.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\timer.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\hmc5883.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\ov7670.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\sccb.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\exti.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\wft_controller.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\moto.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmAsm" --Via "..\obj\startup_stm32f40_41xxx._ia"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\misc.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\stm32f4xx_gpio.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\stm32f4xx_rcc.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\stm32f4xx_syscfg.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\stm32f4xx_usart.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\stm32f4xx_flash.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\stm32f4xx_exti.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\stm32f4xx_i2c.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\stm32f4xx_tim.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\stm32f4xx_spi.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\delay.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\sys.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\usart.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\os_core.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\os_flag.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\os_mbox.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\os_mem.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\os_mutex.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\os_q.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\os_sem.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\os_task.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\os_time.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\os_tmr.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmAsm" --Via "..\obj\os_cpu_a._ia"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\os_cpu_c.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\data_transfer.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\i2c.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\kalmanfilter.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\pwm_capture.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\spi.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\control.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\imu.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmCC" --Via "..\obj\pid.__i"
"F:\Develop Tools\keil\ARM\ARMCC\bin\ArmLink" --Via "..\OBJ\Template.lnp"
"F:\Develop Tools\keil\ARM\ARMCC\bin\fromelf.exe" "..\OBJ\Template.axf" --i32combined --output "..\OBJ\Template.hex"
