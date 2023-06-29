#include "cpp_target_config.h"

//static const std::vector<OutputPin> motor_spi_cspins{OutputPin(*SPI1_SS1_GPIO_Port, SPI1_SS1_Pin), OutputPin(*SPI1_SS2_GPIO_Port, SPI1_SS2_Pin),OutputPin(*SPI1_SS3_GPIO_Port, SPI1_SS3_Pin)};
//extern SPI_HandleTypeDef hspi1;
//SPIPort motor_spi{hspi1,motor_spi_cspins,96000000,false};

#ifdef GPIO_MOTOR
const OutputPin gpMotor{*DRV_GP1_GPIO_Port,DRV_GP1_Pin};
#endif


#ifdef CANBUS
/*
 * Can BTR register for different speed configs
 * 50, 100, 125, 250, 500, 1000 kbit
 */
const OutputPin canSilentPin = OutputPin(*CAN_S_GPIO_Port, CAN_S_Pin);
CANPort canport{CANPORT,&canSilentPin};
const uint32_t canSpeedBTR_preset[] = { 0x001b0037,0x001b001b,0x001c0014,0x001a000b,0x001a0005,0x001a0002};

#endif


#ifdef PWMDRIVER

const PWMConfig pwmTimerConfig =
{
	.channel_1 = TIM_CHANNEL_1,
	.channel_2 = TIM_CHANNEL_2,
	.channel_3 = TIM_CHANNEL_3,
	.channel_4 = TIM_CHANNEL_4,

	.pwm_chan = 1,
	.dir_chan = 2, // Changed from default 3 for OSW pinout
	.dir_chan_n = 4, // Should be PE 10. Check if required by any application

	.centerpwm_chan = 1,

	.rcpwm_chan = 1,

	.dualpwm1 = 1,
	.dualpwm2 = 2,

	.timer = &TIM_PWM,
	.timerFreq = 168000000
};
#endif
