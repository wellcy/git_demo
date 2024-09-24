#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "main.h"
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
extern CAN_HandleTypeDef hcan1;


typedef struct
{
	float rotor_angle;
	float rotor_speed;
	float torque_current;
	float temp;
}moto_info_t;

void Can_filter_init(void);
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);





#endif
