#include "main.h"
#include "bsp_can.h"

extern CAN_HandleTypeDef hcan1;

moto_info_t motor_yaw_info;
uint8_t rx_data[8];

//筛选器的定义，是接收报文的必备函数
void Can_filter_init(void)
{
	CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

//当Fifo0接收到符合条件的数据时进入接收中断函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	
	if(hcan->Instance == CAN1)
  {
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //CAN接收数据
  switch(rx_header.StdId)
	{
	  case 0x201://此处仅接收了id为0x201电机的报文
	{
    motor_yaw_info.rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_yaw_info.rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_yaw_info.torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_yaw_info.temp           =   rx_data[6];
		break;
	}
	}
  }
}

//CAN发送数据给电机使其转动
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{	
    chassis_tx_message.StdId = 0x200;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

 HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, (uint32_t *)CAN_TX_MAILBOX0);
}
