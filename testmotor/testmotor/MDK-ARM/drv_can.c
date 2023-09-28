#include "struct_typedef.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

typedef struct
{
    uint16_t can_id;		//ID?
    int16_t  set_current;		//????
    uint16_t rotor_angle;		//?????
    int16_t  rotor_speed;		//?????
    int16_t  torque_current;		//??????
    uint8_t  temp;		//????
}motor_info_t;



void set_motor_current_can2( int16_t v)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[2];
    
  tx_header.StdId = 0x200;//??id_range==0???0x200,id_range==1???0x1ff(ID?)
  tx_header.IDE   = CAN_ID_STD;//???
  tx_header.RTR   = CAN_RTR_DATA;//???
	
  tx_header.DLC   = 8;		//??????(??)

	tx_data[0] = (v>>8)&0xff;	//?????		
  tx_data[1] =    (v)&0xff;
  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
	
}