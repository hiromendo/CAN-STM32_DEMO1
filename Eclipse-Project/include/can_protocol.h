#ifndef INCLUDE_CAN_PROTOCOL_H
#define INCLUDE_CAN_PROTOCOL_H

typedef enum can_command_e {
	CAN_CMD_QUERY 		= 0xFF,
	CAN_CMD_MASTER_ACK 	= 0xAA,
	CAN_CMD_SLAVE_ACK 	= 0xCC,
	CAN_CMD_SET 		= 0x02
} can_command_t;

#endif


TODO move the file!!!
