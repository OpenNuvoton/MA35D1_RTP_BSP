/****************************************************************************
 * @file     mbox_whc.h
 *
 * @brief    mbox_whc module header file.
 *
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef MAILBOX_WHC_H_
#define MAILBOX_WHC_H_

#define COMMAND_SEND_MSG_TO_A35 0x60
#define COMMAND_RECEIVE_A35_ACK 0x61

#define COMMAND_RECEIVE_A35_MSG 0x80
#define COMMAND_RETURN_ACK      0x81

#define COMMAND_RECEIVE_A35_TEST_START 0x90
#define COMMAND_RECEIVE_A35_TEST_TX    0x91
#define COMMAND_RECEIVE_A35_TEST_RX    0x92

#define RX_NO_MSG           1
#define RX_NEW_MSG          2
#define RX_BUF_FREE         3

#define TX_NO_ACK           4
#define TX_ACK              5

#define TEST_TX             6
#define TEST_RX             7

int Mbox_Notify(void *priv, uint32_t id);
int Mbox_Init(void);
int Mbox_Poll(struct rpmsg_endpoint *ept);


#endif /* MAILBOX_WHC_H_ */
