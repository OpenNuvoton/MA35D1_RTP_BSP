/****************************************************************************
 * @file     mbox_whc.h
 *
 * @brief    mbox_whc module header file.
 *
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef MAILBOX_WHC_H_
#define MAILBOX_WHC_H_

int Mbox_Notify(void *priv, uint32_t id);
int Mbox_Init(void);
int Mbox_Poll(struct virtio_device *vdev);


#endif /* MAILBOX_WHC_H_ */
