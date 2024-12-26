/**************************************************************************//**
 * @file     main.c
 *
 * @brief    This project demonstrates AMP between Core A and Core M,
 *           with Core A running Linux and Core M running FreeRTOS.
 *           Fill in your endpoints in "eptinst[]" to complete the design.
 *
 * Core0      CoreM (this core)
 *   A  <----->  B (Tx & Rx) (High freq. short packet)
 *   C  <----->  D (Tx & Rx) (Low freq. long packet)
 *   E  <----->  F (Tx & Rx) (CRC test)
 *
 * @note     HWSEM 6 & 7 have been assigned to OpenAMP for IPI.
 *
 * @copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include "NuMicro.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "platform_info.h"
#include "rsc_table.h"

#define TASKB_TX_SIZE   0x400
#define TASKD_TX_SIZE   0x2800
#define TASKF_TX_SIZE   0x400

int ReadTaskB_cb(struct rpmsg_endpoint *ept, void *data, size_t len, uint32_t src, void *priv);
int ReadTaskD_cb(struct rpmsg_endpoint *ept, void *data, size_t len, uint32_t src, void *priv);
int ReadTaskF_cb(struct rpmsg_endpoint *ept, void *data, size_t len, uint32_t src, void *priv);
void vPollTaskB( void *pvParameters );
void vSendTaskB( void *pvParameters );
void vPollTaskD( void *pvParameters );
void vSendTaskD( void *pvParameters );
void vPollTaskF( void *pvParameters );
void vSendTaskF( void *pvParameters );

/* User defined endpoints */
struct amp_endpoint eptinst[] = {
	{ {"eptA->B", EPT_TYPE_RX}, ReadTaskB_cb, vPollTaskB },        /* Task B */
	{ {"eptB->A", EPT_TYPE_TX, TASKB_TX_SIZE}, NULL, vSendTaskB }, /* Task B */
	{ {"eptC->D", EPT_TYPE_RX}, ReadTaskD_cb, vPollTaskD },        /* Task D */
	{ {"eptD->C", EPT_TYPE_TX, TASKD_TX_SIZE}, NULL, vSendTaskD }, /* Task D */
    { {"eptE->F", EPT_TYPE_RX}, ReadTaskF_cb, vPollTaskF },        /* Task F */
	{ {"eptF->E", EPT_TYPE_TX, TASKF_TX_SIZE}, NULL, vSendTaskF }, /* Task F */
};
char tx_bufB[TASKB_TX_SIZE];
char tx_bufD[TASKD_TX_SIZE];

TaskHandle_t createTaskHandle;
unsigned long throughput[4];

#define rpmsgEpt_to_ampEpt(ept) metal_container_of(ept, struct amp_endpoint, ept)

void UART16_Init()
{
    /* Enable UART16 clock */
    CLK_EnableModuleClock(UART16_MODULE);
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV3_UART16(1));

    /* Set multi-function pins */
    SYS->GPK_MFPL &= ~(SYS_GPK_MFPL_PK2MFP_Msk | SYS_GPK_MFPL_PK3MFP_Msk);
    SYS->GPK_MFPL |= (SYS_GPK_MFPL_PK2MFP_UART16_RXD | SYS_GPK_MFPL_PK3MFP_UART16_TXD);

	/* Init UART to 115200-8n1 for print message */
	UART_Open(UART16, 115200);
}

void SYS_Init()
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable IP clock */
    CLK_EnableModuleClock(UART16_MODULE);
    /* Select IP clock source */
    CLK_SetModuleClock(UART16_MODULE, CLK_CLKSEL3_UART16SEL_HXT, CLK_CLKDIV3_UART16(1));
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();
    /* Set multi-function pins for UART */
    SYS->GPK_MFPL &= ~(SYS_GPK_MFPL_PK2MFP_Msk | SYS_GPK_MFPL_PK3MFP_Msk);
    SYS->GPK_MFPL |= (SYS_GPK_MFPL_PK2MFP_UART16_RXD | SYS_GPK_MFPL_PK3MFP_UART16_TXD);
    /* Lock protected registers */
    SYS_LockReg();
    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART16, 115200);
}

/**
 * @brief Create endpoint and its task
 *
 * @param rpdev pointer to rpmsg_device
 * @param amp_ept pointer to an inst of amp_endpoint
 * @return int
 */
int amp_create_ept(struct rpmsg_device *rpdev, struct amp_endpoint *amp_ept)
{
    struct rpmsg_endpoint_info *info = &amp_ept->eptinfo;
    int ret;
    static int id = 0; // serial number

    if(info->type == EPT_TYPE_TX) {
        ret = ma35_rpmsg_create_txept(&amp_ept->ept, rpdev, info->name, info->size);
    } else if (info->type == EPT_TYPE_RX) {
        ret = ma35_rpmsg_create_rxept(&amp_ept->ept, rpdev, info->name, amp_ept->cb);
    } else {
        printf("Invalid endpoint type.\n");
        ret = -1;
    }

    if(ret >= 0) {
        xTaskCreate( amp_ept->task_fn, amp_ept->eptinfo.name, configMINIMAL_STACK_SIZE, &amp_ept->ept, tskIDLE_PRIORITY + 2, &amp_ept->taskHandle );
        printf("rpmsg%d: %s endpoint \"%s\" is created.\n", id++,
            (info->type == EPT_TYPE_TX) ? "Tx" : "Rx",
            info->name);
    } else {
        printf("Failed to create endpoint \"%s\", err: 0x%x.\n", info->name, ret);
    }

	return ret;
}

/**
 * @brief Start AMP
 *
 * @param rpdev pointer to rpmsg_device
 * @return int
 */
int amp_open(struct rpmsg_device *rpdev)
{
    int i, ret, inst;
    inst = sizeof(eptinst) / sizeof(eptinst[0]);

    for(i = 0; i < inst; i++) {
		ret = amp_create_ept(rpdev, &eptinst[i]);
		if (ret < 0) {
			return -EINVAL;
		}
	}

    return 0;
}

/**
 * @brief Recycle endpoint
 *
 * @param amp_ept pointer to an inst of amp_endpoint
 * @return int
 */
int amp_destroy_ept(struct amp_endpoint *amp_ept)
{
    int ret;

    if(amp_ept->taskHandle)
    {
        vTaskDelete(amp_ept->taskHandle);
        amp_ept->taskHandle = NULL;
    }

    ret = ma35_rpmsg_destroy_ept(&amp_ept->ept);
    if(ret)
        printf("Failed to destroy endpoint \"%s\".\n", amp_ept->eptinfo.name);
    else
        printf("Endpoint \"%s\" is destroyed.\n", amp_ept->eptinfo.name);

    return ret;
}

/**
 * @brief End AMP
 *
 * @return int
 */
int amp_close(void)
{
    int i, ret, inst;
    inst = sizeof(eptinst) / sizeof(eptinst[0]);

    for(i = 0; i < inst; i++) {
		ret = amp_destroy_ept(&eptinst[i]);
	}
    
    return 0;
}

/**
 * @brief User Rx callback, do not call this directly.
 *        The function is called when a POLLIN event is received.
 *
 * @param ept rpmsg endpoint
 * @param data pointer to rx buffer provided by driver
 * @param len length of rx buffer
 * @param src unused
 * @param priv unused
 * @return int always return RPMSG_SUCCESS
 */
int ReadTaskB_cb(struct rpmsg_endpoint *ept, void *data, size_t len, uint32_t src, void *priv)
{
	char *rxbuf = data;
	int rxlen = len;
	(void)src;
	(void)priv;

    /* Start of user read function */
    throughput[0] += rxlen;
    /* End of user read function */

	return RPMSG_SUCCESS;
}

/**
 * @brief User poll task, call ma35_rpmsg_poll to get status
 *
 * @param pvParameters ept
 */
void vPollTaskB( void *pvParameters )
{
    int ret;

    for( ;; )
    {
        ret = ma35_rpmsg_poll(pvParameters);
        if(ret == RPMSG_ERR_PERM)
        {
            printf("%s: Remote closed.\n", pcTaskGetName(NULL));
            /* 1. do nothing and try reconnecting 2. call ma35_rpmsg_destroy_ept and exit */
            // ma35_rpmsg_destroy_ept(pvParameters);
            // vTaskDelete(NULL);
        }
        else if(ret == RPMSG_ERR_NO_BUFF)
            printf("%s: Rx buffer is full.\n", pcTaskGetName(NULL));

        vTaskDelay(2);
    }
}

/**
 * @brief User Tx task, call ma35_rpmsg_send to send data
 *
 * @param pvParameters ept
 */
void vSendTaskB( void *pvParameters )
{
	int i, ret, size, j = 0;

    for( ;; )
    {
        /* Start of user write function */
        size = rand();
        size = size % TASKB_TX_SIZE;
        size = size ? size : TASKD_TX_SIZE;

        for(i = 0; i < size; i++)
            tx_bufB[i] = size + i;
        /* End of user write function */

        ret = ma35_rpmsg_send(pvParameters, tx_bufB, size);
        if(ret < 0) {
			if(ret == RPMSG_ERR_PERM)
			{
                printf("%s: Remote closed.\n", pcTaskGetName(NULL));
                /* 1. do nothing and try reconnecting 2. call ma35_rpmsg_destroy_ept and exit */
                // ma35_rpmsg_destroy_ept(pvParameters);
                // vTaskDelete(NULL);
            }
			else if(ret == RPMSG_ERR_NO_BUFF)
                printf("%s: Tx blocking.\n", pcTaskGetName(NULL));
			// else if(ret == RPMSG_ERR_INIT)
			// 	printf("%s: Remote ept not ready.\n", pcTaskGetName(NULL));
		}
        else {
            throughput[1] += ret;
        }

        vTaskDelay(10);
    }
}

/**
 * @brief User Rx callback, do not call this directly.
 *        The function is called when a POLLIN event is received.
 *
 * @param ept rpmsg endpoint
 * @param data pointer to rx buffer provided by driver
 * @param len length of rx buffer
 * @param src unused
 * @param priv unused
 * @return int always return RPMSG_SUCCESS
 */
int ReadTaskD_cb(struct rpmsg_endpoint *ept, void *data, size_t len, uint32_t src, void *priv)
{
	char *rxbuf = data;
	int rxlen = len;
	(void)src;
	(void)priv;

    /* Start of user read function */
    throughput[2] += rxlen;
    /* End of user read function */

	return RPMSG_SUCCESS;
}

/**
 * @brief User poll task, call ma35_rpmsg_poll to get status
 *
 * @param pvParameters ept
 */
void vPollTaskD( void *pvParameters )
{
    int ret;

    for( ;; )
    {
        ret = ma35_rpmsg_poll(pvParameters);
        if(ret == RPMSG_ERR_PERM)
        {
            printf("%s: Remote closed.\n", pcTaskGetName(NULL));
            /* 1. do nothing and try reconnecting 2. call ma35_rpmsg_destroy_ept and exit */
            // ma35_rpmsg_destroy_ept(pvParameters);
            // vTaskDelete(NULL);
        }
        else if(ret == RPMSG_ERR_NO_BUFF)
            printf("%s: Rx buffer is full.\n", pcTaskGetName(NULL));

        vTaskDelay(10);
    }
}

/**
 * @brief User Tx task, call ma35_rpmsg_send to send data
 *
 * @param pvParameters ept
 */
void vSendTaskD( void *pvParameters )
{
	int i, ret, size, j = 0;

    for( ;; )
    {
        /* Start of user write function */
        size = rand();
        size = size % TASKD_TX_SIZE;
        size = size ? size : TASKD_TX_SIZE;

        for(i = 0; i < size; i++)
            tx_bufD[i] = size + i;
        /* End of user write function */

        ret = ma35_rpmsg_send(pvParameters, tx_bufD, size);
        if(ret < 0) {
			if(ret == RPMSG_ERR_PERM)
			{
                printf("%s: Remote closed.\n", pcTaskGetName(NULL));
                /* 1. do nothing and try reconnecting 2. call ma35_rpmsg_destroy_ept and exit */
                // ma35_rpmsg_destroy_ept(pvParameters);
                // vTaskDelete(NULL);
            }
			else if(ret == RPMSG_ERR_NO_BUFF)
                printf("%s: Tx blocking.\n", pcTaskGetName(NULL));
			// else if(ret == RPMSG_ERR_INIT)
			// 	printf("%s: Remote ept not ready.\n", pcTaskGetName(NULL));
		}
        else {
            throughput[3] += ret;
        }

        vTaskDelay(100);
    }
}

extern uint32_t crc32(uint32_t crc, void *data, size_t length);
/**
 * @brief User Rx callback, do not call this directly.
 *        The function is called when a POLLIN event is received.
 *
 * @param ept rpmsg endpoint
 * @param data pointer to rx buffer provided by driver
 * @param len length of rx buffer
 * @param src unused
 * @param priv unused
 * @return int always return RPMSG_SUCCESS
 */
int ReadTaskF_cb(struct rpmsg_endpoint *ept, void *data, size_t len, uint32_t src, void *priv)
{
    struct amp_endpoint *eptRead = rpmsgEpt_to_ampEpt(ept);
    struct amp_endpoint *eptSend = eptRead + 1;
	char *rxbuf = data;
	int rxlen = len;
	(void)src;
	(void)priv;

    /* Start of user read function */
    uint32_t crc_cal = crc32(0UL, rxbuf, rxlen);
    // signal to vSendTaskF
    xTaskNotify(eptSend->taskHandle, crc_cal, eSetValueWithOverwrite);
    /* End of user read function */

	return RPMSG_SUCCESS;
}

/**
 * @brief User poll task, call ma35_rpmsg_poll to get status
 *
 * @param pvParameters ept
 */
void vPollTaskF( void *pvParameters )
{
    int ret;

    for( ;; )
    {
        ret = ma35_rpmsg_poll(pvParameters);
        if(ret == RPMSG_ERR_PERM)
        {
            printf("%s: Remote closed.\n", pcTaskGetName(NULL));
            /* 1. do nothing and try reconnecting 2. call ma35_rpmsg_destroy_ept and exit */
            // ma35_rpmsg_destroy_ept(pvParameters);
            // vTaskDelete(NULL);
        }
        else if(ret == RPMSG_ERR_NO_BUFF)
            printf("%s: Rx buffer is full.\n", pcTaskGetName(NULL));

        vTaskDelay(100);
    }
}

/**
 * @brief User Tx task, call ma35_rpmsg_send to send data
 *
 * @param pvParameters ept
 */
void vSendTaskF( void *pvParameters )
{
	int i, ret, size = sizeof(uint32_t);
    uint32_t txbuf;

    for( ;; )
    {
        /* Start of user write function */
        // This task is woken up by ReadTaskF_cb
        txbuf = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        /* End of user write function */

        ret = ma35_rpmsg_send(pvParameters, &txbuf, size);
        if(ret < 0) {
            if(ret == RPMSG_ERR_NO_BUFF)
                printf("%s: Tx blocking.\n", pcTaskGetName(NULL));
        }
    }
}

void vEndpointCreateTask( void *pvParameters )
{
    struct rpmsg_device *rpdev = pvParameters;
    int delay = 1000;

    for( ;; )
    {
        if(ma35_rpmsg_remote_ready())
        {
            amp_open(rpdev);
            delay = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }

        vTaskDelay(delay);
    }
}

void vCmdTask( void *pvParameters )
{
    struct amp_endpoint *amp_ept = pvParameters;
    char c;
    unsigned long stattick = 0;
    int inst, avail, total;

    while(!ma35_rpmsg_remote_ready());

    for( ;; )
    {
        ++stattick;
        if (UART_IS_RX_READY(UART16))
        {
            c = getchar();
            printf("%c\n", c);
            switch(c)
            {
                case '0':
                    amp_destroy_ept(&amp_ept[c-'0']);
                    break;
                case '1':
                    amp_destroy_ept(&amp_ept[c-'0']);
                    break;
                case '2':
                    amp_destroy_ept(&amp_ept[c-'0']);
                    break;
                case '3':
                    amp_destroy_ept(&amp_ept[c-'0']);
                    break;
                case 'g':
                    ma35_rpmsg_get_buffer_size(&avail, &total);
                    printf("Avail/Total shared memory: %d/%d bytes\n", avail, total);
                    break;
                case 'h':
                    printf("Heap available: %d bytes\n", xPortGetFreeHeapSize());
                    break;
                case 'r':
                    printf("Restart tasks.\n");
                    amp_close();
                    stattick = 0;
                    throughput[0] = throughput[1] = throughput[2] = throughput[3] = 0;
                    xTaskNotify(createTaskHandle, 10, eSetValueWithOverwrite);
                    break;
                case 's':
                    if(stattick)
                        printf("Statistics: %lu, %lu, %lu, %lu\n",
                            throughput[0]/stattick, throughput[1]/stattick, throughput[2]/stattick, throughput[3]/stattick);
                    break;
                case 'z':
                    stattick = 0;
                    throughput[0] = throughput[1] = throughput[2] = throughput[3] = 0;
                    printf("Reset statistics.\n");
                    break;
                default:
                    break;
            }
        }

        vTaskDelay(1000);
    }
}

int main(void)
{
    void *platform;
	struct rpmsg_device *rpdev;
	int ret;

    SYS_Init();

    printf("\n\nMA35D1 AMP CoreM demo\n");
    printf("Please refer to source code for detailed command info.\n\n");
    fflush(stdout);

	/* Initialize platform */
	ret = platform_init(0, NULL, &platform);
	if (ret) {
		printf("Failed to initialize platform.\r\n");
		ret = -1;
	} else {
		rpdev = platform_create_rpmsg_vdev(platform, 0,
						  VIRTIO_DEV_DRIVER, // only driver, we support both tx/rx
						  NULL,
						  NULL);
        if (!rpdev) {
			printf("Failed to create rpmsg virtio device.\r\n");
			ret = -1;
		} else {
			ret = 0;
		}
	}

    if(!ret)
    {
        xTaskCreate( vEndpointCreateTask, "CreateEpt", configMINIMAL_STACK_SIZE, rpdev, tskIDLE_PRIORITY + 1, &createTaskHandle );
        xTaskCreate( vCmdTask, "CmdTask", configMINIMAL_STACK_SIZE, eptinst, tskIDLE_PRIORITY + 1, NULL );
        vTaskStartScheduler();
        /* Should never be reached */
    }

    platform_release_rpmsg_vdev(rpdev, platform);
    platform_cleanup(platform);

    return 0;
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created.  It is also called by various parts of the
    demo application.  If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */
    taskDISABLE_INTERRUPTS();
    for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
    to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
    task.  It is essential that code added to this hook function never attempts
    to block in any way (for example, call xQueueReceive() with a block time
    specified, or call vTaskDelay()).  If the application makes use of the
    vTaskDelete() API function (as this demo application does) then it is also
    important that vApplicationIdleHook() is permitted to return to its calling
    function, because it is the responsibility of the idle task to clean up
    memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    taskDISABLE_INTERRUPTS();
    for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
    /* This function will be called by each tick interrupt if
    configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
    added here, but the tick hook is called from an interrupt context, so
    code must not attempt to block, and only the interrupt safe FreeRTOS API
    functions can be used (those that end in FromISR()).  */
}
/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
    /* If the buffers to be provided to the Idle task are declared inside this
    function then they must be declared static - otherwise they will be allocated on
    the stack and so not exists after this function exits. */
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
application must provide an implementation of vApplicationGetTimerTaskMemory()
to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
    /* If the buffers to be provided to the Timer task are declared inside this
    function then they must be declared static - otherwise they will be allocated on
    the stack and so not exists after this function exits. */
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

    /* Pass out a pointer to the StaticTask_t structure in which the Timer
    task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
/*-----------------------------------------------------------*/

void vMainAssertCalled( const char *pcFileName, uint32_t ulLineNumber )
{
    printf( "ASSERT!  Line %u of file %s\r\n", ulLineNumber, pcFileName );
    taskENTER_CRITICAL();
    for( ;; );
}
/*-----------------------------------------------------------*/
