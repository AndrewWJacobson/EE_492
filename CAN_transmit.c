/* CAN Transmit for crane sensor project by Andrew Jacobson & Wyatt Syhlman *
 * will send a CAN message length 8 every 10ms */

/* DriverLib Includes */
#include "ti/devices/msp432e4/driverlib/driverlib.h"

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include "uartstdio.h"

/* CAN variables */
bool errFlag = false;

/* Scheduler variables */
uint8_t schedulerTimer;
uint8_t counter10ms;

/* Function prototypes */
void configureUART(void);
void configureCAN(void);
void configureSchedulerTimer(void);

/**
 * main.c
 */
int main(void)
{
       tCANMsgObject sCANMessage[4];
       uint8_t msgData[8] = {0x00, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

       /* system clock set */
       sysClock = MAP_SysCtlClockFreqSet((SYSCT sysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);L_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

       /* UART initialization */
       configureUART();

       /* Enable the clock to the GPIO Port J and wait for it to be ready */
       MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
       while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ)))
       {
       }

       /* Enable the GPIO port that is used for the on-board LED */
       MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

       /* Check if the peripheral access is enabled */
       while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPION))
       {
       }

       /* Enable the GPIO pin for the LED (PN0).  Set the direction as output, and
       * enable the GPIO pin for digital function */
       MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);

       /* Initialize the CAN */
       configureCAN();

       /* Initialize message objects to be able to send CAN message 1 */
       sCANMessage[0].ui32MsgID = 0x100;
       sCANMessage[0].ui32MsgIDMask = 0;                  /* No mask needed for TX  */
       sCANMessage[0].ui32Flags = MSG_OBJ_TX_INT_ENABLE;  /* Enable interrupt on TX */
       sCANMessage[0].ui32MsgLen = sizeof(msgData);       /* Size of message is 8   */
       sCANMessage[0].pui8MsgData = msgData;              /* Ptr to message content */

       /* Initialize the timer for the scheduler */
       configureSchedulerTimer();

       /* Scheduler */
       while(1)
       {
           /*  10 ms */
           if (schedulerTimer >= 1)
           {
               schedulerTimer = 0;
               counter10ms++;

               /* Send the CAN message using object number 1 */
               MAP_CANMessageSet(CAN0_BASE, 1, &sCANMessage[0], MSG_OBJ_TYPE_TX);

               /* Toggle LED  every 10ms */
               if(MAP_GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_0) == GPIO_PIN_0)
               {
                   MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, ~(GPIO_PIN_0));
               }
               else
               {
                   MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);
               }

           }

           /* Check the error flag to see if errors occurred */
           if(errFlag)
           {
               UARTprintf("error - cable connected?\n");
               while(errFlag);
           }
       }
   }
void configureUART(void)
{
    /* Configure the UART and its pins.
     * This must be called before UARTprintf() */

    /* Enable the GPIO Peripheral used by the UART */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)))
    {
    }

    /* Enable UART2 */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);

    /* Configure GPIO Pins for UART mode */
    MAP_GPIOPinConfigure(GPIO_PD4_U2RX);
    MAP_GPIOPinConfigure(GPIO_PD5_U2TX);
    MAP_GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    /* Initialize the UART for console I/O */
    UARTStdioConfig(2, 115200, sysClock);
}

void configureCAN(void)
{
    /* Configure the CAN and its pins PA0 and PA1 @ 500Kbps */

    /* Enable the clock to the GPIO Port A and wait for it to be ready */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)))
    {
    }

    /* Enable CAN0 */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);

    /* Configure GPIO Pins for CAN mode */
    MAP_GPIOPinConfigure(GPIO_PA0_CAN0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_CAN0TX);
    MAP_GPIOPinTypeCAN(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /* Initialize the CAN controller */
    MAP_CANInit(CAN0_BASE);

    /* Set up the bit rate for the CAN bus.  CAN bus is set to 500 Kbps */
    MAP_CANBitRateSet(CAN0_BASE, sysClock, 500000);

    /* Enable interrupts on the CAN peripheral */
    MAP_CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

    /* Enable auto-retry on CAN transmit */
    MAP_CANRetrySet(CAN0_BASE, true);

    /* Enable the CAN interrupt */
    MAP_IntEnable(INT_CAN0);

    /* Enable the CAN for operation */
    MAP_CANEnable(CAN0_BASE);
}

void configureSchedulerTimer(void)
{
    uint32_t timerPeriod;

    /* Enable Timer peripheral */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    /* Configure TIMER0 / TIMER_A as periodic timer */
    MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    /* 100Hz - 10ms */
    timerPeriod = (sysClock / 100);
    MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, timerPeriod - 1);

    /* Enable interrupt for the timer timeout */
    MAP_IntEnable(INT_TIMER0A);
    MAP_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    MAP_TimerEnable(TIMER0_BASE, TIMER_A);
}

void CAN0_IRQHandler(void)
{
    uint32_t canStatus;

    /* Read the CAN interrupt status to find the cause of the interrupt */
    canStatus = MAP_CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

    /* If the cause is a controller status interrupt, then get the status */
    if(canStatus == CAN_INT_INTID_STATUS)
    {
        /* Read the controller status.  This will return a field of status
         * error bits that can indicate various errors */
        canStatus = MAP_CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

        /* Set a flag to indicate some errors may have occurred */
        errFlag = true;
    }

    /* Check if the cause is message object 1, which what we are using for
     * sending messages */
    else if(canStatus == 1)
    {
        /* Getting to this point means that the TX interrupt occurred on
         * message object 1, and the message TX is complete.  Clear the
         * message object interrupt */
        MAP_CANIntClear(CAN0_BASE, 1);

        /* Since the message was sent, clear any error flags */
        errFlag = false;
    }

    else
    {
    }
}

void TIMER0A_IRQHandler(void)
{
    /* Clear the timer interrupt */
    MAP_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    /* Increment scheduler */
    schedulerTimer++;
}

