#include "msp.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "lib_PAE.h"

typedef uint8_t byte;
#define TXD0_READY (UCA2IFG & UCTXIFG)

volatile bool Byte_Recibido;
volatile byte DatoLeido_UART;

void Sentit_Dades_Rx(void)
{
    P3OUT &= ~BIT0;   // El pin P3.0 (Direction_PORT) a 0 (Rx)
}

void Sentit_Dades_Tx(void)
{
    P3OUT |= BIT0;      // El pin P3.0 (DIRECTION_PORT) a 1 (Tx)
}

void TxUAC0(uint8_t bTxdData)
{
    while (!TXD0_READY)
        ;         // Wait a que este preparado el buffer
    UCA2TXBUF = bTxdData;
}

byte TxPacket(byte bID, byte bParameterLength, byte bInstruction,
              byte Parametros[16])
{
    byte bCount, bCheckSum, bPacketLength;
    byte TxBuffer[32];
    Sentit_Dades_Tx();
    TxBuffer[0] = 0xff;
    TxBuffer[1] = 0xff;
    TxBuffer[2] = bID;
    TxBuffer[3] = bParameterLength + 2;
    TxBuffer[4] = bInstruction;
    char error[] = "adr. no permitida";
    if ((Parametros[0] < 6) && (bInstruction == 3)){
        halLcdPrintLine(error, 8, INVERT_TEXT);
        return 0;

    }

    for (bCount = 0; bCount < bParameterLength; bCount++)
    {
        TxBuffer[bCount + 5] = Parametros[bCount];
    }
    bCheckSum = 0;
    bPacketLength = bParameterLength + 4 + 2;
    for (bCount = 2; bCount < bPacketLength - 1; bCount++)
    {
        bCheckSum += TxBuffer[bCount];
    }
    TxBuffer[bCount] = ~bCheckSum;
    for (bCount = 0; bCount < bPacketLength; bCount++)
    {
        TxUAC0(TxBuffer[bCount]);
    }
    while ((UCA2STATW & UCBUSY))
        ;
    Sentit_Dades_Rx();
    return (bPacketLength);
}



void init_UART(void)
{
    UCA0CTLW0 |= UCSWRST;                   // Desactiva la USCI
    UCA0CTLW0 |= UCSSEL__SMCLK;            // SMCLK (24MHz) como fuente de reloj
    UCA0MCTLW = UCOS16;                     // Sobre mostreo
    UCA0BRW = 13;                           // prescaler para fijar el Baud Rate a 115200 bps
                                           // 24Mhz / 115200 = 208.33, 208.33 > 16, 208.33/16 = 13
    UCA0MCTLW |= (0x25 << 8);               // TODO: Tratar de enteneder
    // Configurar Pines
    P1SEL0 |= BIT2 | BIT3;        // I/0 Funcion: P1.3 = UART0TX, P1.2 = UART0RX
    P1SEL1 &= ~(BIT2 | BIT3 );
    UCA0CTLW0 &= ~UCSWRST;             // Reactivamos la linea de comunicaciones
    EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;    // Clear eUSCI RX interrrup flag
    //EUSCI_A0->IE |= EUSCI_A_IE_RXIE;

}


void init_UART_Robot(void)
{
    UCA2CTLW0 |= UCSWRST;                   // Desactiva la USCI
    UCA2CTLW0 |= UCSSEL__SMCLK;            // SMCLK (24MHz) como fuente de reloj
    UCA2MCTLW = UCOS16;                     // Sobre mostreo
    UCA2BRW = 3;                            // prescaler para fijar el Baud Rate a 500000 bps
                                            // 24Mhz / 115200 = 208.33, 208.33 > 16, 208.33/16 = 13
    // Configurar Pines
    P3SEL0 |= BIT2 | BIT3;        // I/0 Funcion: P1.3 = UART0TX, P1.2 = UART0RX
    P3SEL1 &= ~(BIT2 | BIT3 );
    P3SEL0 &= ~BIT0;
    P3SEL1 &= ~BIT0;
    P3DIR |= BIT0;
    P3OUT &= ~BIT0;

    UCA2CTLW0 &= ~UCSWRST;             // Reactivamos la linea de comunicaciones
    UCA2IE |= UCRXIE;

}

void turn_on_led(uint8_t dir){
    byte parameters[2];
    parameters[0] = 0x19;
    parameters[1] = 0x01;
    if(dir == 0){
        TxPacket(0x02, 0x02, 0x03, parameters);
    }else{
        TxPacket(0x03, 0x02, 0x03, parameters);
    }

}

void init_interrupts() {
        NVIC->ICPR[0] |= 1 << EUSCIA0_IRQn; //Primer, ens assegurem de que no quedi cap interrupciÛ residual pendent per aquest port,
        NVIC->ISER[0] |= 1 << EUSCIA0_IRQn; //i habilitem les interrupcions del port

        __enable_interrupts();  //Habilitem el tercer nivell de les interrupcions
    }

/**
 * main.c
 */
void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    init_ucs_24MHz();

    init_UART_Robot();

    init_interrupts();

    uint16_t i = 0;


    turn_on_led(0);
    for(i = 10000; i>0; i--);
    turn_on_led(1);

    while (1)
    {

    }

}


void EUSCIA0_IRQHandler()
{ //interrupciÛ de recepciÛ de la UART A0
    UCA2IE &= ~UCRXIE; //Interrupcions desactivades en RX
    DatoLeido_UART = UCA2RXBUF;
    Byte_Recibido = true;
    UCA2IE |= UCRXIE; //Interrupcions reactivades en RX
}

