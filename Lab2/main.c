/******************************
 *
 * Practica_02_PAE Programacio de Ports
 * i practica de les instruccions de control de flux:
 * "do ... while", "switch ... case", "if" i "for"
 * UB, 04/2023.
 *****************************/

#include <msp432p401r.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define LED_V_BIT BIT0

#define SW1_POS 1
#define SW2_POS 4
#define SW1_INT 0x04
#define SW2_INT 0x0A
#define SW1_BIT BIT(SW1_POS)
#define SW2_BIT BIT(SW2_POS)

#define RETRASO 100000

volatile uint8_t estado = 0;
volatile uint16_t count = 0;

/**************************************************************************
 * INICIALIZACIúN DEL CONTROLADOR DE INTERRUPCIONES (NVIC).
 *
 * Sin datos de entrada
 *
 * Sin datos de salida
 *
 **************************************************************************/
void init_interrupciones()
{
    // Configuracion al estilo MSP430 "clasico":
    // --> Enable Port 4 interrupt on the NVIC.
    // Segun el Datasheet (Tabla "6-39. NVIC Interrupts", apartado "6.7.2 Device-Level User Interrupts"),
    // la interrupcion del puerto 1 es la User ISR numero 35.
    // Segun el Technical Reference Manual, apartado "2.4.3 NVIC Registers",
    // hay 2 registros de habilitacion ISER0 y ISER1, cada uno para 32 interrupciones (0..31, y 32..63, resp.),
    // accesibles mediante la estructura NVIC->ISER[x], con x = 0 o x = 1.
    // Asimismo, hay 2 registros para deshabilitarlas: ICERx, y dos registros para limpiarlas: ICPRx.

    //Int. port 1 = 35 corresponde al bit 3 del segundo registro ISER1:
    NVIC->ICPR[1] |= BIT3; //Primero, me aseguro de que no quede ninguna interrupcion residual pendiente para este puerto,
    NVIC->ISER[1] |= BIT3; //y habilito las interrupciones del puerto

}

/**************************************************************************
 * INICIALIZACIúN DE LOS BOTONES & LEDS DEL BOOSTERPACK MK II.
 *
 * Sin datos de entrada
 *
 * Sin datos de salida
 *
 **************************************************************************/
void init_botons(void)
{
    //Configuramos botones i LED vermell
    //***************************
    P1SEL0 &= ~(BIT0 + BIT1 + BIT4 );    //Els polsadors son GPIOs
    P1SEL1 &= ~(BIT0 + BIT1 + BIT4 );    //Els polsadors son GPIOs

    //LED vermell = P1.0
    P1DIR |= LED_V_BIT;      //El LED es una sortida
    P1OUT &= ~LED_V_BIT;     //El estat inicial del LED es apagat

    //Botú S1 = P1.1 i S2 = P1.4
    P1DIR &= ~(SW1_BIT + SW2_BIT);    //Un polsador es una entrada
    P1REN |= (SW1_BIT + SW2_BIT);     //Pull-up/pull-down pel pulsador
    P1OUT |= (SW1_BIT + SW2_BIT); //Donat que l'altra costat es GND, volem una pull-up
    P1IE |= (SW1_BIT + SW2_BIT);      //Interrupcions activades
    P1IES &= ~(SW1_BIT + SW2_BIT);    // amb transicio L->H
    P1IFG = 0;                  // Netegem les interrupcions anteriors
}

/**************************************************************************
 * DELAY - A CONFIGURAR POR EL ALUMNO - con bucle while
 *
 * Datos de entrada: Tiempo de retraso. 1 segundo equivale a un retraso de 1000000 (aprox)
 *
 * Sin datos de salida
 *
 **************************************************************************/
void delay_t(uint32_t temps)
{
    volatile uint32_t i;

    /**************************
     * TODO PER PART DEL ALUMNE AMB UN BUCLE FOR
     * Un cop implementat, comenteu o elimineu la segúent funciú
     **************************/

    //Soluciú
    for (i = temps; i > 0; i--)
        ;
}

/*****************************************************************************
 * CONFIGURACIúN DE LOS LEDs DEL PUERTO 2. A REALIZAR POR EL ALUMNO
 *
 * Sin datos de entrada
 *
 * Sin datos de salida
 *
 ****************************************************************************/
void config_RGB_LEDS(void)
{

    /**************************
     * TODO PER PART DEL ALUMNE
     **************************/

    P2DIR |= (BIT0 + BIT1 + BIT2 );         // RGB son salidas
    P2OUT &= ~(BIT0 + BIT1 + BIT2 );      // Estado inicial de los RGB's apagado

}

void main(void)
{
    uint8_t estado_anterior = 0;

    unsigned char filter;

    WDTCTL = WDTPW + WDTHOLD;       // Stop watchdog timer

    //Inicializaciones:
    init_botons();                  // Configuramos botones y leds

    init_interrupciones(); // Configurar y activar las interrupciones de los botones

    config_RGB_LEDS();              // Configuramos RGB Leds

    __enable_interrupts();

    //Bucle principal (infinito):
    while (true)
    {

        if (estado_anterior != estado) // Dependiendo del valor del estado se encenderú un LED u otro.
        {
            estado_anterior = estado;

            /**********************************************************+
             A RELLENAR POR EL ALUMNO BLOQUE  switch (estado) ... case
             Para gestionar las acciones:
             Boton S1 presionado un número impar de veces, estado = 1
             Boton S1 presionado un nmero par de veces, estado = 2
             Boton S2, estado = 3

             ***********************************************************/
            switch (estado)
            {
            case 1:
                P2OUT |= (BIT0 + BIT1 + BIT2 ); // Si el estado es 1, se encienden todos los LEDS RGB(blanco)
                break;
            case 2:
                P2OUT ^= 0x07; // Estado 2, se invierten los LEDS rgb, ej: 010 -> 101
                break;
            case 3:
                filter = P2OUT & 0x07; // Obtenemos los primeros 3 bits de P2OUT
                if (filter == 0x00)
                {                   // si son igual a 0x00 es que estan apagados
                    P2OUT ^= BIT0;           // entonces se enciende el LED Rojo
                }
                else
                {
                    filter <<= 1; // si estan encendidos, desplazo los 3 bits a la izquierda
                    P2OUT = (P2OUT & ~0x07) | (filter & 0x07); // limpia los primeros 3 bits de P2OUT y luego agrega los dezplazados
                }
                estado = 0;
                break;
            }

        }

        P1OUT ^= LED_V_BIT;     // Conmutamos el estado del LED R
        delay_t(RETRASO);       // periodo del parpadeo
    }

}

/**************************************************************************
 * RUTINAS DE GESTION DE LOS BOTONES:
 * Mediante estas rutinas, se detectarú quú botún se ha pulsado
 *
 * Sin Datos de entrada
 *
 * Sin datos de salida
 *
 * Actualizar el valor de la variable global estado
 *
 **************************************************************************/

//ISR para las interrupciones del puerto 1:
void PORT1_IRQHandler(void)
{
    static bool impar = false;
    uint8_t flag = P1IV; //guardamos el vector de interrupciones. De paso, al acceder a este vector, se limpia automaticamente.
    P1IE &= ~(SW1_BIT + SW2_BIT); //interrupciones del boton S1 y S2 en port 1 desactivadas

    switch (flag)
    {
    case SW1_INT:
        count += 1;
        impar = count % 2 == 0 ? false : true;
        if (impar)
        {
            estado = 1;
        }
        else
        {
            estado = 2;
        }
        break;
    case SW2_INT:
        estado = 3;
        break;
    default:
        break;
    }

    /**********************************************************+
     A RELLENAR POR EL ALUMNO
     Para gestionar los estados:
     Boton S1 presionado un número impar de veces, estado = 1
     Boton S1 presionado un número par de veces, estado = 2
     Boton S2, estado = 3
     ***********************************************************/

    P1IE |= (SW1_BIT + SW2_BIT);  //interrupciones S1 y S2 en port 1 reactivadas
}
