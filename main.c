/*
Description: this is the main file for the Hilandero project. A project waste recycling project that aims to turn plastic wastes into 3D printing filament.

Procesador: MSP430G2553
Fecha: 7-11-2020
Autor: Alvaro Guzman Juan
*/

#include <msp430.h> 

/**********************************************************DEFINES*******************************************************************/

#define MAX6675_CS      BIT0        // PORT2.0
#define MAX6675_CLK     BIT1        // PORT2.1
#define MAX6675_MISO    BIT2        // PORT2.2
#define RELE            BIT3        // PORT2.3

#define LCD_CS          BIT5        // PORT1.5
#define LCD_CLK         BIT4        // PORT1.4
#define LCD_MOSI        BIT2        // PORT1.2
#define LCD_D_C         BIT1        // PORT1.1
#define LCD_RES         BIT0        // PORT1.0
#define LED_VERDE       BIT6        // PORT1.6
#define POT_TEMP        BIT7        // PORT1.7
#define POS_TEMP_X      2
#define POS_TEMP_Y      2
#define POS_TEMP_OBJ_X  2
#define POS_TEMP_OBJ_Y  3
#define POS_TITLE_X     20
#define POS_TITLE_Y     0
#define proportional_coeff  2
#define integral_coeff      2
#define derivative_coeff    2


/**************************************************************************************************************************************/


/**********************************************************FUNCTIONS*******************************************************************/

void ReadTempActual( unsigned int* p_temp );
void ShowTempActual( unsigned int* p_temp );
void ReadTempObjetivo( unsigned int* p_temp );
void ShowTempObjetivo( unsigned int* p_temp );
void comando(unsigned char);
void dato(unsigned char);
void lcd_init(void);
void muestra_titulo();
void letra(char);
void vete_a_xy(int,char);
void abre_rele();
void cierra_rele();
void main_loop();
void process_PID();
int proportional_error();
int integral_error();
int derivative_error();



/**************************************************************************************************************************************/
unsigned int a,b;
unsigned char c;

unsigned int  t_temp = 0;
unsigned int  o_temp = 0;
static int current_count = 0;
static int t_duty_cycle = 0;
static int t_prev_error = 0;

static const char FontLookup [][5] =
{

   { 0x00, 0x00, 0x2f, 0x00, 0x00 },  // !
   { 0x00, 0x07, 0x00, 0x07, 0x00 },  // "
   { 0x14, 0x7f, 0x14, 0x7f, 0x14 },  // #
   { 0x24, 0x2a, 0x7f, 0x2a, 0x12 },  // $
   { 0xc4, 0xc8, 0x10, 0x26, 0x46 },  // %
   { 0x36, 0x49, 0x55, 0x22, 0x50 },  // &
   { 0x00, 0x05, 0x03, 0x00, 0x00 },  // '
   { 0x00, 0x1c, 0x22, 0x41, 0x00 },  // (
   { 0x00, 0x41, 0x22, 0x1c, 0x00 },  // )
   { 0x14, 0x08, 0x3E, 0x08, 0x14 },  // *
   { 0x08, 0x08, 0x3E, 0x08, 0x08 },  // +
   { 0x00, 0x00, 0x50, 0x30, 0x00 },  // ,
   { 0x10, 0x10, 0x10, 0x10, 0x10 },  // -
   { 0x00, 0x60, 0x60, 0x00, 0x00 },  // .
   { 0x20, 0x10, 0x08, 0x04, 0x02 },  // /
   { 0x3E, 0x51, 0x49, 0x45, 0x3E },  // 0
   { 0x00, 0x42, 0x7F, 0x40, 0x00 },  // 1
   { 0x42, 0x61, 0x51, 0x49, 0x46 },  // 2
   { 0x21, 0x41, 0x45, 0x4B, 0x31 },  // 3
   { 0x18, 0x14, 0x12, 0x7F, 0x10 },  // 4
   { 0x27, 0x45, 0x45, 0x45, 0x39 },  // 5
   { 0x3C, 0x4A, 0x49, 0x49, 0x30 },  // 6
   { 0x01, 0x71, 0x09, 0x05, 0x03 },  // 7
   { 0x36, 0x49, 0x49, 0x49, 0x36 },  // 8
   { 0x06, 0x49, 0x49, 0x29, 0x1E },  // 9
   { 0x00, 0x36, 0x36, 0x00, 0x00 },  // :
   { 0x00, 0x56, 0x36, 0x00, 0x00 },  // ;
   { 0x08, 0x14, 0x22, 0x41, 0x00 },  // <
   { 0x14, 0x14, 0x14, 0x14, 0x14 },  // =
   { 0x00, 0x41, 0x22, 0x14, 0x08 },  // >
   { 0x02, 0x01, 0x51, 0x09, 0x06 },  // ?
   { 0x32, 0x49, 0x59, 0x51, 0x3E },  // @
   { 0x00, 0x00, 0x00, 0x00, 0x00 },  // sp
   { 0x7E, 0x11, 0x11, 0x11, 0x7E },  // A
   { 0x7F, 0x49, 0x49, 0x49, 0x36 },  // B
   { 0x3E, 0x41, 0x41, 0x41, 0x22 },  // C
   { 0x7F, 0x41, 0x41, 0x22, 0x1C },  // D
   { 0x7F, 0x49, 0x49, 0x49, 0x41 },  // E
   { 0x7F, 0x09, 0x09, 0x09, 0x01 },  // F
   { 0x3E, 0x41, 0x49, 0x49, 0x7A },  // G
   { 0x7F, 0x08, 0x08, 0x08, 0x7F },  // H
   { 0x00, 0x41, 0x7F, 0x41, 0x00 },  // I
   { 0x20, 0x40, 0x41, 0x3F, 0x01 },  // J
   { 0x7F, 0x08, 0x14, 0x22, 0x41 },  // K
   { 0x7F, 0x40, 0x40, 0x40, 0x40 },  // L
   { 0x7F, 0x02, 0x0C, 0x02, 0x7F },  // M
   { 0x7F, 0x04, 0x08, 0x10, 0x7F },  // N
   { 0x3E, 0x41, 0x41, 0x41, 0x3E },  // O
   { 0x7F, 0x09, 0x09, 0x09, 0x06 },  // P
   { 0x3E, 0x41, 0x51, 0x21, 0x5E },  // Q
   { 0x7F, 0x09, 0x19, 0x29, 0x46 },  // R
   { 0x46, 0x49, 0x49, 0x49, 0x31 },  // S
   { 0x01, 0x01, 0x7F, 0x01, 0x01 },  // T
   { 0x3F, 0x40, 0x40, 0x40, 0x3F },  // U
   { 0x1F, 0x20, 0x40, 0x20, 0x1F },  // V
   { 0x3F, 0x40, 0x38, 0x40, 0x3F },  // W
   { 0x63, 0x14, 0x08, 0x14, 0x63 },  // X
   { 0x07, 0x08, 0x70, 0x08, 0x07 },  // Y
   { 0x61, 0x51, 0x49, 0x45, 0x43 },  // Z
   { 0x00, 0x7F, 0x41, 0x41, 0x00 },  // [
   { 0x55, 0x2A, 0x55, 0x2A, 0x55 },  // 55
   { 0x00, 0x41, 0x41, 0x7F, 0x00 },  // ]
   { 0x04, 0x02, 0x01, 0x02, 0x04 },  // ^
   { 0x40, 0x40, 0x40, 0x40, 0x40 },  // _
   { 0x00, 0x01, 0x02, 0x04, 0x00 },  // '
   { 0x20, 0x54, 0x54, 0x54, 0x78 },  // a
   { 0x7F, 0x48, 0x44, 0x44, 0x38 },  // b
   { 0x38, 0x44, 0x44, 0x44, 0x20 },  // c
   { 0x38, 0x44, 0x44, 0x48, 0x7F },  // d
   { 0x38, 0x54, 0x54, 0x54, 0x18 },  // e
   { 0x08, 0x7E, 0x09, 0x01, 0x02 },  // f
   { 0x0C, 0x52, 0x52, 0x52, 0x3E },  // g
   { 0x7F, 0x08, 0x04, 0x04, 0x78 },  // h
   { 0x00, 0x44, 0x7D, 0x40, 0x00 },  // i
   { 0x20, 0x40, 0x44, 0x3D, 0x00 },  // j
   { 0x7F, 0x10, 0x28, 0x44, 0x00 },  // k
   { 0x00, 0x41, 0x7F, 0x40, 0x00 },  // l
   { 0x7C, 0x04, 0x18, 0x04, 0x78 },  // m
   { 0x7C, 0x08, 0x04, 0x04, 0x78 },  // n
   { 0x38, 0x44, 0x44, 0x44, 0x38 },  // o
   { 0x7C, 0x14, 0x14, 0x14, 0x08 },  // p
   { 0x08, 0x14, 0x14, 0x18, 0x7C },  // q
   { 0x7C, 0x08, 0x04, 0x04, 0x08 },  // r
   { 0x48, 0x54, 0x54, 0x54, 0x20 },  // s
   { 0x04, 0x3F, 0x44, 0x40, 0x20 },  // t
   { 0x3C, 0x40, 0x40, 0x20, 0x7C },  // u
   { 0x1C, 0x20, 0x40, 0x20, 0x1C },  // v
   { 0x3C, 0x40, 0x30, 0x40, 0x3C },  // w
   { 0x44, 0x28, 0x10, 0x28, 0x44 },  // x
   { 0x0C, 0x50, 0x50, 0x50, 0x3C },  // y
   { 0x44, 0x64, 0x54, 0x4C, 0x44 }   // z
};


/**
 * main.c
 */
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;                 // stop watchdog timer
    BCSCTL1 = CALBC1_8MHZ;                    // Set DCO
    DCOCTL = CALDCO_8MHZ;

    // Set port1 configuration
    P1DIR |= LCD_D_C + LCD_CS + LCD_RES + LED_VERDE;   // P1.5 SCE Y P1.6 LEDv
    P1SEL = LCD_MOSI + LCD_CLK;                        // P1.2 MOSI y P1.4 SCK
    P1SEL2 = LCD_MOSI + LCD_CLK;                       // P1.2 MOSI y P1.4 SCK

    // Set port2 configuration
    P2DIR |= MAX6675_CS + MAX6675_CLK + RELE;                        // P2.0 is Chip Select, P2.1 is SCK and P2.2 is MISO
    P2OUT = 0;

    // Set ADC
    ADC10CTL0 = ADC10SHT_3 + ADC10ON;           // ADC10ON
    ADC10CTL1 = + ADC10DIV_7 + INCH_7;          // input A7 CLOCK DIVIDER7
    ADC10AE0 |= 0x80;                           // PA.7 ADC option select

    // Set Timer A
    CCTL0 = CCIE;                                // CCR0 interrupt enabled
    CCR0 = 9999;                                  // It will count to 9999 + 1. This will give us a 10ms main cycle ( 10000 / 1MHz counts )
    TACTL = TASSEL_2 + MC_1 + ID_3;              // TSMCLK, contmode, TACLK divided by 8.

    // Set SPI
    UCA0CTL0 |=  UCMSB + UCMST + UCSYNC + UCCKPH;  // 3-pin, 8-bit SPI master ¡¡¡¡IMPORTANTE TENER EN CUENTA ESTE BIT: UCCKPH!!!!!!!!
    UCA0CTL1 |= UCSSEL_2;                     // SMCLK
    UCA0BR0 |= 0x00;                          // /2
    UCA0BR1 = 0;                              //
    UCA0MCTL = 0;                             // No modulation
    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**

    _BIS_SR(GIE);                             // TURN ON INTERRUPTS

    P1OUT |= LCD_D_C;                            // D/C = 1 Y ENCIENDO EL LCD
    P1OUT &= ~LCD_CS;                           // habilito el chip
    _delay_cycles(10000);                     //ESPERO 10ms

    P2OUT &= ~LCD_RES;
    _delay_cycles(25000);                     // reset del display, 25ms
    P2OUT |= LCD_RES;

    lcd_init();                               // inicio el lcd
    muestra_titulo();


    return 0;
}

void main_loop()
{

    ReadTempActual( &t_temp );
    ShowTempActual( &t_temp );
    ReadTempObjetivo( &o_temp );
    ShowTempObjetivo( &o_temp );

    t_duty_cycle = proportional_coeff*proportional_error() + integral_coeff*integral_error() + derivative_coeff*derivative_error();

    if( t_duty_cycle > 1000 )
    {
        t_duty_cycle = 1000;
    }
    else if ( t_duty_cycle < 0 )
    {
        t_duty_cycle = 0;
    }

    /* Temp control loop */
    if( ++current_count > t_duty_cycle )            // If we are beyond duty cycle for this period, we open the relay
    {
        abre_rele();
        P1OUT &= ~LED_VERDE;                     // Toggle P1.0
    }
    else
    {
        cierra_rele();
        P1OUT |= LED_VERDE;                     // Toggle P1.0
    }

    if( current_count >= 1000 )
    {
        current_count = 0;
    }
    /* End temp control loop */
}

int proportional_error()
{
    int t_error = 0;

    t_error = o_temp - t_temp;

    return ( t_error );
}

int integral_error()
{
    int t_integral_error = 0;

    t_integral_error = t_prev_error + ( o_temp - t_temp );
    t_prev_error = ( o_temp - t_temp );

    return ( t_integral_error );
}

int derivative_error()
{
  return ( 0 );
}

void abre_rele()
{
    P2OUT |= RELE;
}

void cierra_rele()
{
    P2OUT &= ~RELE;
}

void ReadTempActual( unsigned int *p_temp )
{
    unsigned int temp = 0;
    int l_var0 = 0;

    P2OUT |= MAX6675_CS;                                        // MAX6675 is deselected
    _delay_cycles(8000);                                        // delay 1ms
    P2OUT &= ~( MAX6675_CS + MAX6675_CLK );                     // MAX6675 is selected

    // Reception
    for( l_var0 = 15; l_var0 >= 0; l_var0-- )
    {
        P2OUT |= MAX6675_CLK;
        _delay_cycles(80);
        //temp |= ( ( 15 - l_var0 ) << ( ( P1IN && BIT2 ) & 0x01 ) );
        if( MAX6675_MISO & P2IN  )
        {
            temp |= ( 1 << l_var0  );
        }
        _delay_cycles(80);
        P2OUT &= ~MAX6675_CLK;
        _delay_cycles(150);
    }

    // Unset ports
    P2OUT |= MAX6675_CS;                                  // MAX6675 is deselected
    temp >>= 3;

    /* According to MAX6675 datasheet a sample with all 12 bits being '1' would mean a temperature of 1023.75º, and all bits '0' means 0º
     * so, the conversion formula is: read_number* ( 1023.75 / (2^12), which is rougly 0.25, so to say, dividing by 4 */

    *p_temp = temp>>2;
}

void ShowTempActual( unsigned int *p_temp )
{
    unsigned char t_pos_x = POS_TEMP_X;
    unsigned char t_temp[4]={ 0, 0, 0, 0};
    unsigned char l_var0 = 0;

    t_temp[0]  = *p_temp / 1000;
    t_temp[1]  =(*p_temp % 1000) / 100;
    t_temp[2]  = (*p_temp % 100) / 10;
    t_temp[3] = *p_temp % 10;

    vete_a_xy( t_pos_x, POS_TEMP_Y );
    letra( 12 );
    t_pos_x += 5;
    vete_a_xy( t_pos_x, POS_TEMP_Y );
    letra( 'T' );
    t_pos_x += 5;
    vete_a_xy( t_pos_x, POS_TEMP_Y );
    letra( 13 );
    t_pos_x += 5;
    vete_a_xy( t_pos_x, POS_TEMP_Y );
    letra( 'A' );
    t_pos_x += 5;
    vete_a_xy( t_pos_x, POS_TEMP_Y );
    letra( 'C' );
    t_pos_x += 5;
    vete_a_xy( t_pos_x, POS_TEMP_Y );
    letra( 'T' );
    t_pos_x += 5;
    vete_a_xy( t_pos_x, POS_TEMP_Y );
    letra( 25 );
    t_pos_x += 5;

    for( l_var0 = 0; l_var0 < 4; l_var0++ )
    {
        if( t_temp[l_var0] >= 0 )
        {
            vete_a_xy( ( t_pos_x+(l_var0 * 5 ) ), POS_TEMP_Y );
            letra( 15 + t_temp[l_var0] );
        }
    }

}

void ReadTempObjetivo( unsigned int* p_temp )
{

    unsigned char l_var0 = 0;
    signed int t_temp= 0;

    for ( l_var0 = 0; l_var0 < 8; l_var0++ )
    {
        ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
        while( ADC10BUSY & ADC10CTL1 );
        t_temp += ADC10MEM;
        ADC10CTL0 &= ~ENC;                      // Sampling and conversion start
    }

    /* The voltage divider gives values between 150->850 ADC counts.
     * As we can be able to configure temps between [10, 300] celsius degree each degree represents ( 850-150 ) / ( 300 - 10 ) = 700/290 = 2.41 counts
     * Thus, conversion formulae is: Degree celsius = counts / 2.41
     * */
    *p_temp = t_temp>>6;

}

void ShowTempObjetivo( unsigned int* p_temp )
{
    unsigned char t_pos_x = POS_TEMP_OBJ_X;
    unsigned char t_temp[4]={ 0, 0, 0, 0 };
    unsigned char l_var0 = 0;

    t_temp[0]  = *p_temp / 1000;
    t_temp[1]  =(*p_temp % 1000) / 100;
    t_temp[2]  = (*p_temp % 100) / 10;
    t_temp[3] = *p_temp % 10;

    vete_a_xy( t_pos_x, POS_TEMP_OBJ_Y );
    letra( 12 );
    t_pos_x += 5;
    vete_a_xy( t_pos_x, POS_TEMP_OBJ_Y );
    letra( 'T' );
    t_pos_x += 5;
    vete_a_xy( t_pos_x, POS_TEMP_OBJ_Y );
    letra( 13 );
    t_pos_x += 5;
    vete_a_xy( t_pos_x, POS_TEMP_OBJ_Y );
    letra( 'O' );
    t_pos_x += 5;
    vete_a_xy( t_pos_x, POS_TEMP_OBJ_Y );
    letra( 'B' );
    t_pos_x += 5;
    vete_a_xy( t_pos_x, POS_TEMP_OBJ_Y );
    letra( 'J' );
    t_pos_x += 5;
    vete_a_xy( t_pos_x, POS_TEMP_OBJ_Y );
    letra( 25 );
    t_pos_x += 5;

    for( l_var0 = 0; l_var0 < 4; l_var0++ )
    {
        if( t_temp[l_var0] >= 0 )
        {
            vete_a_xy( ( t_pos_x+(l_var0 * 5 ) ), POS_TEMP_OBJ_Y );
            letra( 15 + t_temp[l_var0] );
        }
    }
}

void lcd_init(void){

        unsigned int a;

        comando(0x21);                     // CONJUNTO DE INSTRUCCIONES EXTENSO

        comando(0xC8);                     // CONFIGURO EL Vop

        comando(0x13);                     // ENVIO PD=0, V=0, H=0

        comando(0x20);                     // ENVIO D=1 Y E=0

        comando(0x09);                     // ENVIO D=1 Y E=0

        _delay_cycles(500000);             // ESPERO 25MS

        for(a = 0; a <= 503; a++){
            dato(0x00);                     // ENVIO datos
        }

        comando(0x08);                     // todas off
        _delay_cycles(10000);              // ESPERO 10MS
        comando(0x0C);                     // todas off

        vete_a_xy(0,0);

}

void comando(unsigned char comand){


        P1OUT &= ~LCD_D_C;                           // PONGO D/C A 0 (COMANDO)
        P1OUT &= ~LCD_CS;                           // CHIP SELECCIONADO


        _delay_cycles(20);

        UCA0TXBUF = comand;                   // ENVIO PD=0, V=0, H=1

        while ((UCA0STAT&UCBUSY));

        P1OUT |= LCD_CS;                           // CHIP DESELECCIONADO

        _delay_cycles(20);

}

void dato(unsigned char dat){

        P1OUT |= LCD_D_C;                            // PONGO D/C A 1 (dato)
        P1OUT &= ~LCD_CS;                           // CHIP SELECCIONADO


        _delay_cycles(20);

        UCA0TXBUF = dat;                   // ENVIO PD=0, V=0, H=1

        while ((UCA0STAT&UCBUSY));


        P1OUT |= LCD_CS;                           // CHIP DESELECCIONADO

        _delay_cycles(20);

}

void letra(char b)
{
    unsigned char a;
    for(a = 0; a <= 4; a++)
    {
        dato(FontLookup[b][a]);
    }
}

void vete_a_xy(int x,char y){
    comando(0x40|(y&0x07));
    comando(0x80|(x&0x7f));

}

void muestra_titulo()
{
    unsigned int t_pos_x = POS_TITLE_X;
    // Imprime "Hilandero"
    vete_a_xy( t_pos_x, POS_TITLE_Y );
    letra( 'H' );
    t_pos_x += 5;
    vete_a_xy( t_pos_x, POS_TITLE_Y );
    letra( 'I' );
    t_pos_x += 5;
    vete_a_xy( t_pos_x, POS_TITLE_Y );
    letra( 'L' );
    t_pos_x += 5;
    vete_a_xy( t_pos_x, POS_TITLE_Y );
    letra( 'A' );
    t_pos_x += 5;
    vete_a_xy( t_pos_x, POS_TITLE_Y );
    letra( 'N' );
    t_pos_x += 5;
    vete_a_xy( t_pos_x, POS_TITLE_Y );
    letra( 'D' );
    t_pos_x += 5;
    vete_a_xy( t_pos_x, POS_TITLE_Y );
    letra( 'E' );
    t_pos_x += 5;
    vete_a_xy( t_pos_x, POS_TITLE_Y );
    letra( 'R' );
    t_pos_x += 5;
    vete_a_xy( t_pos_x, POS_TITLE_Y );
    letra( 'O' );
}

// Timer A0 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer_A (void)
#else
#error Compiler not supported!
#endif
{
    main_loop();
}

