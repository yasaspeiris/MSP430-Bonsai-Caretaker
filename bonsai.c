#include "msp430g2553.h"
#include <stdio.h>
#include <string.h>

#define TXLED BIT0
#define RXLED BIT6
#define TXD BIT1
#define RXD BIT2

char cwm_command[20] = {"AT+CWMODE=1\r\n"};
char cwlp_command[50] = {"AT+CWLAP\r\n"};
char login_command[50] = { "AT+CWJAP=\"dhare\",\"12345678abc\"\r\n" };//SSID,pw
char cipstart[100] = {"AT+CIPSTART=4,\"TCP\",\"api.thingspeak.com\",80\r\n"};
char cipclose[20] = {"AT+CIPCLOSE\r\n"};
char cipmux[20] = {"AT+CIPMUX=1\r\n"};
//char cipserver[30] = {"AT+CIPSERVER=1,80\r\n"};


char recv_string[] = {};
unsigned int i; //Counter


int adc[2] = {0,0};

//No touchy touchy
#define Kp 50 //proportional gain
#define Kd 100 //differential gain

#define BaseSpeed 80



#define Setpoint1 300//Setpoint for water level. Increase this to increase water level.
int error_tray = 0;
int lasterror_tray = 0;

#define Setpoint2 900//Setpoint for soil moisture. Decrease this to increase moisture.
int error_soil = 0;
int lasterror_soil = 0;


void Flow_control();
void Motor_run();
void Solenoid_control(int solenoid);
void GPIO_init();
void ADC_init();
void ADC_read();
void UART_init();
void UART_send(char buffer[],int length);
void UART_receive();
void ESP8266_connect();
void Post();

int main(void)
{
   WDTCTL = WDTPW + WDTHOLD; // Stop WDT

   //Init functions
   UART_init();
   ADC_init();
   GPIO_init();

   //Configure ESP8266 as AP and connect to WiFi
   ESP8266_connect();
   while (1)
   {
       Flow_control();
       Post();


   }
}


void UART_init(){
    DCOCTL = 0; // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_1MHZ; // Set DCO
    DCOCTL = CALDCO_1MHZ;
    P2DIR |= 0xFF; // All P2.x outputs
    P2OUT &= 0x00; // All P2.x reset
    P1SEL |= RXD + TXD ; // P1.1 = RXD, P1.2=TXD
    P1SEL2 |= RXD + TXD ; // P1.1 = RXD, P1.2=TXD
    P1DIR |= RXLED + TXLED;
    P1OUT &= 0x00;
    UCA0CTL1 |= UCSSEL_2; // SMCLK
    UCA0BR0 = 104; // 1MHz 115200
    UCA0BR1 = 0x00; // 1MHz 115200
    UCA0MCTL = UCBRS2 + UCBRS0; // Modulation UCBRSx = 5
    UCA0CTL1 &= ~UCSWRST; // **Initialize USCI state machine**
}


void UART_send(char buffer[],int length){
    int i;
    for (i = 0; i < length; i++)
    {
        UCA0TXBUF = buffer[i];
        while (!(UC0IFG & UCA0TXIFG))
            ;

    }
}

void ESP8266_connect()
{
    __delay_cycles(5000000);
    UART_send(cwm_command, strlen(cwm_command));
    __delay_cycles(5000000);
    UART_send(cwlp_command, strlen(cwlp_command));
    __delay_cycles(5000000);
    UART_send(login_command, strlen(login_command));
    __delay_cycles(5000000);
    UART_send(cipmux, strlen(cipmux));
   __delay_cycles(5000000);
    //UART_send(cipserver, strlen(cipserver));
   // __delay_cycles(5000000);
}

void ADC_init(){
    ADC10CTL0 = ADC10SHT_3 + SREF_0  + REFON + ADC10ON;
    ADC10AE0 = BIT0+ BIT3;
}


void ADC_read(){

        ADC10CTL1 = INCH_0 + ADC10DIV_3;

        ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
        while (ADC10CTL1 & ADC10BUSY)
            ;          // check for ADC conversion is completed

        adc[0] = ADC10MEM;                 // Read ADC value
        ADC10CTL0 &= ~(ENC + ADC10SC);          // Sampling and conversion start

        ADC10CTL1 = INCH_3 + ADC10DIV_3;
        ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
        while (ADC10CTL1 & ADC10BUSY)
            ;          // check for ADC conversion is completed

        adc[1] = ADC10MEM;                 // Read ADC value
        ADC10CTL0 &= ~(ENC + ADC10SC);

}


void GPIO_init(){
    P2DIR |= BIT1+BIT4+ BIT5; //MotorPWM, Motor IN1 and IN2
    P2SEL |= BIT1;

    /*** Timer1_A Set-Up ***/
    TA1CCR0 |= 200 - 1;
    TA1CCTL1 |= OUTMOD_7;
    TA1CCR1 = 0;
    TA1CTL |= TASSEL_2 + MC_1;

    P2DIR |=BIT0+BIT2; //Solenoid 0 and Solenoid 1
    P2OUT &= ~(BIT0+BIT2);

}


void Motor_run(int speed){
    if(speed >81){
        P2OUT |= BIT4;
        P2OUT &= ~BIT5;
        if(speed>199){
            speed = 199;
        }
        TA1CCR1 = speed;
    }else{
        P2OUT &= ~BIT4;
        P2OUT &= ~BIT5;
        TA1CCR1 = 0;
    }

}

void Flow_control(){

    while (1)
    {

        ADC_read();
        error_tray = Setpoint1 - adc[1];

        if (error_tray > 0)
        {
            Solenoid_control(0);
            int delta_tray = error_tray - lasterror_tray;
            int change_tray = Kp * error_tray + Kd * delta_tray;
            lasterror_tray = error_tray;

            int PumpPWM = BaseSpeed + change_tray;
            //constrain PWM
            if (PumpPWM > 140)
            {
                PumpPWM = 140;
            }
            else if (PumpPWM < 0)
            {
                PumpPWM = 0;
            }

            Motor_run(PumpPWM);
        }
        else
        {
            Motor_run(0);
            break;
        }
    }

    while(1){


        ADC_read();
        error_soil = (adc[0] - Setpoint2);;

        if (error_soil > 0)
        {
            Solenoid_control(1);
            int delta_soil = error_soil - lasterror_soil;
            int change_soil = Kp * error_soil + Kd * delta_soil;
            lasterror_soil = error_soil;

            int PumpPWM = BaseSpeed + change_soil;
            //constrain PWM
            if (PumpPWM > 140)
            {
                PumpPWM = 140;
            }
            else if (PumpPWM < 0)
            {
                PumpPWM = 0;
            }

            Motor_run(PumpPWM);

        }
        else
        {
            Motor_run(0);
            break;
        }

    }

}

void Solenoid_control(int solenoid){
    if(solenoid ==0){
        P2OUT |= BIT0;
        P2OUT &= ~BIT2; }
    else if(solenoid ==1){
        P2OUT |= BIT2;
        P2OUT &= ~BIT0;

    }
}

void Post(){

    char cipsend[20] = {"AT+CIPSEND=4,48\r\n"};
   // char getrequest1[100] = {"GET /update?api_key=10JBLAYEA7XQ7X8Z&field1=03\r\n"};
    //char getrequest2[100] = {"GET /update?api_key=10JBLAYEA7XQ7X8Z&field2=03\r\n"};
    //UART_send(cipstart, strlen(cipstart));

    UART_send(cipsend, strlen(cipsend));
        __delay_cycles(5000000);

        UART_send("GET /update?api_key='your api key'\r\n",50);
                        __delay_cycles(10000000);
   /* UART_send(getrequest1, strlen(getrequest1));
                __delay_cycles(10000000);

 UART_send(cipsend, strlen(cipsend));
                        __delay_cycles(5000000);

                UART_send(getrequest2, strlen(getrequest2));
                                __delay_cycles(10000000);
*/
    UART_send(cipclose,strlen(cipclose));
    __delay_cycles(5000000);


}
