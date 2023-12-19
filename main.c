#include <msp430.h> 

unsigned int GPS_val;
unsigned int comma_count, date_i, time_i, tx_i, j;
unsigned char time[10];
unsigned char date[6];
unsigned int rmc_count;

/**
 * Author: Zachary Becker
 * Date: August 1st, 2021
 * Description: This program pulls GPS RMC NMEA0138 "sentences" from a PMB-648 chip over UART (4800 bps)
 *              This RMC "sentence" is then parsed for time and date. Seconds and minutes are then
 *              sent over I2C to a PCF8523 Real Time Clock to set the values.
 *
 *              Desired feature additions beyond the scope of 1st deadline:
 *                  * Save RMC messages to RAM or FRAM for looking at on http://freenmea.net/decoder
 */



int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	    // stop watchdog timer
	
	//------------------------------------ setup I2C for RTC
	UCB0CTLW0 |= UCSWRST;           // SW RESET ON

	UCB0CTLW0 |= UCSSEL_3;          // SMCLK
    UCB0BRW = 10;                   // prescalar to 10

    UCB0CTLW0 |= UCMODE_3;          // Put into I2C mode
    UCB0CTLW0 |= UCMST;             // set as MASTER
    UCB0CTLW0 |= UCTR;              // Put into Tx mode
    UCB0I2CSA = 0x0068;             // Slave address = 0x68

    UCB0CTLW1 |= UCASTP_2;          // enable automatic stop bit
    UCB0TBCNT = 3;                  // Transfer byte count

	// setup ports
    P1SEL1 &= ~BIT3;                // P1.3 SCL
    P1SEL0 |= BIT3;

    P1SEL1 &= ~BIT2;                // P1.2 SDA
    P1SEL0 |= BIT2;

    UCB0CTLW0 &= ~UCSWRST;          // SW RESET OFF
    UCB0IE |= UCTXIE0;              // enable I2C B0 Tx IRQ

	//------------------------------------ setup UART communication with TERMINAL
    // put UCA1 into SW reset mode
    UCA1CTLW0 |= UCSWRST;

    // 115200 baud rate
    UCA1CTLW0 |= UCSSEL__SMCLK;     // BRCLK = SMCLK
    UCA1BRW = 8;                    // prescaler = 8
    UCA1MCTLW = 0xD600;             // modulation

    // setup pins for UART A1 TX
    P4SEL1 &= ~BIT3;                // Tx
    P4SEL0 |= BIT3;

    UCA1CTLW0 &= ~UCSWRST;          // put UCA1 into operating mode

	//-- Setup LEDS
	P1DIR |= BIT0;                  // set P1.0 as output (LED1) for Rx indicator
	P1OUT &= BIT0;                  // default LOW

	P6DIR |= BIT6;                  // set P6.6 as output LED2 for Tx Indicator
	P6OUT &= ~BIT6;                 // default LOW

    //------------------------------------ setup UART communication with GPS
    // put UCA0 into SW reset mode
    UCA0CTLW0 |= UCSWRST;

    // 4800 baud rate
    UCA0CTLW0 |= UCSSEL__ACLK;      // BRCLK = ACLK
    UCA0BRW = 6;                    // prescaler = 6
    UCA0MCTLW = 0xEE00;             // modulation

    // setup pins for UART A0 RX
    P1SEL1 &= ~BIT6;                // Rx
    P1SEL0 |= BIT6;

    UCA0CTLW0 &= ~UCSWRST;          // UCA0 operating mode
    UCA0IE |= UCRXIE;               // enable A0 RXIFG

    // setup pins for UART A1 TX
    PM5CTL0 &= ~LOCKLPM5;           // enable I/O

    __enable_interrupt();           // global interrupt enable

//    while(1) {}
    int i;
    while(1){
        UCB0CTLW0 |= UCTXSTT;       // generate START condition
        for(i=0;i<1500; i=i+1){}    // delay
    }

	return 0;
}

/**
 * Interrupt for 4800 UART Rx complete (GPS feed)
 * Parsing the date and time out of the NMEA0138 RMC "sentence"
 */
#pragma vector = EUSCI_A0_VECTOR
__interrupt void ISR_EUSCI_A0(void){
    P1DIR ^= BIT0;
    GPS_val = UCA0RXBUF;

    //---- Useful ASCII <--> Hex lookups
    // , - 0x2C
    // $ - 0x24
    // \r (CR) - 0x0D
    // \n (LF) - 0x0A

    if(GPS_val == 0x24){                // Start of NMEA0183 string ("$")
        rmc_count = 0;
    } else if (GPS_val == 0x2C){        // if character is a comma
        comma_count++;
    } else if (GPS_val == 0x52 | GPS_val == 0x4D | GPS_val == 0x43){        // if "R" or "M" or "C"
        rmc_count++;
        if(rmc_count == 2){             // RMC message confirmed
            P6OUT ^= BIT6;
            time_i = 0;
            date_i = 0;
            comma_count = 0;
        }
    } else {
        if(comma_count == 1){
            time[time_i] = GPS_val;
            time_i++;
        } else if(comma_count == 9){
            date[date_i] = GPS_val;
            date_i++;
        } else if(comma_count > 9 && GPS_val == 0x0A){
            // NMEA0138 RMC "sentence" is complete

            //--- (begin characters for UART to terminal)
            UCA1IE |= UCTXCPTIE;        // enable TX complete IRQ
            UCA1IFG &= ~UCTXCPTIFG;     // clear TX complete flag

            tx_i=0;
            UCA1TXBUF = time[tx_i];     // send first character
        }
    }
}

/**
 * Interrupt for A1 (UART) Tx complete
 * Used for sending to UART (to terminal)
 */
#pragma vector = EUSCI_A1_VECTOR
__interrupt void ISR_EUSCI_A1(void){
    if(tx_i==sizeof(time)){
        UCA1TXBUF = 0x0D;               // \r
    } else if (tx_i == sizeof(time)+1){
        UCA1TXBUF = 0x0A;               // \n
    } else if (tx_i == sizeof(time)+2){
        UCA1TXBUF = 0x0A;               // \n
        UCA1IE &= ~UCTXCPTIE;           // TURN OFF TX COMPLETE IRQ
    } else {
        tx_i++;
        UCA1TXBUF = time[tx_i];         // send next character to UART (for monitoring)
    }
    UCA1IFG &= ~UCTXCPTIFG;             // clear TX complete flag
}

/**
 * Interrupt for B0 (I2C) Tx complete
 * Used for sending to I2C
 */
#pragma vector=EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_TX_ISR(void){
    if(j == 0){
        UCB0TXBUF = 0x03;
        j++;
    } else if(j==1) {
        UCB0TXBUF = ((time[4]-0x30)*10)+((time[5]-0x30)*1);     // seconds (sub 30h, multiply by 10^n for radix position, do same process for LSB)
        j++;
    } else if (j==2){
        UCB0TXBUF = ((time[2]-0x30)*10)+((time[3]-0x30)*1);     // minutes (sub 30h, multiply by 10^n for radix position, do same process for LSB)
        j=0;
    }
}
