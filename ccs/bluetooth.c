#include <msp430.h>

#define LED BIT0

void setup() {
    WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5; // Disable the GPIO power-on default high-impedance mode to activate configured port settings

    P1DIR |= LED; // Set P1.0 as output
    P1OUT &= ~LED; // Initially turn off LED

    UCA0CTLW0 |= UCSWRST;                      // **Put state machine in reset**
    UCA0CTLW0 |= UCSSEL__SMCLK;                 // SMCLK
    UCA0BR0 = 0;                                // 9600 baud
    UCA0MCTLW = UCBRS0;                         // Modulation UCBRSx = 1
    UCA0BR1 = 6;
    UCA0CTL1 &= ~UCSWRST;                       // **Initialize USCI state machine**
}

void loop() {
    if (UCA0IFG & UCRXIFG) { // If receive buffer has data
        char data_received = UCA0RXBUF;

        if (data_received == '1') {
            P1OUT |= LED; // Turn on LED
            UCA0TXBUF = 'R'; // Send confirmation
        }

        if (data_received == '2') {
            P1OUT &= ~LED; // Turn off LED
            UCA0TXBUF = 'T'; // Send confirmation
        }
    }
}

int main(void) {
    setup();

    while(1) {
        loop();
    }
}