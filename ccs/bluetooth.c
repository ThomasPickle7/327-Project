#include <msp430.h>

#define TXLED BIT0
#define RXLED BIT1
#define TXD BIT4
#define RXD BIT5

const char string[] = { "AT+" };
unsigned int i; //Counter

void setup() {
    WDTCTL = WDTPW + WDTHOLD;   // Stop watchdog timer

    P1DIR |= TXLED | RXLED;     // Set P2 as output
    P1OUT &= ~(TXLED | RXLED);  // Initially turn off LED
    P1SEL0 |= RXD | TXD ;       // P1.5 = RXD, P1.4 = TXD

    UCA0CTLW0 |= UCSWRST;                      // **Put state machine in reset**
    UCA0CTLW0 |= UCSSEL__SMCLK;                // SMCLK
    UCA0BR0 = 52;                              // 9600 baud
    UCA0MCTLW |= UCOS16 | UCBRF_1 | 0x4900;
    UCA0BR1 = 0;
    UCA0CTLW0 &= ~UCSWRST;                     // **Initialize USCI state machine**
    UCA0IE |= UCRXIE;                          // Enables interrupts
}

void loop() {
    if (!(UCA0IFG & UCRXIFG)) {
        char data_received = UCA0RXBUF;

        if (data_received == 'a') {
            i = 0;
            UCA0TXBUF = string[i++]; // Start sending the string
        }
    }
}

int main(void) {
    setup();
    __enable_interrupt();
    while(1) {
        loop();
    }
}

#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void) {
    switch(__even_in_range(UCA0IV, USCI_UART_UCTXCPTIFG)) {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
            P2OUT |= RXLED;                 // Turn on LED for debugging
            char received_byte = UCA0RXBUF; // Received byte from buffer
            P2OUT &= ~RXLED;                // Turn off LED for debugging
            break;
        case USCI_UART_UCTXIFG:
            P2OUT |= TXLED;                 // Turn on LED for debugging
            UCA0TXBUF = string[i++];        // TX next character
            if (i == sizeof(string) - 1)    // Checks to see if transmission is over
                UCA0IE &= ~UCTXIE;          // Disable USCI_A0 TX interrupt
            P2OUT &= ~TXLED;                // Turn off TXLED
            break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
    }
}