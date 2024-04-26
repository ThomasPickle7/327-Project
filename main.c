// ELEC 327 Final Project
// Authors: Thomas Pickell and Noah Villa
// PINOUT:
// P1.6: SCL
// P1.7: SDA
// P1.0: LED

#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define DEV_ADDR  0x68

// I2C CONSTANTS
#define DEV_ADDR 0x68 // Worker Device Address
#define POWER_ON_CMD 0x6B //Power Config Register
#define GYRO_CONFIG_CMD 0x1B //Gyro Config Register
#define ACCEL_CONFIG_CMD 0x1C //Accel Config Register

/*DATA LENGTHS*/
#define TYPE_1_LENGTH   2
#define TYPE_2_LENGTH   6

#define MAX_BUFFER_SIZE     20

/*Bit Masks*/
uint8_t PowerOnSeq[TYPE_1_LENGTH] = {0x00};
uint8_t GyroConfig[TYPE_1_LENGTH] = {0xF0};
uint8_t AccelConfig[TYPE_1_LENGTH] ={0xF0};

// Bluetooth
#define TXLED BIT0
#define RXLED BIT1
#define TXD BIT4
#define RXD BIT5

const char string[] = { "AT+" };
unsigned int i; //Counter



//******************************************************************************
// General I2C State Machine ***************************************************
//******************************************************************************


/*STATES*/
typedef enum I2C_ModeEnum{
    IDLE_MODE,  //Idle state
    NACK_MODE,  //NACK received
    TX_REG_ADDRESS_MODE,    //Transmitting register address
    RX_REG_ADDRESS_MODE,    //Receiving register address
    TX_DATA_MODE,   //Transmitting data
    RX_DATA_MODE,   //Receiving data
    SWITCH_TO_RX_MODE,  //Switching to receive mode
    SWITHC_TO_TX_MODE,  //Switching to transmit mode
    TIMEOUT_MODE    //Timeout mode
} I2C_Mode;

/* Used to track the state of the software state machine*/
I2C_Mode controllerMode = IDLE_MODE;



/* The Register Address/Command to use*/
uint8_t TransmitRegAddr = 0;

uint8_t ReceiveBuffer[MAX_BUFFER_SIZE] = {0};   // Buffer used to receive data
uint8_t RXByteCtr = 0;  // Number of bytes left to receive
uint8_t ReceiveIndex = 0;   // Index of the next byte to be received in ReceiveBuffer
uint8_t TransmitBuffer[MAX_BUFFER_SIZE] = {0};  // Buffer used to transmit data
uint8_t TXByteCtr = 0;  // Number of bytes left to transfer
uint8_t TransmitIndex = 0;  // Index of the next byte to be transmitted in TransmitBuffer

/*GYRO_DATA*/
// Used to store and interpret the raw gyro data
volatile uint16_t xGyroFull = 0;
volatile uint16_t yGyroFull = 0;
volatile uint16_t zGyroFull = 0;
volatile uint16_t xGyroFinal = 0;
volatile uint16_t yGyroFinal = 0;
volatile uint16_t zGyroFinal = 0;
volatile uint16_t magnitude = 0;

/* I2C Write and Read Functions */

/* For worker device with dev_addr, writes the data specified in *reg_data
 *
 * dev_addr: The device address to write to.
 * reg_addr: The register or command to send to the device.
 * *reg_data: The buffer to write
 *           Example: controllerType0
 * count: The length of *reg_data
 *           Example: TYPE_1_LENGTH
 *  */
I2C_Mode I2C_controller_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count);

/* For worker device with dev_addr, read the data specified in workers reg_addr.
 * The received data is available in ReceiveBuffer
 *
 * dev_addr: The worker device address.
 *           Example: DEV_ADDR
 * reg_addr: The register or command to send to the worker.
 *           Example: CMD_TYPE_0_worker
 * count: The length of data to read
 *           Example: TYPE_1_LENGTH
 *  */
I2C_Mode I2C_controller_ReadReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t count);
void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count);


I2C_Mode I2C_controller_ReadReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t count)
{
    /* Initialize state machine */
    controllerMode = TX_REG_ADDRESS_MODE;   // Set the state machine to transmit the register address
    TransmitRegAddr = reg_addr;            // Set the register address to read from
    RXByteCtr = count;                      // Set the number of bytes to read
    TXByteCtr = 0;                        // Reset the number of bytes to transmit
    ReceiveIndex = 0;                  // Reset the receive index
    TransmitIndex = 0;              // Reset the transmit index

    UCB0I2CSA = dev_addr;                // Set the worker address
    IFG2 &= ~(UCB0TXIFG + UCB0RXIFG);       // Clear any pending interrupts
    UCB0IE &= ~UCB0RXIE;                       // Disable RX interrupt
    UCB0IE |= UCB0TXIE;                        // Enable TX interrupt

    UCB0CTLW0 |= UCTR + UCTXSTT;             // I2C TX, start condition
    __bis_SR_register(GIE);              // Enter LPM0 w/ interrupts

    return controllerMode;

}


I2C_Mode I2C_controller_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
    /* Initialize state machine */
    controllerMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = reg_addr;

    //Copy register data to TransmitBuffer
    CopyArray(reg_data, TransmitBuffer, count);

    TXByteCtr = count;
    RXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    /* Initialize worker address and interrupts */
    UCB0I2CSA = dev_addr;
    IFG2 &= ~(UCB0TXIFG + UCB0RXIFG);       // Clear any pending interrupts
    UCB0IE &= ~UCB0RXIE;                       // Disable RX interrupt
    UCB0IE |= UCB0TXIE;                        // Enable TX interrupt

    UCB0CTLW0 |= UCTR + UCTXSTT;             // I2C TX, start condition
    __bis_SR_register(GIE);              // Enter LPM0 w/ interrupts

    return controllerMode;
}

/*
* Copies the data from source to dest
* source: The source array
* dest: The destination array
* count: The number of elements to copy
*/
void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count)
{
    uint8_t copyIndex = 0;
    // Copy data from source to dest
    for (copyIndex = 0; copyIndex < count; copyIndex++)
    {
        // Copy data from source to dest
        dest[copyIndex] = source[copyIndex];
    }
}



//******************************************************************************
// Device Initialization *******************************************************
//******************************************************************************


void initClockTo16MHz()
{
    if (CALBC1_16MHZ==0xFF)                  // If calibration constant erased
    {
        while(1);                               // do not load, trap CPU!!
    }
    DCOCTL = 0;                               // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_16MHZ;                    // Set DCO
    DCOCTL = CALDCO_16MHZ;
}

void initGPIO()
{
    P1DIR |= BIT0 + BIT1 + BIT2 + BIT3;
    P1OUT &= ~(BIT0 + BIT1 + BIT2 + BIT3);

    P1SEL |= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0
    P1SEL2|= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0
}

void initI2C()
{
    UCB0CTLW0 |= UCSWRST;                      // Enable SW reset
    UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C controller, synchronous mode
    UCB0CTLW0 = UCSSEL_2 + UCSWRST;            // Use SMCLK, keep SW reset
    UCB0BR0 = 160;                            // fSCL = SMCLK/160 = ~100kHz
    UCB0BR1 = 0;
    UCB0I2CSA = DEV_ADDR;                   // worker Address
    UCB0CTLW0 &= ~UCSWRST;                     // Clear SW reset, resume operation
    UCB0I2CIE |= UCNACKIE;
}

void setupBT() {
    WDTCTL = WDTPW + WDTHOLD;   // Stop watchdog timer

    P1DIR |= TXLED | RXLED;     // Set P1 as output
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

void I2CLoop(){
        I2C_controller_ReadReg(DEV_ADDR, 0x43, TYPE_2_LENGTH); // Read the accelerometer and gyro data from the MPU6050
        xGyroFull = ReceiveBuffer[8] << 8 | ReceiveBuffer[9];
        xGyroFinal = xGyroFull / 32.8;
        yGyroFull = ReceiveBuffer[10] << 8 | ReceiveBuffer[11];
        yGyroFinal = yGyroFull / 32.8;
        zGyroFull = ReceiveBuffer[12] << 8 | ReceiveBuffer[13];
        zGyroFinal = zGyroFull / 32.8;
        // Calculate the magnitude of the gyro data
        magnitude = sqrt(xGyroFinal * xGyroFinal + yGyroFinal * yGyroFinal + zGyroFinal * zGyroFinal);
        if(xGyroFull > 300){
            P1OUT ^= 0X01;
            __delay_cycles(1000000);
            P1OUT ^= 0X01;
        }
        __delay_cycles(50000);
}

void BTloop() {
    if (!(UCA0IFG & UCRXIFG)) {
        char data_received = UCA0RXBUF;

        if (data_received == 'a') {
            i = 0;
            UCA0TXBUF = string[i++]; // Start sending the string
        }
    }
}

void initGyro(){
    I2C_controller_WriteReg(DEV_ADDR, POWER_ON_CMD,     PowerOnSeq,  TYPE_1_LENGTH); // Power on the MPU6050
    __delay_cycles(500000);
    I2C_controller_WriteReg(DEV_ADDR, GYRO_CONFIG_CMD,  GyroConfig,  TYPE_1_LENGTH); // Configure the gyro
    __delay_cycles(500000);
    I2C_controller_WriteReg(DEV_ADDR, ACCEL_CONFIG_CMD, AccelConfig, TYPE_1_LENGTH); // Configure the accelerometer
    __delay_cycles(500000);

}

int main(void) {

    WDTCTL = WDTPW | WDTHOLD;


    initClockTo16MHz();
    initGPIO();
    initI2C();
    setupBT();
    initGyro();
    __enable_interrupt();
    while(1){
        I2CLoop();
        BTloop();
    }
    return 0;
}
//******************************************************************************
// I2C Interrupt For Received and Transmitted Data******************************
//******************************************************************************

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0TX_VECTOR))) USCIAB0TX_ISR (void)
#else
#error Compiler not supported!
#endif
{
  if (IFG2 & UCB0RXIFG)                 // Receive Data Interrupt
  {
      //Must read from UCB0RXBUF
      uint8_t rx_val = UCB0RXBUF;

      if (RXByteCtr)
      {
        // Stores received data in ReceiveBuffer
          ReceiveBuffer[ReceiveIndex++] = rx_val;
          // Decrement RX byte counter
          RXByteCtr--;
      }
        //If the RX byte counter is 1, then we must send a NACK
      if (RXByteCtr == 1)
      {
          UCB0CTLW0 |= UCTXSTP;
      }
      else if (RXByteCtr == 0)
      {
        // Once the RX byte counter is 0, we must send a stop condition
        // and disable the RX interrupt
          UCB0IE &= ~UCB0RXIE;
          controllerMode = IDLE_MODE;
          __bic_SR_register_on_exit(GIE);      // Exit LPM0
      }
  }
  else if (IFG2 & UCB0TXIFG)            // Transmit Data Interrupt
  {
      switch (controllerMode)
      {
          case TX_REG_ADDRESS_MODE:
              UCB0TXBUF = TransmitRegAddr;
              if (RXByteCtr != RXByteCtr){
                  controllerMode = SWITCH_TO_RX_MODE;   // Need to start receiving now
              }
              else
              {
                  controllerMode = TX_DATA_MODE;  // Continue to transmision with the data in Transmit Buffer
              }
              break;

          case SWITCH_TO_RX_MODE:
              UCB0IE |= UCB0RXIE;              // Enable RX interrupt
              UCB0IE &= ~UCB0TXIE;             // Disable TX interrupt
              UCB0CTLW0 &= ~UCTR;            // Switch to receiver
              controllerMode = RX_DATA_MODE;    // State state is to receive data
              UCB0CTLW0 |= UCTXSTT;          // Send repeated start
              if (RXByteCtr == 1)
              {
                  //Must send stop since this is the N-1 byte
                  while((UCB0CTLW0 & UCTXSTT));
                  UCB0CTLW0 |= UCTXSTP;      // Send stop condition
              }
              break;

          case TX_DATA_MODE:
              if (TXByteCtr)
              {
                  UCB0TXBUF = TransmitBuffer[TransmitIndex++];
                  TXByteCtr--;
              }
              else
              {
                  //Done with transmission
                  UCB0CTLW0 |= UCTXSTP;     // Send stop condition
                  controllerMode = IDLE_MODE;
                  UCB0IE &= ~UCB0TXIE;                       // disable TX interrupt
                  __bic_SR_register_on_exit(GIE);      // Exit LPM0
              }
              break;

          default:
              __no_operation();
              break;
      }
  }
}


//******************************************************************************
// I2C Interrupt For Start, Restart, Nack, Stop ********************************
//******************************************************************************

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0RX_VECTOR))) USCIAB0RX_ISR (void)
#else
#error Compiler not supported!
#endif
{
    if (UCB0STAT & UCNACKIFG)
    {
        UCB0STAT &= ~UCNACKIFG;             // Clear NACK Flags
    }
    if (UCB0STAT & UCSTPIFG)                        //Stop or NACK Interrupt
    {
        UCB0STAT &=
            ~(UCSTTIFG + UCSTPIFG + UCNACKIFG);     //Clear START/STOP/NACK Flags
    }
    if (UCB0STAT & UCSTTIFG)
    {
        UCB0STAT &= ~(UCSTTIFG);                    //Clear START Flags
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