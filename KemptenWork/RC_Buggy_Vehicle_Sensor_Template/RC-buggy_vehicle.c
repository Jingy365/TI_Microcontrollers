/*
 * RC-buggy_receiver.c
 *
 *  Created on: Nov 23, 2022
 *      Author: stix
 */

#include <RC-buggy_vehicle.h>

/*** globals ***********************************************************/



/*** Functions *********************************************************/

/*************************************************************************
 * read RTC and convert to millis
 * overflow @ 10.000, output 0...9999
 ************************************************************************/
uint16_t readRTCmillis(void){
    static uint16_t timeRAW = 0;
    static uint16_t RTC_old = 0;

    uint16_t RTC_now = RTCCNT12;
    uint16_t diff = RTC_now - RTC_old;
    RTC_old = RTC_now;

    timeRAW += diff;
    if (timeRAW >= 10240)
        timeRAW -= 10240;

    return (timeRAW * 0.9765625);
}


void readUART(int16_t* steering_p, int16_t* drive_p){

    static int errorCount = 0;

    uint16_t receivedSteering = unPack2x7bit(&rxData_uart[0]);
    uint16_t receivedDrive = unPack2x7bit(&rxData_uart[2]);
    uint16_t crc = unPack2x7bit(&rxData_uart[4]);

    // calculate CRC checksum
    CRCINIRES = 0x4BEE;
    CRCDI = receivedSteering;
    CRCDI = receivedDrive;
    // no error:
    if(crc == (CRCINIRES & 0x3FFF)){
        if (startupCounter == 0){
            *drive_p = receivedDrive - SERVO_NEUTRAL;
        }
        *steering_p = receivedSteering - SERVO_NEUTRAL;
        errorCount = 0;
    }
    // hold outputs, count errors
    else if(errorCount < 5){
        errorCount++;
    }
    // fail safe: set outputs neutral
    else{
        *steering_p = CONTROL_NEUTRAL;
        *drive_p = CONTROL_NEUTRAL;
    }

    // visualize control signals, make sure to reach 0 %
    pwmRed = (steering_g + SERVO_NEUTRAL) - (int16_t)(PERIOD_LED * 0.01);
    pwmGreen = (drive_g + SERVO_NEUTRAL) - (int16_t)(PERIOD_LED * 0.01);
}



void sendNames(uint16_t index, char names[TELEMETRY_SIZE][30]){
    index %= TELEMETRY_SIZE;
    int i = 0;


    u.txName.nameByte = 0xFF;
    u.txName.index = index;
    while (i < 30){
        u.txName.nameString[i] = names[index][i];
        if (u.txName.nameString[i++] == 0)
            break;
    }
    pack2x7bit(u.tx.crc16, calcCRC16(&u.txData[1], 2 * TELEMETRY_SIZE));

    UCA3IFG &= ~UCTXIFG;
    DMA4CTL |= DMAEN;   // enable TX DMA
    UCA3IFG |= UCTXIFG; // set flag to start DMA
}



/*************************************************************************
 * Use 16 bit CRC module to calculate CRC checksum
 ************************************************************************/
uint16_t calcCRC16(uint8_t* data, int size){
    CRCINIRES = 0xFFFF;
    while(size--){
        CRCDIRB_L = *data++;
    }
    return CRCINIRES;
}



/*************************************************************************
 * pack a 14 bit value in two 7-bit bytes,
 * H-bit is always low
 * in bits:     fedc.ba98.7654.3210
 * out[0] bits: _dcb.8765 (_ = low)
 * out[1] bits: _654.3210
 ************************************************************************/
void pack2x7bit(uint8_t* packed, uint16_t hex14bit){
    packed[0] = (hex14bit >> 7) & 0x7F;
    packed[1] = hex14bit & 0x7F;
}



/*************************************************************************
 * Combine two 7-bit bytes into 14 bit,
 * highest two bits are always low
 * in[0] bits:  fedc.ba98 (high 7 bits)
 * in[1] bits:  7654.3210 (low 7 bits)
 * ( todo: out bits:    eeed.cba9.8654.3210 (e = sign pos/neg) )
 * out bits:    __ed.cba9.8654.3210 (_ = low)
 ************************************************************************/
uint16_t unPack2x7bit(uint8_t* packed){
    int16_t hex14bit;
    hex14bit = packed[0] << 9;
    hex14bit >>= 2;
//    hex14bit = (packed[0] & 0x7F) << 7;
    hex14bit |= packed[1] & 0x7F;
    return (uint16_t)hex14bit & 0x3FFF;
}



void setup_ADC(void){

    P1SEL1 |= BIT4; // Configure P1.4 for ADC
    P1SEL0 |= BIT4;

     // Configure ADC12
    ADC12CTL0 =
            ADC12ON |       // Turn on ADC
            ADC12MSC|       // automatic restart conversion
            ADC12SHT0_10;   // Sampling timer 512 ADC_CLK cycles

    ADC12CTL1 |=
            ADC12SHP |      // SAMPCON signal is sourced from the sampling timer.
            ADC12CONSEQ_3;  // Repeat-sequence-of-channels

    ADC12CTL2 |=
            ADC12RES_2;     // 12-bit conversion

    ADC12MCTL1 |=
            ADC12INCH_4 |   // A4 select
            ADC12EOS |      // End of sequence
            ADC12VRSEL_0;   // Vref = VCC

    ADC12CTL0 |=
            ADC12ENC |      // enable conversion
            ADC12SC;        // Start first conversion
}



void setup_DMA_SPI_RX(void){

    //set source address
    DMA_INIT(&UCB0RXBUF, DMA0SA);

    // set destination address
    uint8_t* destAdress_ptr = (uint8_t*)&spiBuff_RX;
    destAdress_ptr++; // start in address dummy, address is 1 byte only
    DMA_INIT(destAdress_ptr, DMA0DA);

    // set block size: data bytes plus one address byte
    DMA0SZ = DMA_SPI_RX_SIZE;

    // DMA control:
    DMA0CTL =
            DMAEN__DISABLE |    // DMA interrupt
            DMASRCBYTE |        // source: byte mode
            DMADSTBYTE |        // dest: byte mode
            DMALEVEL__EDGE |    // trigger type todo: edge!
            DMASRCINCR_0 |      // source: no increment
            DMADSTINCR_3 |      // dest: increment
            DMADT_0;            // single transfer (and decrement size)

    // DMA: triggered block
    DMACTL0 |= DMA0TSEL__UCB0RXIFG;

    DMACTL4 =
            ENNMI_0 |
            0 |     //ROUNDROBIN
            DMARMWDIS;

    // DMA will be enabled at EOF
    //DMA0CTL |= DMAEN;   // enable SPI DMA

}



void setup_DMA_SPI_TX(void){

    //set source address, first byte (= address) will be transfered manually
    // start in data section, address is skipped
    DMA_INIT(&spiBuff_TX[1], DMA1SA);

    //set destination address
    DMA_INIT(&UCB0TXBUF, DMA1DA);

    // set block size
    DMA1SZ = DMA_SPI_TX_SIZE;

    // DMA control:
    DMA1CTL =
            DMAEN__DISABLE |    // DMA interrupt
            DMASRCBYTE |        // source: byte mode
            DMADSTBYTE |        // dest: byte mode
            DMALEVEL__EDGE |    // trigger type todo: edge!
            DMASRCINCR_3 |      // source: increment
            DMADSTINCR_0 |      // dest: no increment
            DMADT_0;            // single transfer (and decrement size)

    // DMA: triggered block
    DMACTL0 |= DMA1TSEL__UCB0TXIFG;

    DMACTL4 =
            ENNMI_0 |
            0 |     //ROUNDROBIN
            DMARMWDIS;

    // DMA will be enabled at EOF
    //DMA1CTL |= DMAEN;   // enable SPI DMA

}



void setup_DMA_RX(void){

    //set source address
    DMA_INIT(&UCA3RXBUF, DMA5SA);

    // set destination address
    DMA_INIT(rxData_uart, DMA5DA);

    // set block size
    DMA5SZ = 6;

    // DMA control:
    DMA5CTL =
            DMAIE__ENABLE |     // interrupt enabled?
            DMASRCBYTE__BYTE |  // source mode: byte/word
            DMADSTBYTE__BYTE |  // dest mode: byte/word
            DMASRCINCR_0 |      // source: no increment
            DMADSTINCR_3 |      // dest: increment
            DMADT_0;            // single transfer (and decrement size)

    // DMA: triggered block
    DMACTL2 |= DMA5TSEL__UCA3RXIFG;

    DMACTL4 =
            ENNMI_0 |
            0 |     //ROUNDROBIN
            DMARMWDIS;

    // DMA will be started in timer ISR
}



void setup_DMA_TX(void){

    //set source address
    DMA_INIT(&u.txData[0], DMA4SA);

    // set destination address
    DMA_INIT(&UCA3TXBUF, DMA4DA);

    // set block size
    DMA4SZ = sizeof(u);

    // DMA control:
    DMA4CTL =
            DMAIE__DISABLE |    // no interrupt
            DMASRCBYTE |        // source: byte mode
            DMADSTBYTE |        // dest: byte mode
            DMASRCINCR_3 |      // source: increment
            DMADSTINCR_0 |      // dest: no increment
            DMADT_0;            // single transfer (and decrement size)

    // DMA: triggered block
    DMACTL2 |= DMA4TSEL__UCA3TXIFG;

    DMACTL4 =
            ENNMI_0 |
            0 |     //ROUNDROBIN
            DMARMWDIS;

    // DMA will be started in writeUART()
}



void setup_hardware(void){                  // Module           Pins
    setSYSCLK(MCLK, SMCLK);                 //------------------------------------------
    setup_io(4, "PWM_RED", "PWM_GREEN", "KEY_S1", "KEY_S2");    // Timer A0
    setup_UART();                           // eUSCI_A3,        P6.0, P6.1
    setup_DMA_SPI_RX();                     // DMA0 (from Nucleo board)
    setup_DMA_SPI_TX();                     // DMA1 (to Nucleo board)
    setup_DMA_TX();                         // DMA4
    setup_DMA_RX();                         // DMA5
  //setup_controlPWM();                     // Timer A1         P1.2, P1.3    --> main()
    setup_syncWatchdogTimer();              // Timer A2
    setup_triggerTimer();                   // Timer A3
    setup_capture();                        // Timer B0         P3.4, P3.5, P3.6, P3.7
                                            //                  P3.1, P3.2, P3.3, P3.4
    setup_ADC();                            // ADC Ch.4         P1.4
    setup_referenceTimer();                 // RTC
    setup_UCB0_SPI_Slave();                 // eUSCI_B1         (P5.0), P5.1, P5.3
    setup_I2C();                            // eUSCI_B2         P7.0, P7.1
    __enable_interrupt();
    setup_acceleration_sensor_ADXL346();
#if defined(_MODIFY_ACS37800_ADDRESS_)
    setACS378address(CURR_SENSOR_ADDRESS);
#endif
    setup_current_sensor_ACS37800();
    setup_angle_sensor_AS5048B();

}



void setup_referenceTimer(void){

    PJSEL0 = BIT4 | BIT5;                   // Initialize LFXT pins

    // Configure LFXT 32kHz crystal
    CSCTL0_H = CSKEY >> 8;                  // Unlock CS registers
    CSCTL4 &= ~LFXTOFF;                     // Enable LFXT
    do
    {
      CSCTL5 &= ~LFXTOFFG;                  // Clear LFXT fault flag
      SFRIFG1 &= ~OFIFG;
    } while (SFRIFG1 & OFIFG);              // Test oscillator fault flag
    CSCTL0_H = 0;                           // Lock CS registers

    // Setup RTC Timer
    RTCCTL0_H = RTCKEY_H;           // Unlock RTC

    //RTCCTL0_L = RTCTEVIE;           // RTC event interrupt enable

    RTCCTL1 =
         // RTCMODE_0 |             // Counter Mode (no fitting #define)
            RTCSSEL_2 |             // Count RT1prescaler output
            RTCHOLD;                // hold during configuration

    /*RTCPS0CTL =
            RT0PSHOLD_0 |           // RT0prescaler is used
            RT0IP__32 |             // Prescaler value
            RT0PSIE__ENABLE;        // Interrupt enable*/

    RTCPS1CTL =
            RT1SSEL_0 |             // RT1prescaler from 32 kHz crystal
            RT1PSDIV__32 |          // Prescaler value (counter)
            RT1IP__32 |             // Prescaler value (ISR, debug only)
            RT1PSIE__DISABLE;       // no Interrupt enable

    RTCCTL1 &= ~(RTCHOLD);          // Start RTC

}



void setup_triggerTimer(void){

    // Filter timing
    TA3CCR0 = FILTER_CYCLE;
    TA3CCTL0 =
            CAP__COMPARE |       // Compare Mode
            CCIE_1;              // interrupt enabled


    // Telemetry TX timing
    TA3CCR1 = TELEMETRY_CYCLE;
    TA3CCTL1 =
            CAP__COMPARE |       // Compare Mode
            CCIE_1;              // interrupt enabled

    // start Timer A3
    TA3EX0 = TAIDEX__2;          // Prescaler

    TA3CTL =
            TASSEL__SMCLK |      // Count SMCLK
            ID__4 |              // Prescaler: 4 usec/tick @ TAIDEX__2
                                 //
            MC__CONTINUOUS;      // Start Countersetup_triggerTimer
}



void setup_UART(void){
    // 8N1
    UCA3CTLW0 =
            UCPEN_0 |           // no parity
            UC7BIT__8BIT |      // 8
            UCSPB_0 |           // one stop bit
            UCSYNC__ASYNC |     //
            UCMODE_0 |          // UART mode
            UCSSEL__SMCLK |     // source clock
            UCSWRST__ENABLE;    // hold in RESET to enable baud rate setup

    // baud rate
    //UCA3BRW = 13;           // for SMCLK = 2 MHz, 9600 baud
    UCA3BRW = 1;            // for SMCLK = 2 MHz, 115200 baud
    UCA3MCTLW_H = 0x4A;     // for SMCLK = 2 MHz, 115200 baud
    UCA3MCTLW |=
            UCBRF_1 |
            UCOS16_1;

    // connect TX pin, RX pin to UART
    P6SEL0 |= (BIT0 | BIT1);
    P6SEL1 &= ~(BIT0 | BIT1);

    // release RESET
    UCA3CTLW0 &= ~UCSWRST;

    // prepare TX string
    u.tx.startOfFrame = 0xA2;
    u.tx.endOfFrame = 0xA3;

}



void setup_syncWatchdogTimer(void){

    // syncDMA
    TA2CCR0 = UART_SYNC_TIME;
    TA2CCTL0 =
            CAP__COMPARE |       // Compare Mode
            CCIE_1;              // interrupt enabled

    // watchdog
    TA2CCR1 = WATCHDOG_TIMEOUT;
    TA2CCTL1 =
            CAP__COMPARE |       // Compare Mode
            CCIE_1;              // interrupt enabled


    // start Timer A2
    TA2EX0 = TAIDEX__2;          // Prescaler

    TA2CTL =
            TASSEL__SMCLK |      // Count SMCLK
            ID__4 |              // Prescaler: 4 usec/tick @ TAIDEX__2
                                 //
            MC__CONTINUOUS;      // Start Counter
}


void prepareSPImessage(void){

    accData_t acceleration;
    power_t power;
    int16_t servoAngle;

    // read sensors
    acceleration = readAcceleration(ACC_SENSOR_ADDRESS);
    power = readPower();
    servoAngle = readAngle(ANGLE_SENSOR_ADDRESS);

    spi.TX.data[0] = drive_g;
    spi.TX.data[1] = calcRPM(FRONT_L);
    spi.TX.data[2] = calcRPM(FRONT_R);
    spi.TX.data[3] = calcRPM(REAR_L);
    spi.TX.data[4] = calcRPM(REAR_R);
    spi.TX.data[5] = power.voltage;
    spi.TX.data[6] = power.current;
    spi.TX.data[7] = steering_g;
    spi.TX.data[8] = servoAngle;
    spi.TX.data[9] = acceleration.x;
    spi.TX.data[10] = acceleration.y;
    spi.TX.data[11] = acceleration.z;
    spi.TX.data[12] = acceleration.angX;
    spi.TX.data[13] = acceleration.angY;
    spi.TX.data[14] = acceleration.angZ;
    spi.TX.data[15] = 0x1234; // fixed values to detect health of communication
    spi.TX.data[16] = 0xABCD;
}



void writeUART(void){

    // TX has 14 Bit Range: -8192 .. 8191
    // ==> frameCount: -5000 .. 4999 translates to 0 .. 9999
    static int16_t frameCount = -5000;
    static uint16_t startCount = NAME_TRANSFERS * TELEMETRY_SIZE;

    //accData_t acceleration;
    power_t power;
    int16_t servoAngle;

    // TX has 14 Bit Range: -8192 .. 8191
    // ==> millis: -5000 .. 4999 translates to 0 .. 9999
    uint16_t millis = readRTCmillis() -5000;

    static char dataNames[TELEMETRY_SIZE][30] =
    {
        /* 0*/ "Frame Number",
        /* 1*/ "Milliseconds",
        /* 2*/ "Drive",
        /* 3*/ "Steering Angle",
        /* 4*/ "Servo Angle",
        /* 5*/ "rpm Front L",
        /* 6*/ "rpm Front R",
        /* 7*/ "rpm Rear L",
        /* 8*/ "rpm Rear R",
        /* 9*/ "Voltage",
        /*10*/ "Current",
        /*11*/ "Control Action",
        /*12*/ "PWM out",
        /*13*/ "Slip Ratio",
        /*14*/ "km/h Front",
        /*15*/ "km/h Rear"
    };


    if(startCount){
        sendNames(--startCount, dataNames);
    }
    // send message, skip if already sending
    else if(S1_RELEASED && !(DMA4CTL & DMAEN)) {

        // read sensors
        //acceleration = readAcceleration(ACC_SENSOR_ADDRESS);
        power = readPower();
        servoAngle = readAngle(ANGLE_SENSOR_ADDRESS);

        pack2x7bit(u.tx.data0, frameCount++);
        pack2x7bit(u.tx.data1, millis);
        pack2x7bit(u.tx.data2, drive_g);
        pack2x7bit(u.tx.data3, steering_g);
        pack2x7bit(u.tx.data4, servoAngle);
        pack2x7bit(u.tx.data5, calcRPM(FRONT_L));
        pack2x7bit(u.tx.data6, calcRPM(FRONT_R));
        pack2x7bit(u.tx.data7, calcRPM(REAR_L));
        pack2x7bit(u.tx.data8, calcRPM(REAR_R));
        pack2x7bit(u.tx.data9, power.voltage);
        pack2x7bit(u.tx.data10, power.current);
        pack2x7bit(u.tx.data11, spi.RX.debug[0]);
        pack2x7bit(u.tx.data12, spi.RX.debug[1]);
        pack2x7bit(u.tx.data13, spi.RX.debug[2]);
        pack2x7bit(u.tx.data14, spi.RX.debug[3]);
        pack2x7bit(u.tx.data15, spi.RX.debug[4]);
        //pack2x7bit(u.tx.data15, ADC12MEM1); // analog servo angle
        pack2x7bit(u.tx.crc16, calcCRC16(&u.txData[1], 2 * TELEMETRY_SIZE));
        if (frameCount > 4999)
            frameCount = -5000;

        UCA3IFG &= ~UCTXIFG;
        DMA4CTL |= DMAEN;   // enable TX DMA
        UCA3IFG |= UCTXIFG; // set flag to start DMA
    }
}



/*** Interrupt Handlers ************************************************/

/********************************************************************//**
 *   @brief TA2 ISR
 *   restarts DMA when UART RX is idle to synchronize on start of frame
 ***********************************************************************/
#pragma vector = TIMER2_A0_VECTOR
__interrupt void ISR_syncDMA(void) {

    UCA3IFG &= ~UCRXIFG;
    DMA5CTL |= DMAEN;   // enable RX DMA
}



/********************************************************************//**
 *   @brief TA2 watchdog ISR
 ***********************************************************************/
#pragma vector = TIMER2_A1_VECTOR
__interrupt void ISR_watchdog(void) {

    switch( __even_in_range(TA2IV, TAIV__TACCR1) )
    {
    case TAIV__NONE:
        break; // no interrupt pending

    case TAIV__TACCR1: // watchdog
        isOnline = false;
        break;

    default: __never_executed();
    }

}



/********************************************************************//**
 *   @brief TA3 ISR
 *   timer for cyclic filter calculation
 ***********************************************************************/
#pragma vector = TIMER3_A0_VECTOR
__interrupt void ISR_filter(void) {

    // prepare next filter request
    TA3CCR0 += FILTER_CYCLE;
}



/********************************************************************//**
 *   @brief TA3 ISR
 *   timer for cyclic telemetry transmission
 ***********************************************************************/
#pragma vector = TIMER3_A1_VECTOR
__interrupt void ISR_telemetry(void) {

    // reset Interrupt
    TA3CCTL1 &= ~CCIFG;

    isTelemetryRequest = true;
    // prepare next telemetry request
    TA3CCR1 += TELEMETRY_CYCLE;

}



/********************************************************************//**
 *   @brief ISR_Nucleo_SPI_CS ISR
 *   restarts DMA (when CS goes passive) to synchronize on start of frame
 ***********************************************************************/
#pragma vector = PORT5_VECTOR
__interrupt void ISR_Nucleo_SPI_CS(void) {

    uint16_t i = 0;

    DEBUG_ON;
    switch (__even_in_range(P5IV, P5IV_P5IFG7))
    {
        case P5IV_NONE: break;
        case P5IV_P5IFG0: break;
        case P5IV_P5IFG1: break;
        case P5IV_P5IFG2: break;

        // EOF End of Frame
        case P5IV_P5IFG3:
            // stop DMA (in case of false frame alignment); will be restarted at SOF
            DMA0CTL &= ~DMAEN;
            DMA1CTL &= ~DMAEN;

            // reset SPI module
            UCB0CTLW0 |= UCSWRST;
            // CLK should be already low, but just in case:
            while(P2IN & BIT2); //wait until CLK is low: workaround, see Error USCI47
            // while(!(UCB0IFG & UCTXIFG)); //wait for SPI module ready

            // copy SPI message to a locked TX buffer
            for(i = 0; i < MATLAB_SPI_SIZE; i++){
                spiBuff_TX[i + 1] = spi.TX.data[i];
            }

            // wait until RX DMA0 is stopped (todo: timeout counter)
            // while(DMA0CTL & DMAEN); todo: notwendig???

            // read locked RX buffer to dbg1 .. dbg5
            for(i = 0; i < MATLAB_SPI_SIZE; i++){
                spi.RX.debug[i] = spiBuff_RX[i + 1];
            }

            // restore block size, (in case of false frame alignment)
            DMA0SZ = DMA_SPI_RX_SIZE;
            DMA1SZ = DMA_SPI_TX_SIZE;

            // release SPI RESET
            UCB0CTLW0 &= ~UCSWRST;

            DMA1CTL |= DMAEN;   // start SPI TX DMA
            DMA0CTL |= DMAEN;   // start SPI RX DMA

            UCB0TXBUF = 0x80; // Address

            break;


        case P5IV_P5IFG4: break;
        case P5IV_P5IFG5: break;
        case P5IV_P5IFG6: break;
        case P5IV_P5IFG7: break;
        default: __never_executed();
    }
    DEBUG_OFF;

}



/********************************************************************//**
 *   @brief UART RX DMA ISR
 *   called at end of UART RX frame
 ***********************************************************************/
#pragma vector = DMA_VECTOR
__interrupt void ISR_DMA(void) {

    switch( __even_in_range(DMAIV, DMAIV__DMA7IFG) )
    {
    case DMAIV__NONE:
        break; // no interrupt pending

    case DMAIV__DMA3IFG:
        // DMA3 handles SPI out, no interrupt
        break;

    case DMAIV__DMA5IFG:
        /*** Read UART ***************/
        // stop DMA, will be restarted by timer ISR
        /*DMA5CTL &= ~DMAEN;*/

        // reset DMA interrupt flag
        /*DMA5CTL &= ~DMAIFG;*/

        // restart trigger timer
        TA2CCR0 = TA2R + UART_SYNC_TIME;

        // renew watchdog timeout
        TA2CCR1 = TA2R + WATCHDOG_TIMEOUT;

        // report to main():
        isUARTDataReceived = true;
        isOnline = true;
        break;

    default: __never_executed();
    }

}

