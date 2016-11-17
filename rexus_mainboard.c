/*
 *  rexus_mainboard.c
 *
 *  Created on: 02.11.2016
 *      Author: Ulrich Nordmeyer
 *     Version: v2.2
 *       
 *  Mainboard Firmware for DIANE Experiment on REXUS 21 rocket.
 *  MCU is an ATmega32
 */

//#define F_CPU -> makefile
//#define MCU   -> makefile

#include <inttypes.h>       // uint8_t etc.
#include <avr/io.h>         // port registers
#include <avr/interrupt.h>  // interrupts
#include <avr/eeprom.h>     // EEPROM access
#include <util/twi.h>       // TWI

// this sh*t dont work properly with damn avr-libc
//#define EEPROM      __attribute__ ((section (".eeprom"))) // fuer EEPROM-Zugriffe


#define START_byte  0b11001100  // 0xCC start byte for USART transmission
#define STOP_byte   0b00011111  // 0x1F stop byte for USART transmission

#define PRS0_id     0b00110001  // 0x31 ID for data of PRS_sensor_0
#define PRS1_id     0b00110010  // 0x32 ID for data of PRS_sensor_1
#define TEMP_id     0b00110011  // 0x33 ID for data of TEMP_sensor
#define CAM0_id     0b00110100  // 0x34 ID for status of CAM_0
#define CAM1_id     0b00110101  // 0x35 ID for status of CAM_1
#define ARM_id      0b00110110  // 0x36 ID for status of ARM_sensor
#define RF_id       0b00110111  // 0x37 ID for status of RF-Board
#define VLV_id      0b00111000  // 0x38 ID for status of Valve_0
#define RXSM_id     0b00110000  // 0x30 ID for RXSM signal reception answer

#define GET_prs0    0b01100001  // 0x61 ID for request of single PRS0 value
#define GET_prs1    0b01100010  // 0x62 ID for request of single PRS1 value
#define GET_temp    0b01100011  // 0x63 ID for request of single TEMP value
#define GET_arm     0b01100110  // 0x66 ID for request of arm state
#define OPN_vlv     0b01100111  // 0x67 ID for command to open Valve_0
#define CLS_vlv     0b01101000  // 0x68 ID for command to close Valve_0
#define REQ_pwr_dwn 0b01101111  // 0x6F ID for power down request

#define CAM0_ok     0b10100100  // 0xA4 ID for cam0 status ok
#define CAM1_ok     0b10100101  // 0xA5 ID for cam1 status ok
#define ARM_ok      0b10100110  // 0xA6 ID for ARM sensor ok
#define RF_ok       0b10100111  // 0xA7 ID for RF board ok
#define VLV_opnd    0b10101000  // 0xA8 ID for Valve_0 opened
#define VLV_clsd    0b10101001  // 0xA9 ID for Valve_0 closed
#define ERR_stat    0b10100000  // 0xA0 ID for error status

#define ARM_success 0b11100110  // 0xE6 ID for successful CubeSat ejection
#define SODS_ok     0b11101000  // 0xE8 ID for successful SODS reception
#define LO_ok       0b11101001  // 0xE9 ID for successful LO reception
#define SOE_ok      0b11101010  // 0xEA ID for successful SOE reception

#define SLA_R       0b10010001  // TWI slave address read temperature sensor
#define PRS0_mux    0b01000001  // ADMUX ADC channel address for PRS0_sensor (0- 3 bar)
#define PRS1_mux    0b01000000  // ADMUX ADC channel address for PRS1_sensor (0-12 bar)

#define data_array_length   840 // space for measured data for 210 s at 2 byte/s

// defining timeline events in temporal dependence of SODS at T-120
#define T_start_data_logging     99 // = T- 21
#define T_LO                    120 // = T-  0
#define T_SOE                   240 // = T+120
#define T_start_rf_sig_transm   245 // = T+125
#define T_close_valve           260 // = T+140
#define T_stop_rf_sig_transm    295 // = T+175
#define T_disable_prs_sampling  299 // = T+179
#define T_stop_arm_evaluation   305 // = T+185
#define T_stop_video_recording  325 //700 // = T+580

// defining Timer2 Compare Match purposes
#define IDLE                    0
#define TOGGLE_CAM_RECORDING    1
#define DEBOUNCE_RXSM_SIGNAL    2


// I/O Pin assignment ------------------------------------------------------------------------------

  #define PRS_sensor_1  PINA0   // adc
  #define PRS_sensor_0  PINA1   // adc
  #define ARM_sensor    PINA2   // void
  #define PRS_valve_0   PA3     // void
  #define PRS_valve_1   PA4     // void
  #define TMP_alarm     PINA5   // void
  #define RFB_silence   PA6     // void
  #define RFB_transmit  PA7     // void

//#define (gnd)         PB0     //
//#define (gnd)         PB1     //
  #define RXSM_SOE      PINB2   // interrupt
  #define CAM_0_status  PINB3   // void
  #define CAM_0_control PB4     // void
//#define SPI_MOSI      PB5     //
//#define SPI_MISO      PB6     //
//#define SPI_SCK       PB7     //

//#define TWI_SCL       PC0     //
//#define TWI_SDA       PC1     //
  #define CAM_1_status  PINC2   // void
  #define CAM_1_control PC3     // void
  #define debugLED_3    PC4     // void
  #define debugLED_2    PC5     // void
  #define debugLED_1    PC6     // void
  #define debugLED_0    PC7     // void

//#define USART_RxD     PD0     //
//#define USART_TxD     PD1     //
  #define RXSM_LO       PIND2   // interrupt
  #define RXSM_SODS     PIND3   // interrupt
  #define P_12V_enable  PD4     // void
  #define P_5V_enable   PD5     // void
//#define (gnd)         PD6     //
//#define (gnd)         PD7     //

//--------------------------------------------------------------------------------------------------


// I/O Port and Register assignment ----------------------------------------------------------------

#define port_prs_sens       PORTA
#define port_arm_sens       PORTA
#define port_prs_valv       PORTA
#define port_tmp_alrm       PORTA
#define port_rf_board       PORTA

#define port_rxsm_soe       PORTB
#define port_cam_0          PORTB

#define port_cam_1          PORTC
#define port_dbg_leds       PORTC

#define port_rxsm_sods      PORTD
#define port_rxsm_lo        PORTD
#define port_voltage_en     PORTD

//---

#define pin_prs_sens        PINA
#define pin_arm_sens        PINA
#define pin_prs_valv        PINA
#define pin_tmp_alrm        PINA
#define pin_rf_board        PINA

#define pin_rxsm_soe        PINB
#define pin_cam_0           PINB

#define pin_cam_1           PINC
#define pin_dgb_leds        PINC

#define pin_rxsm_sods       PIND
#define pin_rxsm_lo         PIND
#define pin_voltage_en      PIND

//---

#define dir_prs_sens        DDRA
#define dir_arm_sens        DDRA
#define dir_prs_valv        DDRA
#define dir_tmp_alrm        DDRA
#define dir_rf_board        DDRA

#define dir_rxsm_soe        DDRB
#define dir_cam_0           DDRB

#define dir_cam_1           DDRC
#define dir_dbg_leds        DDRC

#define dir_rxsm_sods       DDRD
#define dir_rxsm_lo         DDRD
#define dir_voltage_en      DDRD

//--------------------------------------------------------------------------------------------------

uint8_t tmp_sreg;

uint16_t usart_msg_cnt = 0;         // continuous counter of transmitted messages via USART
uint8_t  F_6Hz_ref_counter = 0;     // Timer0 auxiliary counter to generate 6Hz clock
uint8_t  F_2Hz_ref_counter = 0;     // Timer0 auxiliary counter to generate 2Hz clock
uint16_t timeline_counter = 0;      // auxiliary counter in seconds as reference for timeline activities
uint16_t timer_2_ref_counter  = 0;  // Timer2 auxiliary counter to generate event after 5s / 100ms

uint8_t  log_data_to_eeprom = 0;    // boolean whether to log data to eeprom or not
uint16_t eeprom_mom_address = 0;    // control variable for next free eeprom address

uint8_t prs_sampling_enabled = 1;   // boolean to stop pressure sampling directly
                                    // before CubeSat ejection
uint8_t arm_polling_enabled = 0;    // boolean to start/stop arm_sensor polling

uint8_t LO_active = 0;              // boolean reference for LO = timeline actions at T+n
uint8_t SOE_active = 0;             // boolean reference for SOE
uint8_t SODS_active = 0;            // boolean reference for SODS = multiple sods events

uint8_t timer_2_purpose = IDLE;     // reference for Timer2 Compare Match ISR if debouncing or cam controlling active

uint8_t LO_debounce_pending = 0;    // boolean reference for debouncing LO interrupt
uint8_t SOE_debounce_pending = 0;   // boolean reference for debouncing SOE interrupt
uint8_t SODS_debounce_pending = 0;  // boolean reference for debouncing SODS interrupt

uint8_t pending_usart_msg = 0;      // boolean to prevent usart transmissions while message reception

uint8_t cams_recording = 0;         // boolean whether or not cams are recording
uint8_t cams_shutdown = 0;          // boolean to decide whether or not to shutdown cams after having stopped recording
uint16_t cams_shutdown_timeref = 0xFFF0;    // timeline reference for 1 s delay between stop recording and shutdown when power-cycle request

void open_valve_0  (void) { port_prs_valv |=  (1<<PRS_valve_0);}
void close_valve_0 (void) { port_prs_valv &= ~(1<<PRS_valve_0);}
void open_valve_1  (void) { port_prs_valv |=  (1<<PRS_valve_1);}
void close_valve_1 (void) { port_prs_valv &= ~(1<<PRS_valve_1);}

void start_rfb_transm (void) { port_rf_board |=  (1<<RFB_transmit);}
void stop_rfb_transm  (void) { port_rf_board &= ~(1<<RFB_transmit);}
void start_rfb_silenc (void) { port_rf_board |=  (1<<RFB_silence);}
void stop_rfb_silenc  (void) { port_rf_board &= ~(1<<RFB_silence);}

void turn_on_led_0  (void) { port_dbg_leds |=  (1<<debugLED_0);}
void turn_off_led_0 (void) { port_dbg_leds &= ~(1<<debugLED_0);}
void turn_on_led_1  (void) { port_dbg_leds |=  (1<<debugLED_1);}
void turn_off_led_1 (void) { port_dbg_leds &= ~(1<<debugLED_1);}
void turn_on_led_2  (void) { port_dbg_leds |=  (1<<debugLED_2);}
void turn_off_led_2 (void) { port_dbg_leds &= ~(1<<debugLED_2);}
void turn_on_led_3  (void) { port_dbg_leds |=  (1<<debugLED_3);}
void turn_off_led_3 (void) { port_dbg_leds &= ~(1<<debugLED_3);}

void turn_on_5V   (void) { port_voltage_en &= ~(1<<P_5V_enable);}
void turn_off_5V  (void) { port_voltage_en |=  (1<<P_5V_enable);}

void turn_on_12V  (void) { port_voltage_en |=  (1<<P_12V_enable);}
void turn_off_12V (void) { port_voltage_en &= ~(1<<P_12V_enable);}

uint8_t get_cam_0_status (void) { return 1; }//pin_cam_0 & (1<<CAM_0_status);}
uint8_t get_cam_1_status (void) { return 1; }//pin_cam_1 & (1<<CAM_1_status);}

uint8_t get_arm_sens_status (void) { return pin_arm_sens & (1<<ARM_sensor);}

uint8_t get_tmp_alarm_status (void) { return pin_tmp_alrm & (1<<TMP_alarm);}


void enable_prs_sampling (void)         { prs_sampling_enabled = 1;}
void disable_prs_sampling (void)        { prs_sampling_enabled = 0;}

void start_data_eeprom_logging (void)   { log_data_to_eeprom = 1;}
void stop_data_eeprom_logging (void)    { log_data_to_eeprom = 0;}

void start_arm_polling (void)           { arm_polling_enabled = 1;}
void stop_arm_polling (void)            { arm_polling_enabled = 0;}


// USART utility routines --------------------------------------------------------------------------
void usart_transmit_byte (uint8_t byte)
{
    uint16_t transm_timeref = timeline_counter+2;                           // timestamp to avoid deadlock when usart not connected
    while ( !(UCSRA & (1<<UDRE)) && (transm_timeref>=timeline_counter))     // wait for empty transmit buffer
        ;
    UDR = byte;  
}

// transmit the given data bytes via USART
void usart_transmit_data (uint8_t prs_0_byte, uint8_t prs_1_byte, uint8_t temp_byte)
{
    usart_msg_cnt++;

    usart_transmit_byte (START_byte);       // transmit first start byte
    usart_transmit_byte (START_byte);       // transmit second start byte


    usart_transmit_byte (0xFF & usart_msg_cnt>>8);  // transmit high byte of message counter
    usart_transmit_byte (0xFF & usart_msg_cnt);     // ransmit low byte of message counter

    usart_transmit_byte (0x06);             // transmit quantity of following ID+data bytes


    usart_transmit_byte (PRS0_id);          // transmit ID for first data byte
    usart_transmit_byte (prs_0_byte);       // transmit first data byte


    usart_transmit_byte (PRS1_id);          // transmit ID for second data byte
    usart_transmit_byte (prs_1_byte);       // transmit second data byte

    usart_transmit_byte (TEMP_id);          // transmit ID for third data byte
    usart_transmit_byte (temp_byte);        // transmit third data byte


    usart_transmit_byte (STOP_byte);        // transmit first stop byte
    usart_transmit_byte (STOP_byte);        // transmit second stop byte
}

// transmit a message containing just one byte
void usart_transmit_single_byte_message (uint8_t id_byte, uint8_t data_byte)
{
    usart_msg_cnt++;

    usart_transmit_byte (START_byte);       // transmit first start byte
    usart_transmit_byte (START_byte);       // transmit second start byte

    usart_transmit_byte (0xFF & usart_msg_cnt>>8);  // transmit high byte of message counter
    usart_transmit_byte (0xFF & usart_msg_cnt);     // ransmit low byte of message counter

    usart_transmit_byte (0x02);             // transmit quantity of following ID+data bytes

    usart_transmit_byte (id_byte);          // transmit ID for this message
    usart_transmit_byte (data_byte);        // transmit data of this message

    usart_transmit_byte (STOP_byte);        // transmit first stop byte
    usart_transmit_byte (STOP_byte);        // transmit second stop byte
}


// transmit the message of successful SODS reception
void usart_transmit_sods_successful (void)
{
    usart_transmit_single_byte_message (RXSM_id, SODS_ok);
}

// transmit the message of successful LO reception
void usart_transmit_lo_successful (void)
{
    usart_transmit_single_byte_message (RXSM_id, LO_ok);
}

// transmit the message of successful SOE reception
void usart_transmit_soe_successful (void)
{
    usart_transmit_single_byte_message (RXSM_id, SOE_ok);
}

// transmit the message of successful CubeSat ejection
void usart_transmit_ejection_successful (void)
{
    usart_transmit_single_byte_message (ARM_id, ARM_success);
}

// transmit the message of opened state of valve_0
void usart_transmit_valve_0_opened (void)
{
    usart_transmit_single_byte_message (VLV_id, VLV_opnd);
}

// transmit the message of closed state of valve_0
void usart_transmit_valve_0_closed (void)
{
    usart_transmit_single_byte_message (VLV_id, VLV_clsd);
}


//--------------------------------------------------------------------------------------------------


// Measuring Temperature and Pressures -------------------------------------------------------------
//
void twi_error (void)
{
    turn_off_led_3();
}

// read then return both temperature bytes from temperature sensor via TWI
uint16_t sample_temp (void)
{
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN); // send START condition
    while ( !(TWCR & (1<<TWINT)))           // wait for TWINT flag = START condition transmitted
        ;
    if ((TWSR & 0xF8) != TW_START)          // if TWI status != START -> error
    {
        twi_error();
    }
//turn_off_led_1();

    TWDR = SLA_R;                           // load address of slave (read) into register
    TWCR = (1<<TWINT) | (1<<TWEN);          // start transmission
    
    while ( !(TWCR & (1<<TWINT)))           // wait for TWINT flag = SLA_R transmitted + ACK received
        ;
    if ((TWSR & 0xF8) != TW_MR_SLA_ACK)     // if TWI status != MR_SLA_ACK -> error
    {
        twi_error();
    }
//turn_off_led_2();
    
    TWCR = (1<<TWINT)|(1<<TWEA)|(1<<TWEN);  // receive data byte (msb), respond with ACK
    
    while ( !(TWCR & (1<<TWINT)))           // wait for TWINT flag = data byte received + ACK transmitted
        ;
    if ((TWSR & 0xF8) != TW_MR_DATA_ACK)    // if TWI status != MR_SLA_ACK -> error
    {
        twi_error();
    }
//turn_off_led_3();

    uint8_t temp_sample = 0;
    temp_sample |= TWDR;                    // store received byte inside TWDR to temp_sample 
    
    TWCR = (1<<TWINT) | (1<<TWEN);          // receive next data byte (lsb), respond with NACK
    
    while ( !(TWCR & (1<<TWINT)))           // wait for TWINT flag = data byte received + NACK transmitted
        ;
    if ((TWSR & 0xF8) != TW_MR_DATA_NACK)   // if TWI status != MR_SLA_NACK -> error
    {
        twi_error();
    }

    //temp_sample |= TWDR;                    // store received byte inside TWDR to twi_lsb
    // not storing lsb causes loss of information (+-0.5 degree) but returns uint8_t instead of uint16_t
    
    TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN); // send STOP condition
    
    return temp_sample;
}


// sample the analog voltage of the given pressure sensor with the internal ADC
uint8_t sample_prs (uint8_t sensor_id)
{
    uint8_t prs_sample = 0;
    uint16_t prs_sample_raw = 0;
    switch (sensor_id)
    {
        case 0:
            ADMUX = PRS0_mux;           // set channel for PRS_sensor_0 active
            break;
        case 1:
            ADMUX = PRS1_mux;           // set channel for PRS_sensor_1 active
            break;
    }
    ADCSRA |= (1<<ADEN) | (1<<ADSC);    // enable ADC, start conversion
    while (ADCSRA & (1<<ADSC))          // waiting for conversion to be completed
        ;
    ADCSRA &= ~(1<<ADIF);               // clear interrupt flag
    prs_sample_raw  = ADC;              // reading low byte
    //prs_sample_raw |= (ADCH<<8);        // reading high byte
    prs_sample_raw -= 102;              // subtract offset equivalent to 0.5 V

    // conversion to 8-bit value
    switch (sensor_id)
    {
        case 0:
            prs_sample = (prs_sample_raw>>1) & 0xFF;    // right-shift, division in half = 16.82 mbar/bit
            break;
        case 1:
            prs_sample = (prs_sample_raw>>2) & 0xFF;    // double right-shift = 84,18 mbar/bit
            break;
    } 

    return prs_sample;
}
//--------------------------------------------------------------------------------------------------


void start_cam_timer_on_off (void)
{
    TCCR2 &= ~(1<<WGM21);               // CTC-Mode off
    TIMSK &= ~(1<<OCIE2);               // disable compare match interrupt
    TIMSK |=  (1<<TOIE2);               // enable overflow interrupt
    timer_2_ref_counter = 0;
    //TCNT2 = 0;
    TCCR2   |= (1<<CS22) | (1<<CS21) | (1<<CS20);   // start Timer2 with clock = F_CPU/1024
}

void start_cam_timer_rec (void)
{
    TCCR2 |=  (1<<WGM21);               // CTC-Mode on
    TIMSK |=  (1<<OCIE2);               // enable compare match interrupt
    TIMSK &= ~(1<<TOIE2);               // disable overflow interrupt
    timer_2_ref_counter = 0;
    timer_2_purpose = TOGGLE_CAM_RECORDING;
    //TCNT2 = 0;
    TCCR2   |= (1<<CS22) | (1<<CS21) | (1<<CS20);   // start Timer2 with clock = F_CPU/1024
}

void start_debouncing_timer (void)
{
    TCCR2 |=  (1<<WGM21);               // CTC-Mode on
    TIMSK |=  (1<<OCIE2);               // enable compare match interrupt
    TIMSK &= ~(1<<TOIE2);               // disable overflow interrupt
    timer_2_ref_counter = 0;
    timer_2_purpose = DEBOUNCE_RXSM_SIGNAL;
    //TCNT2 = 0;
    TCCR2   |= (1<<CS22) | (1<<CS21) | (1<<CS20);   // start Timer2 with clock = F_CPU/1024
}

void turn_on_cams  (void)
{
    port_cam_0 |=  (1<<CAM_0_control);
    port_cam_1 |=  (1<<CAM_1_control);
    start_cam_timer_on_off();
}
void turn_off_cams (void)
{
    port_cam_0 |=  (1<<CAM_0_control);
    port_cam_1 |=  (1<<CAM_1_control);
    start_cam_timer_on_off();
}

void start_rec_cams (void)
{
    cams_recording = 1;
    port_cam_0 |=  (1<<CAM_0_control);
    port_cam_1 |=  (1<<CAM_1_control);
    start_cam_timer_rec();
}
void stop_rec_cams  (void)
{
    cams_recording = 0;
    port_cam_0 |=  (1<<CAM_0_control);
    port_cam_1 |=  (1<<CAM_1_control);
    start_cam_timer_rec();
}

void start_measurements (void)
{
    enable_prs_sampling();
    F_6Hz_ref_counter = 0;
    TCCR0 |= (1<<CS02) | (1<<CS00);     // start Timer0 with clock = F_CPU/1024
}

void start_timeline_counter (void)
{
    TCCR1B |= (1<<CS12) | (1<<CS10);    // start Timer1 with clock = F_CPU/1024
}

void init_pwr_dwn (void)
{
    cams_shutdown = 1;
    if (cams_recording)
    {
        stop_rec_cams();
        // cam shutdown is executed from within stop_recording
    }
    else
    {
        turn_off_cams();
    }

    stop_rfb_transm();
    disable_prs_sampling();

    usart_transmit_single_byte_message (RF_id , RF_ok);
}


void eeprom_log_data (uint8_t prs_0_byte, uint8_t prs_1_byte)
{
    if (eeprom_mom_address < (data_array_length-1))
    {
        tmp_sreg = SREG;
        cli();

        /* Wait for completion of previous write */
        while(EECR & (1<<EEWE))
        ;
        /* Set up address and data registers */
        EEAR = eeprom_mom_address;
        EEDR = prs_0_byte;
        /* Write logical one to EEMWE */
        EECR |= (1<<EEMWE);
        /* Start eeprom write by setting EEWE */
        EECR |= (1<<EEWE);

        eeprom_mom_address++;

        /* Wait for completion of previous write */
        while(EECR & (1<<EEWE))
        ;
        /* Set up address and data registers */
        EEAR = eeprom_mom_address;
        EEDR = prs_1_byte;
        /* Write logical one to EEMWE */
        EECR |= (1<<EEMWE);
        /* Start eeprom write by setting EEWE */
        EECR |= (1<<EEWE);

        eeprom_mom_address++;
        sei();
        SREG = tmp_sreg;
    }
    else
        turn_on_led_1();
}


void sods_approved (void)
{
    SODS_debounce_pending = 0;
    tmp_sreg = SREG;
    cli();
    SODS_active = 1;
turn_off_led_0();
    start_rec_cams();
    turn_on_5V();
    start_measurements();
    start_timeline_counter();
    sei();
    SREG = tmp_sreg;
    usart_transmit_sods_successful();
}

void lo_approved (void)
{
    LO_debounce_pending = 0;
    tmp_sreg = SREG;
    cli();
    LO_active= 1;
    timeline_counter = T_LO;
    start_data_eeprom_logging();
    if (!SODS_active)
    {
        SODS_active = 1;
        start_rec_cams();
        turn_on_5V();
        eeprom_mom_address = 100;
        start_measurements();
        start_timeline_counter();
    }
turn_off_led_1();
    sei();
    SREG = tmp_sreg;
    usart_transmit_lo_successful();
}

void soe_approved (void)
{
    SOE_debounce_pending = 0;
    tmp_sreg = SREG;
    cli();
    if (LO_active)
    {
        SOE_active = 1;
        timeline_counter = T_SOE;
        start_timeline_counter();
        turn_on_12V();
        open_valve_0();
turn_off_led_2();
    }
    sei();
    SREG = tmp_sreg;
    usart_transmit_soe_successful();
}


// ISRs for INT0, INT1, INT2, Timer0, Timer1, Timer2 -----------------------------------------------
 

// ISR for LO Signal from RXSM
ISR (INT0_vect)
{
    if (!LO_active && ! LO_debounce_pending)
    {
        LO_debounce_pending = 1;
        start_debouncing_timer();
    }
}

// ISR for SODS Signal from RXSM
ISR (INT1_vect)
{
    if (!SODS_active && ! SODS_debounce_pending)
    {
        SODS_debounce_pending = 1;
        start_debouncing_timer();
    }
}

// ISR for SOE Signal from RXSM
ISR (INT2_vect)
{
    if (!SOE_active && ! SOE_debounce_pending)
    {
        SOE_debounce_pending = 1;
        start_debouncing_timer();
    }
}


// ISR for USART receive complete
ISR (USART_RXC_vect)
{
    tmp_sreg = SREG;
    cli();
    uint8_t rx_byte = UDR;
    switch (rx_byte)
    {
        case START_byte:
            pending_usart_msg = 1;
            break;
        case REQ_pwr_dwn:
            init_pwr_dwn();
            break;
        case GET_prs0:
            usart_transmit_single_byte_message(PRS0_id, sample_prs(0));
            break;
        case GET_prs1:
            usart_transmit_single_byte_message(PRS1_id, sample_prs(1));
            break;
        case GET_temp:
            usart_transmit_single_byte_message(TEMP_id, sample_temp());
            break;
        case GET_arm:
            if (!get_arm_sens_status())
            {
                usart_transmit_single_byte_message(ARM_id, ARM_ok);
            }
            else
            {
                usart_transmit_single_byte_message(ARM_id, ERR_stat);
            } 
            break;
        case OPN_vlv:
            if (!SODS_active && !LO_active && !SOE_active)
            {
                open_valve_0();
                usart_transmit_valve_0_opened();
            }
            else
            {
                usart_transmit_valve_0_closed();
            }
            break;
        case CLS_vlv:
            if (!SODS_active && !LO_active && !SOE_active)
            {
                close_valve_0();
                usart_transmit_valve_0_closed();
            }
            else
            {
                usart_transmit_valve_0_opened();
            }
            break;
        case STOP_byte:
            pending_usart_msg = 0;
            break;
    }
    sei();
    SREG = tmp_sreg;
}


// ISR for Timer0 generating 6 Hz for MEASUREMENT sampling
ISR (TIMER0_COMP_vect)
{
    tmp_sreg = SREG;
    cli();
    F_6Hz_ref_counter++;
    if (F_6Hz_ref_counter == 10)
    {    
        F_6Hz_ref_counter = 0;
        F_2Hz_ref_counter++;

        uint8_t mom_prs0 = 0;
        uint8_t mom_prs1 = 0;
        uint8_t mom_temp = sample_temp();       // sample TEMP_sensor
        
        if (prs_sampling_enabled)               // dont sample pressure after CubeSat ejection
        {
            mom_prs0 = sample_prs(0);           // sample PRS_sensor_0
            mom_prs1 = sample_prs(1);           // sample PRS_sensor_1
        }

        uint8_t timeout = timeline_counter;
        while (pending_usart_msg && !(timeline_counter-timeout))
            ;
        usart_transmit_data (mom_prs0, mom_prs1, mom_temp);

        if (F_2Hz_ref_counter == 3)
        {
            F_2Hz_ref_counter = 0;
            if (log_data_to_eeprom && prs_sampling_enabled)
            {
                eeprom_log_data (mom_prs0, mom_prs1);
            }
            if (arm_polling_enabled)
            {
                if (! get_arm_sens_status())
                {
                    stop_arm_polling();
                    usart_transmit_ejection_successful();
                }   
            }
        }
    }
    sei();
    SREG = tmp_sreg;
}

// ISR for Timer1 generating 1 Hz for general control purposes on TIMELINE
ISR(TIMER1_COMPA_vect)
{
    tmp_sreg = SREG;
    cli();
    timeline_counter++;
    if (cams_shutdown && (cams_shutdown_timeref == timeline_counter))
    {
        TCCR2 &= 0b11111000;        // stop Timer2
        turn_off_cams();
    }
    switch (timeline_counter)
    {
        case T_start_data_logging:
            start_data_eeprom_logging();
            break;

        case T_start_rf_sig_transm:
            if (LO_active && SOE_active)
            {
turn_off_led_3();
                start_rfb_transm();
            }
            break;

        case T_close_valve:
            close_valve_0();
turn_on_led_2();
            break;

        case T_stop_rf_sig_transm:
            stop_rfb_transm();
turn_on_led_3();
            break;

        case T_disable_prs_sampling:
            disable_prs_sampling();
            stop_data_eeprom_logging();
            turn_off_5V();
            turn_off_12V();
            if (LO_active)
            {
                start_arm_polling();
turn_on_led_1();
            }
            break;

        case T_stop_arm_evaluation:
            stop_arm_polling();
            break;

        case T_stop_video_recording:
            stop_rec_cams();
turn_on_led_0();
            break;

        case T_stop_video_recording+10:
            turn_off_cams();
turn_off_led_0();
turn_off_led_1();
turn_off_led_2();
turn_off_led_3();
            break;
    }
    sei();
    SREG = tmp_sreg;
}

// ISR for Timer2 pulling cam_control lines low after >= 5s
ISR(TIMER2_OVF_vect)
{
    tmp_sreg = SREG;
    cli();
    timer_2_ref_counter++;
    if (timer_2_ref_counter > 300)
    {
        port_cam_0 &= ~(1<<CAM_0_control);
        port_cam_1 &= ~(1<<CAM_1_control);
        TCCR2 &= 0b11111000;        // stop Timer2
        if (cams_shutdown)
        {
            cams_shutdown = 0;
            usart_transmit_single_byte_message (CAM0_id, CAM0_ok);
            usart_transmit_single_byte_message (CAM1_id, CAM1_ok);
        }
    }
    sei();
    SREG = tmp_sreg;
}

// ISR for Timer2 pulling cam_control lines low after >= 100ms
ISR(TIMER2_COMP_vect)
{
    tmp_sreg = SREG;
    cli();
    timer_2_ref_counter++;
    if (timer_2_ref_counter > 10)
    {
        uint8_t stop_timer = 0;
        switch (timer_2_purpose)
        {
            case TOGGLE_CAM_RECORDING:
                port_cam_0 &= ~(1<<CAM_0_control);
                port_cam_1 &= ~(1<<CAM_1_control);
                TCCR2 &= 0b11111000;        // stop Timer2
                stop_timer++;
                break;
            case DEBOUNCE_RXSM_SIGNAL:
                if (LO_debounce_pending)
                {
                    if (!(pin_rxsm_lo & (1<<RXSM_LO)))
                    {
                        TCCR2 &= 0b11111000;        // stop Timer2
                        stop_timer++;
                        lo_approved();
                    }
                }
                else if (SOE_debounce_pending)
                {
                    if (!(pin_rxsm_soe & (1<<RXSM_SOE)))
                    {
                        TCCR2 &= 0b11111000;        // stop Timer2
                        stop_timer++;
                        soe_approved();
                    }
                }
                else if (SODS_debounce_pending)
                {
                    if (!(pin_rxsm_sods & (1<<RXSM_SODS)))
                    {
                        TCCR2 &= 0b11111000;        // stop Timer2
                        stop_timer++;
                        sods_approved();
                    }
                }
                break;
        }
        if (stop_timer)
        {
            if (cams_shutdown)  // if there is a shutdown request of RXSM
            {
                cams_shutdown_timeref = timeline_counter+3; // wait 3 s, then shutdown cams (inside timeline)
            }
        }
        else
        {
            if (timer_2_ref_counter > 50)
            {
                TCCR2 &= 0b11111000;        // stop Timer2
                SODS_debounce_pending = 0;
                LO_debounce_pending = 0;
                SOE_debounce_pending = 0;
            }
        }
    }
    sei();
    SREG = tmp_sreg;
}

//--------------------------------------------------------------------------------------------------


int main(void)
{

    // inputs
    dir_prs_sens    &= ~(1<<PRS_sensor_0);
    dir_prs_sens    &= ~(1<<PRS_sensor_1);
    dir_arm_sens    &= ~(1<<ARM_sensor);
    dir_tmp_alrm    &= ~(1<<TMP_alarm);
    dir_cam_0       &= ~(1<<CAM_0_control);
    dir_cam_1       &= ~(1<<CAM_1_control);
    dir_rxsm_soe    &= ~(1<<RXSM_SOE);
    dir_rxsm_sods   &= ~(1<<RXSM_SODS);
    dir_rxsm_lo     &= ~(1<<RXSM_LO);

    // outputs
    dir_prs_valv    |= (1<<PRS_valve_0) | (1<<PRS_valve_1);
    dir_rf_board    |= (1<<RFB_silence) | (1<<RFB_transmit);
    dir_voltage_en  |= (1<<P_5V_enable) | (1<<P_12V_enable);
    turn_off_5V();
    turn_off_12V();
    dir_dbg_leds    |= (1<<debugLED_0)  | (1<<debugLED_1) | (1<<debugLED_2) | (1<<debugLED_3);
    dir_cam_0       |= (1<<CAM_0_control);
    dir_cam_1       |= (1<<CAM_1_control);

    // external interrupt configs
    MCUCR  &= ~(1<<ISC10) & ~(1<<ISC00);        // int0 and int1 at falling edge
    MCUCR  |=  (1<<ISC11) |  (1<<ISC01);        // int0 and int1 at falling edge
    MCUCSR &= ~(1<<ISC2);                       // int2 at falling edge
    GICR   |=  (1<<INT0)|(1<<INT1)|(1<<INT2);   // activate external interrupts int0,1,2

    // USART initialization (RXSM)
    UBRRL  = 0x17;  // setting UBRR value to 23 for baudrate of 38.4k at 14.7456MHz
    UCSRB |= (1<<RXCIE)|(1<<RXEN)|(1<<TXEN);    // activate interrupt on receive, enable transceiver

    // TWI initialization (temperature)
    TWBR = 0x80;        // TWI clock speed to 54,2kHz
    TWCR |= (1<<TWEN);  // enable TWI

    // ADC initialization (pressure)
    ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); // set ADC prescaler to 128 for 115,2kHz clock
    
    // Timer0
    TCCR0   |= (1<<WGM01);      // CTC-Mode on = clear Timer on compare match
    OCR0     = 0xEF;            // compare value = 239 to generate 60Hz
    TIFR    |= (1<<OCF0);       // clear pending output compare interrupts
    TIMSK   |= (1<<OCIE0);      // enable Timer0 output compare interrupt

    // Timer1 for seconds clock
    TCCR1B  |= (1<<WGM12);      // CTC-Mode on = clear Timer on compare match
    OCR1AH   = 0x38;            // compare value = 14399, high byte
    OCR1AL   = 0x3F;            // compare value = 14399, low byte  to generate 1Hz
    TIFR    |= (1<<OCF1A);      // clear pending output compare interrupts
    TIMSK   |= (1<<OCIE1A);     // enable Timer1 output compare A interrupt

    // Timer2 for camera control uses
    OCR2     = 0x8F;                    // compare value = 143 to generate 100 Hz 
    TIFR    |= (1<<TOV2)  | (1<<OCF2);  // clear pending overflow and compare match interrupts
    TIMSK   |= (1<<TOIE2) | (1<<OCIE2); // enable Timer2 overflow and compare match interrupt 


    sei();

    turn_on_cams();

    turn_on_led_0();
    turn_on_led_1();
    turn_on_led_2();
    turn_on_led_3();

    while (1)
    {

    }

    return 1;
}
