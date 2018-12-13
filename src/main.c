/**
 **********************************************************************
 * @file    main.c
 * @mainpage
 * @author  Kevin Andoni, Engin Bajrami
 * @version V2.0
 * @date    Dec 13, 2018
 * @brief   Smart Incubator consisting of PIR, Active Motion and DHT12 Sensors
 **********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "settings.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "twi.h"
#include "uart.h"
#include <util/delay.h>

/* Constants and macros ------------------------------------------------------*/
/**
 *  @brief Standard hatching temperature of a chicken egg
 */
#define HATCHING_TEMP 38
/**
 *  @brief The upper boundary for a standard chicken egg to hatch
 */
#define UPPER_TEMPERATURE_LIMIT 40
/**
 *  @brief The lower boundary for a standard chicken egg to hatch
 */
#define LOWER_TEMPERATURE_LIMIT 36
/**
 *  @brief Communication speed of the AVR
 */
#define UART_BAUD_RATE 9600
/**
 *  @brief Address of the DHT12 Sensor
 */
#define DHT12_ADDRESS 0x5c

/* Structure definitions -----------------------------------------------------*/
/**
 *  @brief Save the received values of the DHT12 Sensor
 */
struct values{
    uint8_t humidity_integer;
    uint8_t humidity_decimal;
    uint8_t temperature_integer;
    uint8_t temperature_decimal;
};
struct values DHT12_values;

/**
 *  @brief Save the received values of the DHT12 Sensor
 */
void setup(void);
/**
 *  @brief Save the received values of the DHT12 Sensor
 */
void check_motion();
/**
 *  @brief Save the received values of the DHT12 Sensor
 */
void fsm_twi_scanner(void);

typedef enum {
    IDLE_STATE = 1,
    HUMIDITY_STATE,
    TEMPERATURE_STATE,
    RELAY_STATE,

} state_t;

state_t twi_state = IDLE_STATE;

unsigned int PIR_sensor_state	= 1;

/* Functions -----------------------------------------------------------------*/
/**
  * @brief Main function.
  */
int main(void){
    /* Initialize the setup */
    setup();
    /* Enable all interrupts (Set Enable Interrupt)*/
    sei();

    /* Wait for interrupts while waitin for a condition state change*/
    while (!PIR_sensor_state) {
        /* Check constantly for motion */
        check_motion();
    }
    /* Job done! */
    uart_puts("Congratulations! Egg successfully hatched :)");
    return 0;
}

/**
  * @brief Initialize the entire setup
  */
void setup(void){
    /* Timer/Counter1: update FSM state */
    /* Clock prescaler 64 => overflows every 262 ms */
    TCCR1B |= _BV(CS11) | _BV(CS10);
    /* Overflow interrupt enable */
    TIMSK1 |= _BV(TOIE1);
    /* Enable Pin Change Interrupt 0 */
    PCICR |= _BV(PCIE0);
    /* Enable pin change interrupt at pin 11 */
    PCMSK0 |= _BV(PB3);
    /* Set input pin 11 (PB3) - PIR */
    DDRB &= ~_BV(PB3);
    /* Set output pin 12 (PB4) - Alarm */
    DDRD |= _BV(PB4);
    /* Set output pin 13 (PB5) - Relay */
    DDRB |=_BV(PB5);
    /* Set current state of the PIR Sensor */
    PIR_sensor_state = 0;
    /* Initialize uart with the selected Baud Rate */
    uart_init(UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU));
    /* Initialize Two Wire Communication */
    twi_init();
}

/**
  * @brief Set up the PIR Sensor to change state if motion is detected
  */
void check_motion(void){

    /* Save new value if motion detected */
    int new_sensor_value = 0;
    /* Save previous value */
    int old_sensor_value = 0;

    /* Save vlaue */
    new_sensor_value = PINB & (1 << PORTB3);

    if((new_sensor_value > old_sensor_value) && (!PIR_sensor_state)){
      /* Change global state variable */
      PIR_sensor_state = 1;
    }
}

/**
  * @brief Timer 1 based on the overflow vector, to run peridically the method
  */
ISR(TIMER1_OVF_vect)
{
    /* Final State Machine to heat the egg constantly */
    fsm_twi_scanner();
}

/**
  * @brief Active Buzzer to take charge of the Alarm system
  */
void alarm(int active_time)
{   /* Here three different types of alarms defined based on duration */

    for(int i = 0; i  < active_time; i++){  //Short pulses
      /* Short alarm */
      if(active_time <= 1000){
        PORTD &= ~_BV(PD3);
        _delay_ms(5);
        PORTD |= _BV(PB3);
        _delay_ms(50);
      }
      /* Long alarm */
      else if((active_time > 1000)&&(active_time < 5000)){ //Long pulses
        PORTD &= ~_BV(PD3);
        _delay_ms(5);
        PORTD |= _BV(PB3);
        _delay_ms(200);
      }
      /* Malfunction alarm */
      else{
        PORTD |= _BV(PB3);        //All time active
      }
    }
}


/**
 *  @brief Final State Machine to peridically monitor and set up accordingly
 *         the right conditions for an egg to hatch
 */
void fsm_twi_scanner(void)
{
    /* Save temporary the status of the twi sensor (DHT12) */
    uint8_t twi_status;

    switch (twi_state) {
        /* First step of FSM, will notify that the system is up and running */
        case IDLE_STATE:
            uart_puts("System is idle!");
            alarm(500);
            twi_state = HUMIDITY_STATE;
            break;

        /* Second step of FSM, will start to read humidity values of the sorrounding of the egg */
        case HUMIDITY_STATE:
            uart_puts("Reading humidity values...");
            alarm(200);
            twi_status = twi_start((DHT12_ADDRESS<<1) + TWI_WRITE);

            if (twi_status==0){

                twi_write (0x00);
                twi_stop ();
                twi_start((DHT12_ADDRESS<<1) + TWI_READ);
                DHT12_values.humidity_integer = twi_read_ack();
                DHT12_values.humidity_decimal = twi_read_nack();
                twi_stop ();
                twi_state = TEMPERATURE_STATE;
            }
            else{
                uart_puts("Error reading humidity!");
                alarm(3000);
                twi_state = IDLE_STATE;
            }
            break;
        /* Third step of FSM, will start to read humidity values of the sorrounding of the egg */
        case TEMPERATURE_STATE:
              uart_puts("Reading temperature values...");
              alarm(200);
              twi_status = twi_start((DHT12_ADDRESS<<1) + TWI_WRITE);
              if (twi_status==0){
              twi_write (0x02);
              twi_stop ();
              twi_start((DHT12_ADDRESS<<1) + TWI_READ);
              DHT12_values.temperature_integer = twi_read_ack();
              DHT12_values.temperature_decimal = twi_read_nack();
              twi_stop ();
              twi_state = RELAY_STATE;
              }
              else{
                  twi_state = IDLE_STATE;
                  uart_puts("Humidity reading is correct, but temperature failed!\n");
                  uart_puts("DHT Sensor might be damaged!!");
                  alarm(6000);
              }
              break;
        /* Final step of FSM, will control the heating light accorndingly */
        case RELAY_STATE:

            if((DHT12_values.temperature_integer < UPPER_TEMPERATURE_LIMIT)&&(DHT12_values.temperature_integer > LOWER_TEMPERATURE_LIMIT)){
                alarm(5);
                PORTB |= _BV(PB5);
              }
            else{
                alarm(10);
                PORTB &= ~_BV(PB5);
              }

            twi_state = IDLE_STATE;
            break;

        /* Stay in IDLE state by default */
        default:
            twi_state = IDLE_STATE;
        }

}
/* END OF FILE ****************************************************************/
