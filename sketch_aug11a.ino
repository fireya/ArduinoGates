#include <Automaton.h>
#include <RCSwitch.h>
#include <Array.h>
#include "ACS712.h"

class Atm_leaf : public Machine {

  public:
    Atm_leaf( void ) : Machine() {};

    short pin_open;
    short pin_close;

    enum { IDLE, OPENING, CLOSING, OPENED, CLOSED, OPEN_WAIT, CLOSE_WAIT }; // STATES
    enum { EVT_CLOSE_TIMER, EVT_OPEN_TIMER, EVT_CLOSED, EVT_OPENED, EVT_STOP, EVT_TOGGLE, ELSE }; // EVENTS
    enum { ENT_IDLE, ENT_OPENING, ENT_CLOSING, ENT_OPENED, ENT_CLOSED }; // ACTIONS


    float closeCurrent = 3.0;
    float standbyCurrent = 0.5;
    float openCurrent = 2.0;
    ACS712 *sensor;

    unsigned long last_moving_time;
    unsigned long last_closing_time;
    unsigned long last_opening_time;

    atm_timer_millis open_timer, close_timer;

    Atm_leaf & setOpenDelay(int seconds) {
      open_timer.set(seconds * 1000);
      return *this;
    }

    Atm_leaf & setCloseDelay(int seconds) {
      close_timer.set(seconds * 1000);
      return *this;
    }

    Atm_leaf & setCloseForce(float force) {
      this->closeCurrent = force;
      return *this;
    }

    Atm_leaf & setOpenForce(float force) {
      this->openCurrent = force;
      return *this;
    }

    Atm_leaf & begin( int openPin, int closePin, int acsPin)
    {
      // clang-format off
      const static state_t state_table[] PROGMEM = {
        /*                  ON_ENTER  ON_LOOP  ON_EXIT   EVT_CLOSE_TIMER  EVT_OPEN_TIMER  EVT_CLOSED  EVT_OPENED  EVT_STOP  EVT_TOGGLE  ELSE */
        /*       IDLE */    ENT_IDLE,      -1,      -1,               -1,             -1,         -1,         -1,       -1,  OPEN_WAIT,   -1,
        /*    OPENING */ ENT_OPENING,      -1,      -1,               -1,             -1,         -1,     OPENED,     IDLE,         -1,   -1,
        /*    CLOSING */ ENT_CLOSING,      -1,      -1,               -1,             -1,     CLOSED,         -1,     IDLE,         -1,   -1,
        /*     OPENED */  ENT_OPENED,      -1,      -1,               -1,             -1,         -1,         -1,       -1, CLOSE_WAIT,   -1,
        /*     CLOSED */  ENT_CLOSED,      -1,      -1,               -1,             -1,         -1,         -1,       -1,  OPEN_WAIT,   -1,
        /*  OPEN_WAIT */          -1,      -1,      -1,               -1,        OPENING,         -1,         -1,     IDLE,         -1,   -1,
        /* CLOSE_WAIT */          -1,      -1,      -1,          CLOSING,             -1,         -1,         -1,     IDLE,         -1,   -1,
      };
      // clang-format on
      Machine::begin( state_table, ELSE );
      sensor = new ACS712(ACS712_05B, acsPin);

      pin_open = openPin;
      pin_close = closePin;

      digitalWrite(pin_open, HIGH);
      digitalWrite(pin_close, HIGH);
      pinMode( pin_open, OUTPUT );
      pinMode( pin_close, OUTPUT );
      delay(100);
      sensor->calibrate();

      return *this;
    }

    unsigned long last_measure_time;
    float last_measured_value;
    float measureCurrent() {
      if (millis() - last_measure_time > 100)
      {
        last_measure_time = millis();
        last_measured_value = sensor->getCurrentDC();
        Serial.println(last_measured_value);
      }
      return last_measured_value;
    }

    int event( int id ) {
      switch ( id ) {
        case EVT_CLOSE_TIMER:
          return close_timer.expired(this);
        case EVT_OPEN_TIMER:
          return open_timer.expired(this);
        case EVT_CLOSED:
          if (abs(measureCurrent()) > standbyCurrent) {
            last_moving_time = millis();
          }
          if (abs(measureCurrent()) < closeCurrent) {
            last_closing_time = millis();
          }
          return (millis() - last_closing_time > 1000) || (millis() - last_moving_time > 4000);
        case EVT_OPENED:
          if (abs(measureCurrent()) > standbyCurrent) {
            last_moving_time = millis();
          }
          if (abs(measureCurrent()) < closeCurrent) {
            last_opening_time = millis();
          }
          return (millis() - last_opening_time > 1000) || (millis() - last_moving_time > 4000);
        case EVT_STOP:
          return 0;
        case EVT_TOGGLE:
          return 0;
      }
      return 0;
    }

    void action( int id ) {
      //Serial.println(id);
      switch ( id ) {
        case ENT_IDLE:
          digitalWrite(pin_open, HIGH);
          digitalWrite(pin_close, HIGH);
          return;
        case ENT_OPENING:
          digitalWrite(pin_open, HIGH);
          digitalWrite(pin_close, LOW);
          last_moving_time = millis();
          return;
        case ENT_CLOSING:
          digitalWrite(pin_open, LOW);
          digitalWrite(pin_close, HIGH);
          last_moving_time = millis();
          return;
        case ENT_OPENED:
          digitalWrite(pin_open, HIGH);
          digitalWrite(pin_close, HIGH);
          return;
        case ENT_CLOSED:
          digitalWrite(pin_open, HIGH);
          digitalWrite(pin_close, HIGH);
          return;
      }
    }
    Atm_leaf& closed() {
      trigger( EVT_CLOSED );
      return *this;
    }

    Atm_leaf& opened() {
      trigger( EVT_OPENED );
      return *this;
    }

    Atm_leaf& stop() {
      trigger( EVT_STOP );
      return *this;
    }

    Atm_leaf& toggle() {
      trigger( EVT_TOGGLE );
      return *this;
    }

    Atm_leaf& trace( Stream & stream ) {
      Machine::setTrace( &stream, atm_serial_debug::trace,
                         "LEAF\0EVT_STOP\0EVT_CLOSE\0EVT_OPEN\0ELSE\0IDLE\0OPENING\0CLOSING" );
      return *this;
    }
};

Atm_leaf rightLeafG;
Atm_leaf leftLeafG;
Atm_leaf rightLeafU;
Atm_leaf leftLeafU;
Atm_leaf rightLeafU2;
Atm_leaf leftLeafU2;

Atm_led led;
Atm_analog lightSensor;
uint16_t avgbuffer[10];

RCSwitch sender = RCSwitch();
RCSwitch receiver = RCSwitch();

void setup() {
  led.begin(4);
  lightSensor.begin( A14, 5000 )
  .average( avgbuffer, 10 )
  .onChange( []( int idx, int v, int up ) {
    if (v > 500 && up) led.on();
    else if (v < 300 && !up) led.off();
  });;

  rightLeafG
  .setOpenDelay(0)
  .setCloseDelay(7)
  .setCloseForce(3)
  .begin(22, 23, A8)
  .trace(Serial);

  leftLeafG
  .setOpenDelay(0)
  .setCloseDelay(0)
  .setCloseForce(3)
  .begin(24, 25, A9)
  .trace(Serial);

  rightLeafU
  .setOpenDelay(0)
  .setCloseDelay(5)
  .setCloseForce(3)
  .begin(26, 27, A10)
  .trace(Serial);

  leftLeafU
  .setOpenDelay(5)
  .setCloseDelay(0)
  .setCloseForce(3)
  .begin(28, 29, A11)
  .trace(Serial);

  rightLeafU2
  .setOpenDelay(5)
  .setCloseDelay(0)
  .setCloseForce(2)
  .begin(31, 30, A12)
  .trace(Serial);

  leftLeafU2
  .setOpenDelay(0)
  .setCloseDelay(5)
  .setCloseForce(2)
  .begin(32, 33, A13)
  .trace(Serial);

  receiver.enableReceive(digitalPinToInterrupt(21));
  sender.enableTransmit(6);
  sender.setProtocol(1);
  sender.setRepeatTransmit(5);

  Serial.begin(9600);
  Serial.print("Started");


  //timer.repeat(1);

}

void loop() {
  automaton.run();
  if (led.state() == led.ON && millis() - led.state_millis > 10000)
  {
    led.off();
  }
  if (receiver.available()) {
    unsigned long data = receiver.getReceivedValue();

    //Serial.println(data);
    //receiver.resetAvailable();
    //return;
    if (data == 31010101)
    {
      rightLeafG.toggle();
      leftLeafG.toggle();
      led.on();
    }
    if (data == 31010102)
    {
      rightLeafU.toggle();
      leftLeafU.toggle();
      led.on();
    }
    if (data == 31010103)
    {
      rightLeafU2.toggle();
      leftLeafU2.toggle();
      led.on();
    }
    if (data == 31010100)
    {
      rightLeafG.stop();
      leftLeafG.stop();
      rightLeafU.stop();
      leftLeafU.stop();
      rightLeafU2.stop();
      leftLeafU2.stop();
      led.on();
    }
    receiver.resetAvailable();

  }
  //Serial.println(analogRead(9));

  //sender.send(64902,16);
}
