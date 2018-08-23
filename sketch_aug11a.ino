#include <Automaton.h>
#include <RCSwitch.h>

class Atm_leaf : public Machine {

  public:
    Atm_leaf( void ) : Machine() {};

    short pin_open;
    short pin_close;

    enum { IDLE, OPENING, CLOSING, OPENED, CLOSED, OPEN_WAIT, CLOSE_WAIT }; // STATES
    enum { EVT_OPENING_TIMER, EVT_CLOSE_TIMER, EVT_OPEN_TIMER, EVT_CLOSED, EVT_OPENED, EVT_STOP, EVT_TOGGLE, ELSE }; // EVENTS
    enum { ENT_IDLE, ENT_OPENING, ENT_CLOSING, ENT_OPENED, ENT_CLOSED }; // ACTIONS

    Atm_comparator cmpE1;
    uint16_t avgbufferE1[10];

    Atm_comparator cmpE2;
    uint16_t avgbufferE2[10];

    Atm_comparator cmpI1;
    uint16_t avgbufferI1[10];

    Atm_comparator cmpI2;
    uint16_t avgbufferI2[10];

    uint16_t closeForce = 700;

    atm_timer_millis open_timer, close_timer, opening_timer;

    Atm_leaf & setOpenDelay(int seconds) {
      open_timer.set(seconds * 1000);
      return *this;
    }

    Atm_leaf & setCloseDelay(int seconds) {
      close_timer.set(seconds * 1000);
      return *this;
    }

    Atm_leaf & setCloseForce(uint16_t force) {
      this->closeForce = force;
      return *this;
    }

    Atm_leaf & begin( int openPin, int closePin, int acsPin)
    {
      // clang-format off
      const static state_t state_table[] PROGMEM = {
        /*                  ON_ENTER  ON_LOOP  ON_EXIT  EVT_OPENING_TIMER  EVT_CLOSE_TIMER  EVT_OPEN_TIMER  EVT_CLOSED  EVT_OPENED  EVT_STOP  EVT_TOGGLE  ELSE */
        /*       IDLE */    ENT_IDLE,      -1,      -1,                -1,              -1,             -1,         -1,         -1,       -1,  OPEN_WAIT,   -1,
        /*    OPENING */ ENT_OPENING,      -1,      -1,            OPENED,              -1,             -1,         -1,     OPENED,     IDLE,         -1,   -1,
        /*    CLOSING */ ENT_CLOSING,      -1,      -1,                -1,              -1,             -1,     CLOSED,         -1,     IDLE,         -1,   -1,
        /*     OPENED */  ENT_OPENED,      -1,      -1,                -1,              -1,             -1,         -1,         -1,       -1, CLOSE_WAIT,   -1,
        /*     CLOSED */  ENT_CLOSED,      -1,      -1,                -1,              -1,             -1,         -1,         -1,       -1,    OPENING,   -1,
        /*  OPEN_WAIT */          -1,      -1,      -1,                -1,              -1,        OPENING,         -1,         -1,     IDLE,         -1,   -1,
        /* CLOSE_WAIT */          -1,      -1,      -1,                -1,         CLOSING,             -1,         -1,         -1,     IDLE,         -1,   -1,
      };
      // clang-format on
      Machine::begin( state_table, ELSE );
      opening_timer.set(60000);

      static uint16_t threshold_list_E1[] = { closeForce };

      cmpE1.begin( acsPin, 50 )
      .threshold( threshold_list_E1, sizeof( threshold_list_E1 ) )
      .average( avgbufferE1, sizeof( avgbufferE1 ) )
      .onChange(true, *this, EVT_CLOSED);

      static uint16_t threshold_list_E2[] = { 200 };

      cmpE2.begin( acsPin, 50 )
      .threshold( threshold_list_E2, sizeof( threshold_list_E2 ) )
      .average( avgbufferE2, sizeof( avgbufferE2 ) )
      .onChange(false, *this, EVT_STOP);

      static uint16_t threshold_list_I1[] = { 500 };

      cmpI1.begin( acsPin, 50 )
      .threshold( threshold_list_I1, sizeof( threshold_list_I1 ) )
      .average( avgbufferI1, sizeof( avgbufferI1 ) )
      .onChange(true, *this, EVT_OPENED);

      static uint16_t threshold_list_I2[] = { 530 };

      cmpI2.begin( acsPin, 50 )
      .threshold( threshold_list_I2, sizeof( threshold_list_I2 ) )
      .average( avgbufferI2, sizeof( avgbufferI2 ) )
      .onChange(false, *this, EVT_STOP);

      pin_open = openPin;
      pin_close = closePin;
      pinMode( pin_open, OUTPUT );
      pinMode( pin_close, OUTPUT );
      return *this;
    }

    int Atm_leaf::event( int id ) {
      switch ( id ) {
        case EVT_OPENING_TIMER:
          return opening_timer.expired(this);
        case EVT_CLOSE_TIMER:
          return close_timer.expired(this);
        case EVT_OPEN_TIMER:
          return open_timer.expired(this);
        case EVT_CLOSED:
          return 0;
        case EVT_OPENED:
          return 0;
        case EVT_STOP:
          return 0;
        case EVT_TOGGLE:
          return 0;
      }
      return 0;
    }

    void Atm_leaf::action( int id ) {
      //Serial.println(id);
      switch ( id ) {
        case ENT_IDLE:
          digitalWrite(pin_open, HIGH);
          digitalWrite(pin_close, HIGH);
          return;
        case ENT_OPENING:

          digitalWrite(pin_open, HIGH);
          digitalWrite(pin_close, LOW);
          return;
        case ENT_CLOSING:

          digitalWrite(pin_open, LOW);
          digitalWrite(pin_close, HIGH);
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
    Atm_leaf& Atm_leaf::closed() {
      trigger( EVT_CLOSED );
      return *this;
    }

    Atm_leaf& Atm_leaf::opened() {
      trigger( EVT_OPENED );
      return *this;
    }

    Atm_leaf& Atm_leaf::stop() {
      trigger( EVT_STOP );
      return *this;
    }

    Atm_leaf& Atm_leaf::toggle() {
      trigger( EVT_TOGGLE );
      return *this;
    }

    Atm_leaf& Atm_leaf::trace( Stream & stream ) {
      Machine::setTrace( &stream, atm_serial_debug::trace,
                         "LEAF\0EVT_STOP\0EVT_CLOSE\0EVT_OPEN\0ELSE\0IDLE\0OPENING\0CLOSING" );
      return *this;
    }
};

Atm_leaf rightLeafG;
Atm_leaf leftLeafG;
Atm_leaf rightLeafU;
Atm_leaf leftLeafU;
RCSwitch sender = RCSwitch();
RCSwitch receiver = RCSwitch();

void setup() {
  rightLeafG
  .setOpenDelay(0)
  .setCloseDelay(7)
  .setCloseForce(800)
  .begin(22, 23, A8)
  .trace(Serial);

  leftLeafG
  .setOpenDelay(0)
  .setCloseDelay(0)
  .setCloseForce(800)
  .begin(24, 25, A9)
  .trace(Serial);
  
  rightLeafU
  .setOpenDelay(0)
  .setCloseDelay(2)
  .setCloseForce(590)
  .begin(26, 27, A10)
  .trace(Serial);
  
  leftLeafU
  .setOpenDelay(5)
  .setCloseDelay(0)
  .setCloseForce(590)
  .begin(28, 29, A11)
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
  if (receiver.available()) {
    unsigned long data = receiver.getReceivedValue();

    Serial.print(data);
    if (data == 31010101)
    {
      rightLeafG.toggle();
      leftLeafG.toggle();
    }
    if (data == 31010102)
    {
      rightLeafU.toggle();
      leftLeafU.toggle();
    }
    if (data == 31010100)
    {
      rightLeafG.stop();
      leftLeafG.stop();
      rightLeafU.stop();
      leftLeafU.stop();
    }

    receiver.resetAvailable();
  }
  //Serial.println(analogRead(9));
  //delay(500);
}
