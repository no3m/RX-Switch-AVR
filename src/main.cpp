/*
 *  RX Switching Controller
 *  NO3M, E. Tichansky
 *
 *  v2021.A
 *
 *  Note: will not prevent invalid states due to A&B | C&D selection
 */
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include "Messenger.h"
#include "uart.h"

//#define DEBUG
#define OUTPUTS_ENABLE
#define UART_BAUD_RATE 38400

const uint8_t radios     =  4;
const uint8_t nBytes     =  5;
const uint8_t ssPin      = 10;
const uint8_t oePin      =  9;
const uint8_t rs485TxPin =  2;

int antenna[(radios * 2) + 1];
uint8_t busData[nBytes];

uint8_t radios5_8 = 0;
uint8_t ants_A    = 0;
uint8_t ants_B    = 0;
bool    mode2x4   = false;

Messenger message = Messenger();

void processMessage();
void processData(int,int);
void buildData();
void busWrite();
void dumpConfig();
void toggle_relays();

int main (void)
{
  // rs485 rx/tx pin
  PORTD &= ~(1 << PD2); // assert low for rx
  DDRD  |=  (1 << PD2); // output

  // config pins
  DDRB  &=  ~(1<<PB0);                                               // inputs
  PORTB |=   (1<<PB0);                                               // pullup
  DDRC  &= ~((1<<PC0)|(1<<PC1)|(1<<PC2)|(1<<PC3)|(1<<PC4)|(1<<PC5)); // inputs
  PORTC |=   (1<<PC0)|(1<<PC1)|(1<<PC2)|(1<<PC3)|(1<<PC4)|(1<<PC5);  // pullup
  DDRD  &= ~((1<<PD3)|(1<<PD4)|(1<<PD5)|(1<<PD6)|(1<<PD7));          // inputs
  PORTD |=   (1<<PD3)|(1<<PD4)|(1<<PD5)|(1<<PD6)|(1<<PD7);           // pullup

  // SPI setup
  PORTB |= (1<<PB2);                   // SS/RCK high
  DDRB  |= (1<<PB2)|(1<<PB3)|(1<<PB5); // SS, MOSI, SCK outputs
  SPCR   = (1<<SPE)|(1<<MSTR);         // SPI enable, Master
                                       // Defaults: MSBfirst, Mode0, F_CPU/4

  uart0_init(UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU));
  message.attach(processMessage);

  /**
   * Get configuration, jumpers active low
   * NOTES:
   * 1. More than one ants_A selection forces 2x4 MODE
   * 2. ants_A selection will override conflicting ants_B in 2x4 MODE
   * 3. No selections or only ants_B 0-0 will result in unresponsive switch
   */
  radios5_8 = ((~PIND) & (1<<PD3)) >> PD3;
  ants_A    = ((~PIND) & ((1<<PD4)|(1<<PD5)|(1<<PD6)|(1<<PD7))) >> PD4;
  ants_A   |= ((~PINB) & (1<<PB0)) << PD4;
  ants_B    = ((~PINC) & ((1<<PC1)|(1<<PC2)|(1<<PC3)|(1<<PC4)|(1<<PC5))) >> PC1;
  mode2x4   = ((~PINC) & (1<<PC0)) || ants_B || (ants_A & (ants_A-1));

  memset(antenna, 0, sizeof(antenna));
  memset(busData, 0, sizeof(busData));
  // clear relay driver registers before enabling outputs
  busWrite();
  // TPIC6C596 /G (OE)
  PORTB &= ~(1<<PB1); // assert low
  DDRB  |=  (1<<PB1); // output

  while(true) {
    while (uart0_available()) message.process(uart0_getc());
  }

  return 0;
}

void processMessage ()
{
  while ( message.available() ) {
    if ( message.checkString((char *)"DATA") || message.checkString((char *)"DAT")) {
      message.readInt();                  // address
      int msgRadio   = message.readInt(); // radio
      message.readInt();                  // band
      message.readInt();                  // bearing
      int msgAntenna = message.readInt(); // antenna
      processData(msgRadio, msgAntenna);
    } else if (message.checkString((char *)"CFG")) {
      dumpConfig();
    } else if (message.checkString((char *)"RLY")) { // cycle relays
      toggle_relays();
      buildData();
      busWrite();
    } else if (message.checkString((char *)"RST")) { // soft reset
      memset(antenna, 0, sizeof(antenna));
      memset(busData, 0, sizeof(busData));
      busWrite();
    } else {
      message.readInt(); // discard extra data (virt ant, gain, HPF, BPF)
    }
  }
}

void processData(int msgRadio, int msgAntenna)
{
  if (msgRadio > radios5_8*radios && msgRadio <= ((radios5_8*radios)+radios)) { // radio in our range
    if ( antenna[msgRadio] != msgAntenna ) { // update only if antenna changed
      antenna[msgRadio] = msgAntenna;
      buildData();
      busWrite();
    }
  }
}

void buildData ()
{
  memset(busData, 0, sizeof(busData));
  for (uint8_t radio = (radios5_8*radios) + 1; radio <= (radios5_8*radios)+radios; ++radio) { // loop through our radios
    uint8_t _radio_idx = (radio - 1) % radios; // zero index radio number

    if ( mode2x4 ) { // 2x4 mode
      if ( ( ants_A&1  && antenna[radio] >= 1  && antenna[radio] <= 8  ) || // antenna in our range
           ( ants_A&2  && antenna[radio] >= 9  && antenna[radio] <= 16 ) || // port 1
           ( ants_A&4  && antenna[radio] >= 17 && antenna[radio] <= 24 ) ||
           ( ants_A&8  && antenna[radio] >= 25 && antenna[radio] <= 32 ) ||
           ( ants_A&16 && antenna[radio] >= 33 && antenna[radio] <= 40 )
      ) {
        busData[nBytes-1] |= (1 << _radio_idx); // activate radio's buss relay
        busData[_radio_idx] |= (1 << ((_radio_idx / 2) * 2)); // bit shift 0 (A,B) or 2 (C,D)
      }
      else
      if ( ( ants_B&1  && antenna[radio] >= 1  && antenna[radio] <= 8  ) || // antenna in our range
           ( ants_B&2  && antenna[radio] >= 9  && antenna[radio] <= 16 ) || // port 2
           ( ants_B&4  && antenna[radio] >= 17 && antenna[radio] <= 24 ) ||
           ( ants_B&8  && antenna[radio] >= 25 && antenna[radio] <= 32 ) ||
           ( ants_B&16 && antenna[radio] >= 33 && antenna[radio] <= 40 )
      ) {
        busData[nBytes-1] |= (1 << _radio_idx);
        busData[_radio_idx] |= (1 << (((_radio_idx / 2) * 2) + 4 )); // bit shift 4 (A,B) or 6 (C,D)
      }
    }
    else { // 8x4 mode
      if ( ( ants_A&1  && antenna[radio] >= 1  && antenna[radio] <= 8  ) || // antenna in our range
           ( ants_A&2  && antenna[radio] >= 9  && antenna[radio] <= 16 ) ||
           ( ants_A&4  && antenna[radio] >= 17 && antenna[radio] <= 24 ) ||
           ( ants_A&8  && antenna[radio] >= 25 && antenna[radio] <= 32 ) ||
           ( ants_A&16 && antenna[radio] >= 33 && antenna[radio] <= 40 )
      ) {
        busData[nBytes-1] |= (1 << _radio_idx); // active radio's buss relay
        uint8_t _antenna_idx = (antenna[radio] - 1) % 8; // map antenna number to port number
        uint8_t _byte_idx = _antenna_idx / 2; // find data byte number for this antenna
        busData[_byte_idx] |= (1 << (((_antenna_idx % 2) * 4) + ((_radio_idx / 2) * 2))); // (A,B) or (C,D) antenna enable
        if (_radio_idx % 2) { // only if radio B (1) or D (3)
          busData[_byte_idx] |= (1 << (((_antenna_idx % 2) * 4) + _radio_idx )); // B or D active
        }
      }
    }
  }
}

void busWrite ()
{
#ifdef OUTPUTS_ENABLE
   PORTB &= ~(1<<PB2); // RCK assert low
#endif
   for (int8_t i = nBytes-1; i >= 0; --i) {
#ifdef OUTPUTS_ENABLE
     SPDR = busData[i];
     while (!(SPSR & (1<<SPIF))) ;
#endif
#ifdef DEBUG
     char s[9];
     itoa(busData[i], s, 2);
     uart0_puts(s);
     if (i) uart0_puts("|");
      else uart0_puts("\r\n");
#endif
   }
#ifdef OUTPUTS_ENABLE
   PORTB |= (1<<PB2); // RCK assert high
#endif
}

void dumpConfig ()
{
  char s[2];
  uart0_puts("Radios: ");
  itoa((radios5_8*radios) + 1, s, 10);
  uart0_puts(s);
  uart0_puts("-");
  itoa((radios5_8*radios)+radios, s, 10);
  uart0_puts(s);
  uart0_puts("\r\n");
  uart0_puts("Mode: ");
  if (mode2x4)   uart0_puts("2x4\r\n");
   else uart0_puts("8x4\r\n");
  uart0_puts("ants_A: ");
  if (ants_A&1)  uart0_puts("1-8 ");
  if (ants_A&2)  uart0_puts("9-16 ");
  if (ants_A&4)  uart0_puts("17-24 ");
  if (ants_A&8)  uart0_puts("25-32 ");
  if (ants_A&16) uart0_puts("33-40");
  uart0_puts("\r\n");
  uart0_puts("ants_B: ");
  if (ants_B&1)  uart0_puts("1-8 ");
  if (ants_B&2)  uart0_puts("9-16 ");
  if (ants_B&4)  uart0_puts("17-24 ");
  if (ants_B&8)  uart0_puts("25-32 ");
  if (ants_B&16) uart0_puts("33-40");
  uart0_puts("\r\n");
}

void toggle_relays ()
{
  memset(busData, 0, sizeof(busData));
  for (uint8_t i = 0; i < nBytes; ++i) {
    for (uint8_t j = 0; j < 8; ++j) {
      busData[i] = (1<<j);
      busWrite();
      _delay_ms(10); // G5V-1 5ms pullin
      busData[i] = 0;
    }
  }
  busWrite();
}
