/*
  BitWire - Bitbanged TWI/I2C library
  Copyright (c) 2022 Robert M. Zhudetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  ...

  This library is a fork of the Wire library by Nicholas Zambetti, with
  modifications by Todd Krein, Chuck Todd, and Greyson Christoforo. It
  is meant to be a drop-in replacement for Wire to implement bitbanged
  I2C without a dependency on the Wire library itself and which can be
  used outside as well as in an arduino environment.
*/

extern "C" {
  #include <stdlib.h>
  #include <string.h>
  #include <inttypes.h>
}

#include <UMPins/UMPins.h>
#include <avr/interrupt.h>
#include <errno.h>
#include "timeutils.h"
#include "BitWire.h"

void (*resetFunc) (void) = 0; //declare reset function @ address 0

// Constructors ////////////////////////////////////////////////////////////////

BitWire::BitWire()
{
  _rxBuffer [BUFFER_LENGTH];
  _rxBufferIndex = 0;
  _rxBufferLength = 0;

  _address = 0;

  _targetAddress = 0;
  _txBuffer [BUFFER_LENGTH];
  _txBufferIndex = 0;
  _txBufferLength = 0;

  _clockPulseLengthNOPS = ((F_CPU / 1000) - 16) / 4;
  _timeout = 25000;

  _transmitting = 0;
  _user_onRequest = (void (*)(void))0;
  _user_onReceive = (void (*)(int))0;
}

// Public Methods //////////////////////////////////////////////////////////////

void BitWire::begin(pinid_t sda, pinid_t scl)
{
  _pin_sda = GetPinRef(sda);
  _pin_scl = GetPinRef(scl);

  /* The LOW output value in the registers won't get changed when switching the pins
   * to input. However, it's better to switch them to inputs first to avoid high current
   * bus contention as much as possible. Keeping the LOW value on the PORT register
   * will enable us to use external pullup registers (the PORT register controls the
   * pullups during input mode) to pull the lines high and switching a pin to
   * output will automatically pull the line low because the 0 value is still stored
   * in the register.
   */
  SetPinInput(_pin_sda);
  SetPinInput(_pin_scl);
  SetPinLow(_pin_sda);
  SetPinLow(_pin_scl);

  _rxBufferIndex = 0;
  _rxBufferLength = 0;

  _txBufferIndex = 0;
  _txBufferLength = 0;

  _slaveRxBufEnd = 0;
  _slaveRxBufBegin = 0;

  _slaveBusState = 0x01; // IDLE

  //twi_init();
  //twi_attachSlaveTxEvent(onRequestService); // default callback must exist
  //twi_attachSlaveRxEvent(onReceiveService); // default callback must exist
}

void BitWire::begin(pinid_t sda, pinid_t scl, uint8_t address)
{
  begin(sda, scl);
  _address = address;
}

void BitWire::begin(pinid_t sda, pinid_t scl, int address)
{
  begin(sda, scl, (uint8_t)address);
}

void BitWire::end(void)
{
  //twi_disable();
}

void BitWire::setClock(uint32_t clock)
{
  _clockPulseLengthNOPS = ((F_CPU / clock) - 16) / 4;
}

/***
 * Sets the TWI timeout.
 *
 * This limits the maximum time to wait for the TWI hardware. If more time passes, the bus is assumed
 * to have locked up (e.g. due to noise-induced glitches or faulty slaves) and the transaction is aborted.
 * Optionally, the TWI hardware is also reset, which can be required to allow subsequent transactions to
 * succeed in some cases (in particular when noise has made the TWI hardware think there is a second
 * master that has claimed the bus).
 *
 * When a timeout is triggered, a flag is set that can be queried with `getWireTimeoutFlag()` and is cleared
 * when `clearWireTimeoutFlag()` or `setWireTimeoutUs()` is called.
 *
 * Note that this timeout can also trigger while waiting for clock stretching or waiting for a second master
 * to complete its transaction. So make sure to adapt the timeout to accommodate for those cases if needed.
 * A typical timeout would be 25ms (which is the maximum clock stretching allowed by the SMBus protocol),
 * but (much) shorter values will usually also work.
 *
 * In the future, a timeout will be enabled by default, so if you require the timeout to be disabled, it is
 * recommended you disable it by default using `setWireTimeoutUs(0)`, even though that is currently
 * the default.
 *
 * @param timeout a timeout value in microseconds, if zero then timeout checking is disabled
 * @param reset_with_timeout if true then TWI interface will be automatically reset on timeout
 *                           if false then TWI interface will not be reset on timeout

 */
void BitWire::setWireTimeout(uint32_t timeout, bool reset_with_timeout)
{
  _timeout = timeout;
  _resetWithTimeout = reset_with_timeout;
}

/***
 * Returns the TWI timeout flag.
 *
 * @return true if timeout has occurred since the flag was last cleared.
 */
bool BitWire::getWireTimeoutFlag(void)
{
  return _seenTimeout;
}

/***
 * Clears the TWI timeout flag.
 */
void BitWire::clearWireTimeoutFlag(void)
{
  _seenTimeout = false;
}

uint8_t BitWire::requestFrom(uint8_t address, uint8_t quantity, uint32_t iaddress, uint8_t isize, uint8_t sendStop)
{
  if (isize > 0) {
    // send internal address; this mode allows sending a repeated start to access
    // some devices' internal registers. This function is executed by the hardware
    // TWI module on other processors (for example Due's TWI_IADR and TWI_MMR registers)

    beginTransmission(address);

    // the maximum size of internal address is 3 bytes
    if (isize > 3)
      isize = 3;

    // write internal register address - most significant byte first
    while (isize-- > 0)
      write((uint8_t)(iaddress >> (isize*8)));

    endTransmission(false);
  }

  // clamp to buffer length
  if (quantity > BUFFER_LENGTH)
    quantity = BUFFER_LENGTH;

  // perform blocking read into buffer
  uint8_t read = twiReadFrom(address, _rxBuffer, quantity, sendStop);

  // set rx buffer iterator vars
  _rxBufferIndex = 0;
  _rxBufferLength = read;

  return read;
}

uint8_t BitWire::requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop)
{
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint32_t)0, (uint8_t)0, (uint8_t)sendStop);
}

uint8_t BitWire::requestFrom(uint8_t address, uint8_t quantity)
{
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true);
}

uint8_t BitWire::requestFrom(int address, int quantity)
{
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true);
}

uint8_t BitWire::requestFrom(int address, int quantity, int sendStop)
{
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)sendStop);
}

void BitWire::beginTransmission(uint8_t address)
{
  // indicate that we are transmitting
  _transmitting = 1;
  // set address of targeted slave
  _targetAddress = address;
  // reset tx buffer iterator vars
  _txBufferIndex = 0;
  _txBufferLength = 0;
}

void BitWire::beginTransmission(int address)
{
  beginTransmission((uint8_t)address);
}

//
//	Originally, 'endTransmission' was an f(void) function.
//	It has been modified to take one parameter indicating
//	whether or not a STOP should be performed on the bus.
//	Calling endTransmission(false) allows a sketch to 
//	perform a repeated start. 
//
//	WARNING: Nothing in the library keeps track of whether
//	the bus tenure has been properly ended with a STOP. It
//	is very possible to leave the bus in a hung state if
//	no call to endTransmission(true) is made. Some I2C
//	devices will behave oddly if they do not see a STOP.
//
uint8_t BitWire::endTransmission(uint8_t sendStop)
{
  // transmit buffer (blocking)
  uint8_t ret = twiWriteTo(_targetAddress, _txBuffer, _txBufferLength, 1, sendStop);
  // reset tx buffer iterator vars
  _txBufferIndex = 0;
  _txBufferLength = 0;

  // indicate that we are done transmitting
  _transmitting = 0;
  return ret;
}

//	This provides backwards compatibility with the original
//	definition, and expected behaviour, of endTransmission
//
uint8_t BitWire::endTransmission(void)
{
  return endTransmission(true);
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
size_t BitWire::write(uint8_t data)
{
  // in master transmitter mode
  // don't bother if buffer is full
  if (_txBufferLength >= BUFFER_LENGTH) {
    setWriteError(ENOBUFS);
    return 0;
  }

  // put byte in tx buffer
  _txBuffer[_txBufferIndex] = data;
  ++_txBufferIndex;
  // update amount in buffer
  _txBufferLength = _txBufferIndex;

  return 1;
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
size_t BitWire::write(const uint8_t *data, size_t quantity)
{
  // in master transmitter mode
  for (size_t i = 0; i < quantity; ++i)
    if (write(data[i]) < 1)
    {
      _transmitting = 0;
      _txBufferLength = 0;
      _txBufferIndex = 0;
      return 0;
    }
  return quantity;
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int BitWire::available(void)
{
  return _rxBufferLength - _rxBufferIndex;
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int BitWire::read(void)
{
  int value = -1;

  // get each successive byte on each call
  if (_rxBufferIndex < _rxBufferLength) {
    value = _rxBuffer[_rxBufferIndex];
    ++_rxBufferIndex;
  }

  return value;
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int BitWire::peek(void)
{
  int value = -1;

  if (_rxBufferIndex < _rxBufferLength) {
    value = _rxBuffer[_rxBufferIndex];
  }

  return value;
}

void BitWire::flush(void)
{
  // XXX: to be implemented.
}

// behind the scenes function that is called when data is received
void BitWire::onReceiveService(volatile uint8_t* inBytes, int numBytes)
{
  // don't bother if user hasn't registered a callback
  if (!_user_onReceive)
    return;

  // don't bother if rx buffer is in use by a master requestFrom() op
  // i know this drops data, but it allows for slight stupidity
  // meaning, they may not have read all the master requestFrom() data yet
  if (_rxBufferIndex < _rxBufferLength)
    return;

  // copy twi rx buffer into local read buffer
  // this enables new reads to happen in parallel
  for (uint8_t i = 0; i < numBytes; ++i)
    _rxBuffer[i] = inBytes[i];

  // set rx iterator vars
  _rxBufferIndex = 0;
  _rxBufferLength = numBytes;
  // alert user program
  _user_onReceive(numBytes);
}

// behind the scenes function that is called when data is requested
void BitWire::onRequestService(void)
{
  // don't bother if user hasn't registered a callback
  if (!_user_onRequest)
    return;

  // reset tx buffer iterator vars
  // !!! this will kill any pending pre-master sendTo() activity
  _txBufferIndex = 0;
  _txBufferLength = 0;

  // alert user program
  _user_onRequest();
}

// sets function called on slave write
void BitWire::onReceive( void (*function)(int) )
{
  _user_onReceive = function;
}

// sets function called on slave read
void BitWire::onRequest( void (*function)(void) )
{
  _user_onRequest = function;
}

void BitWire::onAddressMatchEvent( void (*handler)(BitWire*, bool&, bool&) )
{
  _onAddressMatchEvent = handler;
}

void BitWire::delayNops(int nops)
{
  int i = nops / 2;
  while (i > 0)
  {
    __asm__ __volatile__ (
    "nop\n\t"
    "nop");
    i--;
  }
}

void BitWire::setWriteError(int32_t err)
{
  _lastError = err;
}

uint32_t BitWire::getLastError()
{
  return _lastError;
}

void BitWire::clearLastError()
{
  _lastError = 0;
}

bool BitWire::sendBusClear()
{
  /* From the NXP Semiconductors I2C-bus specification and user manual:
   * If the data line (SDA) is stuck LOW, the controller should send
   * nine clock pulses. The device that held the bus LOW should release
   * it sometime within those nine clocks. */

  // Just to make sure we're not holding SDA low because of some code bug..
  twiSDAHigh();

  for (int i = 0; i < 9; i++)
  {
    twiSCLHigh();
    delayNops(_clockPulseLengthNOPS);
    twiSCLLow();
    delayNops(_clockPulseLengthNOPS);
  }
  return ReadBidiPin(_pin_sda);
}

inline void BitWire::twiSCLHigh()
{
  SetPinInput(_pin_scl);
}

inline void BitWire::twiSCLLow()
{
  SetPinOutput(_pin_scl);
}

inline void BitWire::twiSDAHigh()
{
  SetPinInput(_pin_sda);
}

inline void BitWire::twiSDALow()
{
  SetPinOutput(_pin_sda);
}

inline bool BitWire::twiSDARead()
{
  return ReadPin(_pin_sda);
}

void BitWire::twiSendStart()
{
  // The clock line shouldn't be low at the moment unless it was left low
  // by some function, or there is another master on the bus or some other
  // (slave) device is keeping the line low.
  if (!ReadBidiPin(_pin_scl))
  {
    twiSCLHigh();
    if (!twiWaitBusIdle())
    {
      _seenTimeout = true;
      return;
    }
  }

  // SDA Going low while SCL is high is a start condition.
  twiSDALow();
  delayNops(_clockPulseLengthNOPS);
}

void BitWire::twiSendStop()
{
  // A stop condition is communicated by allowing the SDA line to go back
  // high while the SCL line is high. So first make sure the SCL line is High.
  if (!ReadBidiPin(_pin_scl))
  {
    twiSCLHigh();
    delayNops(_clockPulseLengthNOPS);
  }
  twiSDAHigh();
  delayNops(_clockPulseLengthNOPS);
}

void BitWire::twiSendByte(uint8_t data)
{
  twiSCLLow();
  delayNops(_clockPulseLengthNOPS);
  uint8_t i = 8;
  while (i > 0)
  {
    if ((data & 0x80) == 0x80)
      twiSDAHigh();
    else
      twiSDALow();

    delayNops(_clockPulseLengthNOPS);

    twiSCLHigh();
    delayNops(_clockPulseLengthNOPS);

    twiSCLLow();
    delayNops(_clockPulseLengthNOPS);

    i--;
    data = data << 1;
  }
  twiSDAHigh();
}

bool BitWire::twiReadByte(volatile uint8_t *buf)
{
  if (ReadBidiPin(_pin_scl))
  {
    twiSCLLow();
    delayNops(_clockPulseLengthNOPS);
  }
  if (!ReadBidiPin(_pin_sda))
  {
    twiSDAHigh();
    delayNops(_clockPulseLengthNOPS);
  }
  uint8_t rxByte = 0;

  twiSCLHigh();
  if (!twiWaitClockHigh(_timeout))
  {
    twiSCLHigh();
    delayNops(_clockPulseLengthNOPS);
    twiSDALow();
    delayNops(_clockPulseLengthNOPS);
    twiSDAHigh();
    _lastError = EBITWIRE_TIMEOUT;
    _seenTimeout = true;
    return false;
  }

  for (uint8_t bit = 0x80; bit > 0; bit = bit >> 1)
  {
    twiSCLHigh();
    if (ReadPin(_pin_sda))
      rxByte |= bit;
    delayNops(_clockPulseLengthNOPS);

    twiSCLLow();
    delayNops(_clockPulseLengthNOPS);
  }

  *buf = rxByte;
  return true;
}

bool BitWire::twiWaitBusIdle()
{
  delayNops(_clockPulseLengthNOPS);
  return true;
}

bool BitWire::twiWaitClockHigh(int32_t timeout)
{
  Stopwatch sw;

  sw.Start();
  while (!ReadBidiPin(_pin_scl))
  {
    if (sw.GetTime() > timeout)
      return false;
  }
  return true;
}

bool BitWire::twiReadAck()
{
  twiSCLHigh();
  delayNops(_clockPulseLengthNOPS);

  // Waiting for the SCL line to go high makes it possible for a slave device
  // to use clock stretching: if it needs more time to process input from the
  // master, the slave can keep the SCL line low, which tells the master the
  // slave device is busy processing data.
  if (!ReadBidiPin(_pin_scl))
  {
    twiSCLHigh();
    if (!twiWaitClockHigh(_timeout))
    {
      _seenTimeout = true;
      _lastError = EBITWIRE_TIMEOUT;
      return false;
    }
  }

  bool ack = !ReadPin(_pin_sda);
  delayNops(_clockPulseLengthNOPS);

  twiSCLLow();
  delayNops(_clockPulseLengthNOPS);

  return ack;
}

void BitWire::twiSendAck()
{
  twiSDALow();
  delayNops(_clockPulseLengthNOPS);

  twiSCLHigh();
  delayNops(_clockPulseLengthNOPS);

  twiSCLLow();
  delayNops(_clockPulseLengthNOPS);
}

uint8_t BitWire::twiReadFrom(uint8_t address, volatile uint8_t *rxBuffer, uint8_t quantity, uint8_t sendStop)
{
  twiSendStart();

  uint8_t a = (address << 1) | 0x1; // Set the read/write bit 1 to indicate a read action.
  twiSendByte(a);
  delayNops(_clockPulseLengthNOPS);

  bool ack = twiReadAck();
  if (!ack)
  {
    twiSCLHigh();
    delayNops(_clockPulseLengthNOPS);
    twiSDALow();
    delayNops(_clockPulseLengthNOPS);
    twiSDAHigh();
    _lastError = EBITWIRE_NODEV;
    return 0;
  }

  uint8_t i = 0;
  while (i < quantity)
  {
    if (!twiReadByte(rxBuffer + i))
      return i;

    delayNops(_clockPulseLengthNOPS);

    twiSendAck();
    i++;
  }

  if (sendStop)
    twiSendStop();

  return i;
}

uint8_t BitWire::twiWriteTo(uint8_t address, uint8_t* data, uint8_t length, uint8_t wait, uint8_t sendStop)
{
  //twiInterruptsOff();

  twiSendStart();

  uint8_t a = address << 1; // Leave the read/not write bit 0 to indicate a write action.
  twiSendByte(a);
  delayNops(_clockPulseLengthNOPS);

  bool ack = twiReadAck();
  if (!ack)
  {
    twiSCLHigh();
    delayNops(_clockPulseLengthNOPS);
    twiSDALow();
    delayNops(_clockPulseLengthNOPS);
    twiSDAHigh();
    _lastError = EBITWIRE_NODEV;
    return 0;
  }

  uint8_t i = 0;
  while (ack && (i < length))
  {
    twiSendByte(*(data + i));
    delayNops(_clockPulseLengthNOPS);

    bool ack = twiReadAck();
    if (ack)
      i++;
    else
    {
      twiSCLHigh();
      delayNops(_clockPulseLengthNOPS);
      twiSDALow();
      delayNops(_clockPulseLengthNOPS);
      twiSDAHigh();
      _lastError = EBITWIRE_NACK;
      return i;
    }

    if (_seenTimeout || (_lastError > 0))
    {
      if (_seenTimeout)
        _lastError = EBITWIRE_TIMEOUT;
      twiSCLHigh();
      delayNops(_clockPulseLengthNOPS);
      twiSDALow();
      delayNops(_clockPulseLengthNOPS);
      twiSDAHigh();
      return i;
    }
  }

  delayNops(_clockPulseLengthNOPS);

  if (sendStop)
  {
    twiSDALow();
    delayNops(_clockPulseLengthNOPS);
    twiSCLHigh();
    delayNops(_clockPulseLengthNOPS);
    twiSDAHigh();
    delayNops(_clockPulseLengthNOPS);
  }

  return length;
  //twiInterruptsOn();
}

uint8_t BitWire::twiTransmit(const uint8_t* data, uint8_t length)
{
  return 0;
}

void BitWire::onInterruptsOn( void (*handler)(BitWire*) )
{
  _user_onInterruptsOn = handler;
}

void BitWire::onInterruptsOff( void (*handler)(BitWire*) )
{
  _user_onInterruptsOff = handler;
}

void BitWire::twiInterruptsOff()
{
  if (_user_onInterruptsOff)
    _user_onInterruptsOff(this);
}

void BitWire::twiInterruptsOn()
{
  if (_user_onInterruptsOn)
    _user_onInterruptsOn(this);
}

#define BWI2CSTATE_BUSIDLE      0x01
#define BWI2CSTATE_BUSBUSY      0x02
#define BWI2CSTATE_Start        0x03
#define BWI2CSTATE_PreAddr6     0x04
#define BWI2CSTATE_Addr6        0x05
#define BWI2CSTATE_PreAddr5     0x06
#define BWI2CSTATE_Addr5        0x07
#define BWI2CSTATE_PreAddr4     0x08
#define BWI2CSTATE_Addr4        0x09
#define BWI2CSTATE_PreAddr3     0x0A
#define BWI2CSTATE_Addr3        0x0B
#define BWI2CSTATE_PreAddr2     0x0C
#define BWI2CSTATE_Addr2        0x0D
#define BWI2CSTATE_PreAddr1     0x0E
#define BWI2CSTATE_Addr1        0x0F
#define BWI2CSTATE_PreAddr0     0x10
#define BWI2CSTATE_Addr0        0x11
#define BWI2CSTATE_PreRW        0x12
#define BWI2CSTATE_ReadWrite    0x13
#define BWI2CSTATE_PreAddrAck   0x14
#define BWI2CSTATE_AddrWrAck    0x15
#define BWI2CSTATE_AddrRdAck    0x16
#define BWI2CSTATE_PreWrData7   0x17
#define BWI2CSTATE_WrData7      0x18
#define BWI2CSTATE_PreWrData6   0x19
#define BWI2CSTATE_WrData6      0x1A
#define BWI2CSTATE_PreWrData5   0x1B
#define BWI2CSTATE_WrData5      0x1C
#define BWI2CSTATE_PreWrData4   0x1D
#define BWI2CSTATE_WrData4      0x1E
#define BWI2CSTATE_PreWrData3   0x1F
#define BWI2CSTATE_WrData3      0x20
#define BWI2CSTATE_PreWrData2   0x21
#define BWI2CSTATE_WrData2      0x22
#define BWI2CSTATE_PreWrData1   0x23
#define BWI2CSTATE_WrData1      0x24
#define BWI2CSTATE_PreWrData0   0x25
#define BWI2CSTATE_WrData0      0x26
#define BWI2CSTATE_PreDataWrAck 0x27
#define BWI2CSTATE_DataWrAck    0x28
#define BWI2CSTATE_PreRdData7   0x29
#define BWI2CSTATE_RdData7      0x2A
#define BWI2CSTATE_PreRdData6   0x2B
#define BWI2CSTATE_RdData6      0x2C
#define BWI2CSTATE_PreRdData5   0x2D
#define BWI2CSTATE_RdData5      0x2E
#define BWI2CSTATE_PreRdData4   0x2F
#define BWI2CSTATE_RdData4      0x30
#define BWI2CSTATE_PreRdData3   0x31
#define BWI2CSTATE_RdData3      0x32
#define BWI2CSTATE_PreRdData2   0x33
#define BWI2CSTATE_RdData2      0x34
#define BWI2CSTATE_PreRdData1   0x35
#define BWI2CSTATE_RdData1      0x36
#define BWI2CSTATE_PreRdData0   0x37
#define BWI2CSTATE_RdData0      0x38
#define BWI2CSTATE_PreDataRdAck 0x39
#define BWI2CSTATE_DataRdAck    0x3A


void BitWire::ClearSlaveBuffers()
{
	for (uint8_t i = 0; i < BUFFER_LENGTH; i++)
    _slaveRxBuffer[i] = 0;
  _slaveRxBufEnd = 0;
  _slaveRxBufBegin = 0;
}

void BitWire::ClearSlaveByteBuffer()
{
	_slaveByteBuffer = 0;
}

bool BitWire::MatchReceivedAddress()
{
	PORTD = (PORTD & 0x3) | (_slaveByteBuffer << 2);
	PORTB &= 0xFE;
	return _slaveByteBuffer == _address;
}

void BitWire::StretchClock()
{
	twiSCLLow();
}

void BitWire::StoreAddressBit(uint8_t bitPos, bool bitValue)
{
	StoreDataBit(bitPos, bitValue);
}

void BitWire::StoreDataBit(uint8_t bitPos, bool bitValue)
{
	if (bitValue)
		_slaveByteBuffer |= _BV(bitPos);
	else
		_slaveByteBuffer &= ~_BV(bitPos);
}

bool BitWire::StoreDataByte()
{
  if (_slaveRxBufEnd >= BUFFER_LENGTH)
    return false;

	PORTD = (PORTD & 0x3) | (_slaveByteBuffer << 2);
	PORTB &= 0xFE;

  _slaveRxBuffer[_slaveRxBufEnd++] = _slaveByteBuffer;
  return true;
}

void BitWire::SlaveReceiveEvent()
{
  onReceiveService(_slaveRxBuffer, (uint8_t)(_slaveRxBufEnd - _slaveRxBufBegin));
  _slaveRxBufBegin = 0;
  _slaveRxBufEnd = 0;
}

void BitWire::StoreReadNotWriteBit(bool readNotWriteBit)
{
  _slaveReadNotWriteBit = readNotWriteBit;
}

void BitWire::AddressMatchEvent()
{
  if (_onAddressMatchEvent)
  {
    bool releaseClock = false;
    bool ack = false;
    _onAddressMatchEvent(this, releaseClock, ack);
    if (!releaseClock)
      return; // Handle continuing of communication from outside ISR *TODO*.

    if (!ack)
    {
      twiSDAHigh();
      delayNops(2);
      twiSCLHigh();
      SetNextState(BWI2CSTATE_BUSBUSY);
      return;
    }
  }

  twiSDALow();
  delayNops(2);
  twiSCLHigh();

  if (_slaveReadNotWriteBit)
  {
    onRequestService();
    CopySlaveTxBuffer();
  }

  if (_slaveReadNotWriteBit)
    SetNextState(BWI2CSTATE_AddrRdAck);
  else
    SetNextState(BWI2CSTATE_AddrWrAck);
}

void BitWire::CopySlaveTxBuffer()
{
  if (_txBufferLength == 0)
  {
    _slaveTxBufLength = 0;
    _slaveTxBufIndex = 0;
    return;
  }

  memcpy((void *)_slaveTxBuffer, (void *)_txBuffer, BUFFER_LENGTH);
  _slaveTxBufIndex = 0;
  _slaveTxBufLength = _txBufferLength;
}

bool BitWire::LoadSlaveTxByteBuffer()
{
  if (_slaveTxBufIndex >= _slaveTxBufLength)
    return false;

  _slaveByteBuffer = _slaveTxBuffer[_slaveTxBufIndex];
  return true;
}

void BitWire::SetSDAForSlaveTxBit(uint8_t bitPos)
{
  uint8_t bitValue = _BV(bitPos);
  if ((_slaveByteBuffer & bitValue) == bitValue)
    twiSDAHigh();
  else
    twiSDALow();
}

void BitWire::removeByteFromSlaveTxBuffer()
{
  _slaveTxBufIndex++;
  if (_slaveTxBufIndex == _slaveTxBufLength)
  {
    _slaveTxBufIndex = 0;
    _slaveTxBufLength = 0;
  }
}

void BitWire::SetNextState(uint8_t nextState)
{
  /* DEBUG */
  if ((nextState < 0x10) | (nextState > 0x20))
  {
    PORTD = (PORTD & 0x3) | (nextState << 2);
    PORTB |= 0x1;
    delayNops(10);
  }
  /* */

  _slaveBusState = nextState;
  switch (nextState)
  {
    // When BUSIDLE    enter { ClearSlaveBuffers() }
    case BWI2CSTATE_BUSIDLE:
      ClearSlaveBuffers();
      break;

    // When Start      enter { ClearByteBuffer() }
    case BWI2CSTATE_Start:
      ClearSlaveByteBuffer();
      break;

    // When PreAddrAck enter { if (MatchReceivedAddress()) { StretchClock(); AddressMatchEvent(); } else SetNextState(BUSBUSY); }
    case BWI2CSTATE_PreAddrAck:
      if (MatchReceivedAddress())
      {
        StretchClock();
        AddressMatchEvent();
      }
      else
        SetNextState(BWI2CSTATE_BUSBUSY);
      break;

    // When PreDataWrAck enter { StretchClock(); if (StoreDataByte()) SDALow(); ReleaseSCL(); }
    case BWI2CSTATE_PreDataWrAck:
      StretchClock();
      if (StoreDataByte())
        twiSDALow();
      else
        twiSDAHigh();

      delayNops(2);
      twiSCLHigh();
      break;

    // When PreRdData7 enter { if (LoadSlaveTxByteBuffer()) SetSDAForSlaveTXBit(7); else SetNextState(BUSBUSY); }
    case BWI2CSTATE_PreRdData7:
      if (LoadSlaveTxByteBuffer())
        SetSDAForSlaveTxBit(7);
      else
        SetNextState(BWI2CSTATE_BUSBUSY);
      break;
  }
}

void BitWire::HandleInterrupt()
{
  bool scl = ReadPin(_pin_scl);
  bool sda = ReadPin(_pin_sda);
  bool sclUp = scl && !_lastSCL;
  bool sclDn = !scl && _lastSCL;
  bool sdaUp = sda && !_lastSDA;
  bool sdaDn = !sda && _lastSDA;
  _lastSDA = sda;
  _lastSCL = scl;

  // Begin Bus State statemachine
  switch (_slaveBusState) {
    // When BUSIDLE    enter { ClearSlaveBuffers() }
    // When BUSIDLE    and * while SCL LO -> BUSBUSY
    // When BUSIDLE    and SDA Dn while SCL HI -> Start
    case BWI2CSTATE_BUSIDLE:
      if (!scl)
        SetNextState(BWI2CSTATE_BUSBUSY);
      else if (scl && sdaDn);
        SetNextState(BWI2CSTATE_Start);
      break;

    // When BUSBUSY    and SDA Up while SCL HI -> BUSIDLE // Stop condition
    // When BUSBUSY    and SDA Dn while SCL HI -> Start // Start condition
    case BWI2CSTATE_BUSBUSY:
      if (sdaUp && scl) // Stop condition
        SetNextState(BWI2CSTATE_BUSIDLE);
      else if (sdaDn && scl) // Start condition
        SetNextState(BWI2CSTATE_Start);
      break;

    // When Start      enter { ClearByteBuffer() }
    // When Start      and SDA Up -> BUSIDLE
    // When Start      and SCL Dn -> PreAddr6
    case BWI2CSTATE_Start:
      if (sdaUp)
        SetNextState(BWI2CSTATE_BUSIDLE);
      else if (sclDn)
        SetNextState(BWI2CSTATE_PreAddr6);
      break;

    // When PreAddr6   and SCL Up -> Addr6     { StoreAddressBit(6) }
    case BWI2CSTATE_PreAddr6:
      if (sclUp)
      {
        StoreAddressBit(6, sda);
        SetNextState(BWI2CSTATE_Addr6);
      }
      else if (sclDn)
        // This situation shouldn't occur. However, we're getting stuck here somehow..
        resetFunc();
      else if (scl)
        resetFunc();
      break;

    // When Addr6      and SDA Up -> BUSIDLE
    // When Addr6      and SCL Dn -> PreAddr5
    case BWI2CSTATE_Addr6:
      if (sdaUp)
        SetNextState(BWI2CSTATE_BUSIDLE);
      else if (sclDn)
        SetNextState(BWI2CSTATE_PreAddr5);
      break;

    // When PreAddr5   and SCL Up -> Addr5     { StoreAddressBit(5) }
    case BWI2CSTATE_PreAddr5:
      if (sclUp)
      {
        StoreAddressBit(5, sda);
        SetNextState(BWI2CSTATE_Addr5);
      }
      else if (sclDn)
        resetFunc();
      else if (scl)
        resetFunc();
      break;

    // When Addr5      and SDA Up -> BUSIDLE
    // when Addr5      and SCL Dn -> PreAddr4
    case BWI2CSTATE_Addr5:
      if (sdaUp)
        SetNextState(BWI2CSTATE_BUSIDLE);
      else if (sclDn)
        SetNextState(BWI2CSTATE_PreAddr4);
      break;

    // When PreAddr4   and SCL Up -> Addr4     { StoreAddressBit(4) }
    case BWI2CSTATE_PreAddr4:
      if (sclUp)
      {
        StoreAddressBit(4, sda);
        SetNextState(BWI2CSTATE_Addr4);
      }
      break;

    // When Addr4      and SDA Up -> BUSIDLE
    // When Addr4      and SCL Dn -> PreAddr3
    case BWI2CSTATE_Addr4:
      if (sdaUp)
        SetNextState(BWI2CSTATE_BUSIDLE);
      else if (sclDn)
        SetNextState(BWI2CSTATE_PreAddr3);
      break;

    // When PreAddr3   and SCL Up -> Addr3     { StoreAddressBit(3) }
    case BWI2CSTATE_PreAddr3:
      if (sclUp)
      {
        StoreAddressBit(3, sda);
        SetNextState(BWI2CSTATE_Addr3);
      }
      break;

    // When Addr3      and SDA Up -> BUSIDLE
    // When Addr3      and SCL Dn -> PreAddr2
    case BWI2CSTATE_Addr3:
      if (sdaUp)
        SetNextState(BWI2CSTATE_BUSIDLE);
      else if (sclDn)
        SetNextState(BWI2CSTATE_PreAddr2);
      break;

    // When PreAddr2   and SCL Up -> Addr2     { StoreAddressBit(2) }
    case BWI2CSTATE_PreAddr2:
      if (sclUp)
      {
        StoreAddressBit(2, sda);
        SetNextState(BWI2CSTATE_Addr2);
      }
      break;

    // When Addr2      and SDA Up -> BUSIDLE
    // When Addr2      and SCL Dn -> PreAddr1
    case BWI2CSTATE_Addr2:
      if (sdaUp)
        SetNextState(BWI2CSTATE_BUSIDLE);
      else if (sclDn)
        SetNextState(BWI2CSTATE_PreAddr1);
      break;

    // When PreAddr1   and SCL Up -> Addr1     { StoreAddressBit(1) }
    case BWI2CSTATE_PreAddr1:
      if (sclUp)
      {
        StoreAddressBit(1, sda);
        SetNextState(BWI2CSTATE_Addr1);
      }
      break;

    // When Addr1      and SDA Up -> BUSIDLE
    // When Addr1      and SCL Dn -> PreAddr0
    case BWI2CSTATE_Addr1:
      if (sdaUp)
        SetNextState(BWI2CSTATE_BUSIDLE);
      else if (sclDn)
        SetNextState(BWI2CSTATE_PreAddr0);
      break;

    // When PreAddr0   and SCL Up -> Addr0     { StoreAddressBit(0) }
    case BWI2CSTATE_PreAddr0:
      if (sclUp)
      {
        StoreAddressBit(0, sda);
        SetNextState(BWI2CSTATE_Addr0);
      }
      break;

    // When Addr0      and SDA Up -> BUSIDLE
    // When Addr0      and SCL Dn -> PreRW
    case BWI2CSTATE_Addr0:
      if (sdaUp)
        SetNextState(BWI2CSTATE_BUSIDLE);
      if (sclDn)
        SetNextState(BWI2CSTATE_PreRW);
      break;

    // When PreRW      and SCL Up -> ReadWrite { StoreReadNotWriteBit() }
    case BWI2CSTATE_PreRW:
      if (sclUp)
      {
        StoreReadNotWriteBit(sda);
        SetNextState(BWI2CSTATE_ReadWrite);
      }
      break;

    // When ReadWrite  and SDA Up -> BUSIDLE
    // When ReadWrite  and SCL Dn -> PreAddrAck
    case BWI2CSTATE_ReadWrite:
      if (sdaUp)
        SetNextState(BWI2CSTATE_BUSIDLE);
      else if (sclDn)
        SetNextState(BWI2CSTATE_PreAddrAck);
      break;

    // When PreAddrAck enter { if (MatchReceivedAddress()) { StretchClock(); AddressMatchEvent(); } else SetNextState(BUSBUSY); }
    case BWI2CSTATE_PreAddrAck:
      break;

    ////////////////////////////////////////////////////////////////////
    // Write mode states
    
    // When AddrWrAck  and SCL Dn -> PreWrData7   { twiSDAHigh(); }
    // When AddrWrAck  and SDA Up while SCL High -> BUSIDLE   { SlaveReceiveEvent(); }
    case BWI2CSTATE_AddrWrAck:
      if (sclDn)
      {
        twiSDAHigh();
        SetNextState(BWI2CSTATE_PreWrData7);
      }
      if (sdaUp and scl)
      {
        SlaveReceiveEvent();
        SetNextState(BWI2CSTATE_BUSIDLE);
      }
      break;

    // When PreWrData7 and SCL Up -> WrData7      { ClearByteBuffer(); StoreDataBit(7) }
    case BWI2CSTATE_PreWrData7:
      if (sclUp)
      {
        ClearSlaveByteBuffer();
        StoreDataBit(7, sda);
        SetNextState(BWI2CSTATE_WrData7);
      }
      break;

    // When WrData7    and SDA Up -> BUSIDLE      { SlaveReceiveEvent(); } // Stop condition
    // When WrData7    and SDA Dn -> Start        { SlaveReceiveEvent(); } // Restart condition
    // When WrData7    and SCL Dn -> PreWrData6
    case BWI2CSTATE_WrData7:
      if (sdaUp)
      {
        SlaveReceiveEvent();
        SetNextState(BWI2CSTATE_BUSIDLE);
      }
      else if (sdaDn)
      {
        SlaveReceiveEvent();
        SetNextState(BWI2CSTATE_Start);
      }
      else if (sclDn)
        SetNextState(BWI2CSTATE_PreWrData6);
      break;

    // When PreWrData6 and SCL Up -> WrData6      { StoreDataBit(6) }
    case BWI2CSTATE_PreWrData6:
      if (sclUp)
      {
        StoreDataBit(6, sda);
        SetNextState(BWI2CSTATE_WrData6);
      }
      break;

    // When WrData6    and SDA Up -> BUSIDLE
    // When WrData6    and SCL Dn -> PreWrData5
    case BWI2CSTATE_WrData6:
      if (sdaUp)
        SetNextState(BWI2CSTATE_BUSIDLE);
      else if (sclDn)
        SetNextState(BWI2CSTATE_PreWrData5);
      break;

    // When PreWrData5 and SCL Up -> WrData5      ( StoreDataBit(5) }
    case BWI2CSTATE_PreWrData5:
      if (sclUp)
      {
        StoreDataBit(5, sda);
        SetNextState(BWI2CSTATE_WrData5);
      }
      break;

    // When WrData5    and SDA Up -> BUSIDLE
    // When WrData5    and SCL Dn -> PreWrData4
    case BWI2CSTATE_WrData5:
      if (sdaUp)
        SetNextState(BWI2CSTATE_BUSIDLE);
      else if (sclDn)
        SetNextState(BWI2CSTATE_PreWrData4);
      break;

    // When PreWrData4 and SCL Up -> WrData4      { StoreDataBit(4) }
    case BWI2CSTATE_PreWrData4:
      if (sclUp)
      {
        StoreDataBit(4, sda);
        SetNextState(BWI2CSTATE_WrData4);
      }
      break;

    // When WrData4    and SDA Up -> BUSIDLE
    // When WrData4    and SCL Dn -> PreWrData3
    case BWI2CSTATE_WrData4:
      if (sdaUp)
        SetNextState(BWI2CSTATE_BUSIDLE);
      else if (sclDn)
        SetNextState(BWI2CSTATE_PreWrData3);
      break;

    // When PreWrData3 and SCL Up -> WrData3      { StoreDataBit(3) }
    case BWI2CSTATE_PreWrData3:
      if (sclUp)
      {
        StoreDataBit(3, sda);
        SetNextState(BWI2CSTATE_WrData3);
      }
      break;

    // When WrData3    and SDA Up -> BUSIDLE
    // When WrData3    and SCL Dn -> PreWrData2
    case BWI2CSTATE_WrData3:
      if (sdaUp)
        SetNextState(BWI2CSTATE_BUSIDLE);
      else if (sclDn)
        SetNextState(BWI2CSTATE_PreWrData2);
      break;

    // When PreWrData2 and SCL Up -> WrData2      { StoreDataBit(2) }
    case BWI2CSTATE_PreWrData2:
      if (sclUp)
      {
        StoreDataBit(2, sda);
        SetNextState(BWI2CSTATE_WrData2);
      }
      break;

    // When WrData2    and SDA Up -> BUSIDLE
    // When WrData2    and SCL Dn -> PreWrData1
    case BWI2CSTATE_WrData2:
      if (sdaUp)
        SetNextState(BWI2CSTATE_BUSIDLE);
      else if (sclDn)
        SetNextState(BWI2CSTATE_PreWrData1);
      break;

    // When PreWrData1 and SCL Up -> WrData1      { StoreDataBit(1) }
    case BWI2CSTATE_PreWrData1:
      if (sclUp)
      {
        StoreDataBit(1, sda);
        SetNextState(BWI2CSTATE_WrData1);
      }
      break;

    // When WrData1    and SDA Up -> BUSIDLE
    // When WrData1    and SCL Dn -> PreWrData0
    case BWI2CSTATE_WrData1:
      if (sdaUp)
        SetNextState(BWI2CSTATE_BUSIDLE);
      else if (sclDn)
        SetNextState(BWI2CSTATE_PreWrData0);
      break;

    // When PreWrData0 and SCL Up -> WrData0      { StoreDataBit(0) }
    case BWI2CSTATE_PreWrData0:
      if (sclUp)
      {
        StoreDataBit(0, sda);
        SetNextState(BWI2CSTATE_WrData0);
      }
      break;

    // When WrData0    and SDA Up -> BUSIDLE
    // When WrData0    and SCL Dn -> PreDataWrAck
    case BWI2CSTATE_WrData0:
      if (sdaUp)
        SetNextState(BWI2CSTATE_BUSIDLE);
      else if (sclDn)
        SetNextState(BWI2CSTATE_PreDataWrAck);
      break;

    // When PreDataWrAck enter { StretchClock(); if (StoreDataByte()) SDALow(); ReleaseSCL(); }
    // When PreDataWrAck and SCL Up -> DataWrAck
    case BWI2CSTATE_PreDataWrAck:
      if (sclUp)
        SetNextState(BWI2CSTATE_DataWrAck);
      break;

    // When DataWrAck  and SCL Dn -> PreWrData7   { RelaseSDA(); }
    case BWI2CSTATE_DataWrAck:
      if (sclDn)
      {
        twiSDAHigh();
        SetNextState(BWI2CSTATE_PreWrData7);
      }
      break;

    ////////////////////////////////////////////////////////////////////
    // Read mode states

    // When AddrRdAck  and SCL Dn -> PreRdData7   { twiSDAHigh(); }
    case BWI2CSTATE_AddrRdAck:
      if (sclDn)
      {
        twiSDAHigh();
        SetNextState(BWI2CSTATE_PreRdData7);
      }
      break;

    // When PreRdData7 enter { if (LoadSlaveTxByteBuffer()) SetSDAForSlaveTXBit(7); else SetNextState(BUSBUSY); }
    // When PreRdData7 and SCL Up -> RdData7
    case BWI2CSTATE_PreRdData7:
      if (sclUp)
        SetNextState(BWI2CSTATE_RdData7);
      break;

    // When RdData7    and SCL Dn -> PreRdData6   { SetSDAForSlaveTXBit(6); }
    case BWI2CSTATE_RdData7:
      if (sclDn)
      {
        SetSDAForSlaveTxBit(6);
        SetNextState(BWI2CSTATE_PreRdData6);
      }
      break;

    // When PreRdData6 and SCL Up -> RdData6
    case BWI2CSTATE_PreRdData6:
      if (sclUp)
        SetNextState(BWI2CSTATE_RdData6);
      break;
    
    // When RdData6    and SCL Dn -> PreRdData5   { SetSDAForSlaveTXBit(5); }
    case BWI2CSTATE_RdData6:
      if (sclDn)
      {
        SetSDAForSlaveTxBit(5);
        SetNextState(BWI2CSTATE_PreRdData5);
      }
      break;
    
    // When PreRdData5 and SCL Up -> RdData5
    case BWI2CSTATE_PreRdData5:
      if (sclUp)
        SetNextState(BWI2CSTATE_RdData5);
      break;
    
    // When RdData5    and SCL Dn -> PreRdData4   { SetSDAForSlaveTXBit(4); }
    case BWI2CSTATE_RdData5:
      if (sclDn)
      {
        SetSDAForSlaveTxBit(4);
        SetNextState(BWI2CSTATE_PreRdData4);
      }
      break;
    
    // When PreRdData4 and SCL Up -> RdData4
    case BWI2CSTATE_PreRdData4:
      if (sclUp)
        SetNextState(BWI2CSTATE_RdData4);
      break;
    
    // When RdData4    and SCL Dn -> PreRdData3   { SetSDAForSlaveTXBit(3); }
    case BWI2CSTATE_RdData4:
      if (sclDn)
      {
        SetSDAForSlaveTxBit(3);
        SetNextState(BWI2CSTATE_PreRdData3);
      }
      break;
    
    // When PreRdData3 and SCL Up -> RdData3
    case BWI2CSTATE_PreRdData3:
      if (sclUp)
        SetNextState(BWI2CSTATE_RdData3);
      break;
    
    // When RdData3    and SCL Dn -> PreRdData2   { SetSDAForSlaveTXBit(2); }
    case BWI2CSTATE_RdData3:
      if (sclDn)
      {
        SetSDAForSlaveTxBit(2);
        SetNextState(BWI2CSTATE_PreRdData2);
      }
      break;
    
    // When PreRdData2 and SCL Up -> RdData2
    case BWI2CSTATE_PreRdData2:
      if (sclUp)
        SetNextState(BWI2CSTATE_RdData2);
      break;
    
    // When RdData2    and SCL Dn -> PreRdData1   { SetSDAForSlaveTXBit(1); }
    case BWI2CSTATE_RdData2:
      if (sclDn)
      {
        SetSDAForSlaveTxBit(1);
        SetNextState(BWI2CSTATE_PreRdData1);
      }
      break;
    
    // When PreRdData1 and SCL Up -> RdData1
    case BWI2CSTATE_PreRdData1:
      if (sclUp)
        SetNextState(BWI2CSTATE_RdData1);
      break;
    
    // When RdData1    and SCL Dn -> PreRdData0   { SetSDAForSlaveTXBit(0); }
    case BWI2CSTATE_RdData1:
      if (sclDn)
      {
        SetSDAForSlaveTxBit(0);
        SetNextState(BWI2CSTATE_PreRdData0);
      }
      break;
    
    // When PreRdData0 and SCL Up -> RdData0
    case BWI2CSTATE_PreRdData0:
      if (sclUp)
        SetNextState(BWI2CSTATE_RdData0);
      break;
    
    // When RdData0    and SCL Dn -> PreDataRdAck { twiSDAHigh(); }
    case BWI2CSTATE_RdData0:
      if (sclDn)
      {
        twiSDAHigh();
        SetNextState(BWI2CSTATE_PreDataRdAck);
      }
      break;
    
    // When PreDataRdAck and SCL Up -> DataRdAck  { if (!sda) removeByteFromSlaveTxBuffer(); }
    case BWI2CSTATE_PreDataRdAck:
      if (sclUp)
      {
        if (!sda)
          removeByteFromSlaveTxBuffer();
        SetNextState(BWI2CSTATE_DataRdAck);
      }
      break;
    
    // When DataRdAck    and SCL Dn -> PreRdData7
    case BWI2CSTATE_DataRdAck:
      if (sclDn)
        SetNextState(BWI2CSTATE_PreRdData7);
      break;

    default:
      resetFunc();
  }
}

/* I2C state machine spec:
 * When BUSIDLE    enter { ClearSlaveBuffers() }
 * When BUSIDLE    and * while SCL LO -> BUSBUSY
 * When BUSIDLE    and SDA Dn while SCL HI -> Start
 * 
 * // BUSBUSY is a state where we know the bus is busy but the data is
 * // not for us, so we can ignore it. We only need to keep track of the
 * // stop and (re)start conditions here.
 * When BUSBUSY    and SDA Up while SCL HI -> BUSIDLE // Stop condition
 * When BUSBUSY    and SDA Dn while SCL HI -> Start // Start condition
 * 
 * When Start      enter { ClearByteBuffer() }
 * When Start      and SDA Up -> BUSIDLE
 * When Start      and SCL Dn -> PreAddr6
 * When PreAddr6   and SCL Up -> Addr6     { StoreAddressBit(6) }
 * When Addr6      and SDA Up -> BUSIDLE
 * When Addr6      and SCL Dn -> PreAddr5
 * When PreAddr5   and SCL Up -> Addr5     { StoreAddressBit(5) }
 * When Addr5      and SDA Up -> BUSIDLE
 * when Addr5      and SCL Dn -> PreAddr4
 * When PreAddr4   and SCL Up -> Addr4     { StoreAddressBit(4) }
 * When Addr4      and SDA Up -> BUSIDLE
 * When Addr4      and SCL Dn -> PreAddr3
 * When PreAddr3   and SCL Up -> Addr3     { StoreAddressBit(3) }
 * When Addr3      and SDA Up -> BUSIDLE
 * When Addr3      and SCL Dn -> PreAddr2
 * When PreAddr2   and SCL Up -> Addr2     { StoreAddressBit(2) }
 * When Addr2      and SDA Up -> BUSIDLE
 * When Addr2      and SCL Dn -> PreAddr1
 * When PreAddr1   and SCL Up -> Addr1     { StoreAddressBit(1) }
 * When Addr1      and SDA Up -> BUSIDLE
 * When Addr1      and SCL Dn -> PreAddr0
 * When PreAddr0   and SCL Up -> Addr0     { StoreAddressBit(0) }
 * When Addr0      and SDA Up -> BUSIDLE
 * When Addr0      and SCL Dn -> PreRW
 * When PreRW      and SCL Up -> ReadWrite { StoreReadNotWriteBit() }
 * When ReadWrite  and SDA Up -> BUSIDLE
 * When ReadWrite  and SCL Dn -> PreAddrAck
 * 
 * // PreAddrAck looks like a "dead end" in the state machine because the next state is
 * // determined by the normal process outside the ISR. The normal processing code
 * // will have to respond to the received slave address by setting SDA accordingly,
 * // setting the next state and releasing SCL. This process also starts a timer that
 * // uses a interrupt after a few ms to release SCL so the bus doesn't become stuck
 * // in the case the normal process fails to respond.
 * When PreAddrAck enter { if (MatchReceivedAddress()) { StretchClock(); AddressMatchEvent(); } else SetNextState(BUSBUSY); }
 * 
 * // When the slave responds fast enough, it could be that it released the SCL line
 * // before the controller did. In that case, the SCL line will still be low when we
 * // get to either Ack state. But we don't need to specify these events because the
 * // SCL line going up will just be ignored if we don't specify a rule for it.
 * When AddrWrAck  and SCL Dn -> PreWrData7   { twiSDAHigh(); }
 * When AddrWrAck  and SDA Up while SCL High -> BUSIDLE   { SlaveReceiveEvent(); }
 * When AddrRdAck  and SCL Dn -> PreRdData7   { twiSDAHigh(); }
 * 
 * When PreWrData7 and SCL Up -> WrData7      { ClearByteBuffer(); StoreDataBit(7) }
 * When WrData7    and SDA Up -> BUSIDLE      { SlaveReceiveEvent(); } // Stop condition
 * When WrData7    and SDA Dn -> Start        { SlaveReceiveEvent(); } // Restart condition
 * When WrData7    and SCL Dn -> PreWrData6
 * When PreWrData6 and SCL Up -> WrData6      { StoreDataBit(6) }
 * When WrData6    and SDA Up -> BUSIDLE
 * When WrData6    and SCL Dn -> PreWrData5
 * When PreWrData5 and SCL Up -> WrData5      ( StoreDataBit(5) }
 * When WrData5    and SDA Up -> BUSIDLE
 * When WrData5    and SCL Dn -> PreWrData4
 * When PreWrData4 and SCL Up -> WrData4      { StoreDataBit(4) }
 * When WrData4    and SDA Up -> BUSIDLE
 * When WrData4    and SCL Dn -> PreWrData3
 * When PreWrData3 and SCL Up -> WrData3      { StoreDataBit(3) }
 * When WrData3    and SDA Up -> BUSIDLE
 * When WrData3    and SCL Dn -> PreWrData2
 * When PreWrData2 and SCL Up -> WrData2      { StoreDataBit(2) }
 * When WrData2    and SDA Up -> BUSIDLE
 * When WrData2    and SCL Dn -> PreWrData1
 * When PreWrData1 and SCL Up -> WrData1      { StoreDataBit(1) }
 * When WrData1    and SDA Up -> BUSIDLE
 * When WrData1    and SCL Dn -> PreWrData0
 * When PreWrData0 and SCL Up -> WrData0      { StoreDataBit(0) }
 * When WrData0    and SDA Up -> BUSIDLE
 * When WrData0    and SCL Dn -> PreDataWrAck
 * 
 * When PreDataWrAck enter { StretchClock(); if (StoreDataByte()) SDALow(); ReleaseSCL(); }
 * When PreDataWrAck and SCL Up -> DataWrAck
 * When DataWrAck    and SCL Dn -> PreWrData7   { RelaseSDA(); }
 *
 * When PreRdData7 enter { if (LoadSlaveTxByteBuffer()) SetSDAForSlaveTxBit(7); else SetNextState(BUSBUSY); }
 * When PreRdData7 and SCL Up -> RdData7
 * When RdData7    and SCL Dn -> PreRdData6   { SetSDAForSlaveTXBit(6); }
 * When PreRdData6 and SCL Up -> RdData6
 * When RdData6    and SCL Dn -> PreRdData5   { SetSDAForSlaveTXBit(5); }
 * When PreRdData5 and SCL Up -> RdData5
 * When RdData5    and SCL Dn -> PreRdData4   { SetSDAForSlaveTXBit(4); }
 * When PreRdData4 and SCL Up -> RdData4
 * When RdData4    and SCL Dn -> PreRdData3   { SetSDAForSlaveTXBit(3); }
 * When PreRdData3 and SCL Up -> RdData3
 * When RdData3    and SCL Dn -> PreRdData2   { SetSDAForSlaveTXBit(2); }
 * When PreRdData2 and SCL Up -> RdData2
 * When RdData2    and SCL Dn -> PreRdData1   { SetSDAForSlaveTXBit(1); }
 * When PreRdData1 and SCL Up -> RdData1
 * When RdData1    and SCL Dn -> PreRdData0   { SetSDAForSlaveTXBit(0); }
 * When PreRdData0 and SCL Up -> RdData0
 * When RdData0    and SCL Dn -> PreDataRdAck { twiSDAHigh(); }
 * When PreDataRdAck and SCL Up -> DataRdAck  { if (!sda) removeByteFromSlaveTxBuffer(); }
 * When DataRdAck    and SCL Dn -> PreRdData7 
 * 
 */

