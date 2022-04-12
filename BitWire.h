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

#ifndef __BitWire_h__
#define __BitWire_h__

#include <inttypes.h>
#include <string.h>
#include <UMPins/UMPins.h>

#define BUFFER_LENGTH 32

// WIRE_HAS_END means Wire has end()
#define WIRE_HAS_END 1

#define EBITWIRE_TIMEOUT 1;
#define EBITWIRE_NODEV   2;
#define EBITWIRE_NACK    3;

class BitWire
{
  private:
    pinref_t _pin_sda;
    pinref_t _pin_scl;
  
    uint16_t _clockPulseLengthNOPS;
    uint8_t _address;
  
    volatile uint8_t _rxBuffer[BUFFER_LENGTH];
    volatile uint8_t _rxBufferIndex;
    volatile uint8_t _rxBufferLength;

    uint8_t _txBuffer[BUFFER_LENGTH];
    uint8_t _targetAddress;
    uint8_t _txBufferIndex;
    uint8_t _txBufferLength;

    uint8_t _transmitting;
    
    uint32_t _timeout;
    bool _resetWithTimeout;
    bool _seenTimeout;
    int32_t _lastError;

    volatile uint8_t _slaveBusState; // State for the pin change ISR state machine. (May sometimes need to be altered when code outside the ISR needs to decide the next state.)
    bool _lastSCL = true; // Only used within the ISR context.
    bool _lastSDA = true; // Only used within the ISR context.

    void (*_user_onRequest)(void);
    void (*_user_onReceive)(int);
    void (*_user_onInterruptsOn)(BitWire*);
    void (*_user_onInterruptsOff)(BitWire*);
    void onRequestService(void);
    void onReceiveService(volatile uint8_t*, int);
    void setWriteError(int32_t err = 1);
    void delayNops(int nops);
  private:
    /* Slave mode members */
    uint8_t _slaveByteBuffer;
    volatile uint8_t _slaveRxBuffer[BUFFER_LENGTH];
    volatile uint8_t _slaveRxBufBegin;
    volatile uint8_t _slaveRxBufEnd;
    volatile uint8_t _slaveTxBuffer[BUFFER_LENGTH];
    volatile uint8_t _slaveTxBufIndex;
    volatile uint8_t _slaveTxBufLength;
    bool _slaveReadNotWriteBit;
    void (*_onAddressMatchEvent)(BitWire*, bool&, bool&);
    
    void CopySlaveTxBuffer();
    void SetSDAForSlaveTxBit(uint8_t bitPos);
    bool LoadSlaveTxByteBuffer();
    void ClearSlaveBuffers();
    void ClearSlaveByteBuffer();
    void StoreAddressBit(uint8_t bitPos, bool bitValue);
    void StoreDataBit(uint8_t bitPos, bool bitValue);
    bool StoreDataByte();
    void removeByteFromSlaveTxBuffer();
    
    void SetNextState(uint8_t nextState);
    bool MatchReceivedAddress();
    void StretchClock();
    void StoreReadNotWriteBit(bool readNotWriteBit);
    void SlaveReceiveEvent();
    void AddressMatchEvent();
  private:
    /* twi functions */
    inline void twiSCLHigh();
    inline void twiSCLLow();
    inline void twiSDAHigh();
    inline void twiSDALow();
    inline bool twiSDARead();
    void twiSendStart();
    void twiSendStop();
    void twiSetTimeoutInMicros(uint32_t timeout, bool reset_with_timeout);
    bool twiManageTimeoutFlag(bool doClear);
    void twiSendByte(uint8_t data);
    bool twiReadByte(volatile uint8_t *buf);
    bool twiWaitBusIdle();
    bool twiWaitClockHigh(int32_t timeout);
    bool twiReadAck();
    void twiSendAck();
    void twiInterruptsOff();
    void twiInterruptsOn();
    uint8_t twiReadFrom(uint8_t address, volatile uint8_t *rxBuffer, uint8_t quantity, uint8_t sendStop);
    uint8_t twiWriteTo(uint8_t address, uint8_t* data, uint8_t length, uint8_t wait, uint8_t sendStop);
    uint8_t twiTransmit(const uint8_t* data, uint8_t length);
  public:
    BitWire();
    void begin(pinid_t sda, pinid_t scl);
    void begin(pinid_t sda, pinid_t scl, uint8_t address);
    void begin(pinid_t sda, pinid_t scl, int address);
    void end();
    void setClock(uint32_t);
    void setWireTimeout(uint32_t timeout = 25000, bool reset_with_timeout = false);
    bool getWireTimeoutFlag(void);
    void clearWireTimeoutFlag(void);
    uint32_t getLastError();
    void clearLastError();
    bool sendBusClear();
    void beginTransmission(uint8_t);
    void beginTransmission(int);
    uint8_t endTransmission(void);
    uint8_t endTransmission(uint8_t);
    uint8_t requestFrom(uint8_t, uint8_t);
    uint8_t requestFrom(uint8_t, uint8_t, uint8_t);
    uint8_t requestFrom(uint8_t, uint8_t, uint32_t, uint8_t, uint8_t);
    uint8_t requestFrom(int, int);
    uint8_t requestFrom(int, int, int);
    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t *, size_t);
    virtual int available(void);
    virtual int read(void);
    virtual int peek(void);
    virtual void flush(void);
    void onReceive( void (*)(int) );
    void onRequest( void (*)(void) );
    
    void HandleInterrupt();
    
    void onInterruptsOn( void (*)(BitWire*) );
    void onInterruptsOff( void (*)(BitWire*) );
    void onAddressMatchEvent( void (*)(BitWire*, bool&, bool&) );

    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }

  private:
};

#endif

