# BitWire

An open source, free (as in freedom) source library (LGPL licensed) for microcontroller applications to implement bitbanged I2C.

Bitbanging I2C may be useful when a microcontroller, such as the AtMega32U2, doesn't have a hardware I2C implementation and there are no (good) alternatives available. The library uses a pinchange interrupt and a state machine to process the SDA and SCL signal in I2C slave mode and general purpose pin IO in I2C master mode, setting the PORTxn bit to 0 and manipulating the DDRxn bit to change the pin pin from low impedance 0 when the value on the line should be low and change the pin to a (high impedance) input when the line should be high (1).

This library was developed because I was making an EEPROM programmer to program some EEPROM's for another project. However, I found that the AtMega32U4 (that has hardware I2C support) was not in stock and not expected for a while at mouser. So I decided to use the AtMega32U2 instead because that still has USB support and I thought I could probably find some bitbanging I2C library online. (I knew of at least one). However, I didn't find many good options so I decided to make my own.

I realise the current implementation is probably a bit messy and might even be unstable. It also probably still contains a lot of debug code used for development of this first working version. Many things could probably also be done in a much more simple or straighforward way. This has two main reasons. For one, because I copied the code for the popular Wire library and removed dependencies to the twi.c and twi.h files and re-implemented those functions to implement the protocol in a bitbanged fashion. And on the other hand, I just wanted a functional bitbanged I2C implementation as soon as possible so that I could continue with my EEPROM programmer project. However, I will try to maintain this library as well as I can. Plans for it are currently:
- to clean up the code
- testing with other I2C devices and make it feature complete (currenly I just used an arduino mega clone as the I2C master and a standalone atmega328p on a breadboard as the I2C slave to develop this first version)
- improve stability and performance
- integrate it with Arduino ide

Contributions to this library are welcomed (though it may take me a little while to respond to a pull request). For instance for integration and easy instalation in the arduino IDE (of which I currently have no clue how to do that). I'm also not great with makefiles so I think a lot could be improved in that respect as well. For now I've just been simlinking this directory into the project directory and adding the files to the makefile for building the hex file, but there are probably better ways. I will add some example code here later.

-----

