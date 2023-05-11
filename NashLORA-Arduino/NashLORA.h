#ifndef NashLORA_H
#define NashLORA_H

#include <Arduino.h>
#include <SPI.h>

//#include <stdint.h>
//#include <stdlib.h>

// This will work for arduino SPI so it can be used on Teensy

class NashLORA
{
    public:

        // can add error handling to these to make them only work if inited, but im not a pussy
        bool init(int csPinIn, int rstPinIn); // init chip 
        void reset(); // reset lora chip by pulling rst pin low then high
        void sleep(); // low power mode, lose FIFO
        void standby(); // set chip to idle in order to access FIFO and alter registers
        void send(uint8_t* buf, uint8_t len);
        void receive(); // set chip into receive mode
        int received(); // check to see if a packet has been received, 0 is no, 1 is yes, 2 is yes with crc error
        void receivePacket(uint8_t* buf, uint8_t* len); // receive packet
        int getPacketRSSI(); // returns rssi of last packet received

        void setPower(uint8_t power); // set transmit power in dBm
        void enableCRC(bool en);

        void setFreq(uint32_t freq);

    private:
    
        void writeRegister(uint8_t reg, uint8_t val); // write value to register
        uint8_t readRegister(uint8_t reg); // read value from register

        uint32_t freq = 434000000;

        SPISettings spiSettings;
        SPIClass* spi;
        int csPin;
        int rstPin;
    
    
};

#endif