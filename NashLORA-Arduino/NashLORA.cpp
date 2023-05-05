#include "NashLORA.h"

// definitions - undecided if these should go here or in header

// register addresses - notes are for info or maybe for future development
#define REG_FIFO                    0x00
#define REG_OP_MODE                 0x01
#define REG_FR_MSB                  0x06
#define REG_FR_MID                  0x07
#define REG_FR_LSB                  0x08
#define REG_PA_CONFIG               0x09
#define REG_PA_RAMP                 0x0A // unsure if needed
#define REG_OCP                     0x0B // unsure if needed
#define REG_LNA                     0x0C
#define REG_FIFO_ADDR_PTR           0x0D
#define REG_FIFO_TX_BASE_PTR        0x0E
#define REG_FIFO_RX_BASE_PTR        0x0F
#define REG_FIFO_RX_CURRENT_PTR     0x10
#define REG_IRQ_FLAGS_MASK          0x11 // unsure if needed
#define REG_IRQ_FLAGS               0x12
#define REG_RX_NUM_BYTES            0x13
// skipped some registers for count of receveived things here, not sure if needed
#define REG_MODEM_STAT              0x18 // unsure if needed
#define REG_PACKET_SNR              0x19
#define REG_PACKET_RSSI             0x1A
// skipped register for current rssi - probably unneccesary with previous rssi register
// skipped hop channel register
#define REG_MODEM_CONFIG_1          0x1D
#define REG_MODEM_CONFIG_2          0x1E
// skipped register about symbol timeout
#define REG_PREAMBLE_MSB            0x20
#define REG_PREAMBLE_LSB            0x21
#define REG_PAYLOAD_LEN             0x22
// skipped some registers about max payload len and frequency hopping
#define REG_FIFO_RX_BYTE_ADDR_PTR   0x25 // unsure if needed
#define REG_MODEM_CONFIG_3          0x26
// skipped some registers about rf frequency error
#define REG_RSSI_WIDEBAND           0x2C // used to generate a random number - interesting
// skipped some whacky IFFreq unlabeled registers
#define REG_DETECTION_OPTIMIZE      0x31
// skipped register about inverting I and Q and optimizing for high bandwidth
#define REG_DETECTION_THRESHOLD     0x37
#define REG_SYNC_WORD               0x39
// skipped register about inverting I and Q and optimizing for high bandwidth part 2
#define REG_DIO_MAPPING_1           0x40 // used for doing things with DIO pins, not used in dogometer v1
#define REG_DIO_MAPPING_2           0x41
#define REG_VERSION                 0x42
// there are some other registers, seems they are not needed

// add more defines here for needed things possibly, like writing certain registers in repeated ways
#define LORA_MAX_PACKET_LEN 255


// register settings for different modes - these are set to always keep the device in LoRa long range mode
#define MODE_SLEEP 0x80
#define MODE_STANDBY 0x81
#define MODE_TX 0x83
#define MODE_RX 0x86
#define MODE_RX_CONT 0x85 // should use RX Continuous mode unless precise arrival time for signal is known, basically just use this

// register settings for IRQ flags
#define IRQ_TX_DONE 0x08
#define IRQ_RX_DONE 0x40
#define IRQ_CRC_ERR 0x20

#define DEFAULT_SPI SPI
#define DEFAULT_SPI_FREQ 8000000

bool NashLORA::init(int csPinIn, int rstPinIn)
{
    // prep pins
    csPin = csPinIn;
    rstPin = rstPinIn;

    pinMode(csPin, OUTPUT);
    pinMode(rstPin, OUTPUT);

    digitalWrite(csPin, HIGH); // this may be communist
    //gpio_set_level(PIN_CS, 1); // this line of code may be communist

    // config and initialize SPI
    spiSettings = SPISettings(DEFAULT_SPI_FREQ, MSBFIRST, SPI_MODE0);
    spi = &DEFAULT_SPI;

    // reset chip
    reset();

    spi->begin();

    uint8_t version;
    version = readRegister(REG_VERSION);

    // debug

    if (version != 0x12)
    {
        return false;
    }

    // put device into sleep mode
    sleep();

    // set default parameters
    setFreq(915);
    setPower(17); // randomly selected default power - this is the max

    // reset FIFO to 0
    writeRegister(REG_FIFO_RX_BASE_PTR, 0);
    writeRegister(REG_FIFO_TX_BASE_PTR, 0);

    writeRegister(REG_LNA, 0x20 | 0x03); // this line will boost LNA current to %150, not sure if necessary, leaving alone for now

    writeRegister(REG_MODEM_CONFIG_3, 0x04); // set gain to auto

    standby();

    return true;

}

void NashLORA::reset()
{
    digitalWrite(rstPin, LOW);
    delay(100);
    digitalWrite(rstPin, HIGH);
    delay(100);
}

uint8_t NashLORA::readRegister(uint8_t reg)
{
    uint8_t response;

    digitalWrite(csPin, LOW);

    spi->beginTransaction(spiSettings);
    spi->transfer(reg & 0x7f);
    response = spi->transfer(0x00);
    spi->endTransaction();

    digitalWrite(csPin, HIGH);

    return response;
}

void NashLORA::writeRegister(uint8_t reg, uint8_t val)
{
    digitalWrite(csPin, LOW);

    spi->beginTransaction(spiSettings);
    spi->transfer(reg | 0x80);
    spi->transfer(val);
    spi->endTransaction();

    digitalWrite(csPin, HIGH);
}

void NashLORA::sleep()
{
    writeRegister(REG_OP_MODE, MODE_SLEEP);
}

void NashLORA::standby()
{
    writeRegister(REG_OP_MODE, MODE_STANDBY);
}

void NashLORA::setFreq(uint32_t freq) // take in freq in mhz
{
    uint32_t frf = freq * 16384; // simplified conversion from datasheet

    // write bytes by shifting and masking
    writeRegister(REG_FR_MSB, frf >> 16 & 0xff);
    writeRegister(REG_FR_MID, frf >> 8 & 0xff);
    writeRegister(REG_FR_LSB, frf >> 0 & 0xff);
}

void NashLORA::setPower(uint8_t power)
{
    // power is between 2 and 17 (dBm)
    if (power < 2)
    {
        power = 2;
    }
    if (power > 17)
    {
        power = 17;
    }

    writeRegister(REG_PA_CONFIG, 0x80 | 0x70 | (power - 2)); // set power register, unsure of 0x70, sets parameter called MaxPower to be its max which is still less than PA_BOOST and seems unused in other code
}

void NashLORA::send(uint8_t* buf, uint8_t len)
{
    standby(); // put radio into standby to transfer data
    writeRegister(REG_FIFO_ADDR_PTR, 0); // put pointer for FIFO to zero to be able to use max size

    // write buf to FIFO
    for (int i = 0; i < len; i++)
    {
        writeRegister(REG_FIFO, buf[i]);
    }

    writeRegister(REG_PAYLOAD_LEN, len);

    // put lora into send mode
    writeRegister(REG_OP_MODE, MODE_TX);

    // wait for lora to be done sending by reading flag for completion
    while ((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE) == 0)
    {
        yield();
    }

    //reset IRQ flag by writing 1 to bit in register
    writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE);

}

void NashLORA::receive()
{
    writeRegister(REG_OP_MODE, MODE_RX_CONT);
}

void NashLORA::enableCRC(bool en)
{
    uint8_t buf = readRegister(REG_MODEM_CONFIG_2);
    if (en)
    {
        writeRegister(REG_MODEM_CONFIG_2, buf | 0x04);
    }
    else
    {
        writeRegister(REG_MODEM_CONFIG_2, buf & 0xDF);
    }
}

int NashLORA::received()
{
    uint8_t buf = readRegister(REG_IRQ_FLAGS);

    if (buf & IRQ_RX_DONE)
    {
        if (buf & IRQ_CRC_ERR)
        {
            writeRegister(REG_IRQ_FLAGS, IRQ_CRC_ERR);
            return 2; // crc error
        }
        return 1;
    }
    return 0;

    /*
    if ((readRegister(REG_IRQ_FLAGS) & IRQ_RX_DONE) == 0)
    {
        return 0;
    }
    else
    {
        if ((readRegister(REG_IRQ_FLAGS) & IRQ_CRC_ERR) == 0)
        {
            return 1;
        }
        else
        {
            writeRegister(REG_IRQ_FLAGS, IRQ_CRC_ERR); // reset crc flag
            return 2; // received packet with crc error    
        }
    }
    */
}

void NashLORA::receivePacket(uint8_t* buf, uint8_t* len) // this function only receives the most recent packet
{
    // get len of received packet and reset rx flag
    standby();
    *len = readRegister(REG_RX_NUM_BYTES);
    writeRegister(REG_IRQ_FLAGS, IRQ_RX_DONE);
    //writeRegister(REG_IRQ_FLAGS, IRQ_CRC_ERR);

    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_PTR)); // pointer to start of packet in FIFO

    for (int i = 0; i < *len; i++) // read FIFO into buffer for length of packet
    {
        buf[i] = readRegister(REG_FIFO); 
    }
    receive();
}