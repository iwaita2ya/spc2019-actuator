//
// Created by iwait on 4/8/19.
//

#ifndef SRAM_TEST_GS47SERIAL_SRAM_H
#define SRAM_TEST_GS47SERIAL_SRAM_H

#include <mbed.h>

#define SRAM_BUFFER_SIZE 8

class Gs47SerialSRAM {

private:
    uint8_t SRAM_REGISTER_READ;
    uint8_t SRAM_REGISTER_WRITE;
    uint8_t CONTROL_REGISTER_READ;
    uint8_t CONTROL_REGISTER_WRITE;
    I2C i2c;
    DigitalOut *hardwareStore;

public:

    // Constructor
    Gs47SerialSRAM(PinName sda, PinName scl, PinName hs, const uint8_t A2=0, const uint8_t A1=0);

    /**
     * SRAM Read Operations
     */
    uint8_t read(char *buffer); // Read from current address (1 Byte)
    uint8_t read(const uint16_t address, char *buffer); // Random read (1 Byte)
    uint8_t read(const uint16_t address, char *buffer, const uint16_t size); // Seq. read w/ address (Multiple Bytes)

    /**
     * SRAM Write Operations
     */
    uint8_t write(const uint16_t address, const uint8_t data); // Byte write (1 Byte)
    uint8_t write(const uint16_t address, const char *data, const uint16_t size); // Seq. write
    //TODO: メモリクリア用のメソッドも用意する

    /**
     * Control Register Operations
     */
    // Write (Control Register)
    uint8_t readControlRegister(char *buffer);
    uint8_t writeControlRegister(const uint8_t address, const uint8_t value);
    uint8_t getAutoStore();
    void setAutoStore(const uint8_t value); //MEMO: 他のメソッドに合わせて uint8_t を返却するべき？

};

#endif //SRAM_TEST_GS47SERIAL_SRAM_H
