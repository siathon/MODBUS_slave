#include "mbed.h"
#include "lib_crc.h"
#include "events/EventQueue.h"

static EventQueue ev_queue(5 * EVENTS_EVENT_SIZE);

AnalogIn sensor(p2);
RawSerial rs(p12, p14, 9600);
DigitalOut nrfLED(p20, 0);
DigitalOut sendEn(p6, 1);

const uint8_t address = 0x01;


#define READ_DO         0x01
#define READ_DI         0x02
#define READ_AO         0x03
#define READ_AI         0x04
#define WRITE_SINGLE_DO 0x05
#define WRITE_SINGLE_AO 0x06
#define WRITE_MULTI_DO  0x0F
#define WRITE_MULTI_AO  0x10

#define FUNCTION_CODE_ERROR 0x01
#define DATA_ADDRESS_ERROR  0x02
#define DATA_VALUE_ERROR    0x03
#define UNRECOVERABLE_ERROR 0x04
#define LONGTIME_NOTIFY     0x05
#define SLAVE_BUSY          0x06

uint16_t AI[1];
uint8_t ans[10];

void blink() {
    nrfLED = !nrfLED;
}

void processPacket(){
    ans[0] = address;
    ans[1] = 0x04;
    ans[2] = 0x02;
    ans[3] = (uint8_t)((AI[0] & 0xFF00) >> 8);
    ans[4] = (uint8_t)(AI[0] & 0x00FF);
}

void sendPacket() {
    nrfLED = 1;
    // rs.printf("Reading Sensor...");
    AI[0] = sensor.read_u16();
    // rs.printf("Done.\n");

    // rs.printf("Generating Packet..");
    processPacket();
    // rs.printf("Done.\n");

    // rs.printf("Calculating CRC...");
    uint16_t crc = calculate_crc16_Modbus((char *) ans, 5);
    ans[5] = (uint8_t)((crc & 0xFF00) >> 8);
    ans[6] = (uint8_t)(crc & 0x00FF);
    // rs.printf("Done\n");

    for (size_t i = 0; i < 7; i++) {
        rs.putc(ans[i]);
    }
    nrfLED = 0;
}
