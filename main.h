#include "mbed.h"
#include "lib_crc.h"
#include "Watchdog.h"
#include "events/EventQueue.h"

#define DEBUG 1

static EventQueue ev_queue(15 * EVENTS_EVENT_SIZE);

AnalogIn sensor(PC_0);
RawSerial rs(PB_10, PB_11, 9600);
RawSerial pc(PA_2, PA_3, 9600);
DigitalOut led0(PC_8, 0);
DigitalOut sendEn(PB_15, 0);
DigitalOut simEn(PB_7, 0);

const uint8_t address = 0x01;

Watchdog wd;

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

uint16_t AI[100], AO[100];
bool DI[100], DO[100];

uint8_t buffer[100];
uint8_t ans[100];
int indx = 0, packetLen = 0, ansPacketLen = 0;
uint8_t errorCode;
uint16_t packetCRC;
char c;
bool newPacket = false, receiving = false, processingPacket = false, noError = true;

void blink() {
    led0 = !led0;
    wd.Service();
}

void initRegs(){
    bool temp = true;
    for(uint16_t i = 0;i < 100;i++){
        DO[i] = temp;
        temp = !temp;
        DI[i] = temp;
        AI[i] = i;
        AO[i] = 100 - i;
    }
}

void rsRx() {
    if(processingPacket){
        return;
    }
    c = rs.getc();
    buffer[indx] = c;
    indx++;
    if(!receiving){
        newPacket = true;
    }
    receiving = true;
}
void processPacketAndRespond();

void waitForPacket(){
    if (!newPacket) {
        return;
    }
    if (DEBUG) {
        pc.printf("New packet.\n");
    }
    ansPacketLen = 0;
    while(1){
        receiving = false;
        if (DEBUG) {
            pc.printf("Still receving...\n");
        }
        wait_ms(1);
        if(!receiving){
            packetLen = indx;
            packetCRC = (buffer[indx - 1] << 8) + buffer[indx - 2];
            indx = 0;
            newPacket = false;
            receiving = false;
            wait_ms(500);
            ev_queue.call(processPacketAndRespond);
            return;
        }
    }
}

bool checkPacketCRC(){
    uint16_t crc = calculate_crc16_Modbus((char *)buffer, packetLen-2);
    return crc == packetCRC;
}

bool checkPacketAddress(){
    return buffer[0] == address;
}

void readDO(){
    uint16_t startAddress = (buffer[2] << 8) + buffer[3];
    uint16_t registerCount = (buffer[4] << 8) + buffer[5];
    if((startAddress < 0 || startAddress >= 100) && (startAddress + registerCount) >=100){
        noError = false;
        errorCode = DATA_ADDRESS_ERROR;
        return;
    }
    uint8_t ansBytes = 0;
    ansBytes += (registerCount / 8);
    if(registerCount % 8 != 0 ){
        ansBytes++;
    }
    ans[2] = ansBytes;
    ansPacketLen++;
    for(size_t i = 0;i < ansBytes;i++){
        uint8_t temp = 0;
        for(size_t j = 0;j < 8;j++){
            if(i * 8 + j < registerCount){
                temp += DO[(i * 8 + j) + startAddress] << j;
            }
        }
        if (DEBUG) {
            pc.printf("byte = %x\n", temp);
        }
        ans[3 + i] = temp;
        ansPacketLen++;
    }
}

void readDI(){
    uint16_t startAddress = (buffer[2] << 8) + buffer[3];
    uint16_t registerCount = (buffer[4] << 8) + buffer[5];
    if((startAddress < 0 || startAddress >= 100) && (startAddress + registerCount) >=100){
        noError = false;
        errorCode = DATA_ADDRESS_ERROR;
        return;
    }
    uint8_t ansBytes = 0;
    ansBytes += (registerCount / 8);
    if(registerCount % 8 != 0 ){
        ansBytes++;
    }
    ans[2] = ansBytes;
    ansPacketLen++;
    for(size_t i = 0;i < ansBytes;i++){
        uint8_t temp = 0;
        for(size_t j = 0;j < 8;j++){
            if(i * 8 + j < registerCount){
                temp += DI[(i * 8 + j) + startAddress] << j;
            }
        }
        if (DEBUG) {
            pc.printf("byte = %x\n", temp);
        }
        ans[3 + i] = temp;
        ansPacketLen++;
    }
}

void readAO(){
    uint16_t startAddress = (buffer[2] << 8) + buffer[3];
    uint16_t registerCount = (buffer[4] << 8) + buffer[5];
    if((startAddress < 0 || startAddress >= 100) && (startAddress + registerCount) >=100){
        noError = false;
        errorCode = DATA_ADDRESS_ERROR;
        return;
    }
    uint8_t ansBytes = registerCount * 2;
    ans[2] = ansBytes;
    ansPacketLen++;
    for(size_t i = 0;i < ansBytes;i+=2){
        uint8_t tempH = (uint8_t)((AO[startAddress] & 0xFF00) >> 8);
        uint8_t tempL = (uint8_t)(AO[startAddress] & 0xFF);
        if (DEBUG) {
            pc.printf("byte = %x - %x\n", tempH, tempL);
        }
        ans[3 + i] = tempH;
        ans[4 + i] = tempL;
        ansPacketLen += 2;
        startAddress++;
    }
}

void readAI(){
    uint16_t startAddress = (buffer[2] << 8) + buffer[3];
    uint16_t registerCount = (buffer[4] << 8) + buffer[5];
    if((startAddress < 0 || startAddress >= 100) && (startAddress + registerCount) >=100){
        noError = false;
        errorCode = DATA_ADDRESS_ERROR;
        return;
    }
    uint8_t ansBytes = (uint8_t)registerCount * 2;
    ans[2] = ansBytes;
    ansPacketLen++;
    for(size_t i = 0;i < ansBytes;i+=2){
        uint8_t tempH = (uint8_t)((AI[startAddress] & 0xFF00) >> 8);
        uint8_t tempL = (uint8_t)(AI[startAddress] & 0xFF);
        if (DEBUG) {
            pc.printf("byte = %x - %x\n", tempH, tempL);
        }
        ans[3 + i] = tempH;
        ans[4 + i] = tempL;
        ansPacketLen += 2;
        startAddress++;
    }
}

void writeSingleDO(){
    uint16_t registerAddress = (buffer[2] << 8) + buffer[3];
    if(registerAddress < 0 || registerAddress >= 100){
        noError = false;
        errorCode = DATA_ADDRESS_ERROR;
        return;
    }
    uint16_t registerValue = (buffer[4] << 8) + buffer[5];
    if(registerValue != 0xFF00 && registerValue != 0x0000){
        noError = false;
        errorCode = DATA_VALUE_ERROR;
        return;
    }

    for(size_t i = 2;i < 6;i++){
        ans[i] = buffer[i];
        ansPacketLen++;
    }
    bool value = false;
    if(registerValue == 0xFF00){
        value = true;
    }
    if (DEBUG) {
        pc.printf("Set output %d = %d\n", registerAddress, value);
    }
    DO[registerAddress] = value;
}

void writeSingleAO(){
    uint16_t registerAddress = (buffer[2] << 8) + buffer[3];
    if(registerAddress < 0 || registerAddress >= 100){
        noError = false;
        errorCode = DATA_ADDRESS_ERROR;
        return;
    }
    uint16_t registerValue = (buffer[4] << 8) + buffer[5];
    if(registerValue < 0 || registerValue >0xFFFF){

    }
    for(size_t i = 2;i < 6;i++){
        ans[i] = buffer[i];
        ansPacketLen++;
    }
    if (DEBUG) {
        pc.printf("Set register %x = %x\n", registerAddress, registerValue);
    }
    AO[registerAddress] = registerValue;
}

void writeMultiDO(){
    uint16_t startAddress = (buffer[2] << 8) + buffer[3];
    uint16_t registerCount = (buffer[4] << 8) + buffer[5];
    if((startAddress < 0 || startAddress >= 100) && (startAddress + registerCount) >=100){
        noError = false;
        errorCode = DATA_ADDRESS_ERROR;
        return;
    }
    for(size_t i = 2;i < 6;i++){
        ans[i] = buffer[i];
        ansPacketLen++;
    }
    uint8_t dataBytes = buffer[6];
    for(size_t i = 0;i < dataBytes;i++){
        uint8_t temp = buffer[7 + i];
        for(size_t j = 0, k = 1;j < 8; j++, k*=2){
            if((i * 8) + j < registerCount){
                bool value = (temp & k) >> j;
                if (DEBUG) {
                    pc.printf("Set Output %d = %d\n", startAddress, value);
                }
                DO[startAddress] = value;
                startAddress++;
            }
        }
    }
}

void writeMultiAO(){
    uint16_t startAddress = (buffer[2] << 8) + buffer[3];
    uint16_t registerCount = (buffer[4] << 8) + buffer[5];
    if((startAddress < 0 || startAddress >= 100) && (startAddress + registerCount) >=100){
        noError = false;
        errorCode = DATA_ADDRESS_ERROR;
        return;
    }

    for(size_t i = 2;i < 6;i++){
        ans[i] = buffer[i];
        ansPacketLen++;
    }
    uint8_t dataBytes = buffer[6];
    for(size_t i = 0;i < dataBytes;i += 2){
        uint8_t tempH = buffer[7 + i];
        uint8_t tempL = buffer[8 + i];
        uint16_t temp = (tempH << 8) + tempL;
        if (DEBUG) {
            pc.printf("Set register %x = %x\n", startAddress, temp);
        }
        AO[startAddress] = temp;
        startAddress++;
    }
}

int processPacket(){
    uint16_t functionCode = buffer[1];
    noError = true;
    switch(functionCode){
        case READ_DO:
            if (DEBUG) {
                pc.printf("Read DO\n");
            }
            readDO();
            break;

        case READ_DI:
            if (DEBUG) {
                pc.printf("Read DI\n");
            }
            readDI();
            break;

        case READ_AO:
            if (DEBUG) {
                pc.printf("Read AO\n");
            }
            readAO();
            break;

        case READ_AI:
            if (DEBUG) {
                pc.printf("Read AI\n");
            }
            readAI();
            break;

        case WRITE_SINGLE_DO:
            if (DEBUG) {
                pc.printf("WRITE_SINGLE_DO\n");
            }
            writeSingleDO();
            break;

        case WRITE_SINGLE_AO:
            if (DEBUG) {
                pc.printf("WRITE_SINGLE_AO\n");
            }
            writeSingleAO();
            break;

        case WRITE_MULTI_DO:
            if (DEBUG) {
                pc.printf("WRITE_MULTI_DO\n");
            }
            writeMultiDO();
            break;

        case WRITE_MULTI_AO:
            if (DEBUG) {
                pc.printf("WRITE_MULTI_AO\n");
            }
            writeMultiAO();
            break;

        default:
            noError = false;
            errorCode = FUNCTION_CODE_ERROR;
            if (DEBUG) {
                pc.printf("Unknown funtion code!\n");
            }
    }
    if(noError){
        ans[1] = buffer[1];
    }
    else {
        ans[1] = buffer[1] + 0x80;
        ans[2] = errorCode;
        ansPacketLen++;
    }
    ansPacketLen++;
    return 0;
}

void calculateAnsCRC(){
    uint16_t crc = calculate_crc16_Modbus((char *) ans, ansPacketLen);
    if (DEBUG) {
        pc.printf("CRC = %x\n", crc);
    }
    ans[ansPacketLen + 1] = (uint8_t)((crc & 0xFF00) >> 8);
    ans[ansPacketLen] = (uint8_t)(crc & 0xFF);
    ansPacketLen += 2;
}

void sendAnswer(){
    for(size_t i = 0;i < ansPacketLen;i++){
        rs.putc(ans[i]);
    }
    wait_ms(10);
}

void processPacketAndRespond() {
    processingPacket = true;
    if (DEBUG) {
        for (size_t i = 0; i < packetLen; i++) {
            pc.printf("%x ", buffer[i]);
        }
        pc.printf("\n");
    }
    if(checkPacketCRC()){
        if (DEBUG) {
            pc.printf("CRC done\n");
        }
    }
    else{
        if (DEBUG) {
            pc.printf("CRC failed\n");
        }
        processingPacket = false;
        return;
    }

    if(checkPacketAddress()){
        if (DEBUG) {
            pc.printf("Packet is for me\n");
        }
        ans[0] = buffer[0];
        ansPacketLen++;
    }
    else {
        if (DEBUG) {
            pc.printf("Packet is not for me\n");
        }
        processingPacket = false;
        return;
    }

    if (DEBUG) {
        pc.printf("Processing Packet\n");
    }
    processPacket();
    if (DEBUG) {
        pc.printf("Calculate response CRC\n");
    }
    calculateAnsCRC();
    if (DEBUG) {
        pc.printf("Sent response\n");
    }
    sendEn = 1;
    sendAnswer();
    sendEn = 0;
    processingPacket = false;
}
