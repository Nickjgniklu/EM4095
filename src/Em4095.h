//Most of this code is derived from
//https://github.com/Proxmark/proxmark3
#ifndef RFID_H
#define RFID_H
#include <array>
#include <Arduino.h>
#define DELAYVAL 384  // 384 //standard delay for manchster decode
#define TIMEOUT 10000 // standard timeout for manchester decode at  160mhz is 1000000
#define MAX_DEMOD_BUF_LEN 1024
struct RfidResult {             // Structure declaration
  int data;         // Member (int variable)
  bool error;   // Member (string variable)
} ;
class Em4095
{
public:
    void Enable();
    void Disable();
    void Init();
    RfidResult ReadTag(uint8_t address);
    RfidResult WriteTag(uint8_t address, uint32_t data);
    std::array<uint32_t,15> DumpTag();
    double calcResonantFreq();
    Em4095(byte shd,byte mod, byte demodOut,byte rdyClk);


private:
    // pin configuration

    int demodOut = 16;
    int shd = 4;
    int mod = 27;
    int rdyClk = 15;

    int DemodBufferLen = 0;
    unsigned char DemodBuffer[MAX_DEMOD_BUF_LEN];
    uint8_t forwardLink_data[64]; // array of forwarded bits
    uint8_t *forward_ptr;         // ptr for forward message preparation
    uint8_t fwd_bit_sz;           // forwardlink bit counter
    uint8_t *fwd_write_ptr;       // forwardlink bit pointer

    bool decodeTag(unsigned char *buf);
    bool EM4x05testDemodReadData(uint32_t *word, bool readCmd);
    bool preambleSearchEx(uint8_t *BitStream, uint8_t *preamble, size_t pLen, size_t *size, size_t *startIdx, bool findone) ;
    void setDemodBuf(uint8_t *buff, size_t size, size_t startIdx);
    void SendForward(uint8_t fwd_bit_count, bool fast);
    void EM4xLoginEx(uint32_t pwd);
    void EM4xLogin(uint32_t pwd);
    void EM4xReadWord(uint8_t addr, uint32_t pwd, uint8_t usepwd);
    void EM4xProtectWord(uint32_t data, uint32_t pwd, uint8_t usepwd);
    void EM4xWriteWord(uint8_t addr, uint32_t data, uint32_t pwd, uint8_t usepwd);
    uint8_t Prepare_Cmd(uint8_t cmd);
    uint8_t Prepare_Addr(uint8_t addr);
    uint8_t Prepare_Data(uint16_t data_low, uint16_t data_hi);
    void RecordFromAntenna(uint32_t numberOfBits);
    void turn_read_lf_off(uint32_t microseconds);
    void turn_read_lf_on(uint32_t microseconds);


};

#endif