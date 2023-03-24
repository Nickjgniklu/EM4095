// code is adapted from https://github.com/Proxmark/proxmark3
#include "Em4095.h"
const char *RF_TAG = "RFID";
volatile unsigned int clkCount = 0;
volatile bool lastEdgeWasFalling = false;\

// some defines to make copy pasted code from proxmark function
bool g_debugMode = false;
#define prnt(s) ESP_LOGI(RF_TAG, s)
#define prnt(s, v) ESP_LOGI(RF_TAG, s, v)
#define PrintAndLog2(f, v) ESP_LOGI(RF_TAG, f, v)
#define PrintAndLog(f) ESP_LOGI(RF_TAG, f)
#define WaitUS delayMicroseconds
#define turn_read_lf_off(microSeconds) \
    digitalWrite(27, HIGH);            \
    delayMicroseconds(microSeconds);
#define turn_read_lf_on(microSeconds) \
    digitalWrite(27, LOW);            \
    delayMicroseconds(microSeconds);
//-----------------------------------
// EM4469 / EM4305 routines
//-----------------------------------
// Below given command set.
// Commands are including the even parity, binary mirrored
#define FWD_CMD_LOGIN 0xC
#define FWD_CMD_WRITE 0xA
#define FWD_CMD_READ 0x9
#define FWD_CMD_PROTECT 0x3
#define FWD_CMD_DISABLE 0x5



double Em4095::calcResonantFreq(){
    unsigned int countStart = clkCount;
    delay(100);
    unsigned int elapsedCount= clkCount- countStart;
    return elapsedCount/100.0;
}
//====================================================================
// prepares command bits
// see EM4469 spec
//====================================================================
//--------------------------------------------------------------------
//  VALUES TAKEN FROM EM4x function: SendForward
//  START_GAP = 440;       (55*8) cycles at 125kHz (8us = 1cycle)
//  WRITE_GAP = 128;       (16*8)
//  WRITE_1   = 256 32*8;  (32*8)

//  These timings work for 4469/4269/4305 (with the 55*8 above)
//  WRITE_0 = 23*8 , 9*8

uint8_t Em4095::Prepare_Cmd(uint8_t cmd)
{

    *forward_ptr++ = 0; // start bit
    *forward_ptr++ = 0; // second pause for 4050 code

    *forward_ptr++ = cmd;
    cmd >>= 1;
    *forward_ptr++ = cmd;
    cmd >>= 1;
    *forward_ptr++ = cmd;
    cmd >>= 1;
    *forward_ptr++ = cmd;

    return 6; // return number of emitted bits
}

//====================================================================
// prepares address bits
// see EM4469 spec
//====================================================================
uint8_t Em4095::Prepare_Addr(uint8_t addr)
{

    register uint8_t line_parity;

    uint8_t i;
    line_parity = 0;
    for (i = 0; i < 6; i++)
    {
        *forward_ptr++ = addr;
        line_parity ^= addr;
        addr >>= 1;
    }

    *forward_ptr++ = (line_parity & 1);

    return 7; // return number of emitted bits
}

//====================================================================
// prepares data bits intreleaved with parity bits
// see EM4469 spec
//====================================================================
uint8_t Em4095::Prepare_Data(uint16_t data_low, uint16_t data_hi)
{

    register uint8_t column_parity;
    register uint8_t i, j;
    register uint16_t data;

    data = data_low;
    column_parity = 0;

    for (i = 0; i < 4; i++)
    {
        register uint8_t line_parity = 0;
        for (j = 0; j < 8; j++)
        {
            line_parity ^= data;
            column_parity ^= (data & 1) << j;
            *forward_ptr++ = data;
            data >>= 1;
        }
        *forward_ptr++ = line_parity;
        if (i == 1)
            data = data_hi;
    }

    for (j = 0; j < 8; j++)
    {
        *forward_ptr++ = column_parity;
        column_parity >>= 1;
    }
    *forward_ptr = 0;

    return 45; // return number of emitted bits
}
void Em4095::Enable()
{
    digitalWrite(shd, LOW);
}
void Em4095::Disable()
{
    digitalWrite(shd, HIGH);
}
void IRAM_ATTR onClk()
{
    clkCount++;
}

// search for given preamble in given BitStream and return success=1 or fail=0 and startIndex (where it was found) and length if not fineone
// fineone does not look for a repeating preamble for em4x05/4x69 sends preamble once, so look for it once in the first pLen bits
bool Em4095::preambleSearchEx(uint8_t *BitStream, uint8_t *preamble, size_t pLen, size_t *size, size_t *startIdx, bool findone)
{
    // Sanity check.  If preamble length is bigger than bitstream length.
    if (*size <= pLen)
        return false;

    uint8_t foundCnt = 0;
    for (size_t idx = 0; idx < *size - pLen; idx++)
    {
        if (memcmp(BitStream + idx, preamble, pLen) == 0)
        {
            // first index found
            foundCnt++;
            if (foundCnt == 1)
            {
                if (g_debugMode)
                    prnt("DEBUG: preamble found at %u", idx);
                *startIdx = idx;
                if (findone)
                    return true;
            }
            else if (foundCnt == 2)
            {
                *size = idx - *startIdx;
                return true;
            }
        }
    }
    return false;
}
static inline bool oddparity32(uint32_t x)
{
#if !defined __GNUC__
    x ^= x >> 16;
    x ^= x >> 8;
    return oddparity8(x);
#else
    return !__builtin_parity(x);
#endif
}
// by marshmellow
// pass bits to be tested in bits, length bits passed in bitLen, and parity type (even=0 | odd=1) in pType
// returns 1 if passed
bool parityTest(uint32_t bits, uint8_t bitLen, uint8_t pType)
{
    return oddparity32(bits) ^ pType;
}
// by marshmellow
// takes a array of binary values, start position, length of bits per parity (includes parity bit - MAX 32),
//   Parity Type (1 for odd; 0 for even; 2 for Always 1's; 3 for Always 0's), and binary Length (length to run)
size_t removeParity(uint8_t *BitStream, size_t startIdx, uint8_t pLen, uint8_t pType, size_t bLen)
{
    uint32_t parityWd = 0;
    size_t bitCnt = 0;
    for (int word = 0; word < (bLen); word += pLen)
    {
        for (int bit = 0; bit < pLen; bit++)
        {
            if (word + bit >= bLen)
                break;
            parityWd = (parityWd << 1) | BitStream[startIdx + word + bit];
            BitStream[bitCnt++] = (BitStream[startIdx + word + bit]);
        }
        if (word + pLen > bLen)
            break;

        bitCnt--; // overwrite parity with next data
        // if parity fails then return 0
        switch (pType)
        {
        case 3:
            if (BitStream[bitCnt] == 1)
            {
                return 0;
            }
            break; // should be 0 spacer bit
        case 2:
            if (BitStream[bitCnt] == 0)
            {
                return 0;
            }
            break; // should be 1 spacer bit
        default:
            if (parityTest(parityWd, pLen, pType) == 0)
            {
                return 0;
            }
            break; // test parity
        }
        parityWd = 0;
    }
    // if we got here then all the parities passed
    // return size
    return bitCnt;
}
bool EM_EndParityTest(uint8_t *BitStream, size_t size, uint8_t rows, uint8_t cols, uint8_t pType)
{
    if (rows * cols > size)
        return false;
    uint8_t colP = 0;
    // assume last col is a parity and do not test
    for (uint8_t colNum = 0; colNum < cols - 1; colNum++)
    {
        for (uint8_t rowNum = 0; rowNum < rows; rowNum++)
        {
            colP ^= BitStream[(rowNum * cols) + colNum];
        //Serial.print(BitStream[(rowNum * cols) + colNum]);
        }
        //Serial.println();
        if (colP != pType)
            return false;
    }
    return true;
}



//====================================================================
// Forward Link send function
// Requires: forwarLink_data filled with valid bits (1 bit per byte)
// fwd_bit_count set with number of bits to be sent
//====================================================================
void Em4095::SendForward(uint8_t fwd_bit_count, bool fast)
{
// iceman,   21.3us increments for the USclock verification.
// 55FC * 8us == 440us / 21.3 === 20.65 steps.  could be too short. Go for 56FC instead
// 32FC * 8us == 256us / 21.3 ==  12.018 steps. ok
// 16FC * 8us == 128us / 21.3 ==  6.009 steps. ok
#ifndef EM_START_GAP
#define EM_START_GAP 55 * 8
#endif

    fwd_write_ptr = forwardLink_data;
    fwd_bit_sz = fwd_bit_count;

    // force 1st mod pulse (start gap must be longer for 4305)
    fwd_bit_sz--; // prepare next bit modulation
    fwd_write_ptr++;

    turn_read_lf_off(EM_START_GAP);
    turn_read_lf_on(18 * 8);

    // now start writing with bitbanging the antenna. (each bit should be 32*8 total length)
    while (fwd_bit_sz-- > 0)
    { // prepare next bit modulation
        if (((*fwd_write_ptr++) & 1) == 1)
        {
            turn_read_lf_on(32 * 8);
        }
        else
        {
            turn_read_lf_off(23 * 8);
            turn_read_lf_on(18 * 8);
        }
    }
}

void Em4095::EM4xLoginEx(uint32_t pwd)
{
    forward_ptr = forwardLink_data;
    uint8_t len = Prepare_Cmd(FWD_CMD_LOGIN);
    len += Prepare_Data(pwd & 0xFFFF, pwd >> 16);

    SendForward(len, false);
    // WaitUS(20); // no wait for login command.
    //  should receive
    //  0000 1010 ok
    //  0000 0001 fail
}

void Em4095::EM4xLogin(uint32_t pwd)
{

    EM4xLoginEx(pwd);

    WaitUS(400);
}

void Em4095::EM4xWriteWord(uint8_t addr, uint32_t data, uint32_t pwd, uint8_t usepwd)
{

    /* should we read answer from Logincommand?
     *
     * should receive
     * 0000 1010 ok.
     * 0000 0001 fail
     **/
    if (usepwd)
        EM4xLoginEx(pwd);

    forward_ptr = forwardLink_data;
    uint8_t len = Prepare_Cmd(FWD_CMD_WRITE);
    len += Prepare_Addr(addr);
    len += Prepare_Data(data & 0xFFFF, data >> 16);

    // String sdata = "";

    // for (int i = 0; i < len; i++)
    // {
    //     sdata += (char)((forward_ptr[i]&1)+48);
    //     //Serial.print(forward_ptr[i]);
    //     //Serial.print(',');
    // }
    // ESP_LOGI(RF_TAG, "Write Buffer: %s", sdata.c_str());
    SendForward(len, false);
}

void Em4095::EM4xProtectWord(uint32_t data, uint32_t pwd, uint8_t usepwd)
{

    /* should we read answer from Logincommand?
     *
     * should receive
     * 0000 1010 ok.
     * 0000 0001 fail
     **/
    if (usepwd)
        EM4xLoginEx(pwd);

    forward_ptr = forwardLink_data;
    uint8_t len = Prepare_Cmd(FWD_CMD_PROTECT);
    len += Prepare_Data(data & 0xFFFF, data >> 16);

    SendForward(len, false);
}
void Em4095::EM4xReadWord(uint8_t addr, uint32_t pwd, uint8_t usepwd)
{

    // clear buffer now so it does not interfere with timing later

    /* should we read answer from Logincommand?
     *
     * should receive
     * 0000 1010 ok
     * 0000 0001 fail
     **/
    if (usepwd)
        EM4xLoginEx(pwd);

    forward_ptr = forwardLink_data;
    uint8_t len = Prepare_Cmd(FWD_CMD_READ);
    len += Prepare_Addr(addr);

    SendForward(len, false);
}
RfidResult Em4095::WriteTag(uint8_t address, uint32_t data)
{
    // send read
    EM4xWriteWord(address,data, 0, 0);
    //loging here may cause issues with timing!!!!!!
    RecordFromAntenna(127);
    // attempt to find a response
    uint32_t outData=data;
    RfidResult result;

    if (EM4x05testDemodReadData(&outData, false))
    {
        //ESP_LOGI(RF_TAG, "READ COMPLETE %x", result);
        result.data=data;
        result.error=false;
        return result;
    }
    result.error=true;
    result.data=0;
    return result;
}

RfidResult Em4095::ReadTag(uint8_t address)
{
    // send read
    EM4xReadWord(address, 0, 0);
    //loging here may cause issues with timing!!!!!!
    RecordFromAntenna(127);
    // attempt to find a response
    uint32_t data;
        RfidResult result;

    if (EM4x05testDemodReadData(&data, true))
    {
        //ESP_LOGI(RF_TAG, "READ COMPLETE %x", result);
        result.data=data;
        result.error=false;
        return result;
    }
    result.error=true;
    result.data=0;
    return result;
}
    std::array<uint32_t,15> Em4095::DumpTag(){
        std::array<uint32_t,15> data{0};
        for(int i=0; i<data.size();i++){
            if(i!=2||i!=14||i!=15){
            data[i]=ReadTag(i).data;
            }
            WaitUS(400);
        }
        return data;
    }

// set the demod buffer with given array of binary (one bit per byte)
// by marshmellow
void Em4095::setDemodBuf(uint8_t *buff, size_t size, size_t startIdx)
{
    if (buff == NULL)
        return;

    if (size > MAX_DEMOD_BUF_LEN - startIdx)
        size = MAX_DEMOD_BUF_LEN - startIdx;

    size_t i = 0;
    for (; i < size; i++)
    {
        DemodBuffer[i] = buff[startIdx++];
    }
    DemodBufferLen = size;
    return;
}
// least significant bit first
uint32_t bytebits_to_byteLSBF(uint8_t *src, size_t numbits)
{
    uint32_t num = 0;
    for (int i = 0; i < numbits; i++)
    {
        num = (num << 1) | *(src + (numbits - (i + 1)));
    }
    return num;
}
bool Em4095::EM4x05testDemodReadData(uint32_t *word, bool readCmd)
{
    // em4x05/em4x69 command response preamble is 00001010
    // skip first two 0 bits as they might have been missed in the demod
    uint8_t preamble[] = {0, 0, 1, 0, 1, 0};
    size_t startIdx = 0;

    // set size to 20 to only test first 14 positions for the preamble or less if not a read command
    size_t size = (readCmd) ? 20 : 11;
    // sanity check
    size = (size > DemodBufferLen) ? DemodBufferLen : size;
    // test preamble
    if (!preambleSearchEx(DemodBuffer, preamble, sizeof(preamble), &size, &startIdx, true))
    {
        if (g_debugMode)
            PrintAndLog2("DEBUG: Error - EM4305 preamble not found :: %d", startIdx);
        return false;
    }
    // if this is a readword command, get the read bytes and test the parities
    if (readCmd)
    {
        //the column parity row is off on esp32 i think it may be timing/scheduler related? maybe every bit tha is read gets alittle farther off?
         if (!EM_EndParityTest(DemodBuffer + startIdx + sizeof(preamble), 45, 5, 9, 0))
        {
            if (g_debugMode)
                PrintAndLog("DEBUG: Error - End Parity check failed");
            return false;
        }
        //test for even parity bits and remove them. (leave out the end row of parities so 36 bits)
        if (removeParity(DemodBuffer, startIdx + sizeof(preamble), 9, 0, 36) == 0)
        {
            if (g_debugMode)
                PrintAndLog("DEBUG: Error - Parity not detected");
            return false;
        }

        setDemodBuf(DemodBuffer, 32, 0);
        // setClockGrid(0,0);

        *word = bytebits_to_byteLSBF(DemodBuffer, 32);
    }
    return true;
}
void Em4095::Init()
{
    pinMode(shd, OUTPUT);
    pinMode(mod, OUTPUT);
    pinMode(demodOut, INPUT);
    pinMode(rdyClk, INPUT);
    digitalWrite(mod, LOW);
    // todo only attach on read command dont want to affect timming while sending commands
    attachInterrupt(rdyClk, onClk, RISING);
}
Em4095::Em4095(byte shd,byte mod, byte demodOut,byte rdyClk){
    this->shd=shd;
    this->mod =mod;
    this->demodOut=demodOut;
    this->rdyClk= rdyClk;
}

void Em4095::RecordFromAntenna(uint numberOfBits)
{
    if (numberOfBits > MAX_DEMOD_BUF_LEN)
    {
        return;
    }
    // preturn this is wrong
    while (0 == digitalRead(demodOut))
    { // sync on falling edge
    }
    while (1 == digitalRead(demodOut))
    { // sync on falling edge
    }
    clkCount = 0;
    while (clkCount < 5)
        ; //read offset a few lcocks to avoid the edge
    for (int i = 0; i < numberOfBits; i++)
    {
        DemodBuffer[i] = digitalRead(demodOut);
        clkCount = 0;
        while (clkCount < 64)
            ;
    }
    DemodBufferLen = numberOfBits;
    //print raw buffer
    // String data = "";

    // for (int i = 0; i < numberOfBits; i++)
    // {
    //     data += DemodBuffer[i];
    // }
    // ESP_LOGI(RF_TAG, "Read Buffer: %s", data.c_str());
}
