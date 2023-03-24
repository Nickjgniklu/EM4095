#include <Em4095.h>
#define SHD 4
#define MOD 27
#define DEMODOUT 16
#define READYCLK 15
Em4095 rfid(4, 27, 16, 15);

void setup()
{
    rfid.Init();
    // put your setup code here, to run once:
    double freq = rfid.calcResonantFreq();
    Serial.printf("Antenna Freq: %f", freq);
}

void loop()
{
    // put your main code here, to run repeatedly:

    RfidResult data = rfid.ReadTag(0x06);
    if (!data.error)
    {
        Serial.println(data.data);
    }

    rfid.WriteTag(0x06, 0x64);
    // rfid.Disable();

    delay(50);
}