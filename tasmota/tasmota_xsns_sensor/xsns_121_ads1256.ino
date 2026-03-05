/*
  xsns_12_ads1256.ino - ADS1256 A/D Converter support for Tasmota

  Copyright (C) 2026 meMoriZe

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_SPI
#ifdef USE_ADS1256
/*********************************************************************************************\
 * ADS1256 - 8 channel 24BIT A/D converter
 *
 * The ADS1256 is a 24-bit ADC with 8 channels and a programmable gain amplifier. It can be
 * used to measure a wide range of analog signals, including voltage, current, and temperature.
 * The ADS1256 communicates with a microcontroller via the SPI protocol, and it can be configured
 * to operate in either single-ended or differential mode. 
 * The ADS1256 is commonly used in applications such as data acquisition, instrumentation,
 * and industrial control systems where high-resolution analog measurements are required.
 *
 * Required library: none
 *
 * The ADC input range (or gain) can be changed via the following
 * defines, but be careful never to exceed VDD +0.3V max, or to
 * exceed the upper and lower limits if you adjust the input range!
 * Setting these values incorrectly may destroy your ADC!
 * 
 * A typical module with the ADS1256 
 * https://youtu.be/u1zC8Q3vjbo?si=LKUxT6Hc6c55wml6
 * has a REF03 reference of 2.5V and a VDD of 5V, 
 * so the maximum input voltage is 5.3V and the typical input range is +/-2.5V. 
 * 
 \*********************************************************************************************/

#include <SPI.h>

#define XSNS_121                        121


// Example: Single ended channel sequence: AINp=AIN0...AIN7 vs. AINn=GND
// Example: Differential channel sequence: AINp=AIN0, AIN2, AIN4, AIN63 vs. AINn=AIN1, AIN3, AIN5, AIN7
//#define ADS1256_CHANNEL_SEQUENCE    { 0x01, 0x23, 0x45, 0x67 }


/*
static const uint8_t cChannels[6][] PROGMEM =
{
    ADS1256_CHANNEL_SEQUENCE_CS1,
    ADS1256_CHANNEL_SEQUENCE_CS2,
    ADS1256_CHANNEL_SEQUENCE_CS3,
    ADS1256_CHANNEL_SEQUENCE_CS4,
    ADS1256_CHANNEL_SEQUENCE_CS5,
    ADS1256_CHANNEL_SEQUENCE_CS6
};
*/

#define ADS1256_SCLK_FREQUENCY          ((ADS1256_CLKIN_FREQUENCY) / 8) // 1/8 of CLKIN, typically 960kHz
#define ADS1256_DATARATE                (1000)     // 1000 SPS data rate, typical for general purpose measurements, can be adjusted up to 30000 SPS for faster measurements at the cost of resolution and noise performance

/*======================================================================
TIMING AND DELAYS
-----------------------------------------------------------------------*/
#define ADS1256_CLKIN_FREQUENCY         (7680000ul) // 0.1 ... 10 MHz, typically 7.68MHz
#define ADS1256_DRATE_MAX               (30000)    // 30000 SPS max data rate
#define ADS1256_SCLK_MAX_FREQUENCY      ((ADS1256_CLKIN_FREQUENCY) / 4) // 1/4 of CLKIN, typically 1.92MHz
#define ADS1256_SCLK_MIN_FREQUENCY      ((ADS1256_DRATE_MAX) / 10) // 1/10 of max data rate, typically 3kHz

#define ADS1256_MIN_DELAY_IN_OUT_ms     (1 + 50 * 1000ul / ADS1256_CLKIN_FREQUENCY) // t6: Minimum delay last clock edge of DIN to first clock edge of DOUT in ms
#define ADS1256_MIN_CS_HOLD_TIME_ms     (1 + 8 * 1000ul / ADS1256_CLKIN_FREQUENCY)  // t10: Minimum CS need to be held low after the last clock edge of DOUT in ms
#define ADS1256_MIN_DELAY_COMMAND_ms    (1 + 50 * 1000ul / ADS1256_CLKIN_FREQUENCY) // t11: Minimum delay between the last clock edge of DOUT and the first clock edge of the next command in ms


/*======================================================================
COMMANDS
-----------------------------------------------------------------------*/
#define ADS1256_CMD_WAKEUP              0x00      // Completes SYNC and exits standby mode
#define ADS1256_CMD_RDATA               0x01      // Read data once, when in standby mode
#define ADS1256_CMD_RDATAC              0x03      // Read data continuously, i.e. in Data Ready mode
#define ADS1256_CMD_SDATAC              0x0F      // Stop read data continuously, i.e. exit Data Ready mode
#define ADS1256_CMD_RREG(reg)           0x10 + ((reg) & 0x0F) // Read from register,  register number in lower 4 bits
#define ADS1256_CMD_WREG(reg)           0x50 + ((reg) & 0x0F) // Write to register, register number in lower 4 bits
#define ADS1256_CMD2_REG(n)             (((n)-1) & 0x0F)      // Number of registers to read/write minus one in lower 4 bits
#define ADS1256_CMD_SELFCAL             0xF0      // Offset and gain self-calibration
#define ADS1256_CMD_SELFOCAL            0xF1      // Offset self-cal  ibration
#define ADS1256_CMD_SELFGCAL            0xF2      // Gain self-calibration
#define ADS1256_CMD_SYSOCAL             0xF3      // System offset calibration
#define ADS1256_CMD_SYSGCAL             0xF4      // System gain calibration
#define ADS1256_CMD_SYNC                0xFC      // Synchronize the A/D conversion
#define ADS1256_CMD_STANDBY             0xFD      // Begin standby mode
#define ADS1256_CMD_RESET               0xFE      // Reset to power-up values
#define ADS1256_CMD_WAKEUP_FF           0xFF      // Completes SYNC and exits standby mode (same as 0x00)


/*======================================================================
REGISTER MAP
-----------------------------------------------------------------------*/
#define ADS1256_REG_STATUS              0x00      // Device Status Register
#define ADS1256_REG_MUX                 0x01      // Input Multiplexer Configuration Register
#define ADS1256_REG_ADCON               0x02      // A/D Control Register
#define ADS1256_REG_DRATE               0x03      // Data Rate Register
#define ADS1256_REG_IO                  0x04      // GPIO Register
#define ADS1256_REG_OFC0                0x05      // Offset Calibration Register
#define ADS1256_REG_OFC1                0x06      // Offset Calibration Register
#define ADS1256_REG_OFC2                0x07      // Offset Calibration Register
#define ADS1256_REG_FSC0                0x08      // Full-Scale Calibration Register
#define ADS1256_REG_FSC1                0x09      // Full-Scale Calibration Register
#define ADS1256_REG_FSC2                0x0A      // Full-Scale Calibration Register

#define ADS1256_SCDS_OFF                0x00      // Sensor Detect Current Source off
#define ADS1256_SCDS_0_5uA              0x01      // Sensor Detect Current Source 0.5uA
#define ADS1256_SCDS_2uA                0x02      // Sensor Detect Current Source 2uA
#define ADS1256_SCDS_10uA               0x03      // Sensor Detect Current Source 10uA

#define ADS1256_PGA_GAIN_1              0x00      // Gain of 1
#define ADS1256_PGA_GAIN_2              0x01      // Gain of 2
#define ADS1256_PGA_GAIN_4              0x02      // Gain of 4
#define ADS1256_PGA_GAIN_8              0x03      // Gain of 8
#define ADS1256_PGA_GAIN_16             0x04      // Gain of 16
#define ADS1256_PGA_GAIN_32             0x05      // Gain of 32
#define ADS1256_PGA_GAIN_64             0x06      // Gain of 64

#define ADS1256_DRATE_30000SPS          0xF0      // 30000 samples per second
#define ADS1256_DRATE_15000SPS          0xE0      // 15000 samples per second
#define ADS1256_DRATE_7500SPS           0xD0      // 7500 samples per second
#define ADS1256_DRATE_3750SPS           0xC0      // 3750 samples per second
#define ADS1256_DRATE_2000SPS           0xB0      // 2000 samples per second
#define ADS1256_DRATE_1000SPS           0xA1      // 1000 samples per second
#define ADS1256_DRATE_500SPS            0x92      // 500 samples per second
#define ADS1256_DRATE_100SPS            0x82      // 100 samples per second
#define ADS1256_DRATE_60SPS             0x72      // 60 samples per second
#define ADS1256_DRATE_50SPS             0x63      // 50 samples per second
#define ADS1256_DRATE_30SPS             0x53      // 30 samples per second
#define ADS1256_DRATE_25SPS             0x43      // 25 samples per second
#define ADS1256_DRATE_15SPS             0x33      // 15 samples per second
#define ADS1256_DRATE_10SPS             0x23      // 10 samples per second
#define ADS1256_DRATE_5SPS              0x13      // 5 samples per second
#define ADS1256_DRATE_2_5SPS            0x03      // 2.5 samples per second

const struct {
    uint8_t drate; // data rate setting for the current channel, e.g. ADS1256_DRATE_1000SPS for 1000 samples per second
    uint32_t t18_ms; // settling time for the last channel change in ms, used to ensure minimum delay between channel change and next SYNC command
    uint32_t tcal_ms; // calibration time at autocalibtaion, if configuration changed
    uint32_t ideal_fsc; // ideal full-scale calibration value for the current channel, used for reference and diagnostics
} cTabSettlingTime[] = { // clkin is typically 7.68MHz, so t18 is typically 1ms for 30000SPS, 2ms for 15000SPS, etc.
    { ADS1256_DRATE_30000SPS, 1,    1, 0x44AC08 }, // 30kSPS
    { ADS1256_DRATE_15000SPS, 1,    1, 0x44AC08 }, // 15kSPS
    { ADS1256_DRATE_7500SPS,  1,    2, 0x44AC08 }, // 7.5kSPS
    { ADS1256_DRATE_3750SPS,  1,    2, 0x44AC08 }, // 3.75kSPS
    { ADS1256_DRATE_2000SPS,  1,    3, 0x494008 }, // 2kSPS
    { ADS1256_DRATE_1000SPS,  2,    4, 0x494008 }, // 1kSPS
    { ADS1256_DRATE_500SPS,   3,    7, 0x494008 }, // 500SPS
    { ADS1256_DRATE_100SPS,  11,   32, 0x3A99A0 }, // 100SPS
    { ADS1256_DRATE_60SPS,   17,   51, 0x4651F3 }, // 60SPS
    { ADS1256_DRATE_50SPS,   21,   62, 0x3A99A0 }, // 50SPS
    { ADS1256_DRATE_30SPS,   34,  102, 0x4651F3 }, // 30SPS
    { ADS1256_DRATE_25SPS,   41,  124, 0x3A99A0 }, // 25SPS
    { ADS1256_DRATE_15SPS,   67,  203, 0x4651F3 }, // 15SPS
    { ADS1256_DRATE_10SPS,  101,  308, 0x2EE14C }, // 10SPS
    { ADS1256_DRATE_5SPS,   201,  614, 0x2EE14C }, // 5SPS
    { ADS1256_DRATE_2_5SPS, 401, 1228, 0x2EE14C } // 2.5SPS
};

typedef struct ads1256_config_t_
{
    float ref_voltage;                  // Reference voltage in V, typically 2.5V
    uint32_t channels_used;             // Number of active channels
    uint32_t status_mux_adcon_drate[8]; // Configuration for each channel: MUX, ADCON, DRATE, and IO settings
} ads1256_config_t;


#define ADS1256_CONFIG   { 2.5f, 8, 0x000F0023, 0x001F0023, 0x002F0023, 0x003F0023, 0x004F0023, 0x005F0023, 0x006F0023, 0x007F0023 } // example configuration for single-ended channels AIN0...AIN7 vs. AINCOM with 10SPS data rate and gain of 1

const ads1256_config_t ads1256_default_config PROGMEM = ADS1256_CONFIG;

typedef struct ADS1256_DEVICE_T_
{
    int cs_pin;                         // Pin for Chip Select (CS)
    uint8_t actual_channel;             // Currently active channel (0-7)
    uint32_t busy_ms;                   // time left after the last SYNC command in milliseconds
    //
    uint32_t ofc[8];                    // Offset calibration values for each channel
    uint32_t fsc[8];                    // Full-scale calibration values for each channel
    int32_t raw_values[8];              // Last read value from the ADC
    float scaled_values[8];             // Last read value from the ADC converted to voltage using the reference and gain settings
    //
    ads1256_config_t config;            // Configuration for the device
} ADS1256_DEVICE_T;

ADS1256_DEVICE_T * mpAds1256 = nullptr; // single allocation reference for all devices, indexed by CS pin number
static uint32_t ads1256_count = 0;
static uint32_t ads1256_cs = 0;


static void Ads1256_Sync(void);

void Ads1256_BeginSPI(uint8_t dev_idx)
{
    SPI.beginTransaction(SPISettings(ADS1256_SCLK_FREQUENCY, MSBFIRST, SPI_MODE1)); // ADS1256 samples data on the falling edge of SCLK, so SPI mode 1 is used
    digitalWrite(Pin(GPIO_ADS1256_CS, mpAds1256[dev_idx].cs_pin), LOW); // select the device by pulling its CS pin low
//    delay(ADS1256_MIN_DELAY_IN_OUT_ms); // t6: Minimum delay last clock edge of DIN to first clock edge of DOUT in ms
}

void Ads1256_EndSPI(uint8_t dev_idx)
{
    digitalWrite(Pin(GPIO_ADS1256_CS, mpAds1256[dev_idx].cs_pin), HIGH); // deselect the device by pulling its CS pin high
//    delay(ADS1256_MIN_DELAY_IN_OUT_ms); // t6: Minimum delay last clock edge of DIN to first clock edge of DOUT in ms
    SPI.endTransaction();
}

uint32_t Ads1256_GetSettlingTime_ms(uint32_t cfg_last, uint32_t cfg_new)
{
    uint32_t settling_time_ms = cTabSettlingTime[sizeof(cTabSettlingTime) / sizeof(cTabSettlingTime[0]) - 1].t18_ms; // default to the longest settling time
    uint8_t drate = cfg_new & 0xFF;
    for ( uint8_t i = 0; i < sizeof(cTabSettlingTime) / sizeof(cTabSettlingTime[0]); i++ ) {
        if ( cTabSettlingTime[i].drate == drate ) {
            settling_time_ms = cTabSettlingTime[i].t18_ms;
            if (cfg_new & 0x04000000) { // autocalibration enabled, add calibration time to settling time
                if ( (cfg_last ^ cfg_new) & 0x020007FF ) { // only add calibration time if configuration has changed, otherwise use the previously added calibration time for the same configuration
                    settling_time_ms += cTabSettlingTime[i].tcal_ms;
                }
            }
            break;
        }
    }
    return settling_time_ms;
}

uint32_t Ads1256_GetIdealFSC(uint8_t drate)
{
    for ( uint8_t i = 0; i < sizeof(cTabSettlingTime) / sizeof(cTabSettlingTime[0]); i++ ) {
        if ( cTabSettlingTime[i].drate == drate ) {
            return cTabSettlingTime[i].ideal_fsc;
        }
    }
    return cTabSettlingTime[sizeof(cTabSettlingTime) / sizeof(cTabSettlingTime[0]) - 1].ideal_fsc; // return the ideal FSC for the lowest data rate if data rate not found
}

void Ads1256_NextCycle(uint8_t dev_idx)
{
    uint8_t ch, next_ch;
    uint32_t u32;
    uint8_t rx[3];
    int32_t raw_value;

    ch = mpAds1256[dev_idx].actual_channel; // current channel index (=actual configuration and conversion)

    if ( mpAds1256[dev_idx].busy_ms >= 50 ) {
        mpAds1256[dev_idx].busy_ms -= 50; // decrease the busy time by the cycle time
        return; // not yet time for next cycle, wait until minimum settling time has passed
    }
    
    next_ch = (ch + 1) % mpAds1256[dev_idx].config.channels_used; // next channel index in the sequence, wrap around to 0 after the last channel

    Ads1256_BeginSPI(dev_idx);
    SPI.transfer(ADS1256_CMD_WREG(ADS1256_REG_STATUS));
    SPI.transfer(ADS1256_CMD2_REG(4));
    SPI.transfer((uint8_t)(mpAds1256[dev_idx].config.status_mux_adcon_drate[next_ch] >> 24)); // STATUS register
    SPI.transfer((uint8_t)(mpAds1256[dev_idx].config.status_mux_adcon_drate[next_ch] >> 16)); // MUX register
    SPI.transfer((uint8_t)(mpAds1256[dev_idx].config.status_mux_adcon_drate[next_ch] >> 8));  // ADCON register
    SPI.transfer((uint8_t)mpAds1256[dev_idx].config.status_mux_adcon_drate[next_ch]);         // DRATE register
/* not here. shall be init by DRATE defintion with the ideal values or determined by calibration
    SPI.transfer((uint8_t)(mpAds1256[dev_idx].ofc[next_ch] >> 16)); // OFC0 register
    SPI.transfer((uint8_t)(mpAds1256[dev_idx].ofc[next_ch] >> 8));  // OFC1 register
    SPI.transfer((uint8_t)mpAds1256[dev_idx].ofc[next_ch]);         // OFC2 register

    SPI.transfer((uint8_t)(mpAds1256[dev_idx].fsc[next_ch] >> 16)); // FSC0 register
    SPI.transfer((uint8_t)(mpAds1256[dev_idx].fsc[next_ch] >> 8));  // FSC1 register
    SPI.transfer((uint8_t)mpAds1256[dev_idx].fsc[next_ch]);         // FSC2 register
*/
    SPI.transfer(ADS1256_CMD_SYNC); // Send the SYNC command to all selected devices
    SPI.transfer(ADS1256_CMD_WAKEUP); // Send the WAKEUP command to all selected devices
    SPI.transfer(ADS1256_CMD_RDATA); // Send the RDATA command to read the conversion result
    delay(ADS1256_MIN_DELAY_COMMAND_ms); // t11: Minimum delay between the last clock edge of DOUT and the first clock edge of the next command in ms
    rx[0] = SPI.transfer(0xFF); // Read the first byte (MSB)
    rx[1] = SPI.transfer(0xFF); // Read the second byte
    rx[2] = SPI.transfer(0xFF); // Read the third byte
    Ads1256_EndSPI(dev_idx);

    raw_value = ((int32_t)rx[0] << 16) | ((int32_t)rx[1] << 8) | rx[2]; // Combine the three bytes into a 24-bit signed integer
    if (raw_value & 0x800000) { // If the sign bit is set, convert to negative value
        raw_value = raw_value - 0x1000000;
    }
    mpAds1256[dev_idx].raw_values[ch] = raw_value; // Store the raw value for this channel
    mpAds1256[dev_idx].busy_ms = 1 + Ads1256_GetSettlingTime_ms(mpAds1256[dev_idx].config.status_mux_adcon_drate[ch], 
                                                            mpAds1256[dev_idx].config.status_mux_adcon_drate[next_ch]); // Set the busy time for the next cycle based on the current channel's configuration
    mpAds1256[dev_idx].actual_channel = next_ch; // Update the actual channel index to the next channel for the next cycle
}


void Ads1256_Every50ms(void)
{
    uint8_t i;
    for( i = 0; i < ads1256_count; i++ ) {
        Ads1256_NextCycle(i);
    }
}


void Ads1256_EverySecond(void)
{
    // TODO: calculate float values from raw values using the reference voltage and gain settings, and log them or make them available for JSON and web server output

}


void Ads1256Label(char* label, uint32_t maxsize, uint32_t device) {
    // Create the identifier of the the selected sensor
    // "ADS1256":{"A0":3240,"A1":3235,"A2":3269,"A3":3269}
    snprintf_P(label, maxsize, PSTR("ADS1256"));
    // "ADS1256-1":{"A0":3240,"A1":3235,"A2":3269,"A3":3269,"A4":3269,"A5":3269,"A6":3269,"A7":3269},"ADS1256-2":{"A0":3240,"A1":3235,"A2":3269,"A3":3269}
    snprintf_P(label, maxsize, PSTR("%s%cCS%u"), label, IndexSeparator(), mpAds1256[device].cs_pin+1);
}


void Ads1256Show(bool json) {
    uint32_t i,j;
    char label[16];
    for ( i = 0; i < ads1256_count; i++) {
        Ads1256Label(label, sizeof(label), i);
        if (json) {
            ResponseAppend_P(PSTR(",\"%s\":{"), label);
            for (j = 0; j < mpAds1256[i].config.channels_used; j++) {
                ResponseAppend_P(PSTR("%s\"A%d\":%d"), (0 == j) ? "" : ",", j, mpAds1256[i].raw_values[j] );
            }
            ResponseJsonEnd();
        }
#ifdef USE_WEBSERVER
        else {
            for (j = 0; j < mpAds1256[i].config.channels_used; j++) {
                WSContentSend_PD(HTTP_SNS_ANALOG, label, j, mpAds1256[i].raw_values[j] );
            }
        }
    }
#endif  // USE_WEBSERVER
}


/********************************************************************************************/

void Ads1256Init(void) {
    int i, pin;
    uint32_t u32;
    ADS1256_DEVICE_T * pDevice = nullptr;

    u32 = 0; // bitmask for configured CS pins    
    for( i = 0; i < MAX_ADS1256; i++ ) {
        if ( Pin(GPIO_ADS1256_CS, i) >= 0 ) { // check for configured chip select pin
            ads1256_count++; // count configured devices
            u32 |= (1UL << i); // set the bit for this CS pin in the global mask
        }
    }
    if (ads1256_count == 0) {
        return; // No devices configured
    } else {
        mpAds1256 = (ADS1256_DEVICE_T *)(malloc(ads1256_count * sizeof(ADS1256_DEVICE_T)));
        if (mpAds1256 == nullptr) {
            AddLog(LOG_LEVEL_ERROR, PSTR("ADS1256: Memory allocation failed´%u"), ads1256_count);
            return;
        } else {
//    memset(mpAds1256, 0, ads1256_count * sizeof(ADS1256_DEVICE_T)); // Clear the device array
            AddLog(LOG_LEVEL_INFO, PSTR("ADS1256: %u device(s) configured"), ads1256_count);
        }
    }
    
    pDevice = mpAds1256; // first device
    for( i=0; i < MAX_ADS1256; i++ ) {
        if (u32 & 1u) { // if the least significant bit is set
            pDevice->cs_pin = i; // assign the CS pin to the device structure
            pDevice->config = ads1256_default_config; // assign the default configuration to the device structure
            pDevice++; // move to the next device structure for the next configured device
        }
        u32 >>= 1; // shift the bitmask to check the next bit
    }

#ifdef ESP8266
        SPI.begin();
#endif // ESP8266
#ifdef ESP32
        SPI.begin(Pin(GPIO_SPI_CLK), Pin(GPIO_SPI_MISO), Pin(GPIO_SPI_MOSI), -1);
#endif // ESP32

    // Initialize each configured device:
    // Coldstart: delay for power stabilization and the automatic self calibration that happens after power-up
    // RESET, SELFCAL, and STANDBY to prepare for the first conversion
    // 2. STANDBY
    delay(20);
    // all devices share the same SPI bus, so we can initialize them all in one go
    for( i = 0; i < ads1256_count; i++ ) {  
        pin = Pin(GPIO_ADS1256_CS, mpAds1256[i].cs_pin);
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW); // select the device
        delay(2); // Wait for the CS to settle after selecting the device

        SPI.beginTransaction(SPISettings(ADS1256_SCLK_FREQUENCY, MSBFIRST, SPI_MODE1)); // ADS1256 samples data on the falling edge of SCLK, so SPI mode 1 is used
        SPI.transfer(ADS1256_CMD_RREG(ADS1256_REG_STATUS)); // Read the status register to check if the device is responsive
        SPI.transfer(ADS1256_CMD2_REG(1)); // Read the status register to check if the device is responsive
        delay(ADS1256_MIN_DELAY_IN_OUT_ms); // t6: Minimum delay last clock edge of DIN to first clock edge of DOUT in clkin periods
        u32 = SPI.transfer(0xFF); // Read the status register value
        SPI.endTransaction();
        delay(ADS1256_MIN_CS_HOLD_TIME_ms); // Wait for action complete before deselecting the device
        digitalWrite(pin, HIGH); // deselect the device
        if (u32 == 0xFF) {
            AddLog(LOG_LEVEL_ERROR, PSTR("ADS1256.CS%u: No response, check wiring and CS pin configuration"), i+1);
//            mpAds1256[i].cs_pin = -1; // Mark this device as unconfigured to skip it in the future
            continue; // Skip initialization for this device
        } else {
            AddLog(LOG_LEVEL_INFO, PSTR("ADS1256.CS%u: Device present, Status=0x%02X"), i+1, u32); // 0x30
        }
    }
    delay(ADS1256_MIN_DELAY_COMMAND_ms); // ensure min deselect time before next action
}


/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns121(uint32_t function)
{
    bool result = false;
    bool spi_enabled = false;

    spi_enabled = (SPI_MOSI_MISO == TasmotaGlobal.spi_enabled);
    if (!spi_enabled) { return false; }

    if (FUNC_INIT == function) {
        Ads1256Init();
    }
    else if (ads1256_count) {
        switch (function) {
        case FUNC_EVERY_50_MSECOND:
            Ads1256_Every50ms();
            break;
        case FUNC_EVERY_SECOND:
            Ads1256_EverySecond();
            break;
        case FUNC_JSON_APPEND:
            Ads1256Show(1);
            break;
#ifdef USE_WEBSERVER
        case FUNC_WEB_SENSOR:
            Ads1256Show(0);
            break;
#endif  // USE_WEBSERVER
        case FUNC_COMMAND_SENSOR:
//            if (XSNS_12 == XdrvMailbox.index) {
//                result = ADS1256_Command();
//            }
            break;
    }
  }
  return result;
}

#endif  // USE_ADS1256
#endif  // USE_SPI
