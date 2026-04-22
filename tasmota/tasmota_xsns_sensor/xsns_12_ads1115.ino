/*
  xsns_12_ads1115_ada.ino - ADS1115 A/D Converter support for Tasmota

  Copyright (C) 2021  Syssi, stefanbode, meMoriZe

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

#ifdef USE_I2C
#ifdef USE_ADS1115
/*********************************************************************************************\
 * ADS1115 - 4 channel 16BIT A/D converter
 *
 * Required library: none but based on Adafruit Industries ADS1015 library
 *
 * I2C Address: 0x48, 0x49, 0x4A or 0x4B
 *
 * The ADC input range (or gain) can be changed via the following
 * defines, but be careful never to exceed VDD +0.3V max, or to
 * exceed the upper and lower limits if you adjust the input range!
 * Setting these values incorrectly may destroy your ADC!
 *                                                                 ADS1115
 *                                                                 -------
 * ADS1115_REG_CONFIG_PGA_6_144V  // 2/3x gain +/- 6.144V  1 bit = 0.1875mV (default)
 * ADS1115_REG_CONFIG_PGA_4_096V  // 1x gain   +/- 4.096V  1 bit = 0.125mV
 * ADS1115_REG_CONFIG_PGA_2_048V  // 2x gain   +/- 2.048V  1 bit = 0.0625mV
 * ADS1115_REG_CONFIG_PGA_1_024V  // 4x gain   +/- 1.024V  1 bit = 0.03125mV
 * ADS1115_REG_CONFIG_PGA_0_512V  // 8x gain   +/- 0.512V  1 bit = 0.015625mV
 * ADS1115_REG_CONFIG_PGA_0_256V  // 16x gain  +/- 0.256V  1 bit = 0.0078125mV
\*********************************************************************************************/

#define XSNS_12                         12
#define XI2C_13                         13        // See I2CDEVICES.md

#define ADS1115_ADDRESS_ADDR_GND        0x48      // address pin low (GND)
#define ADS1115_ADDRESS_ADDR_VDD        0x49      // address pin high (VCC)
#define ADS1115_ADDRESS_ADDR_SDA        0x4A      // address pin tied to SDA pin
#define ADS1115_ADDRESS_ADDR_SCL        0x4B      // address pin tied to SCL pin

#define ADS1115_DEBUG_ALERT_DRDY_PIN    1         // set to 1 to enable ALERT/RDY pin as Data Ready signal for debugging, pull-up resistor required

#ifdef USE_RULES
// +-1 percent fullscale change threshold for rules processing
#define ADS1115_RULES_CHANGE_THRESHOLD  ((1 * (1UL << 15)) / 100)
#endif

#define ADS1115_SINGLE_CHANNELS         (4)
#define ADS1115_DIFFERENTIAL_CHANNELS   (2)

/*======================================================================
POINTER REGISTER
-----------------------------------------------------------------------*/
#define ADS1115_REG_POINTER_MASK        (0x03)
#define ADS1115_REG_POINTER_CONVERT     (0x00)
#define ADS1115_REG_POINTER_CONFIG      (0x01)
#define ADS1115_REG_POINTER_LOWTHRESH   (0x02)
#define ADS1115_REG_POINTER_HITHRESH    (0x03)

/*======================================================================
CONFIG REGISTER
-----------------------------------------------------------------------*/
#define ADS1115_REG_CONFIG_OS_MASK      (0x8000)
#define ADS1115_REG_CONFIG_OS_SINGLE    (0x8000)  // Write: Set to start a single-conversion
#define ADS1115_REG_CONFIG_OS_BUSY      (0x0000)  // Read: Bit = 0 when conversion is in progress
#define ADS1115_REG_CONFIG_OS_NOTBUSY   (0x8000)  // Read: Bit = 1 when device is not performing a conversion

#define ADS1115_REG_CONFIG_MUX_MASK     (0x7000)
#define ADS1115_REG_CONFIG_MUX_DIFF_0_1 (0x0000)  // Differential P = AIN0, N = AIN1 (default)
#define ADS1115_REG_CONFIG_MUX_DIFF_0_3 (0x1000)  // Differential P = AIN0, N = AIN3
#define ADS1115_REG_CONFIG_MUX_DIFF_1_3 (0x2000)  // Differential P = AIN1, N = AIN3
#define ADS1115_REG_CONFIG_MUX_DIFF_2_3 (0x3000)  // Differential P = AIN2, N = AIN3
#define ADS1115_REG_CONFIG_MUX_SINGLE_0 (0x4000)  // Single-ended AIN0
#define ADS1115_REG_CONFIG_MUX_SINGLE_1 (0x5000)  // Single-ended AIN1
#define ADS1115_REG_CONFIG_MUX_SINGLE_2 (0x6000)  // Single-ended AIN2
#define ADS1115_REG_CONFIG_MUX_SINGLE_3 (0x7000)  // Single-ended AIN3
#define ADS1115_REG_CONFIG_MUX_SINGLE(ch)   (0x4000 + (ch)*0x1000)  // Single-ended

#define ADS1115_REG_CONFIG_PGA_MASK     (0x0E00)
#define ADS1115_REG_CONFIG_PGA_6_144V   (0x0000)  // +/-6.144V range = Gain 2/3 (default)
#define ADS1115_REG_CONFIG_PGA_4_096V   (0x0200)  // +/-4.096V range = Gain 1
#define ADS1115_REG_CONFIG_PGA_2_048V   (0x0400)  // +/-2.048V range = Gain 2
#define ADS1115_REG_CONFIG_PGA_1_024V   (0x0600)  // +/-1.024V range = Gain 4
#define ADS1115_REG_CONFIG_PGA_0_512V   (0x0800)  // +/-0.512V range = Gain 8
#define ADS1115_REG_CONFIG_PGA_0_256V   (0x0A00)  // +/-0.256V range = Gain 16

#define ADS1115_REG_CONFIG_MODE_MASK    (0x0100)
#define ADS1115_REG_CONFIG_MODE_CONTIN  (0x0000)  // Continuous conversion mode
#define ADS1115_REG_CONFIG_MODE_SINGLE  (0x0100)  // Power-down single-shot mode (default)

#define ADS1115_REG_CONFIG_DR_MASK      (0x00E0)
#define ADS1115_REG_CONFIG_DR_8SPS      (0x0000)  // 8 samples per second (for cyle time 250ms = 4 cycles per second)
#define ADS1115_REG_CONFIG_DR_16SPS     (0x0020)  // 16 samples per second (for cycle time 100ms = 10 cycles per second)
#define ADS1115_REG_CONFIG_DR_32SPS     (0x0040)  // 32 samples per second (for cycle time 50ms = 20 cycles per second)
#define ADS1115_REG_CONFIG_DR_64SPS     (0x0060)  // 64 samples per second
#define ADS1115_REG_CONFIG_DR_128SPS    (0x0080)  // 128 samples per second (default)
#define ADS1115_REG_CONFIG_DR_250SPS    (0x00A0)  // 250 samples per second
#define ADS1115_REG_CONFIG_DR_475SPS    (0x00C0)  // 475 samples per second
#define ADS1115_REG_CONFIG_DR_860SPS    (0x00E0)  // 860 samples per second

#define ADS1115_REG_CONFIG_CMODE_MASK   (0x0010)
#define ADS1115_REG_CONFIG_CMODE_TRAD   (0x0000)  // Traditional comparator with hysteresis (default)
#define ADS1115_REG_CONFIG_CMODE_WINDOW (0x0010)  // Window comparator

#define ADS1115_REG_CONFIG_CPOL_MASK    (0x0008)
#define ADS1115_REG_CONFIG_CPOL_ACTVLOW (0x0000)  // ALERT/RDY pin is low when active (default)
#define ADS1115_REG_CONFIG_CPOL_ACTVHI  (0x0008)  // ALERT/RDY pin is high when active

#define ADS1115_REG_CONFIG_CLAT_MASK    (0x0004)  // Determines if ALERT/RDY pin latches once asserted
#define ADS1115_REG_CONFIG_CLAT_NONLAT  (0x0000)  // Non-latching comparator (default)
#define ADS1115_REG_CONFIG_CLAT_LATCH   (0x0004)  // Latching comparator

#define ADS1115_REG_CONFIG_CQUE_MASK    (0x0003)
#define ADS1115_REG_CONFIG_CQUE_1CONV   (0x0000)  // Assert ALERT/RDY after one conversions
#define ADS1115_REG_CONFIG_CQUE_2CONV   (0x0001)  // Assert ALERT/RDY after two conversions
#define ADS1115_REG_CONFIG_CQUE_4CONV   (0x0002)  // Assert ALERT/RDY after four conversions
#define ADS1115_REG_CONFIG_CQUE_NONE    (0x0003)  // Disable the comparator and put ALERT/RDY in high state (default)

#define ADS1115_DEFAULT_DATARATE  ADS1115_REG_CONFIG_DR_32SPS

#define ADS1115_CONFIG_A         (ADS1115_REG_CONFIG_OS_SINGLE    | \
                                  ADS1115_REG_CONFIG_MODE_SINGLE  | \
                                  ADS1115_REG_CONFIG_CLAT_NONLAT  | \
                                  ADS1115_REG_CONFIG_CPOL_ACTVHI  | \
                                  ADS1115_REG_CONFIG_CMODE_TRAD   | \
                                  ADS1115_DEFAULT_DATARATE          ) 
#if ADS1115_DEBUG_ALERT_DRDY_PIN == 1  
#define ADS1115_CONFIG_COMMON     (ADS1115_CONFIG_A                | \
                                   ADS1115_REG_CONFIG_CQUE_1CONV     )
#else
#define ADS1115_CONFIG_COMMON     (ADS1115_CONFIG_A                | \
                                   ADS1115_REG_CONFIG_CQUE_NONE      )
#endif

#define ADS1115_MAX_CONFIGS 4

/********************************************************************************************/
/* types */
typedef struct 
{
  int16_t last_values[ADS1115_MAX_CONFIGS];
  uint16_t config[ADS1115_MAX_CONFIGS];
  uint8_t cfg_idx;        // actual config index
  uint8_t address; 
  uint8_t bus;
#ifdef USE_RULES
  uint8_t changed_bit;
#endif
} ADS1115_t;

/********************************************************************************************/
/* ROM (const) */
const uint8_t ads1115_addresses[] PROGMEM = { ADS1115_ADDRESS_ADDR_GND, 
                                              ADS1115_ADDRESS_ADDR_VDD, 
                                              ADS1115_ADDRESS_ADDR_SDA, 
                                              ADS1115_ADDRESS_ADDR_SCL };

const float ads1115_fs_factors[] PROGMEM = { (float)(6.144 / 32768.0),
                                             (float)(4.096 / 32768.0),
                                             (float)(2.048 / 32768.0),
                                             (float)(1.024 / 32768.0),
                                             (float)(0.512 / 32768.0),
                                             (float)(0.256 / 32768.0),
                                             (float)(0.256 / 32768.0),
                                             (float)(0.256 / 32768.0) };

const uint16_t ads1115_ranges[] PROGMEM = { ADS1115_REG_CONFIG_PGA_6_144V, 
                                            ADS1115_REG_CONFIG_PGA_4_096V, 
                                            ADS1115_REG_CONFIG_PGA_2_048V, 
                                            ADS1115_REG_CONFIG_PGA_1_024V, 
                                            ADS1115_REG_CONFIG_PGA_0_512V, 
                                            ADS1115_REG_CONFIG_PGA_0_256V };

const uint16_t ads1115_dr[] PROGMEM = { ADS1115_REG_CONFIG_DR_8SPS, 
                                        ADS1115_REG_CONFIG_DR_16SPS, 
                                        ADS1115_REG_CONFIG_DR_32SPS }; // we do not block, so higher data rates makes no sense with cycling of 50ms

const char ADS1115_HTTP_SNS_F_VOLT[]      PROGMEM = "{s}%s "  D_VOLTAGE             "%d {m}% 1.6f " D_UNIT_VOLT              "{e}";
const char ADS1115_HTTP_SNS_F_MILLIVOLT[] PROGMEM = "{s}%s "  D_VOLTAGE             "%d {m}% 4.3f " D_UNIT_MILLIVOLT         "{e}";
// default configurations
const uint16_t ADS1115_DEF_CFGS_SE[ADS1115_MAX_CONFIGS] PROGMEM = { ADS1115_REG_CONFIG_MUX_SINGLE_0 | ADS1115_CONFIG_COMMON,
                                                                    ADS1115_REG_CONFIG_MUX_SINGLE_1 | ADS1115_CONFIG_COMMON,
                                                                    ADS1115_REG_CONFIG_MUX_SINGLE_2 | ADS1115_CONFIG_COMMON,
                                                                    ADS1115_REG_CONFIG_MUX_SINGLE_3 | ADS1115_CONFIG_COMMON };

const uint16_t ADS1115_DEF_CFGS_DF[ADS1115_MAX_CONFIGS] PROGMEM = { ADS1115_REG_CONFIG_MUX_DIFF_0_1 | ADS1115_CONFIG_COMMON,
                                                                    ADS1115_REG_CONFIG_MUX_DIFF_2_3 | ADS1115_CONFIG_COMMON,
                                                                    0,
                                                                    0};

/********************************************************************************************/
/* RAM */
uint8_t ads1115_count = 0;
uint8_t ads1115_units;
ADS1115_t * Ads1115;


/********************************************************************************************/
void Ads1115Config(uint32_t devices_bit, uint16_t * configs) {
  for (uint32_t t = 0; t < ads1115_count; t++) { // for each device (they can be on different busses and convert same time)
    if (bitRead(devices_bit, t)) {
      memcpy(Ads1115[t].config, configs, sizeof(uint16_t) * ADS1115_MAX_CONFIGS); // update config for this device
    }
  }
}

/********************************************************************************************/
void Ads1115Detect(void) {
#define ADS1115_I2C_MAX_BUSES   MAX_I2C
#if ADS1115_I2C_MAX_BUSES > 8   /* 32 / sizeof(ads1115_addresses) */
  #undef ADS1115_I2C_MAX_BUSES
  #define ADS1115_I2C_MAX_BUSES   8 /* limit to 8 busses */
#endif
  uint32_t found_bits = 0; /* up to 4 device per bus, bit0..bit3: found device on bus0, bit4..bit7: found device on bus1 ... */
  for (uint32_t bus = 0; bus < ADS1115_I2C_MAX_BUSES; bus++) {

    for (uint32_t i = 0; i < sizeof(ads1115_addresses); i++) {
      if (!I2cSetDevice(ads1115_addresses[i], bus)) { continue; }
      uint16_t buffer;
      if (I2cValidRead16(&buffer, ads1115_addresses[i], ADS1115_REG_POINTER_CONVERT, bus) &&
          I2cValidRead16(&buffer, ads1115_addresses[i], ADS1115_REG_POINTER_CONFIG, bus)) {
        found_bits |= (1 << (bus * sizeof(ads1115_addresses) + i));
        ads1115_count++;
      }
    }
  }
  if (!ads1115_count) {
    return; // no device found, driver not active
  }
  Ads1115 = (ADS1115_t*)malloc(sizeof(ADS1115_t) * ads1115_count);
  ads1115_count = 0;
  if (Ads1115 == nullptr) {
//    AddLog(LOG_LEVEL_ERROR, PSTR("ADS1115: Failed to allocate memory for %d devices"), ads1115_count);
    return;
  }
  ads1115_units = 0;
  memset(Ads1115, 0, sizeof(ADS1115_t) * ads1115_count); // init last values and changed bits
  for (uint32_t bus = 0; bus < ADS1115_I2C_MAX_BUSES; bus++) {
    for (uint32_t i = 0; i < sizeof(ads1115_addresses); i++) {
      if (found_bits & 1) {
        // initialize default config
        for( uint32_t cfg = 0; cfg < ADS1115_MAX_CONFIGS; cfg++) {
          Ads1115[ads1115_count].config[cfg] = ADS1115_REG_CONFIG_MUX_SINGLE(cfg % 4) | ADS1115_CONFIG_COMMON | ADS1115_REG_CONFIG_PGA_6_144V;
        }
        Ads1115[ads1115_count].address = ads1115_addresses[i];
        Ads1115[ads1115_count].bus = bus;
        I2cSetActiveFound(Ads1115[ads1115_count].address, "ADS1115", bus);
        ads1115_count++;
#if ADS1115_DEBUG_ALERT_DRDY_PIN == 1        
        I2cWrite16(ads1115_addresses[i], ADS1115_REG_POINTER_LOWTHRESH, 0x7FFF, bus);
        I2cWrite16(ads1115_addresses[i], ADS1115_REG_POINTER_HITHRESH,  0x8000, bus);
#endif
        found_bits >>= 1;
      }
    }
  }
  Ads1115Cycle(true); // only config for first run
}

/********************************************************************************************/
void Ads1115Label(char* label, uint32_t maxsize, uint32_t device) {
  // Create the identifier of the the selected sensor
  // "ADS1115":{"A0":3240,"A1":3235,"A2":3269,"A3":3269}
  snprintf_P(label, maxsize, PSTR("ADS1115"));
  if (ads1115_count > 1) {
    // "ADS1115-48":{"A0":3240,"A1":3235,"A2":3269,"A3":3269},"ADS1115-49":{"A0":3240,"A1":3235,"A2":3269,"A3":3269}
    snprintf_P(label, maxsize, PSTR("%s%c%02X"), label, IndexSeparator(), Ads1115[device].address);
#if MAX_I2C > 1
    if (TasmotaGlobal.i2c_enabled[1] &&                       // Second bus enabled
        (Ads1115[0].bus != Ads1115[ads1115_count -1].bus)) {  // Different busses
      // "ADS1115-48-1":{"A0":3240,"A1":3235,"A2":3269,"A3":3269},"ADS1115-48-2":{"A0":3240,"A1":3235,"A2":3269,"A3":3269}
      snprintf_P(label, maxsize, PSTR("%s%c%d"), label, IndexSeparator(), Ads1115[device].bus +1);
    }
#endif  // MAX_I2C
  }
}

/********************************************************************************************/
void Ads1115Cycle(bool config_only) {
  uint16_t config;
  int16_t value;
  ADS1115_t device;

  for (uint32_t t = 0; t < ads1115_count; t++) { // for each device (they can be on different busses and convert same time)
    device = Ads1115[t]; // optimize struct access by local copy, as it is used multiple times in this loop
    // I2C: read actual status and config
    config = I2cRead16(device.address, ADS1115_REG_POINTER_CONFIG, device.bus);
    if (config & ADS1115_REG_CONFIG_OS_NOTBUSY) {
      if (config_only == false) { // not only config, also read last conversion result
        if (config == device.config[device.cfg_idx]) { // config is as expected
          // I2C: read conversion result
          value = I2cRead16(device.address, ADS1115_REG_POINTER_CONVERT, device.bus);
#ifdef USE_RULES
          if (_abs(value - device.last_values[device.cfg_idx]) > ADS1115_RULES_CHANGE_THRESHOLD) {
            bitSet(device.changed_bit , device.cfg_idx); // relevant change, set bit for this config index
          }      
#endif
          device.last_values[device.cfg_idx] = value; // update stored value
        } // else config is not as expected, keep last stored value TODO: error-counter ?
      } // else only config, no read
      device.cfg_idx++; // next configuration
      if (device.cfg_idx >= ADS1115_MAX_CONFIGS) {
        device.cfg_idx = 0; // reset to first config index
      } else {
        if (!(device.config[device.cfg_idx] & ADS1115_REG_CONFIG_OS_SINGLE)) {
          device.cfg_idx = 0; // reset to first config index
        }
      }
      // I2C: write next config and start next conversion
      I2cWrite16(device.address, ADS1115_REG_POINTER_CONFIG, device.config[device.cfg_idx], device.bus);
    } // else busy, nothing to do now
#ifdef USE_RULES
    if (device.cfg_idx == 0) { // all configs done for this device
      if (device.changed_bit) { // if there are changed values for this device
        char label[16];
        Ads1115Label(label, sizeof(label), t);
        Response_P(PSTR("{\"%s\":{"), label);
        bool first = true;
        for (uint32_t i = 0; i < ADS1115_MAX_CONFIGS; i++) {
          if (bitRead(device.changed_bit, i)) {
            float fValue = 1.0f;
            switch(ads1115_units) {
              case 2: // millivolt
                fValue = 1000.0f; // fallthrough
              case 1: // volt
                fValue *= (float)(device.last_values[i]) * ads1115_fs_factors[(device.config[i] >> 9) & 0x07];
                ResponseAppend_P(PSTR("%s\"A%ddiv10\":%f"), (first) ? "" : ",", i, fValue);
                break;
              default: // raw value
                ResponseAppend_P(PSTR("%s\"A%ddiv10\":%d"), (first) ? "" : ",", i, device.last_values[i]);
            }
            first = false;
          }
        }
        ResponseJsonEndEnd();
        XdrvRulesProcess(0);
        device.changed_bit = 0;
      }
    }
#endif
    Ads1115[t] = device; // update device struct with new config and possibly changed bit and last values
  } // for each device
}

/********************************************************************************************/
void Ads1115Show(bool json) {
  float fValue;

  for (uint32_t t = 0; t < ads1115_count; t++) {
//    AddLog(LOG_LEVEL_INFO, "Logging ADS1115 %02x", Ads1115[t].address);
    char label[16];
    Ads1115Label(label, sizeof(label), t);
    if (json) {
      ResponseAppend_P(PSTR(",\"%s\":{"), label);
      for (uint32_t i = 0; i < ADS1115_MAX_CONFIGS; i++) {
        switch(ads1115_units) {
          case 2: // millivolt
            fValue = 1000.0f; // fallthrough
          case 1: // volt
            fValue *= (float)((Ads1115[t].last_values[i]) * ads1115_fs_factors[(Ads1115[t].config[i] >> 9) & 0x07]);
            ResponseAppend_P(PSTR("%s\"A%d\":%f"), (0 == i) ? "" : ",", i, fValue);
            break;
          default: // raw value
            ResponseAppend_P(PSTR("%s\"A%d\":%d"), (0 == i) ? "" : ",", i, Ads1115[t].last_values[i]);
        }
      }
      ResponseJsonEnd();
    }
#ifdef USE_WEBSERVER
    else {
      for (uint32_t i = 0; i < ADS1115_MAX_CONFIGS; i++) {
        if (ads1115_units == 1 || ads1115_units == 2) { // volt or millivolt
          fValue = (float)(Ads1115[t].last_values[i]) * ads1115_fs_factors[(Ads1115[t].config[i] >> 9) & 0x07];
        }
        if (ads1115_units == 1) { // volt
          WSContentSend_PD(ADS1115_HTTP_SNS_F_VOLT, label, i, fValue);
        } else
        if (ads1115_units == 2) { // millivolt
          fValue *= 1000.0f;
          WSContentSend_PD(ADS1115_HTTP_SNS_F_MILLIVOLT, label, i, fValue);
        } else { // default raw value
          WSContentSend_PD(HTTP_SNS_ANALOG, label, i, Ads1115[t].last_values[i]);
        }
      }
    }
#endif  // USE_WEBSERVER
  }
}

/********************************************************************************************/
// 'Sensor12 Dn' : Differential Channels 01 and 23 (=MUX settings), n:0...5 for range (=PGA settings)
// 'Sensor12 Sn' : Single ended Channels 0123 (=MUX settings), n:0...5 for range (=PGA settings)
// new:
// 'Sensor12 Ux' : Unit selection, x: R=raw (default), V=volt, M=millivolt
// 'Sensor12 Tn' : Data rate selection, n: 0=8SPS, 1=16SPS, 2=32SPS (default)
// TODO 'Sensor12 Caa<bb<cc<dd>>> <dev> : Set Configuration aa=[0],bb=[1],cc=[2], dd=[3], dev=(bus * 100 + I2C address)
bool ADS1115_Command(void) {
  uint16_t configs[ADS1115_MAX_CONFIGS];
  uint8_t mode;
  uint16_t ads1115_range;
  // Sensor12 D2
  // Sensor12 S0
  if (XdrvMailbox.data_len > 1) {
    UpperCase(XdrvMailbox.data, XdrvMailbox.data);
    switch (XdrvMailbox.data[0]) {
      case 'D':
        memcpy( configs, ADS1115_DEF_CFGS_DF, sizeof(configs) );
        mode = 0;
        break;
      case 'S':
        memcpy( configs, ADS1115_DEF_CFGS_SE, sizeof(configs) );  
        mode = 1;
        break;
    }
    uint32_t number = atoi((const char*)XdrvMailbox.data +1);    

    if (XdrvMailbox.data[0] == 'D' || XdrvMailbox.data[0] == 'S') {
      if ((number >= 0) && (number <= 5)) {
        ads1115_range = ads1115_ranges[number];
        for (uint32_t i = 0; i < ADS1115_MAX_CONFIGS; i++) {
          configs[i] |= ads1115_range;
        }
        Ads1115Config(0xFFFFFFFF, configs); // apply to all devices
        const char ds[2][13] = { "Differential", "Single ended" };
        const uint16_t r[6] = { 6144, 4096, 2048, 1024, 512, 256 };
        Response_P("{\"ADS1115\":{\"Settings\":\"%c%u\",\"Mode\":\"%s\",\"Range\":%u,\"Unit\":\"mV\"}}",
          XdrvMailbox.data[0], number, ds[mode], r[number]);
      }
    } else
    if (XdrvMailbox.data[0] == 'U') { // output unit selection (HTTP, JSON and rules output)
      if ((number >= 0) && (number <= 2)) { // 0=raw[+-15bit counts], 1=Volt[V], 2=milliVolt[mV]
        ads1115_units = number;
        Response_P("{\"ADS1115\":{\"Unit\":\"%s\"}}", (ads1115_units == 1) ? "Volt" : (ads1115_units == 2) ? "milliVolt" : "Raw");
      }
    } else
    if (XdrvMailbox.data[0] == 'T') { // data rate selection
      if ((number >= 0) && (number <= 2)) {
        uint16_t dr = ads1115_dr[number];
        for (uint32_t i = 0; i < ADS1115_MAX_CONFIGS; i++) {
          configs[i] = (Ads1115[0].config[i] & ~ADS1115_REG_CONFIG_DR_MASK) | dr;
        }
        Ads1115Config(0xFFFFFFFF, configs); // apply to all devices
        const uint16_t sps[8] = { 8, 16, 32};
        Response_P("{\"ADS1115\":{\"DataRate\":%u}}", sps[number]);
      }
    }
  }
  return true;
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns12(uint32_t function)
{
  if (!I2cEnabled(XI2C_13)) { return false; }

  bool result = false;

  if (FUNC_INIT == function) {
    Ads1115Detect();
  }
  else if (ads1115_count) {
    switch (function) {
      case FUNC_EVERY_50_MSECOND:
        Ads1115Cycle(false);          
        break;
      case FUNC_JSON_APPEND:
        Ads1115Show(1);
        break;
#ifdef USE_WEBSERVER
      case FUNC_WEB_SENSOR:
        Ads1115Show(0);
        break;
#endif  // USE_WEBSERVER
      case FUNC_COMMAND_SENSOR:
        if (XSNS_12 == XdrvMailbox.index) {
          result = ADS1115_Command();
        }
        break;
    }
  }
  return result;
}

#endif  // USE_ADS1115
#endif  // USE_I2C
