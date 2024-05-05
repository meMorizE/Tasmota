/*
  xnrg_25_mk_sky_blu.ino - MakeSkyBlue Solar charger support for Tasmota

  Copyright (C) 2024  meMorizE

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

#ifdef USE_ENERGY_SENSOR
#ifdef USE_MAKE_SKY_BLUE
/*********************************************************************************************\
 * This implementation communicates with solar charge controller of MakeSkyBlue(Shenzen) Co.LTD 
 * http://www.makeskyblue.com
 * Model: S3-30A/40A/50A/60A MPPT 12V/24V/36V/48V with firmwware V118 (or V119 including a Wifi Box)
 * The device should have a Mini-USB socket at bottom right beside the screw terminals.
 * This socket conforms NOT to the USB standard protocol !
 * It is a TTL serial interface with 9600bps 8N1, D+=Tx D-=Rx plus regular 5V out and GND
 * When using a DIY ESP hardware be aware of the 5V levels by using e.g. level shifter 
 * 
 * The orignal V119 Wifi Box hardware includes an ESP8285 with 1MB Flash (as module 2AL3B ESP-M)
 * It specific hardware uses
 *  GPIO1: TX0,  ESP => Charge controller
 *  GPIO3: RXD0, ESP <= Charge controller
 *  GPIO5 red LED, active low, for e.g. LedLink_i
 *  free: GPIO4, GPIO12, GPIO13, GPIO14, GPIO15, GPIO16, GPIO17(ADC)
 *
 * Useful: command VoltRes 1 to select voltage resolution to 0.1 V (native resolution)
 *         command AmpRes 1 to select current resolution to 0.1 A (native resolution)
 *         SetOption72 to read total energy from the charge controller
 * 
 * Note: SetOption129 should be off, because only the solar energy is typical the main interest
 * 
 * This implementation is based on own logic analyzer recordings and information from
 * https://github.com/lasitha-sparrow/Makeskyblue_wifi_Controller
 * https://www.genetrysolar.com/community/forum-134/topic-1219/
 * 
\*********************************************************************************************/

#include <TasmotaSerial.h>

#define XNRG_25                     25

/* compile options */
#define MKSB_VARIANT_ENERGY_IMPORT    2 /* 0=never import, 1=import once, 2=import changed full kWh */
#define MKSB_WITH_CONFIG_REGISTER_SUPPORT
#define MKSB_WITH_ON_OFF_SUPPORT
#define MKSB_WITH_SERIAL_DEBUGGING
//#define MKSB_WITH_PHASE_NAME_SUPPORT /* requires new element Energy->phase_names at xdrv_03_energy and xdrv_03_esp32_energy */


#ifdef MKSB_WITH_SERIAL_DEBUGGING
  #define MKSB_COM_RX_IncrementCnt( cnt )  { mksb_cntRx[cnt]++; if ( cnt ) mksb_lastRxCntError = cnt; }
#else
  #define MKSB_COM_RX_IncrementCnt( cnt )  (void)0
#endif

#define MKSB_BAUDRATE               9600
#define MKSB_TIMEOUT                3     // seconds

// first byte of every valid frame
#define MKSB_START_FRAME            0xAA
// second byte: Request to the charge controller
#define MKSB_CMD_READ_STATUS        0x55
#define MKSB_CMD_READ_CONFIG        0xCB
#define MKSB_CMD_WRITE_CONFIG       0xCA
#define MKSB_CMD_POWER              0xCC
// second byte: Response from the charge controller
#define MKSB_RSP_READ_STATUS        0xBB
#define MKSB_RSP_RW_CONFIG          0xDA
#define MKSB_RSP_POWER              0xDD

#define MKSB_BUFFER_SIZE            24 // bytes

// config registers                   
#define MKSB_REG_SPECIAL            0   //     ?
#define MKSB_REG_VOLTAGE_BULK       1   // D02 MPPT Voltage limit BULK, >= stops charging [mV], e.g. 55000mV
#define MKSB_REG_VOLTAGE_FLOAT      2   // D01 MPPT voltage limit FLOAT, <= restarts charging [mV], SLA battery only
#define MKSB_REG_OUT_TIMER          3   // D00 Time duration the load gets connected [mh], default: 24h = 24000mh
#define MKSB_REG_CURRENT            4   //  -  MPPT current limit [mA] e.g. 1000...60000mA 
                                        //     CAUTION: do not set above hardware-limit 30000/40000/50000/60000
#define MKSB_REG_BATT_UVP_CUT       5   // D03 Battery UnderVoltageProtection, <=limit cuts load [mV], e.g. 42400mV
#define MKSB_REG_BATT_UVP_CONN      6   //  ~  Battery UnderVoltageProtection, >=limit reconnects load [mV], typical: D03 + 200mV, e.g. 44400mV
#define MKSB_REG_COMM_ADDR          7   //  -  Communication Address [mAddress]
#define MKSB_REG_BATT_TYPE          8   // D04 Battery Type: value 0=SLA, 1=LiPo (2=LiLo, 3=LiFE, 4=LiTo) [mSelect], e.g. 1000=LiPo
#define MKSB_REG_BATT_CELLS         9   //  -  Battery System: 1...4 * 12V (Read-Only, set at Batt.connection) [m12V], e.g. 4000m12V
#define MKSB_REG_TOTAL              10  // ^^^ related local parameter at HMI

// serial counters
#define MKSB_COM_RX_GOOD            0
#define MKSB_COM_ERR_RX_TIMEOUT     1
#define MKSB_COM_ERR_RX_OVERFLOW    2
#define MKSB_COM_ERR_RX_CRC         3
#define MKSB_COM_ERR_RX_CMD_LENGTH  4
#define MKSB_COM_RX_COUNTER         5

// module const
const char * mksb_register_names[] PROGMEM = {
  "0 unknown",                 //     dummy ? 
  "1 MPPT bulk [V]",           // D02 MPPT Voltage limit BULK, >= stops charging
  "2 MPPT float [V]",          // D01 MPPT voltage limit FLOAT, <= restarts charging, SLA battery only
  "3 Ontime load [h]",         // D00 Time duration the load gets connected, default: 24h 
  "4 MPPT current max.[A]",    //  -  MPPT current limit, default 30A, 40A, 50A or 60A
                               // CAUTION: do not set above the limit of your specific hardware
  "5 Batt.UVP cutload [V]",    // D03 Battery UnderVoltageProtection, <= cuts load
  "6 Batt.UVP connload [V]",   //  ~  Battery UnderVoltageProtection, >=limit reconnects load, typ D03 + 0.2V
  "7 Comm. address (?)",       //  -  Communication Address (ro)
  "8 Batt.Type [0=SLA, 1=Li]", // D04 Battery Type: value 0=SLA, 1=LiPo (ro)
  "9 Batt.System [n*12V]"      //  -  Battery System: 1...4 * 12V (Read-Only, set at Batt.connection)
};
const char MKSB_HTTP_SNS_str_m_int[] PROGMEM = "{s}%s " "{m}" "%d{e}";
#ifdef MKSB_WITH_PHASE_NAME_SUPPORT
const char * mksb_channels[] PROGMEM = {D_DEVICE_INPUT , D_BATTERY};
#endif
#ifdef MKSB_WITH_SERIAL_DEBUGGING
const char * mksb_rx_counter_names[] PROGMEM = {
  "Good",
  "Timeout",
  "Overflow",
  "CRC",
  "Cmd_Length",
  "Length"
};
#endif

// module data
static TasmotaSerial *mksb_Serial = nullptr;
static char *mksb_pRxBuffer = nullptr;
static uint32_t mksb_time_window;            // for Rx silence detection
static float mksb_temperature;                    // temperature of the charge controller electronics
#ifdef MKSB_WITH_SERIAL_DEBUGGING
static uint32_t mksb_cntTx;                  // requests transmitted to the charge controller
static uint32_t mksb_cntRxBytes;
static uint32_t mksb_cntRx[MKSB_COM_RX_COUNTER];
static uint16_t mksb_lastRxCntError;
#endif
#ifdef MKSB_WITH_CONFIG_REGISTER_SUPPORT
static uint16_t mksb_regs_to_read;                // bit_n = flags register n to read
static uint16_t mksb_regs_to_write;               // bit_n = flags register n to write
static uint16_t mksb_regs_to_report;              // bit_n = flags register n to report (after valid response received)
static uint16_t mksb_regs_value[MKSB_REG_TOTAL];  // temp value storage all known configuration registers
#endif
static uint16_t mksb_energy_for_import;           // totalizer at charge controller (non-volatile there)
//  uint8_t mode;                         // status: mode flags
//  uint8_t error;                        // status: error flags
static uint16_t mksb_status;
static uint8_t mksb_rxIdx;                        // bytecounter at the Rx data buffer
static uint8_t mksb_rxChecksum;
static uint8_t mksb_timeout;                      // active after a request, waiting for reponse
#ifdef MKSB_WITH_ON_OFF_SUPPORT
static bool mksb_actual_state;                    // true = active, false = stop
static bool mksb_target_state;                    // true = active, false = stop
#endif

/*
const char * mksb_status_bits[] PROGMEM = {
                                    // mode
  "",                               // b0 
  "",                               // b1
  D_CHARGE,                         // b2 MPPT Mode Active
  "",                               // b3

  "",                               // b4
  "",                               // b5
  "",                               // b6
  "",                               // b7

                                            // error
  D_BATTERY " " D_VOLTAGE " " D_STR_LOW,    // b0 Battery undervoltage (< register 5 / 6)
  D_BATTERY " " D_VOLTAGE " " D_STR_HIGH,   // b1 Battery overvoltage (> register 1)
  "",                                       // b2
  "",                                       // b3

  "",                                       // b4
  "",                                       // b5
  "",                                       // b6
  ""                                        // b7
};
*/

/********************************************************************************************/
// Extract an unsigned int with size of 1...4 bytes from data stream at offset_lsb 
uint32_t MkSkyBluExtractUint(const char *data, uint8_t offset_lsb, uint8_t offset_msb)
{
	uint32_t result = 0;

  for ( ; offset_msb >= offset_lsb; offset_msb-- ) {
    result = (result << 8) | (uint8_t)data[offset_msb];
  }
	return result;
}

/********************************************************************************************/
// Send serial data which consists of the variable data stream with its length
bool MkSkyBluSend(const uint8_t *data, uint8_t len)
{
  uint32_t i;
  uint8_t crc;

  // check for busy
  if (mksb_timeout) {
    return false; // transaction busy, a request is already waiting for a response
  }
  mksb_timeout = MKSB_TIMEOUT;

  // finish receiver
  if (MKSB_BUFFER_SIZE <= mksb_rxIdx) { // receive buffer was full
    MKSB_COM_RX_IncrementCnt(MKSB_COM_ERR_RX_OVERFLOW);
  }
  while ( mksb_Serial->available() ) { // clear receive buffer before each new request
    (void)mksb_Serial->read();
  }
  mksb_rxIdx = 0; // reset receiver state

  // build request frame
  mksb_Serial->write( MKSB_START_FRAME );
  crc = 0;
  for ( i = 0; i < len; i++ ) { // variable data
    mksb_Serial->write(data[i]);
    crc += data[i];
  }
  mksb_Serial->write(crc); // checksum

#ifdef MKSB_WITH_SERIAL_DEBUGGING
  { // serial debug at LOG_LEVEL_DEBUG_MORE: Request bytes transmitted
    char hex_char[(len * 3) + 2];
    mksb_cntTx++;
    AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("NRG: Serial Tx %02X %s %02X"), 
      MKSB_START_FRAME, 
      ToHex_P(data, len, hex_char, sizeof (hex_char), ' '),
      crc );
  }
#endif

  return true;
}

/********************************************************************************************/
/* COMMAND REQUESTS */

/* a) recording of original Wifi-Box firmware: Requested periodical every 1s, response within 30ms */
bool MkSkyBluRequestStatus(void)
{
  static const uint8_t data[] PROGMEM = {  MKSB_CMD_POWER, 0, 2, 0 };
  return MkSkyBluSend(data, sizeof (data));
}

/* b) recording of original Wifi-Box firmware: Requested periodical every 1s, 380ms after a), response within 30...370ms */
bool MkSkyBluRequestMeasurements(void)
{
  static const uint8_t data[] PROGMEM = {  MKSB_CMD_READ_STATUS, 0, 0, 0 };
  return MkSkyBluSend(data, sizeof (data));
}

/* c) recording of original Wifi-Box firmware: Requested on demand with an interval of 170ms, response within 30ms */
bool MkSkyBluRequestReadRegister(uint8_t reg)
{
  uint8_t data[] = {  MKSB_CMD_READ_CONFIG, 
                      reg, 
                      0, 0,  
                      0,0,0 };
  return MkSkyBluSend(data, sizeof (data));
}

bool MkSkyBluRequestWriteRegister(uint8_t reg, uint16_t value)
{
  uint8_t data[] = {  MKSB_CMD_WRITE_CONFIG, 
                      reg,
                      (uint8_t)(value & 0xFF), (uint8_t)(value >> 8),
                      0,0,0 };
  return MkSkyBluSend(data, sizeof (data));
}

bool MkSkyBluRequestPowerOn(void)
{
  static const uint8_t data[] PROGMEM = {  MKSB_CMD_POWER, 2, 0, 0 };
  return MkSkyBluSend(data, sizeof (data));
}

bool MkSkyBluRequestPowerOff(void)
{
  static const uint8_t data[] PROGMEM = {  MKSB_CMD_POWER, 1, 2, 0 };
  return MkSkyBluSend(data, sizeof (data));
}


/********************************************************************************************/
/* COMMAND RESPONSES */

void MkSkyBluParseStatusDataResponse(void)
{
  uint16_t u16 = 0;
  
  // Energy->power_on: not used, because ESP device acts as gateway to the charge controller
  
                                                                            // response data content:
  Energy->voltage[1] = (float)MkSkyBluExtractUint(mksb_pRxBuffer,  2, 3) / 10.0f; // battery voltage [0.1V]
  Energy->current[1] = (float)MkSkyBluExtractUint(mksb_pRxBuffer,  4, 5) / 10.0f; // battery current [0.1A]
  Energy->voltage[0] = (float)MkSkyBluExtractUint(mksb_pRxBuffer,  6, 7) / 10.0f; // solar voltage   [0.1V]
  Energy->active_power[0] = (float)MkSkyBluExtractUint(mksb_pRxBuffer,  8, 9);    // solar power     [1.0W]
  // calculate: battery power
  Energy->active_power[1] = Energy->voltage[1] * Energy->current[1]; 
  if ( Energy->voltage[0] >= 0.1f ) { // prevent division by 0
    // calculate: solar current
    Energy->current[0] = Energy->active_power[0] / Energy->voltage[0]; 
  } else {
    Energy->current[0] = 0.0f;
  }
  mksb_temperature = (float)MkSkyBluExtractUint(mksb_pRxBuffer, 10, 11) / 10.0f;    // temperature     [0.1degC]

#if MKSB_VARIANT_ENERGY_IMPORT == 1 /* import only once after restart: jitter only every restart */
  if ( !mksb_energy_for_import ) {
    mksb_energy_for_import++;
    u16 = MkSkyBluExtractUint(mksb_pRxBuffer, 12, 13);                             // solar energy    [1.0kWh]
    if ( u16 ) {
      mksb_energy_for_import = u16;
      Energy->import_active[0] = (float)u16;
      Energy->import_active[1] = 0.0f; // there is no energy generated from the battery
      EnergyUpdateTotal();
    }
  }
#elif MKSB_VARIANT_ENERGY_IMPORT == 2 /* import every full kWh integer resolution: jitter during runtime */
  u16 = MkSkyBluExtractUint(mksb_pRxBuffer, 12, 13);                              // solar energy    [1.0kWh]
  if ( mksb_energy_for_import != u16 ) {
    mksb_energy_for_import = u16;
    Energy->import_active[0] = (float)u16;
    Energy->import_active[1] = 0.0f; // there is no energy generated from the battery
    EnergyUpdateTotal();
  }
#endif
    
  // done
  Energy->data_valid[0] = 0;
  Energy->data_valid[1] = 0;

  mksb_status = MkSkyBluExtractUint(mksb_pRxBuffer, 16, 17);  // mode flags [bits]
                                                      //   bit1=solar current limited (register 3)
                                                      //   bit2=MPPT Mode Active
                                                      //   more ?
                                                      // error flags [bits]
                                                      //   bit0=Batt.undervoltage (< register 5 / 6)
                                                      //   bit1=Batt.overvoltage (> register 1)
                                                      //   ?more
  // unused response data:
  // MkSkyBluExtractUint(mksb_pRxBuffer, 14, 15); // ?dummy [14:15]
  // MkSkyBluExtractUint(mksb_pRxBuffer, 18, 18); // ?dummy [18]
}

#ifdef MKSB_WITH_CONFIG_REGISTER_SUPPORT
void MkSkyBluParseRegisterResponse(void)
{
  uint8_t reg;

  reg = mksb_pRxBuffer[2];
  if ( reg < MKSB_REG_TOTAL ) {
    mksb_regs_value[reg] = MkSkyBluExtractUint(mksb_pRxBuffer, 3, 4);
    mksb_regs_to_report |= 1 << reg;
  }
}
#endif

#ifdef MKSB_WITH_ON_OFF_SUPPORT
void MkSkyBluParsePowerResponse(void)
{
  if ( mksb_pRxBuffer[2] == 0 ) { /* ON */
    mksb_actual_state = true;
    AddLog(LOG_LEVEL_INFO, PSTR("NRG: Charging enabled"));
  } else 
  if ( mksb_pRxBuffer[2] == 1 ) {  /* OFF */
    mksb_actual_state = false;
    AddLog(LOG_LEVEL_INFO, PSTR("NRG: Charging disabled"));
  } else { /* unknown content */
  }
}
#endif

/********************************************************************************************/
/* RECEIVE SERIAL DATA */

void MkSkyBluSerialInput(void)
{
  while ( mksb_Serial->available() ) {
    yield();
    if (mksb_rxIdx < MKSB_BUFFER_SIZE) { // buffer not full
      mksb_pRxBuffer[mksb_rxIdx] = mksb_Serial->read();
      mksb_time_window = millis();
      if (MKSB_START_FRAME != mksb_pRxBuffer[0] ) { // no start of frame yet
        mksb_rxIdx = 0; // reset receiver
      } else {                // [0] start valid
        if (0 == mksb_rxIdx) {  // start of frame present
          mksb_rxChecksum = 0;  // reset checksum calc
          mksb_rxIdx++;         // more to receive
        } else {              // [1+] cmd or later
          if ((MKSB_RSP_READ_STATUS == mksb_pRxBuffer[1]) && (19 == mksb_rxIdx)) {
            if ( mksb_rxChecksum == mksb_pRxBuffer[mksb_rxIdx] ) {
              MkSkyBluParseStatusDataResponse();
              mksb_timeout = 0;
              MKSB_COM_RX_IncrementCnt(MKSB_COM_RX_GOOD);
            } else {
              MKSB_COM_RX_IncrementCnt(MKSB_COM_ERR_RX_CRC);
            }
          } else
          if ((MKSB_RSP_RW_CONFIG == mksb_pRxBuffer[1]) && (8 == mksb_rxIdx)) {
            if ( mksb_rxChecksum == mksb_pRxBuffer[mksb_rxIdx] ) {
              MkSkyBluParseRegisterResponse();
              mksb_timeout = 0;
              MKSB_COM_RX_IncrementCnt(MKSB_COM_RX_GOOD);
            } else {
              MKSB_COM_RX_IncrementCnt(MKSB_COM_ERR_RX_CRC);
            }
          } else
          if ((MKSB_RSP_POWER == mksb_pRxBuffer[1]) && (5 == mksb_rxIdx)) {
            if ( mksb_rxChecksum == mksb_pRxBuffer[mksb_rxIdx] ) {
              MkSkyBluParsePowerResponse();
              mksb_timeout = 0;
              MKSB_COM_RX_IncrementCnt(MKSB_COM_RX_GOOD);
            } else {
              MKSB_COM_RX_IncrementCnt(MKSB_COM_ERR_RX_CRC);
            }
          } else
          if (20 == mksb_rxIdx) { // invalid command and / or length
            MKSB_COM_RX_IncrementCnt(MKSB_COM_ERR_RX_CMD_LENGTH);
          } else {            // calc checksum
            mksb_rxChecksum += mksb_pRxBuffer[mksb_rxIdx]; 
            mksb_rxIdx++;       // more to receive     
          }
        } 
      }
    } else { // buffer full
      (void)mksb_Serial->read(); // drop received byte
    }
  }
}

/********************************************************************************************/

void MkSkyBluEverySecond(void)
{
  int i;

  if (Energy->data_valid[0] > ENERGY_WATCHDOG) {
    Energy->voltage[0] = Energy->current[0] = Energy->active_power[0] = 0.0f;
    Energy->voltage[1] = Energy->current[1] = Energy->active_power[1] = 0.0f;
  } else {
    Energy->kWhtoday_delta[0] += Energy->active_power[0] * 1000 / 36; // solar energy only
    EnergyUpdateToday();
  }

  if (mksb_timeout) { // busy, waiting for response
    mksb_timeout--;
    if ( mksb_timeout == 0 ) {
      MKSB_COM_RX_IncrementCnt(MKSB_COM_ERR_RX_TIMEOUT);
    }
  } else { // available
#ifdef MKSB_WITH_SERIAL_DEBUGGING
    if ( mksb_lastRxCntError ) { // new error
      if ( mksb_cntRx[mksb_lastRxCntError] ) { // error available
        AddLog(LOG_LEVEL_ERROR, PSTR("NRG: Serial Tx %u, Rx-good %u, Rx-error-%s %u"), 
          mksb_cntTx,
          mksb_cntRx[MKSB_COM_RX_GOOD], 
          mksb_rx_counter_names[mksb_lastRxCntError], mksb_cntRx[mksb_lastRxCntError] );
      }
      mksb_lastRxCntError = 0;
    } else
    if ( 0 == (mksb_cntTx % (10 * 60) ) ) { // every 10 minutes: Info
      AddLog(LOG_LEVEL_INFO, PSTR("NRG: Serial Tx %u, Rx-good %u"), 
        mksb_cntTx, 
        mksb_cntRx[MKSB_COM_RX_GOOD] );
      for ( i = 1; i < MKSB_COM_RX_COUNTER; i++ ) { // check all errors
        if ( mksb_cntRx[i] ) { // reports only if there are errors 
          AddLog(LOG_LEVEL_ERROR, PSTR("NRG: Serial Rx-error-%s %u"), 
            mksb_rx_counter_names[i], mksb_cntRx[i]);
        }
      }
    } else {}
#endif
    // request new measurements data
    MkSkyBluRequestMeasurements();
  }
}


void MkSkyBluEvery250ms(void)
{ 
  int i;
#ifdef MKSB_WITH_CONFIG_REGISTER_SUPPORT
  float fVal;

  if ( mksb_regs_to_read || mksb_regs_to_write ) {
    for ( i = 0; i < MKSB_REG_TOTAL; i++ ) {
      if ( mksb_regs_to_write & (1 << i) ) { // prio write
        if ( MkSkyBluRequestWriteRegister( i, mksb_regs_value[i] ) == true ) {
          mksb_regs_to_write &= ~(1 << i);
          break; // only one per call
        }
      } else 
      if ( mksb_regs_to_read & (1 << i) ) { 
        if ( MkSkyBluRequestReadRegister( i ) == true ) {
          mksb_regs_to_read &= ~(1 << i);
          break; // only one per call
        }
      } else {
      }
    }
  } else 
#endif
#ifdef MKSB_WITH_ON_OFF_SUPPORT
  if ( mksb_actual_state != mksb_target_state ) {
    if (mksb_timeout) { // waiting for answer  
      return;
    }
    if ( mksb_target_state == false ) { // standby
      MkSkyBluRequestPowerOff();
    } else { // avtivate
      MkSkyBluRequestPowerOn();
    }
  } else 
#endif
  {}
#ifdef MKSB_WITH_CONFIG_REGISTER_SUPPORT
  if ( mksb_regs_to_report ) {
    for ( i = 0; i < MKSB_REG_TOTAL; i++ ) {
      if ( mksb_regs_to_report & (1 << i) ) {
        fVal = ((float)(mksb_regs_value[i])) / 1000.0f;
        // answer requested manually by user
        AddLog( LOG_LEVEL_NONE, PSTR("NRG: Register%s = %3_f"), mksb_register_names[i], &fVal);
        mksb_regs_to_report &= ~(1 << i);
        break; // once per call
      }
    }
  }
#endif
}


bool MkSkyBluEnergyCommand(void)
{
  bool serviced = true;
  uint8_t reg;
  char *str;
  int32_t value;

  if ((CMND_POWERCAL == Energy->command_code) || (CMND_VOLTAGECAL == Energy->command_code) || (CMND_CURRENTCAL == Energy->command_code)) {
    // Service in xdrv_03_energy.ino
  } else 
  if (CMND_ENERGYCONFIG == Energy->command_code) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("NRG: Config index %d, payload %d, data '%s'"),
      XdrvMailbox.index, XdrvMailbox.payload, XdrvMailbox.data ? XdrvMailbox.data : "null" );

    if ( XdrvMailbox.data_len == 0 ) { // no data: Read all registers and report to log
#ifdef MKSB_WITH_CONFIG_REGISTER_SUPPORT
      mksb_regs_to_read = (1 << MKSB_REG_TOTAL) - 1; // flag all registers to be read
#endif
    } else {
      str = XdrvMailbox.data;
#ifdef MKSB_WITH_ON_OFF_SUPPORT
      if ('+' == str[0] ) { // + to set controller active
        mksb_target_state = true;
      } else
      if ('-' == str[0] ) { // - to set controller stop
        mksb_target_state = false;
      } else
#endif
#ifdef MKSB_WITH_CONFIG_REGISTER_SUPPORT
      {
        reg = (uint8_t)strtoul( str, &str, 10 );
        if ( MKSB_REG_TOTAL > reg ) {
          while ((*str != '\0') && isspace(*str)) { str++; }  // Trim spaces
          if ( *str ) {
            value = (int32_t)(CharToFloat(str) * 1000.0f);
            // write Register: no range check here, all on your own risk
            if ( MKSB_REG_VOLTAGE_BULK <= reg && MKSB_REG_BATT_UVP_CONN >= reg ) {
              mksb_regs_value[reg] = value;   // store to prepare write
              mksb_regs_to_write |= 1 << reg; // trigger write
            } // else read only
          } else { // read one register
              mksb_regs_to_read |= 1 << reg; 
          }
        } // invalid register
      }
#else
      {}
#endif
    }
  }
  else serviced = false;  // Unknown command

  return serviced;
}


void MkSkyBluShow(uint32_t function) {
  float fVal;

  if ( Settings->flag.temperature_conversion ) {
    fVal = ConvertTempToFahrenheit( mksb_temperature );
  } else {
    fVal = mksb_temperature; // celsius
  }
 
  if ( FUNC_JSON_APPEND == function ) {
    /* Temperature */
    ResponseAppend_P(JSON_SNS_F_TEMP, PSTR("MkSkyBlu"), Settings->flag2.temperature_resolution, &fVal);
    if (0 == TasmotaGlobal.tele_period) {
#ifdef USE_DOMOTICZ
      DomoticzFloatSensor(DZ_TEMP, fVal);
#endif  // USE_DOMOTICZ
#ifdef USE_KNX
      KnxSensor(KNX_TEMPERATURE, fVal);
#endif // USE_KNX
    }
  }
#ifdef USE_WEBSERVER
  else if ( FUNC_WEB_SENSOR == function ) {
    WSContentSend_Temp("", fVal);
    WSContentSend_P(MKSB_HTTP_SNS_str_m_int, D_INFO, mksb_status );
//    WSContentSend_P( MKSB_HTTP_SNS_sS_m_Se , D_POWERUSAGE ,mksb_actual_state == true ? D_ENABLED: D_DISABLED );
  } else if ( FUNC_WEB_COL_SENSOR == function ) {
    WSContentSend_P( PSTR("MakeSkyBlue") ); // headline before values
  } else {}
#endif  // USE_WEBSERVER
}


void MkSkyBluSnsInit(void)
{
  // Software serial init needs to be done here as earlier (serial) interrupts may lead to Exceptions
  mksb_Serial = new TasmotaSerial(Pin(GPIO_MKSKYBLU_RX), Pin(GPIO_MKSKYBLU_TX), 1);
  if (mksb_Serial->begin(MKSB_BAUDRATE)) {
    if (mksb_Serial->hardwareSerial()) {
      ClaimSerial();
      mksb_pRxBuffer = TasmotaGlobal.serial_in_buffer;  // Use idle serial buffer to save RAM
    } else {
      mksb_pRxBuffer = (char*)(malloc(MKSB_BUFFER_SIZE));
    }
#ifdef ESP32
    AddLog(LOG_LEVEL_DEBUG, PSTR("NRG: MkSkyBlu Serial UART%d"), mksb_Serial->getUart());
#endif
  } else {
    TasmotaGlobal.energy_driver = ENERGY_NONE;
  }
}


void MkSkyBluDrvInit(void)
{
  if (PinUsed(GPIO_MKSKYBLU_RX) && PinUsed(GPIO_MKSKYBLU_TX)) {
    Energy->phase_count = 2; // phases as channels: 0=solar input, 1=battery
#ifdef MKSB_WITH_PHASE_NAME_SUPPORT
    Energy->phase_names = mksb_channels;
#endif
    Energy->voltage_common = false;
    Energy->frequency_common = true;
    Energy->type_dc = true;
    Energy->use_overtemp = false;     // ESP device acts as separated gateway, charge controller has its own temperature management
    Energy->voltage_available = true; // both direct
    Energy->current_available = true; // solar indirect, battery direct 
    // Energy->local_energy_active_export = ?;

    mksb_timeout = 2;                   // Initial wait
#ifdef MKSB_WITH_SERIAL_DEBUGGING
    mksb_cntRx[MKSB_COM_ERR_RX_TIMEOUT] = 0xFFFFFFFF; // occurence at startup results to 0
#endif
    mksb_time_window = 0;
    mksb_temperature = NAN;
    mksb_energy_for_import = 0;
#ifdef MKSB_WITH_CONFIG_REGISTER_SUPPORT
    mksb_regs_to_read = 0;
    mksb_regs_to_write = 0;
    mksb_regs_to_report = 0;
#endif
#ifdef MKSB_WITH_ON_OFF_SUPPORT
    mksb_actual_state = true; // default
    mksb_target_state = true; // default
#endif
    TasmotaGlobal.energy_driver = XNRG_25;
  }
}


/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xnrg25(uint32_t function)
{
  bool result = false;

  switch (function) {
    case FUNC_LOOP:
      if (mksb_Serial) { MkSkyBluSerialInput(); }
      break;
    case FUNC_EVERY_250_MSECOND:
      if (mksb_Serial) { MkSkyBluEvery250ms(); }
      break;

    case FUNC_ENERGY_EVERY_SECOND:
      if (mksb_Serial) { MkSkyBluEverySecond(); }
      break;

    case FUNC_JSON_APPEND:
    case FUNC_WEB_SENSOR:
    case FUNC_WEB_COL_SENSOR:
      MkSkyBluShow(function);
      break;
    case FUNC_ENERGY_RESET:
      mksb_temperature = NAN;
      break;
    case FUNC_COMMAND:
      result = MkSkyBluEnergyCommand();
      break;
    case FUNC_INIT:
      MkSkyBluSnsInit();
      break;
    case FUNC_PRE_INIT:
      MkSkyBluDrvInit();
      break;
  }
  return result;
}

#endif  // USE_MAKE_SKY_BLUE
#endif  // USE_ENERGY_SENSOR
