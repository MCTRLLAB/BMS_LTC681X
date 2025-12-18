#include <Arduino.h>
#include <SPI.h>
#include "driver/twai.h"
#include "LTC6811.h"
#include "LTC681x.h"

// =========================================================================
// PIN CONFIGURATION
// =========================================================================
#define CS_PIN      7
#define CAN_TX_PIN  20
#define CAN_RX_PIN  21
#define TEMP_PIN    0
#define AMSOK_PIN   3

// =========================================================================
// BMS CONFIGURATION
// =========================================================================
#define TOTAL_IC    1
#define NUM_CELLS   12

#ifndef MD_7KHZ_3KHZ
  #define MD_7KHZ_3KHZ 2
#endif
#ifndef DCP_ENABLED
  #define DCP_ENABLED 1
#endif
#ifndef CELL_CH_ALL
  #define CELL_CH_ALL 0
#endif

// =========================================================================
// GLOBAL VARIABLES
// =========================================================================
cell_asic bms_ic[TOTAL_IC];
float cellVoltages[NUM_CELLS];  // Easy access to cell voltages in Volts


unsigned long current = 0;
unsigned long interval = 1000; // Interval between balance checks (ms)

// =========================================================================
// HARDWARE GLUE CODE (DON'T MODIFY)
// =========================================================================
void cs_low(uint8_t pin) { digitalWrite(CS_PIN, LOW); }
void cs_high(uint8_t pin) { digitalWrite(CS_PIN, HIGH); }
void delay_u(uint16_t micro) { delayMicroseconds(micro); }
void spi_write_array(uint8_t len, uint8_t data[]) {
  for (int i = 0; i < len; i++) SPI.transfer(data[i]);
}
void spi_write_read(uint8_t tx_Data[], uint8_t tx_len, uint8_t *rx_data, uint8_t rx_len) {
  for (int i = 0; i < tx_len; i++) SPI.transfer(tx_Data[i]);
  for (int i = 0; i < rx_len; i++) rx_data[i] = SPI.transfer(0xFF);
}

// =========================================================================
// USER FUNCTIONS - USE THESE IN YOUR BALANCING ALGORITHM
// =========================================================================

/**
 * Read all cell voltages from the BMS
 * Results are stored in global array: cellVoltages[0-11]
 * Returns: true if successful, false if error
 */
bool readAllCells() {
  wakeup_sleep(TOTAL_IC);
  LTC6811_adcv(MD_7KHZ_3KHZ, DCP_ENABLED, CELL_CH_ALL);
  delay(5);
  wakeup_sleep(TOTAL_IC);
  int8_t error = LTC6811_rdcv(0, TOTAL_IC, bms_ic);
  
  // Convert to voltage array for easy access
  for (int i = 0; i < NUM_CELLS; i++) {
    cellVoltages[i] = bms_ic[0].cells.c_codes[i] * 0.0001;
  }
  
  return (error == 0);
}

/**
 * Enable discharge on a single cell
 * @param cellNumber: cell to discharge (1-12)
 * @param durationMinutes: how long to discharge (0.5 to 7.5 minutes)
 *                         Default = 2.0 minutes if not specified
 * 
 * Duration guide:
 *   0.5 = 30 seconds
 *   1.0 = 1 minute
 *   2.0 = 2 minutes (default)
 *   4.0 = 4 minutes
 *   7.5 = 7.5 minutes (maximum)
 * 
 * Example: 
 *   enableDischarge(5, 4.0);  // Discharge cell 5 for 4 minutes
 */
void enableDischarge(uint8_t cellNumber) {
  if (cellNumber < 1 || cellNumber > NUM_CELLS) {
    Serial.print("ERROR: Cell number must be 1-");
    Serial.println(NUM_CELLS);
    return;
  }
  wakeup_sleep(TOTAL_IC);
  LTC6811_rdcfg(TOTAL_IC, bms_ic);
  // Clear discharge registers first
  for (int ic = 0; ic < TOTAL_IC; ic++) {
    bms_ic[0].config.tx_data[4] = 0x00; 
    bms_ic[0].config.tx_data[5] &= 0x0F;  // Keep lower 4 bits!
    bms_ic[0].config.tx_data[5] |= (4 << 4);
  }
  
  // Set timeout (0-15, where each step = 0.5 minutes)
  // timeoutValue: 1=0.5min, 2=1min, 4=2min, 8=4min, 15=7.5min
  // uint8_t timeoutValue = constrain((uint8_t)(durationMinutes * 2), 1, 15);
  // bms_ic[0].config.tx_data[5] = (timeoutValue << 4);
  
  // Enable discharge for this cell
  if (cellNumber <= 8) {
    bms_ic[0].config.tx_data[4] = (1 << (cellNumber - 1));
  } else {
    bms_ic[0].config.tx_data[5] |= (1 << (cellNumber - 9));
  }
  
  wakeup_sleep(TOTAL_IC);
  LTC6811_wrcfg(TOTAL_IC, bms_ic);
  
  Serial.print("Discharging Cell ");
  Serial.print(cellNumber);
  Serial.print(" for ");
  // Serial.print(durationMinutes);
  // Serial.println(" minutes");
}

/**
 * Disable discharge on all cells
 */
void clearDischarge() {
  for (int ic = 0; ic < TOTAL_IC; ic++) {
    bms_ic[ic].config.tx_data[4] = 0x00; 
    bms_ic[ic].config.tx_data[5] &= 0xF0;  // Keep timeout, clear discharge bits
  }
  wakeup_sleep(TOTAL_IC);
  LTC6811_wrcfg(TOTAL_IC, bms_ic);
  Serial.println("All discharge cleared");
}

/**
 * Send data over CAN bus
 * @param id: CAN message ID (0x000 to 0x7FF)
 * @param data: pointer to data array
 * @param len: data length (0-8 bytes)
 */
void sendCAN(uint32_t id, uint8_t* data, uint8_t len) {
  if (len > 8) len = 8;
  
  twai_message_t message;
  message.identifier = id;
  message.extd = 0;
  message.rtr = 0;
  message.data_length_code = len;
  
  for (int i = 0; i < len; i++) {
    message.data[i] = data[i];
  }

  esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(100));
  
  if (result == ESP_OK) {
    Serial.print("CAN TX: 0x");
    Serial.print(id, HEX);
    Serial.print(" [");
    for (int i = 0; i < len; i++) {
      if (data[i] < 0x10) Serial.print("0");
      Serial.print(data[i], HEX);
      if (i < len - 1) Serial.print(" ");
    }
    Serial.println("]");
  } else {
    Serial.println("CAN TX FAILED");
  }
}

// =========================================================================
// HELPER FUNCTIONS
// =========================================================================

/**
 * Print all cell voltages to Serial
 */
void printCellVoltages() {
  Serial.println("\n--- Cell Voltages ---");
  for (int i = 0; i < NUM_CELLS; i++) {
    Serial.print("C");
    if (i < 9) Serial.print("0");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(cellVoltages[i], 4);
    Serial.println(" V");
  }
  Serial.println();
}

/**
 * Get pack statistics
 */
void getPackStats(float* minV, float* maxV, float* avgV, float* totalV) {
  *minV = 10.0;
  *maxV = 0.0;
  *totalV = 0.0;
  int validCells = 0;
  
  for (int i = 0; i < NUM_CELLS; i++) {
    if (cellVoltages[i] > 0.5) {  // Valid cell
      if (cellVoltages[i] < *minV) *minV = cellVoltages[i];
      if (cellVoltages[i] > *maxV) *maxV = cellVoltages[i];
      *totalV += cellVoltages[i];
      validCells++;
    }
  }
  
  *avgV = (validCells > 0) ? (*totalV / validCells) : 0.0;
}

/**
 * Print pack statistics
 */
void printPackStats() {
  float minV, maxV, avgV, totalV;
  getPackStats(&minV, &maxV, &avgV, &totalV);
  
  Serial.println("--- Pack Statistics ---");
  Serial.print("Total:   "); Serial.print(totalV, 3); Serial.println(" V");
  Serial.print("Average: "); Serial.print(avgV, 4); Serial.println(" V");
  Serial.print("Min:     "); Serial.print(minV, 4); Serial.println(" V");
  Serial.print("Max:     "); Serial.print(maxV, 4); Serial.println(" V");
  Serial.print("Delta:   "); Serial.print((maxV - minV) * 1000, 1); Serial.println(" mV");
  Serial.println();
}

/**
 * Send pack data over CAN (pre-defined format)
 */
void sendPackDataCAN() {
  float minV, maxV, avgV, totalV;
  getPackStats(&minV, &maxV, &avgV, &totalV);
  
  uint8_t data[8];
  
  // Message 1: Pack summary (ID 0x200)
  uint16_t total_mV = (uint16_t)(totalV * 1000);
  uint16_t min_mV = (uint16_t)(minV * 1000);
  uint16_t max_mV = (uint16_t)(maxV * 1000);
  
  data[0] = (total_mV >> 8) & 0xFF;
  data[1] = total_mV & 0xFF;
  data[2] = (min_mV >> 8) & 0xFF;
  data[3] = min_mV & 0xFF;
  data[4] = (max_mV >> 8) & 0xFF;
  data[5] = max_mV & 0xFF;
  data[6] = NUM_CELLS;
  data[7] = 0;
  sendCAN(0x200, data, 8);
  
  // Messages 2-4: Individual cell voltages (ID 0x300-0x302)
  for (int j = 0; j < 3; j++) {
    int start = j * 4;
    for (int k = 0; k < 4; k++) {
      uint16_t cell_mV = (uint16_t)(cellVoltages[start + k] * 1000);
      data[k * 2] = (cell_mV >> 8) & 0xFF;
      data[k * 2 + 1] = cell_mV & 0xFF;
    }
    sendCAN(0x300 + j, data, 8);
  }
}

/**
 * Read temperature sensor
 */
float readTemperature() {
  int raw = analogRead(TEMP_PIN);
  float voltage = (raw / 4095.0) * 3.3;
  float R_pullup = 10000.0;
  float R_thermistor = R_pullup * (voltage / (3.3 - voltage));
  float temperatureC = 25.0 - ((R_thermistor - 10000.0) / 400.0);
  return temperatureC;
}

// =========================================================================
// EXAMPLE USAGE
// =========================================================================

/**
 * EXAMPLE: Simple balancing test
 * Read cells, discharge highest cell for 1 minute
 */
void exampleSimpleBalance() {
  Serial.println("\n=== EXAMPLE: Simple Balance ===");
  
  readAllCells();
  printCellVoltages();
  
  // Find highest cell
  float maxV = 0;
  uint8_t maxCell = 0;
  
  for (int i = 0; i < NUM_CELLS; i++) {
    if (cellVoltages[i] > maxV && cellVoltages[i] > 0.5) {
      maxV = cellVoltages[i];
      maxCell = i + 1;  // Cell numbers are 1-indexed
    }
  }
  
  if (maxCell > 0) {
    Serial.print("Highest cell: ");
    Serial.print(maxCell);
    Serial.print(" at ");
    Serial.print(maxV, 4);
    Serial.println(" V");
    
    enableDischarge(maxCell);  // Discharge for 1 minute
  }
  
  Serial.println("================================\n");
}

// =========================================================================
// INITIALIZATION
// =========================================================================

void initCAN() {
  Serial.println("Initializing CAN Bus...");
  
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)CAN_TX_PIN, 
    (gpio_num_t)CAN_RX_PIN, 
    TWAI_MODE_NO_ACK
  );
  
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); 
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("✓ CAN Driver Installed");
  } else {
    Serial.println("✗ CAN Driver Install Failed");
  }
  
  if (twai_start() == ESP_OK) {
    Serial.println("✓ CAN Driver Started");
  } else {
    Serial.println("✗ CAN Driver Start Failed");
  }
  
  delay(100);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n╔════════════════════════════════════════╗");
  Serial.println("║   ESP32-C3 BMS - Simplified Interface ║");
  Serial.println("╚════════════════════════════════════════╝\n");
  
  // Initialize pins
  pinMode(AMSOK_PIN, OUTPUT);
  pinMode(TEMP_PIN, INPUT);
  analogSetPinAttenuation(TEMP_PIN, ADC_11db);
  
  // Initialize SPI
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  SPI.begin(4, 5, 6, 7);
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  Serial.println("✓ SPI Initialized");

  // Initialize LTC6811
  LTC6811_init_cfg(TOTAL_IC, bms_ic);
  LTC6811_reset_crc_count(TOTAL_IC, bms_ic);
  LTC6811_init_reg_limits(TOTAL_IC, bms_ic);
  Serial.println("✓ LTC6811 Initialized");
  
  // Initialize CAN
  initCAN();
  
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║            COMMAND MENU                ║");
  Serial.println("╠════════════════════════════════════════╣");
  Serial.println("║  r = Read all cells                    ║");
  Serial.println("║  p = Print voltages + stats            ║");
  Serial.println("║  c = Send CAN data                     ║");
  Serial.println("║  x = Clear all discharge               ║");
  Serial.println("║  t = Read temperature                  ║");
  Serial.println("║                                        ║");
  Serial.println("║  a = Example: Discharge highest cell  ║");
  Serial.println("║                                        ║");
  Serial.println("║  Manual discharge (1-9, q, w, e):     ║");
  Serial.println("║    1-9 = Discharge cells 1-9          ║");
  Serial.println("║    q = Discharge cell 10               ║");
  Serial.println("║    w = Discharge cell 11               ║");
  Serial.println("║    e = Discharge cell 12               ║");
  Serial.println("╚════════════════════════════════════════╝\n");
  
  Serial.println("Ready! Write your balancing algorithm below.\n");
}

// =========================================================================
// MAIN LOOP - ADD YOUR CODE HERE
// =========================================================================

void loop() {
  
  int cell[10] = {1,3,5} ; 
  if(Serial.available()){
    char cmd = Serial.read();
    while (Serial.available())
    Serial.read(); // clear input buffer
    switch(cmd){
      case 'c':
        clearDischarge();
        break;
      case 'd':
        if (millis() - current >= interval) {
          current = millis();
          for (int i = 0; i < 3; i++) {
            enableDischarge(cell[i]); 
            delay(2000);
          }
        } 
        break;
      case 'p':
        readAllCells();
        printCellVoltages();
        break;
      case 's':
        sendPackDataCAN();
        sendCAN(0x123, (uint8_t *)"Hello", 5);
        break;
    }

  }

  

  // =====================================================================
  // YOUR BALANCING ALGORITHM HERE
  // =====================================================================
  // Example:
  // 
  // static unsigned long lastBalance = 0;
  // if (millis() - lastBalance > 60000) {  // Every 60 seconds
  //   readAllCells();
  //   
  //   // Find max voltage
  //   float maxV = 0;
  //   uint8_t maxCell = 0;
  //   for (int i = 0; i < NUM_CELLS; i++) {
  //     if (cellVoltages[i] > maxV) {
  //       maxV = cellVoltages[i];
  //       maxCell = i + 1;
  //     }
  //   }
  //   
  //   // Discharge highest cell for 4 minutes
  //   enableDischarge(maxCell, 4.0);
  //   
  //   lastBalance = millis();
  // }
  // =====================================================================
}
