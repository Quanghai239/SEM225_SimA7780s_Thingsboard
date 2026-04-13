#ifndef SOIL_SENSOR_H
#define SOIL_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdio.h"
#include "stdint.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"

#define UART_PORT      UART_NUM_1 
#define TXD_PIN        13
#define RXD_PIN        14
#define DE_PIN         12    
#define BAUD_RATE      4800 
#define BUFF           1024

// Sử dụng extern để tránh lỗi Multiple Definition khi include vào nhiều file
extern float moisture, Temp, conductivity, ph, nitro, phospho, pota, salinity, TDS;

// Read Data
enum READ_REG {
    MOISTURE_CONTENT = 0x00,
    TEMPERATURE_VALUE,
    CONDUCTIVITY,
    PH_VALUE,
    NITROGEN_CONTENT,
    PHOSPHORUS_CONTENT,
    POTASSIUM_CONTENT,
    SALINITY,
    TOTAL_DISSOLVE_SOLIDS_TDS,
};

enum Coeffi {
    Conduc_Temp_Coeffi = 0x22, // Default value 0.0%
    Salinity_Coeffi,  // Default value 0.55
    TDS_Coeffi
};

enum calib { // default value = 0
    Temp_Calib = 0x50,
    Water_Calib,
    Conduc_Calib,
    PH_Calib,
};

void UART_CONFIG(void);
uint16_t CRC_Cal(const uint8_t *data, uint16_t len);
void Find_Address_sensor(void);
void Find_Baudrate_sensor(uint8_t slave_id);
void Change_address(int nos, uint8_t current_id, uint8_t new_id);
int Change_Baudrate(uint8_t slave_id, uint8_t baud_index);
void Function_write_coeff_calib(uint8_t slave_id, uint8_t coeffi_address, float value);
void Function_read_value(uint8_t slave_id, uint16_t reg);
void Read_Sensor(uint8_t slave_id, uint8_t Things);

#ifdef __cplusplus
}
#endif

#endif