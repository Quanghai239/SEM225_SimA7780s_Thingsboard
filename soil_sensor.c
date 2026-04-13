#include "Soil_Sensor.h"

// Khởi tạo các biến toàn cục ở đây
float moisture, Temp, conductivity, ph, nitro, phospho, pota, salinity, TDS;

void UART_CONFIG(void){
    uart_config_t uart_cfg = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .source_clk = UART_SCLK_APB, // ĐÃ FIX LỖI CLOCK
    };

    uart_param_config(UART_PORT, &uart_cfg);
    uart_set_pin(UART_PORT, TXD_PIN, RXD_PIN, DE_PIN, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT, BUFF, 0, 0, NULL, 0);
    uart_set_mode(UART_PORT, UART_MODE_RS485_HALF_DUPLEX);
}

uint16_t CRC_Cal(const uint8_t *data, uint16_t len){
    uint16_t crc = 0xFFFF;
    for(uint16_t pos = 0; pos < len; pos++){
        crc ^= (uint16_t)data[pos];
        for(int i = 8; i != 0; i--){
            if((crc & 0x0001) != 0){
                crc >>= 1;
                crc ^= 0xA001;
            }
            else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void Find_Address_sensor(void) {
    printf("Scanning the ID \n");
    for (int test_id = 1; test_id <= 247; test_id++) {
        uint8_t frame[8];
        uint8_t response[10];

        frame[0] = test_id;
        frame[1] = 0x03;
        frame[2] = 0x00;
        frame[3] = 0x00;
        frame[4] = 0x00;
        frame[5] = 0x01;
        
        uint16_t crc = CRC_Cal(frame, 6);
        frame[6] = (uint8_t)(crc & 0xFF);
        frame[7] = (uint8_t)((crc >> 8) & 0xFF);
        
        uart_flush(UART_PORT);
        uart_write_bytes(UART_PORT, (const char*)frame, 8);
        
        int len = uart_read_bytes(UART_PORT, response, 7, pdMS_TO_TICKS(100));
        
        if (len > 0) {
            printf("\n FOUND ID: 0x%02X (%d)\n", test_id, test_id);
            return; 
        }
        if (test_id % 10 == 0) printf(".");
    }
    printf("\nNot found any ID devices\n");
}

void Find_Baudrate_sensor(uint8_t slave_id) {
    uint32_t baud;
    uart_get_baudrate(UART_PORT, &baud);
    
    printf("Checking sensor at current MCU Baudrate: %ld\n", baud);

    uint8_t frame[8];
    uint8_t response[10];

    frame[0] = slave_id;
    frame[1] = 0x03;
    frame[2] = 0x07;
    frame[3] = 0xD1;
    frame[4] = 0x00;
    frame[5] = 0x01;
    
    uint16_t crc = CRC_Cal(frame, 6);
    frame[6] = (uint8_t)(crc & 0xFF);
    frame[7] = (uint8_t)((crc >> 8) & 0xFF);
    
    uart_flush(UART_PORT);
    uart_write_bytes(UART_PORT, (const char*)frame, 8);
    
    int len = uart_read_bytes(UART_PORT, response, 10, pdMS_TO_TICKS(200));
    
    if (len > 0) {
        printf(" MATCH! Sensor is running at: %ld\n", baud);
    } else {
        printf("==> NO RESPONSE at %ld. Please change MCU Baudrate and try again.\n", baud);
    }
}

void Change_address(int nos, uint8_t current_id, uint8_t new_id){ 
    uint8_t frame[8];
    frame[0] = current_id;
    frame[1] = 0x06; 
    frame[2] = 0x07;
    frame[3] = 0xD0;
    frame[4] = 0x00;
    frame[5] = new_id;

    uint16_t crc = CRC_Cal(frame,6);
    frame[6] = (uint8_t)(crc & 0xFF); 
    frame[7] = (uint8_t)((crc >> 8) & 0xFF); 

    uart_flush(UART_PORT);

    int ret = uart_write_bytes(UART_PORT,(const char*)frame,sizeof(frame));
    if(ret <= 0){
        printf("Change ID failed \n");
    }
    printf("sending the new data to change address from current_id = 0x%02X to new_id = 0x%02X \n", current_id, new_id);
    uint8_t response[8]= {0};

    int len = uart_read_bytes(UART_PORT,response,sizeof(response),pdMS_TO_TICKS(1000));
    if (len >= 8 && response[5] == new_id){
        printf("Change ID successfully \n");
    }
    else {
        printf("Failed to change address \n");
    }
}

int Change_Baudrate(uint8_t slave_id, uint8_t baud_index) {
    uint8_t frame[8];
    frame[0] = slave_id;
    frame[1] = 0x06; 
    frame[2] = 0x07; 
    frame[3] = 0xD1; 
    frame[4] = 0x00; 
    frame[5] = baud_index; 

    uint16_t crc = CRC_Cal(frame, 6);
    frame[6] = (uint8_t)(crc & 0xFF);
    frame[7] = (uint8_t)((crc >> 8) & 0xFF);

    uart_flush(UART_PORT);
    int ret = uart_write_bytes(UART_PORT, (const char*)frame, 8);
    
    if(ret <= 0) return -1;

    printf("Changing Baudrate for ID 0x%02X sang Index %d...\n", slave_id, baud_index);

    uint8_t response[8] = {0};
    int len = uart_read_bytes(UART_PORT, response, 8, pdMS_TO_TICKS(1000));

    if (len >= 8 && response[5] == baud_index) {
        printf("changed Baud rate successfully \n");
        return 0;
    } else {
        printf("Error \n");
        return -2;
    }
}

void Function_write_coeff_calib(uint8_t slave_id, uint8_t coeffi_address, float value){ 
    int16_t value_int = (int16_t)(value * 100.0f);
    uint8_t frame[8];
    frame[0] = slave_id;
    frame[1] = 0x06; 
    frame[2] = 0x00;
    frame[3] = coeffi_address;
    frame[4] = (uint8_t)((value_int >> 8) & 0xFF); 
    frame[5] = (uint8_t)((value_int) & 0xFF);

    uint16_t crc = CRC_Cal(frame,6);
    frame[6] = (uint8_t)(crc & 0xFF); 
    frame[7] = (uint8_t)((crc >> 8) & 0xFF); 

    uart_flush(UART_PORT);
    uart_write_bytes(UART_PORT,(const char*)frame, sizeof(frame));

    printf("Setting value 0x%02X for coefficient address 0x%02X \n",value_int,coeffi_address);
}

void Function_read_value(uint8_t slave_id, uint16_t reg) {
    uint8_t frame[8], response[7];
    frame[0] = slave_id;
    frame[1] = 0x03;
    frame[2] = (uint8_t)((reg >> 8) & 0xFF); 
    frame[3] = (uint8_t)(reg & 0xFF);
    frame[4] = 0x00;
    frame[5] = 0x01; 

    uint16_t crc = CRC_Cal(frame, 6);
    frame[6] = (uint8_t)(crc & 0xFF);
    frame[7] = (uint8_t)((crc >> 8) & 0xFF);

    uart_flush(UART_PORT);
    uart_write_bytes(UART_PORT, (const char*)frame, sizeof(frame));

    int len = uart_read_bytes(UART_PORT, response, 7, pdMS_TO_TICKS(1000));

    if (len >= 5) { 
        if (response[0] == slave_id && response[1] == 0x03) {
            printf("Read Reg 0x%02X success: ", reg);
            for (int i = 0; i < len; i++) {
                printf("0x%02X ", response[i]);
            }
            if(reg > 0x21 && reg < 0x25){
                uint16_t value = (response[3] << 8) | response[4];
                printf("\n Value (Raw): %d, Value (Float): %.2f\n", value, value / 100.0f);
            }
            else {
                int16_t value = (response[3] << 8) | response[4];
                printf("\n Value (Raw): %d || Real value = %.1f \n", value, value/10.0f);
            }

            // ĐÃ FIX LỖI REDEFINITION VÀ LABEL BẰNG CÁCH BỌC NGOẶC NHỌN KHỐI LỆNH
            switch (reg)
            {
            case 0x22: {
                uint16_t value = (response[3] << 8) | response[4];
                printf("\n Conduc Temp coeffi (Raw): %d, Value (Float): %.2f\n", value, value / 100.0f);
                break;
            }
            case 0x23: {
                uint16_t value = (response[3] << 8) | response[4];
                printf("\n Salinity coeffi (Raw): %d, Value (Float): %.2f\n", value, value / 100.0f);
                break;
            }
            case 0x24: {
                uint16_t value = (response[3] << 8) | response[4];
                printf("\n TDS coeffi (Raw): %d, Value (Float): %.2f\n", value, value / 100.0f);
                break;
            }
            case 0x50: {
                int16_t value = (response[3] << 8) | response[4];
                printf("\n Value (Raw): %d || Real value = %.1f \n", value, value/10.0f);
                break;
            }
            case 0x4EA: {
                int16_t value = (response[3] << 8) | response[4];
                printf("\n Nitro calib value (Raw): %d\n", value);
                break;
            }
            case 0x4F4: {
                int16_t value = (response[3] << 8) | response[4];
                printf("\n Nitro calib value (Raw): %d\n", value);
                break;
            }
            case 0x4FE: {
                int16_t value = (response[3] << 8) | response[4];
                printf("\n Nitro calib value (Raw): %d\n", value);
                break;
            }
            default:
                break;
            }
            
        } else {
            printf("Error: Wrong Response (Code: 0x%02X)\n", response[1]);
        }
    } else {
        printf("Error: Timeout or No response from ID %d\n", slave_id);
    }
}

void Read_Sensor(uint8_t slave_id, uint8_t Things){ 
    uint8_t frame[8];
    uint8_t response[(Things + 1)*2 + 5];
    frame[0] = slave_id;
    frame[1] = 0x03; 
    frame[2] = 0x00; 
    frame[3] = 0x00; 
    frame[4] = 0x00; 
    frame[5] = Things + 1;

    uint16_t crc = CRC_Cal(frame,6);

    frame[6] = (uint8_t)(crc & 0xFF); 
    frame[7] = (uint8_t)((crc >> 8) & 0xFF); 

    uart_flush(UART_PORT);
    uart_write_bytes(UART_PORT,(const char*)frame,sizeof(frame));

    int len = uart_read_bytes(UART_PORT,response,sizeof(response),pdMS_TO_TICKS(1000));
    if (len > 0){
        printf("It has been send %d bytes \n", len);
        for(int i = 0; i < len;i++){
            printf("Data = 0x%02X ",response[i]);
        }
        printf (" \n");
    }

    switch (Things)
    {
    case 0:
        moisture = (float)((uint16_t)((response[3] << 8) | response[4])) / 10.0f;
        printf("The moisture = %.2f \n",moisture);
        break;
    case 1:
        moisture = (float)((uint16_t)((response[3] << 8) | response[4])) / 10.0f;
        Temp = (float)((int16_t)((response[5] << 8) | response[6])) / 10.0f;
        printf("The moisture = %.2f \n The temp = %.2f \n",moisture, Temp);
        break;
    case 2:
        moisture =        (float)((uint16_t)((response[3] << 8) | response[4])) / 10.0f;
        Temp =            (float)((int16_t)((response[5] << 8) | response[6])) / 10.0f;
        conductivity =    (float)((uint16_t)((response[7] << 8) | response[8]));
        printf("The moisture = %.2f \n The temp = %.2f \n The Conduc = %.2f \n",
        moisture, Temp,conductivity);
        break;
    case 3:
        moisture =        (float)((uint16_t)((response[3] << 8) | response[4])) / 10.0f;
        Temp =            (float)((int16_t)((response[5] << 8) | response[6])) / 10.0f;
        conductivity =    (float)((uint16_t)((response[7] << 8) | response[8]));
        ph =              (float)((uint16_t)((response[9] << 8) | response[10])) / 10.0f;
        printf("The moisture = %.2f \n The temp = %.2f \n The Conduc = %.2f \n PH = %.2f \n",
        moisture, Temp,conductivity,ph);
        break;
    case 4:
        moisture =        (float)((uint16_t)((response[3] << 8) | response[4])) / 10.0f;
        Temp =            (float)((int16_t)((response[5] << 8) | response[6])) / 10.0f;
        conductivity =    (float)((uint16_t)((response[7] << 8) | response[8]));
        ph =              (float)((uint16_t)((response[9] << 8) | response[10])) / 10.0f;
        nitro =           (float)((uint16_t)((response[11] << 8) | response[12]));
        printf("The moisture = %.2f \n The temp = %.2f \n The Conduc = %.2f \n PH = %.2f \n Nito = %.2f \n",
        moisture, Temp,conductivity,ph, nitro);
        break;
    case 5: 
        moisture =        (float)((uint16_t)((response[3] << 8) | response[4])) / 10.0f;
        Temp =            (float)((int16_t)((response[5] << 8) | response[6])) / 10.0f;
        conductivity =    (float)((uint16_t)((response[7] << 8) | response[8]));
        ph =              (float)((uint16_t)((response[9] << 8) | response[10])) / 10.0f;
        nitro =           (float)((uint16_t)((response[11] << 8) | response[12]));
        phospho =         (float)((uint16_t)((response[13] << 8) | response[14]));
        printf("The moisture = %.2f \n The temp = %.2f \n The Conduc = %.2f \n PH = %.2f \n Nito = %.2f \n PhosPho = %.2f \n",
        moisture, Temp,conductivity,ph, nitro ,phospho);
        break;
    case 6:
        moisture =        (float)((uint16_t)((response[3] << 8) | response[4])) / 10.0f;
        Temp =            (float)((int16_t)((response[5] << 8) | response[6])) / 10.0f;
        conductivity =    (float)((uint16_t)((response[7] << 8) | response[8]));
        ph =              (float)((uint16_t)((response[9] << 8) | response[10])) / 10.0f;
        nitro =           (float)((uint16_t)((response[11] << 8) | response[12]));
        phospho =         (float)((uint16_t)((response[13] << 8) | response[14]));
        pota =            (float)((uint16_t)((response[15] << 8) | response[16]));
        printf("The moisture = %.2f \n The temp = %.2f \n The Conduc = %.2f \n PH = %.2f \n Nito = %.2f \n PhosPho = %.2f \n Pota = %.2f \n",
        moisture, Temp,conductivity,ph, nitro,phospho, pota);
        break;
    case 7: 
        moisture =        (float)((uint16_t)((response[3] << 8) | response[4])) / 10.0f;
        Temp =            (float)((int16_t)((response[5] << 8) | response[6])) / 10.0f;
        conductivity =    (float)((uint16_t)((response[7] << 8) | response[8]));
        ph =              (float)((uint16_t)((response[9] << 8) | response[10])) / 10.0f;
        nitro =           (float)((uint16_t)((response[11] << 8) | response[12]));
        phospho =         (float)((uint16_t)((response[13] << 8) | response[14]));
        pota =            (float)((uint16_t)((response[15] << 8) | response[16]));
        salinity =        (float)((uint16_t)((response[17] << 8) | response[18]));
        printf("The moisture = %.2f \n The temp = %.2f \n The Conduc = %.2f \n PH = %.2f \n Nito = %.2f \n PhosPho = %.2f \n Pota = %.2f \n Sali = %.2f \n",
        moisture, Temp,conductivity,ph, nitro,phospho, pota, salinity);
        break;
    case 8:
        moisture =        (float)((uint16_t)((response[3] << 8) | response[4])) / 10.0f;
        Temp =            (float)((int16_t)((response[5] << 8) | response[6])) / 10.0f;
        conductivity =    (float)((uint16_t)((response[7] << 8) | response[8]));
        ph =              (float)((uint16_t)((response[9] << 8) | response[10])) / 10.0f;
        nitro =           (float)((uint16_t)((response[11] << 8) | response[12]));
        phospho =         (float)((uint16_t)((response[13] << 8) | response[14]));
        pota =            (float)((uint16_t)((response[15] << 8) | response[16]));
        salinity =        (float)((uint16_t)((response[17] << 8) | response[18]));
        TDS =             (float)((uint16_t)((response[19] << 8) | response[20]));
        printf("The moisture = %.2f \n The temp = %.2f \n The Conduc = %.2f \n PH = %.2f \n Nito = %.2f \n PhosPho = %.2f \n Pota = %.2f \n Sali = %.2f \n TDS = %.2f \n",
        moisture, Temp,conductivity,ph, nitro,phospho, pota, salinity, TDS);
    default:
        break;
    }
}