#include "ICM42688P.h"
#include "spi.h"

#define ICM42688_CS_PIN GPIO_PIN_4
#define ICM42688_CS_PORT GPIOA

// ICM-42688-P寄存器定义
#define WHO_AM_I_REG 0x75
#define PWR_MGMT0_REG 0x4E
#define GYRO_CONFIG0_REG 0x4F
#define ACCEL_CONFIG0_REG 0x50
#define FIFO_CONFIG_REG 0x16
#define FIFO_DATA_REG 0x34


// 初始化函数
void ICM42688_Init(void) {
    uint8_t whoami;
    uint8_t tx_data;

    // 复位设备
    ICM42688_WriteRegister(PWR_MGMT0_REG, 0x00);
    HAL_Delay(10);

    // 检查WHO_AM_I
    whoami = ICM42688_ReadRegister(WHO_AM_I_REG);
    if(whoami != 0x47) { // ICM-42688-P的WHO_AM_I值应为0x42
        while(1);
    }

    // 配置电源管理
    tx_data = 0x0F; // 启用加速度计和陀螺仪
    ICM42688_WriteRegister(PWR_MGMT0_REG, tx_data);

    // 配置陀螺仪
    tx_data = (0x03 << 5) | 0x06; // ±500dps, 1kHz ODR
    ICM42688_WriteRegister(GYRO_CONFIG0_REG, tx_data);

    // 配置加速度计
    tx_data = (0x01 << 5) | 0x06; // ±8g, 1kHz ODR
    ICM42688_WriteRegister(ACCEL_CONFIG0_REG, tx_data);

    // 配置FIFO
    tx_data = 0x03; // 启用加速度计和陀螺仪数据到FIFO
    ICM42688_WriteRegister(FIFO_CONFIG_REG, tx_data);

    HAL_Delay(100); // 等待传感器稳定
}

// 写寄存器函数
void ICM42688_WriteRegister(uint8_t reg, uint8_t data) {
    uint8_t tx_buffer[2] = {reg & 0x7F, data}; // 清除最高位(写操作)

    HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, tx_buffer, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_SET);
}

// 读寄存器函数
uint8_t ICM42688_ReadRegister(uint8_t reg) {
    uint8_t tx_buffer[2] = {reg | 0x80, 0x00}; // 设置最高位(读操作)
    uint8_t rx_buffer[2] = {0};

    HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_SET);

    return rx_buffer[1];
}

// 读取传感器数据
void ReadSensorData(int16_t* accel, int16_t* gyro) {
    // 读取加速度计数据
    accel[0] = (int16_t)((ICM42688_ReadRegister(0x1F) << 8) | ICM42688_ReadRegister(0x20));
    accel[1] = (int16_t)((ICM42688_ReadRegister(0x21) << 8) | ICM42688_ReadRegister(0x22));
    accel[2] = (int16_t)((ICM42688_ReadRegister(0x23) << 8) | ICM42688_ReadRegister(0x24));

    // 读取陀螺仪数据
    gyro[0] = (int16_t)((ICM42688_ReadRegister(0x25) << 8) | ICM42688_ReadRegister(0x26));
    gyro[1] = (int16_t)((ICM42688_ReadRegister(0x27) << 8) | ICM42688_ReadRegister(0x28));
    gyro[2] = (int16_t)((ICM42688_ReadRegister(0x29) << 8) | ICM42688_ReadRegister(0x2A));

}

// 将原始数据转换为实际值
void ConvertRawData(int16_t raw_accel[3], int16_t raw_gyro[3], float* accel_g, float* gyro_dps) {
    // 加速度计转换 (±8g范围)
    for(int i = 0; i < 3; i++) {
        accel_g[i] = (float)raw_accel[i] / 4096.0f; // 4096 LSB/g (对于±8g范围)
    }

    // 陀螺仪转换 (±500dps范围)
    for(int i = 0; i < 3; i++) {
        gyro_dps[i] = (float)raw_gyro[i] / 65.5f; // 65.5 LSB/dps (对于±500dps范围)
    }
}

#define TEMP_SENSITIVITY 132.48f  // LSB/℃
#define TEMP_OFFSET      25.0f    // 25℃时输出0

float ICM42688_ReadTemperature(void) {
    // 读取两个8位寄存器并组合成16位数据
    int16_t temp_raw = (int16_t)((ICM42688_ReadRegister(0x1D) << 8) | ICM42688_ReadRegister(0x1E));

    // 转换为实际温度值
    return ((float)temp_raw / TEMP_SENSITIVITY) + TEMP_OFFSET;
}