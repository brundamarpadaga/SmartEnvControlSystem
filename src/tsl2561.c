#include "tsl2561.h"
#include "main.h"

extern int i2c_soft_reset(XIic* InstancePtr);

int tsl2561_init(XIic* i2c) {
    uint8_t send[2];
    int status;

    if (xSemaphoreTake(i2c_sem, pdMS_TO_TICKS(100)) == pdTRUE) {
        i2c_soft_reset(i2c);
        usleep(20000);

        int timeout = TIMEOUT_COUNTER;
        while (XIic_IsIicBusy(i2c) && --timeout > 0);
        if (timeout <= 0) {
            xSemaphoreGive(i2c_sem);
            return XST_FAILURE;
        }

        if (XIic_SetAddress(i2c, XII_ADDR_TO_SEND_TYPE, TSL2561_ADDR) != XST_SUCCESS) {
            xSemaphoreGive(i2c_sem);
            return XST_FAILURE;
        }

        send[0] = TSL2561_CMD_CONTROL;
        send[1] = TSL2561_POWER_ON;
        status = XIic_Send(i2c->BaseAddress, TSL2561_ADDR, send, 2, XIIC_STOP);
        if (status != 2) {
            xSemaphoreGive(i2c_sem);
            return XST_FAILURE;
        }

        send[0] = TSL2561_CMD_TIMING;
        send[1] = TSL2561_TIMING_402MS_16X;
        status = XIic_Send(i2c->BaseAddress, TSL2561_ADDR, send, 2, XIIC_STOP);
        if (status != 2) {
            xSemaphoreGive(i2c_sem);
            return XST_FAILURE;
        }

        xSemaphoreGive(i2c_sem);
    } else {
        xil_printf("[ERROR] I2C semaphore timeout in tsl2561_init\r\n");
        return XST_FAILURE;
    }

    usleep(410000);
    return XST_SUCCESS;
}

float tsl2561_readChannel(XIic* i2c, tsl2561_channel_t channel) {
    uint8_t buf[2];
    int status;

    if (xSemaphoreTake(i2c_sem, pdMS_TO_TICKS(500)) == pdTRUE) {  // Increased from 100ms to 500ms
        buf[0] = (channel == TSL2561_CHANNEL_0 ? DATA0LOW_REG : DATA1LOW_REG) | 0x80;

        XIic_Stop(i2c);
        usleep(5000);
        status = XIic_Start(i2c);
        if (status != XST_SUCCESS) {
            xil_printf("[ERROR] I2C restart failed before read (status: %d)\r\n", status);
            xSemaphoreGive(i2c_sem);
            return 0;
        }

        status = XIic_Send(i2c->BaseAddress, TSL2561_ADDR, buf, 1, XIIC_REPEATED_START);
        if (status != 1) {
            xil_printf("[ERROR] Send address 0x%02X failed (bytes sent: %d)\r\n", buf[0], status);
            xSemaphoreGive(i2c_sem);
            return 0;
        }

        status = XIic_Recv(i2c->BaseAddress, TSL2561_ADDR, buf, 2, XIIC_STOP);
        if (status != 2) {
            xil_printf("[ERROR] Receive failed for channel %d (bytes read: %d)\r\n", channel, status);
            xSemaphoreGive(i2c_sem);
            return 0;
        }

        xSemaphoreGive(i2c_sem);
        uint16_t value = (buf[1] << 8) | buf[0];
        return (float)value;
    } else {
        xil_printf("[ERROR] I2C semaphore timeout in tsl2561_readChannel\r\n");
        return 0;
    }
}
