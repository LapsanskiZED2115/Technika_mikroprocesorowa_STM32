#include <mpu6050.h>
#include <main.h>
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;

extern UART_HandleTypeDef huart2;

void mpu6050_init()
{
	HAL_StatusTypeDef  ret = HAL_I2C_IsDeviceReady(&hi2c1,(DEVICE_ADDRESS <<1) + 0, 1,100);
	  if (ret == HAL_OK)
	  {
		   const char message[] = "device by i2c working\r\n";
		   HAL_UART_Transmit(&huart2, message, strlen(message), HAL_MAX_DELAY);

		  	   	   uint8_t  temp_data = FS_GYRO_500;
		  	   	   HAL_StatusTypeDef ret =HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS <<1) + 0,  REG_CONFIG_GYRO , 1, &temp_data, 1, 100);
		  	   	   if (ret == HAL_OK)
					   {
						   const char message[] = "succesfully load mode scale to register 27 \r\n";
						   HAL_UART_Transmit(&huart2, message, strlen(message), HAL_MAX_DELAY);

					   }
				   else if (ret != HAL_OK)
				   	   {
						   const char message[] = "failed load to register 27!\r\n";
						   HAL_UART_Transmit(&huart2, message, strlen(message), HAL_MAX_DELAY);
					   }
		  	   	temp_data = FS_ACC_4G;
			    ret =HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS <<1) + 0,  REG_CONFIG_ACC , 1, &temp_data, 1, 100);
			    if (ret == HAL_OK)
				   {
					   const char message[] = "succesfully load G range to register 28 \r\n";
					   HAL_UART_Transmit(&huart2, message, strlen(message), HAL_MAX_DELAY);

				   }
			    else if(ret != HAL_OK)
				   {
					   const char message[] = "failed load G range  to register 28!\r\n";
					   HAL_UART_Transmit(&huart2, message, strlen(message), HAL_MAX_DELAY);
				   }


				temp_data = 0;
				ret =HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS <<1) + 0,  REG_USR_CTRL, 1, &temp_data, 1, 100);
				if (ret == HAL_OK)
				   {
					   const char message[] = "Exiting from sleep mode \r\n";
					   HAL_UART_Transmit(&huart2, message, strlen(message), HAL_MAX_DELAY);

				   }
				else if(ret != HAL_OK)
				   {
					   const char message[] = "failed exiting from sleep mode!\r\n";
					   HAL_UART_Transmit(&huart2, message, strlen(message), HAL_MAX_DELAY);
				   }

	  }
	  else
		  {
			   const char message[] = "device by i2c not working\r\n";
				HAL_UART_Transmit(&huart2, message, strlen(message), HAL_MAX_DELAY);
		  }
	  }

void mpu6050_read()
{
    uint8_t data[12];
    int16_t x_acc, y_acc, z_acc;
    int16_t x_gyro, y_gyro, z_gyro;

    // Odczyt 12 bajtów (akcelerometr + żyroskop)
    HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS << 1) + 1, 0x3B, 1, data, 12, 100);

    // Składanie wartości 16-bitowych
    x_acc  = (int16_t)((data[0] << 8)  | data[1]);
    y_acc  = (int16_t)((data[2] << 8)  | data[3]);
    z_acc  = (int16_t)((data[4] << 8)  | data[5]);
    x_gyro = (int16_t)((data[6] << 8)  | data[7]);
    y_gyro = (int16_t)((data[8] << 8)  | data[9]);
    z_gyro = (int16_t)((data[10] << 8) | data[11]);

    // Przeliczanie na jednostki fizyczne
    float x_acc_g = x_acc / 8192.0f;
    float y_acc_g = y_acc / 8192.0f;
    float z_acc_g = z_acc / 8192.0f;

    float x_gyro_dps = x_gyro / 65.5f;
    float y_gyro_dps = y_gyro / 65.5f;
    float z_gyro_dps = z_gyro / 65.5f;

    // Wysyłanie wyników przez UART
    char msg[150];
    snprintf(msg, sizeof(msg),"ACC X: %.2f g, Y: %.2f g, Z: %.2f g \r GYRO X: %.2f dps, Y: %.2f dps, Z: %.2f dps\n", x_acc_g, y_acc_g, z_acc_g, x_gyro_dps, y_gyro_dps, z_gyro_dps);

    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

