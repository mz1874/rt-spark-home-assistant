//��ʹ��������������
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "mqttclient.h"

#define DBG_TAG "main"
#define DBG_LVL         DBG_LOG
#include <rtdbg.h>

#define AHT_I2C_BUS_NAME    "i2c3"  //AHT20 ���ص�I2C����
#define AHT_ADDR            0x38    //AHT20 I2C��ַ
#define AHT_CALIBRATION_CMD 0xBE    //AHT20 ��ʼ������
#define AHT_NORMAL_CMD      0xA8    //AHT20 ��������ģʽ����
#define AHT_GET_DATA_CMD    0xAC    //AHT20 ��ȡ�������




//д���������ӻ��������ݣ�
rt_err_t write_reg(struct rt_i2c_bus_device *Device, rt_uint8_t reg, rt_uint8_t* data);

//�����ݣ��ӻ��������������ݣ�
rt_err_t read_reg(struct rt_i2c_bus_device *Device, rt_uint8_t len, rt_uint8_t* buf);
//��ȡAHT����ʪ������
void read_temp_humi(float* Temp_Data, float* Humi_Data);

//AHT���г�ʼ��
void AHT_Init(const char* name);

//AHT�豸�����߳�
void AHT_test_entry(void * args);

