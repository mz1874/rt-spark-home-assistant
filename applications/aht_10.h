//不使用软件包的情况下
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "mqttclient.h"

#define DBG_TAG "main"
#define DBG_LVL         DBG_LOG
#include <rtdbg.h>

#define AHT_I2C_BUS_NAME    "i2c3"  //AHT20 挂载的I2C总线
#define AHT_ADDR            0x38    //AHT20 I2C地址
#define AHT_CALIBRATION_CMD 0xBE    //AHT20 初始化命令
#define AHT_NORMAL_CMD      0xA8    //AHT20 正常工作模式命令
#define AHT_GET_DATA_CMD    0xAC    //AHT20 获取结果命令




//写命令（主机向从机传输数据）
rt_err_t write_reg(struct rt_i2c_bus_device *Device, rt_uint8_t reg, rt_uint8_t* data);

//读数据（从机向主机返回数据）
rt_err_t read_reg(struct rt_i2c_bus_device *Device, rt_uint8_t len, rt_uint8_t* buf);
//读取AHT的温湿度数据
void read_temp_humi(float* Temp_Data, float* Humi_Data);

//AHT进行初始化
void AHT_Init(const char* name);

//AHT设备测试线程
void AHT_test_entry(void * args);

