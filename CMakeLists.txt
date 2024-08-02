CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_PROCESSOR cortex-m4)
#SET(CMAKE_VERBOSE_MAKEFILE ON)

SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)

SET(CMAKE_C_COMPILER "D:/env-windows/tools/bin/../../tools/gnu_gcc/arm_gcc/mingw/bin/arm-none-eabi-gcc.exe")
SET(CMAKE_ASM_COMPILER "D:/env-windows/tools/bin/../../tools/gnu_gcc/arm_gcc/mingw/bin/arm-none-eabi-gcc.exe")
SET(CMAKE_C_FLAGS " -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -Dgcc -O0 -gdwarf-2 -g")
SET(CMAKE_ASM_FLAGS " -c -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -x assembler-with-cpp -Wa,-mimplicit-it=thumb  -gdwarf-2")
SET(CMAKE_C_COMPILER_WORKS TRUE)

SET(CMAKE_CXX_COMPILER "D:/env-windows/tools/bin/../../tools/gnu_gcc/arm_gcc/mingw/bin/arm-none-eabi-g++.exe")
SET(CMAKE_CXX_FLAGS " -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -Dgcc -O0 -gdwarf-2 -g")
SET(CMAKE_CXX_COMPILER_WORKS TRUE)

SET(CMAKE_OBJCOPY "D:/env-windows/tools/bin/../../tools/gnu_gcc/arm_gcc/mingw/bin/arm-none-eabi-objcopy.exe")
SET(CMAKE_SIZE "D:/env-windows/tools/bin/../../tools/gnu_gcc/arm_gcc/mingw/bin/arm-none-eabi-size.exe")

SET(CMAKE_EXE_LINKER_FLAGS " -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -Wl,--gc-sections,-Map=rt-thread.map,-cref,-u,Reset_Handler -T ${CMAKE_SOURCE_DIR}/board/linker_scripts/link.lds -Wl,--gc-sections,--print-memory-usage")

SET(CMAKE_C_STANDARD 11)
SET(CMAKE_CXX_STANDARD 17)

PROJECT(rtthread C CXX ASM)

INCLUDE_DIRECTORIES(
	packages/aht10-latest/inc
	packages/ap3216c-latest
	applications
	rt-thread/components/libc/compilers/common/include
	rt-thread/components/libc/compilers/newlib
	rt-thread/components/drivers/include
	rt-thread/components/drivers/spi
	rt-thread/components/drivers/wlan
	board
	board/CubeMX_Config/Inc
	board/ports
	libraries/HAL_Drivers/drivers
	libraries/HAL_Drivers/drivers/config
	libraries/HAL_Drivers
	libraries/HAL_Drivers/CMSIS/Include
	rt-thread/components/finsh
	packages/kawaii-mqtt-latest/common
	packages/kawaii-mqtt-latest/mqtt
	packages/kawaii-mqtt-latest/mqttclient
	packages/kawaii-mqtt-latest/network
	packages/kawaii-mqtt-latest/platform/RT-Thread
	packages/kawaii-mqtt-latest/common/log
	.
	rt-thread/include
	rt-thread/libcpu/arm/common
	rt-thread/libcpu/arm/cortex-m4
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Inc
	libraries/STM32F4xx_HAL/CMSIS/Device/ST/STM32F4xx/Include
	rt-thread/components/net/lwip/lwip-2.0.3/src/include
	rt-thread/components/net/lwip/lwip-2.0.3/src/include/ipv4
	rt-thread/components/net/lwip/lwip-2.0.3/src/include/netif
	rt-thread/components/net/lwip/port
	rt-thread/components/libc/posix/io/epoll
	rt-thread/components/libc/posix/io/eventfd
	rt-thread/components/libc/posix/io/poll
	rt-thread/components/libc/posix/ipc
	packages/rw007-v2.1.0
	packages/rw007-v2.1.0/inc
	rt-thread/components/net/netdev/include
	rt-thread/components/net/sal/include
	rt-thread/components/net/sal/include/socket
	rt-thread/components/net/sal/impl
	rt-thread/components/net/sal/include/socket/sys_socket
)

ADD_DEFINITIONS(
	-DRT_USING_LIBC
	-DRT_USING_NEWLIBC
	-DSTM32F407xx
	-DUSE_HAL_DRIVER
	-D_POSIX_C_SOURCE=1
	-D__RTTHREAD__
)

SET(PROJECT_SOURCES
	packages/aht10-latest/src/aht10.c
	packages/ap3216c-latest/ap3216c.c
	applications/main.c
	applications/mqtt_client.c
	applications/aht_10.c
	applications/usart3.c
	applications/wifi_connection.c
	rt-thread/components/libc/compilers/common/cctype.c
	rt-thread/components/libc/compilers/common/cstdlib.c
	rt-thread/components/libc/compilers/common/cstring.c
	rt-thread/components/libc/compilers/common/ctime.c
	rt-thread/components/libc/compilers/common/cunistd.c
	rt-thread/components/libc/compilers/common/cwchar.c
	rt-thread/components/libc/compilers/newlib/syscalls.c
	rt-thread/components/drivers/core/device.c
	rt-thread/components/drivers/i2c/i2c-bit-ops.c
	rt-thread/components/drivers/i2c/i2c_core.c
	rt-thread/components/drivers/i2c/i2c_dev.c
	rt-thread/components/drivers/ipc/completion_comm.c
	rt-thread/components/drivers/ipc/completion_up.c
	rt-thread/components/drivers/ipc/condvar.c
	rt-thread/components/drivers/ipc/dataqueue.c
	rt-thread/components/drivers/ipc/pipe.c
	rt-thread/components/drivers/ipc/ringblk_buf.c
	rt-thread/components/drivers/ipc/ringbuffer.c
	rt-thread/components/drivers/ipc/waitqueue.c
	rt-thread/components/drivers/ipc/workqueue.c
	rt-thread/components/drivers/misc/rt_drv_pwm.c
	rt-thread/components/drivers/pin/pin.c
	rt-thread/components/drivers/serial/serial.c
	rt-thread/components/drivers/spi/spi_core.c
	rt-thread/components/drivers/spi/spi_dev.c
	rt-thread/components/drivers/wlan/wlan_cfg.c
	rt-thread/components/drivers/wlan/wlan_cmd.c
	rt-thread/components/drivers/wlan/wlan_dev.c
	rt-thread/components/drivers/wlan/wlan_lwip.c
	rt-thread/components/drivers/wlan/wlan_mgnt.c
	rt-thread/components/drivers/wlan/wlan_prot.c
	rt-thread/components/drivers/wlan/wlan_workqueue.c
	board/CubeMX_Config/Src/stm32f4xx_hal_msp.c
	board/board.c
	libraries/HAL_Drivers/drivers/drv_gpio.c
	libraries/HAL_Drivers/drivers/drv_soft_i2c.c
	libraries/HAL_Drivers/drivers/drv_spi.c
	libraries/HAL_Drivers/drivers/drv_usart.c
	libraries/HAL_Drivers/drv_common.c
	libraries/STM32F4xx_HAL/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f407xx.s
	rt-thread/components/finsh/msh.c
	rt-thread/components/finsh/msh_parse.c
	rt-thread/components/finsh/shell.c
	rt-thread/components/finsh/cmd.c
	packages/kawaii-mqtt-latest/network/nettype_tcp.c
	packages/kawaii-mqtt-latest/mqtt/MQTTFormat.c
	packages/kawaii-mqtt-latest/platform/RT-Thread/platform_timer.c
	packages/kawaii-mqtt-latest/mqtt/MQTTPacket.c
	packages/kawaii-mqtt-latest/platform/RT-Thread/platform_thread.c
	packages/kawaii-mqtt-latest/common/log/salof.c
	packages/kawaii-mqtt-latest/mqttclient/mqttclient.c
	packages/kawaii-mqtt-latest/mqtt/MQTTConnectServer.c
	packages/kawaii-mqtt-latest/mqtt/MQTTSubscribeClient.c
	packages/kawaii-mqtt-latest/common/log/arch/rt-thread/arch.c
	packages/kawaii-mqtt-latest/mqtt/MQTTUnsubscribeServer.c
	packages/kawaii-mqtt-latest/common/mqtt_list.c
	packages/kawaii-mqtt-latest/network/network.c
	packages/kawaii-mqtt-latest/mqtt/MQTTSerializePublish.c
	packages/kawaii-mqtt-latest/network/nettype_tls.c
	packages/kawaii-mqtt-latest/common/log/fifo.c
	packages/kawaii-mqtt-latest/platform/RT-Thread/platform_mutex.c
	packages/kawaii-mqtt-latest/mqtt/MQTTUnsubscribeClient.c
	packages/kawaii-mqtt-latest/mqtt/MQTTConnectClient.c
	packages/kawaii-mqtt-latest/common/random.c
	packages/kawaii-mqtt-latest/common/log/format.c
	packages/kawaii-mqtt-latest/platform/RT-Thread/platform_memory.c
	packages/kawaii-mqtt-latest/mqtt/MQTTDeserializePublish.c
	packages/kawaii-mqtt-latest/platform/RT-Thread/platform_net_socket.c
	packages/kawaii-mqtt-latest/mqtt/MQTTSubscribeServer.c
	rt-thread/src/clock.c
	rt-thread/src/components.c
	rt-thread/src/cpu_up.c
	rt-thread/src/idle.c
	rt-thread/src/ipc.c
	rt-thread/src/irq.c
	rt-thread/src/klibc/kstdio.c
	rt-thread/src/klibc/kstring.c
	rt-thread/src/kservice.c
	rt-thread/src/mem.c
	rt-thread/src/mempool.c
	rt-thread/src/object.c
	rt-thread/src/scheduler_comm.c
	rt-thread/src/scheduler_up.c
	rt-thread/src/thread.c
	rt-thread/src/timer.c
	rt-thread/libcpu/arm/common/atomic_arm.c
	rt-thread/libcpu/arm/common/div0.c
	rt-thread/libcpu/arm/common/showmem.c
	rt-thread/libcpu/arm/cortex-m4/context_gcc.S
	rt-thread/libcpu/arm/cortex-m4/cpuport.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cryp.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cryp_ex.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cec.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
	libraries/STM32F4xx_HAL/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_qspi.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rng.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_usart.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_lptim.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_crc.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/api/api_lib.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/api/api_msg.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/api/err.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/api/netbuf.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/api/netdb.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/api/netifapi.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/api/sockets.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/api/tcpip.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/apps/ping/ping.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/core/def.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/core/dns.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/core/inet_chksum.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/core/init.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/core/ip.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/core/ipv4/autoip.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/core/ipv4/dhcp.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/core/ipv4/etharp.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/core/ipv4/icmp.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/core/ipv4/igmp.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/core/ipv4/ip4.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/core/ipv4/ip4_addr.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/core/ipv4/ip4_frag.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/core/memp.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/core/netif.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/core/pbuf.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/core/raw.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/core/stats.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/core/sys.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/core/tcp.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/core/tcp_in.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/core/tcp_out.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/core/timeouts.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/core/udp.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/netif/ethernet.c
	rt-thread/components/net/lwip/lwip-2.0.3/src/netif/lowpan6.c
	rt-thread/components/net/lwip/port/ethernetif.c
	rt-thread/components/net/lwip/port/sys_arch.c
	packages/rw007-v2.1.0/src/spi_wifi_rw007.c
	packages/rw007-v2.1.0/example/rw007_stm32_port.c
	rt-thread/components/net/netdev/src/netdev.c
	rt-thread/components/net/netdev/src/netdev_ipaddr.c
	rt-thread/components/net/sal/impl/af_inet_lwip.c
	rt-thread/components/net/sal/socket/net_netdb.c
	rt-thread/components/net/sal/src/sal_socket.c
)

LINK_DIRECTORIES(
)

LINK_LIBRARIES(
	c
	m
)

ADD_EXECUTABLE(${CMAKE_PROJECT_NAME}.elf ${PROJECT_SOURCES})
ADD_CUSTOM_COMMAND(TARGET ${CMAKE_PROJECT_NAME}.elf POST_BUILD 
	COMMAND ${CMAKE_OBJCOPY} -O ihex ${CMAKE_PROJECT_NAME}.elf rtthread.hex
	COMMAND ${CMAKE_SIZE} ${CMAKE_PROJECT_NAME}.elf
	COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_PROJECT_NAME}.elf rtthread.bin
	COMMAND ${CMAKE_SIZE} ${CMAKE_PROJECT_NAME}.elf
)

# if custom.cmake is exist, add it
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/custom.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/custom.cmake)
endif()
