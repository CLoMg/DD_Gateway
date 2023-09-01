/// ------------------------------------------------------------------------------------------------------------------------------------
///
/// MIT License
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction, including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
/// copies of the Software, and to permit persons to whom the Software is
/// furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all
/// copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
/// SOFTWARE.
///
/// Copyright (c) 2022 ycz. All rights reserved.
///
/// Created by ycz on 2022/9/12.
///
/// @brief
///      y_lora 是 sx127x lora 芯片系列的一种驱动程序。
///
/// ------------------------------------------------------------------------------------------------------------------------------------



/// ------------------------------------------------------------------------------------------------------------------------------------
/// 防止当前头文件被重复引用
/// ------------------------------------------------------------------------------------------------------------------------------------

#ifndef _Y_LORA_H
#define _Y_LORA_H



/// ------------------------------------------------------------------------------------------------------------------------------------
/// 头文件
/// ------------------------------------------------------------------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
//#include "cmsis_os.h"



/// ------------------------------------------------------------------------------------------------------------------------------------
/// 宏定义
/// ------------------------------------------------------------------------------------------------------------------------------------

#define Y_LORA_MAJOR              0  ///< 主版本     ( 主架构变化 )
#define Y_LORA_MINOR              1  ///< 次版本     ( 单个功能增加或修改 )
#define Y_LORA_PATCH              0  ///< 补丁版本    ( bug修复 )

#define LORA_WAKEUP_TIME          1            ///< 无线电从睡眠中唤醒的时间 [ms]
#define LORA_TCXO_WAKEUP_TIME     0            ///< TCXO 唤醒所需的时间 [ms]
#define LORA_MAC_PRIVATE_SYNCWORD 0x12         ///< 私有 LoRa 网络的同步字
#define LORA_MAC_PUBLIC_SYNCWORD  0x34         ///< 公共 LoRa 网络的同步字
#define LORA_RF_MID_THRESH        525000000    ///< 射频中频带阈值
#define LORA_XTAL_FREQ            32000000     ///< 基准频率
#define LORA_FREQ_STEP            61.03515625  ///< 频率步长
#define LORA_RX_BUF_SIZE          256          ///< 接受数据 buf 长度
#define LORA_BANDWIDTH_AFC        0            ///< AFC 带宽(仅限 FSK)
#define LORA_PREAMBLE_LEN         6            ///< 前导码长度
#define LORA_SYMB_TIMEOUT         5            ///< 前导码长度
#define LORA_FIXLEN               false        ///< 固定长度数据包
#define LORA_PAYLOAD_LEN          0            ///< 固定长度时设置有效负载长度
#define LORA_CRC_ON               true         ///< 启用/禁用 CRC
#define LORA_FREQ_HOP_ON          false        ///< 启用/禁用数据包内跳频
#define LORA_HOP_PERIOD           0            ///< 每跳之间的符号数
#define LORA_IQ_INVERTED          false        ///< 反转 IQ 信号（仅限 LoRa）
#define LORA_RX_CONTINUOUS        true         ///< 将接收设置为连续模式
#define LORA_FDEV                 0            ///< 设置频率偏差（仅限 FSK）
#define LORA_TX_TIMEOUT           5000         ///< 传输超时 [ms]



/// ------------------------------------------------------------------------------------------------------------------------------------
/// 枚举
/// ------------------------------------------------------------------------------------------------------------------------------------

/// 无线电驱动程序支持的调制解调器
typedef enum {
    MODEM_FSK = 0,
    MODEM_LORA,
} RadioModem_e;

/// 无线电驱动程序内部状态机状态定义
typedef enum {
    RF_IDLE = 0,    ///< 空闲
    RF_RX_RUNNING,  ///< 接收状态
    RF_TX_RUNNING,  ///< 传输状态
    RF_CAD,         ///< 无线电正在进行信道活动检测
} RadioState_e;



/// ------------------------------------------------------------------------------------------------------------------------------------
/// 结构体
/// ------------------------------------------------------------------------------------------------------------------------------------

/// 无线电 FSK 调制解调器参数
typedef struct {
    int8_t   Power;
    uint32_t Fdev;
    uint32_t Bandwidth;
    uint32_t BandwidthAfc;
    uint32_t Datarate;
    uint16_t PreambleLen;
    bool     FixLen;
    uint8_t  PayloadLen;
    bool     CrcOn;
    bool     IqInverted;
    bool     RxContinuous;
    uint32_t TxTimeout;
    uint32_t RxSingleTimeout;
} RadioFskSettings_t;

/// 无线电 FSK 数据包句柄
typedef struct {
    uint8_t  PreambleDetected;
    uint8_t  SyncWordDetected;
    int8_t   RssiValue;
    int32_t  AfcValue;
    uint8_t  RxGain;
    uint16_t Size;
    uint16_t NbBytes;
    uint8_t  FifoThresh;
    uint8_t  ChunkSize;
} RadioFskPacketHandler_t;

/// 无线电 LoRa 调制解调器参数
typedef struct {
    int8_t   Power;
    uint32_t Bandwidth;
    uint32_t Datarate;
    bool     LowDatarateOptimize;
    uint8_t  Coderate;
    uint16_t PreambleLen;
    bool     FixLen;
    uint8_t  PayloadLen;
    bool     CrcOn;
    bool     FreqHopOn;
    uint8_t  HopPeriod;
    bool     IqInverted;
    bool     RxContinuous;
    uint32_t TxTimeout;
    bool     PublicNetwork;
} RadioLoRaSettings_t;

/// 无线电 LoRa 数据包句柄
typedef struct {
    int8_t  SnrValue;
    int16_t RssiValue;
    uint8_t Size;
} RadioLoRaPacketHandler_t;

/// 无线电设置
typedef struct {
    RadioModem_e             Modem;
    RadioState_e             State;
    uint32_t                 Channel;
    RadioFskSettings_t       Fsk;
    RadioFskPacketHandler_t  FskPacketHandler;
    RadioLoRaSettings_t      LoRa;
    RadioLoRaPacketHandler_t LoRaPacketHandler;
} RadioSettings_t;

typedef struct lora LORA_st;

extern LORA_st *lora_dev;

/// LORA 事件回调函数
typedef struct {
    void (*TxDone)(LORA_st *lora);                                                             ///< Tx Done 回调原型。
    void (*TxTimeout)(LORA_st *lora);                                                          ///< Tx 超时回调原型。
    void (*RxDone)(LORA_st *lora, uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);  ///< Rx Done 回调原型。
    void (*RxTimeout)(LORA_st *lora);                                                          ///< Rx 超时回调原型。
    void (*RxError)(LORA_st *lora);                                                            ///< Rx 错误回调原型。
    void (*FhssChangeChannel)(LORA_st *lora, uint8_t currentChannel);                          ///< FHSS 更改通道回调原型。
    void (*CadDone)(LORA_st *lora, bool channelActivityDetected);                              ///< CAD 完成回调原型。
} RadioEventCb_t;

/// LORA 底层端口回调函数
typedef struct {
    uint8_t (*spi_transmit)(uint8_t outData);  ///< lora spi 发送/接收数据
    void (*spi_nss)(bool on_off);              ///< lora spi 开关
    void (*reset)();                           ///< lora 重启
} RadioPortCb_t;

/// lora 句柄
struct lora {
    RadioEventCb_t  event;                  ///< LORA 事件回调组
    RadioPortCb_t   port;                   ///< LORA 底层端口回调组
    // TimerHandle_t   tx_timer;               ///< 发送超时定时器
    // TimerHandle_t   rx_timer;               ///< 接收超时定时器
    // TimerHandle_t   rx_sync_timer;          ///< 接收同步字超时定时器
    RadioSettings_t setting;                ///< 设置参数
    uint8_t         buf[LORA_RX_BUF_SIZE];  ///< 接收发送缓冲区
    void           *lora_wan;               ///< 保留值 用于确定父对象
};



/// ------------------------------------------------------------------------------------------------------------------------------------
/// 函数 API
/// ------------------------------------------------------------------------------------------------------------------------------------
int          Lora_Init(void);
void         y_lora_print_version();                                                                                                          ///< 打印版本信息
LORA_st     *y_lora_create(RadioEventCb_t *event, RadioPortCb_t *port);                                                                       ///< 创建 lora 实例
void         y_lora_destroy(LORA_st *lora);                                                                                                   ///< 销毁 lora 实例
void         y_lora_write_buf(LORA_st *lora, uint16_t addr, uint8_t *buffer, uint8_t size);                                                   ///< 写入多个数据到寄存器
void         y_lora_read_buf(LORA_st *lora, uint16_t addr, uint8_t *buffer, uint8_t size);                                                    ///< 读取寄存器的值到 buf 中
void         y_lora_write(LORA_st *lora, uint16_t addr, uint8_t data);                                                                        ///< 写入单个数据到寄存器
uint8_t      y_lora_read(LORA_st *lora, uint16_t addr);                                                                                       ///< 读取指定寄存器的值
void         y_lora_send(LORA_st *lora, uint8_t *buffer, uint8_t size);                                                                       ///< 发送指定大小的数据
void         y_lora_start_cad(LORA_st *lora);                                                                                                 ///< 启动通道活动检测
bool         y_lora_is_channel_free(LORA_st *lora, RadioModem_e modem, uint32_t freq, int16_t rssiThresh, uint32_t Timeout);                  ///< 检查频道是否空闲
RadioState_e y_lora_get_status(LORA_st *lora);                                                                                                ///< 获取 lora 运行状态
uint32_t     y_lora_get_random(LORA_st *lora);                                                                                                ///< 根据 RSSI 生成随机值
int16_t      y_lora_get_rssi(LORA_st *lora, RadioModem_e modem);                                                                              ///< 读取当前 RSSI 值
uint32_t     y_lora_get_time_on_air(LORA_st *lora, RadioModem_e modem, uint8_t pktLen);                                                       ///< 计算空中数据包时间
uint32_t     y_lora_get_wakeup_time();                                                                                                        ///< 获取睡眠中醒来的时间
void         y_lora_set_modem(LORA_st *lora, RadioModem_e modem);                                                                             ///< 设置模式
void         y_lora_set_channel(LORA_st *lora, uint32_t freq);                                                                                ///< 设置通道频率
void         y_lora_set_tx_config(LORA_st *lora, RadioModem_e modem, int8_t power, uint32_t bandwidth, uint32_t datarate, uint8_t coderate);  ///< 设置发送参数
void         y_lora_set_tx_continuous_wave(LORA_st *lora, uint32_t freq, int8_t power, uint16_t time);                                        ///< 设置为连续波传输模式
void         y_lora_set_rx_config(LORA_st *lora, RadioModem_e modem, uint32_t bandwidth, uint32_t datarate, uint8_t coderate);                ///< 设置接收参数
void         y_lora_set_rx_time(LORA_st *lora, uint32_t timeout);                                                                             ///< 设置为接收超时时间
void         y_lora_set_max_payload_length(LORA_st *lora, RadioModem_e modem, uint8_t max);                                                   ///< 设置最大有效载荷长度
void         y_lora_set_public_network(LORA_st *lora, bool enable);                                                                           ///< 设置为公共或私有
void         y_lora_set_sleep(LORA_st *lora);                                                                                                 ///< 设置为睡眠模式
void         y_lora_set_stby(LORA_st *lora);                                                                                                  ///< 设置为待机模式
void         y_lora_io0_irq(LORA_st *lora);                                                                                                   ///< DIO0中断回调
void         y_lora_io1_irq(LORA_st *lora);                                                                                                   ///< DIO1中断回调
void         y_lora_io2_irq(LORA_st *lora);                                                                                                   ///< DIO2中断回调
void         y_lora_io3_irq(LORA_st *lora);                                                                                                   ///< DIO3中断回调
void         y_lora_io4_irq(LORA_st *lora);                                                                                                   ///< DIO4中断回调
void         y_lora_io5_irq(LORA_st *lora);                                                                                                   ///< DIO5中断回调



/// ------------------------------------------------------------------------------------------------------------------------------------
/// 底层接口 API         note: 需根据实际修改( 若有多个 lora ,可按以下函数类型增加 )
/// ------------------------------------------------------------------------------------------------------------------------------------

uint8_t y_lora_1_spi_transmit(uint8_t outData);  ///< spi 1 发送/接收数据
void    y_lora_1_spi_nss(bool on_off);           ///< spi 1 开关
void    y_lora_1_reset();                        ///< spi 1 重启

uint8_t y_lora_2_spi_transmit(uint8_t outData);  ///< spi 2 发送/接收数据
void    y_lora_2_spi_nss(bool on_off);           ///< spi 2 开关
void    y_lora_2_reset();                        ///< spi 2 重启



/// ------------------------------------------------------------------------------------------------------------------------------------
/// 条件编译结尾
/// ------------------------------------------------------------------------------------------------------------------------------------

#endif  // _Y_LORA_H
