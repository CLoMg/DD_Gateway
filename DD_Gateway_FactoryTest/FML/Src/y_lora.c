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
///     y_lora 是 sx127x lora 芯片系列的一种驱动程序。
///
/// ------------------------------------------------------------------------------------------------------------------------------------



/// ------------------------------------------------------------------------------------------------------------------------------------
/// 头文件
/// ------------------------------------------------------------------------------------------------------------------------------------

#include "y_lora.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "y_lora_Regs_Fsk.h"
#include "y_lora_Regs_LoRa.h"
//#include "y_log.h"
#include "spi.h"
#include "shell_port.h"



/// ------------------------------------------------------------------------------------------------------------------------------------
/// 宏定义
/// ------------------------------------------------------------------------------------------------------------------------------------

#define RSSI_OFFSET_LF (-164)  ///< 常量值需要计算RSSI值
#define RSSI_OFFSET_HF (-157)  ///< 常量值需要计算RSSI值



/// ------------------------------------------------------------------------------------------------------------------------------------
/// 结构体
/// ------------------------------------------------------------------------------------------------------------------------------------

/// 无线电寄存器定义
typedef struct {
    RadioModem_e Modem;
    uint8_t      Addr;
    uint8_t      Value;
} RadioRegisters_t;

/// FSK 带宽定义
typedef struct {
    uint32_t bandwidth;
    uint8_t  RegValue;
} FskBandwidth_t;



/// ------------------------------------------------------------------------------------------------------------------------------------
/// 私有全局常量
/// ------------------------------------------------------------------------------------------------------------------------------------

LORA_st *lora_dev = NULL;

typedef struct jt_lora_Seting_params{
    uint32_t Freq ;
    uint8_t Power;
    uint8_t Bandwidth;
    uint8_t Datarate;
    uint8_t Coderate;
};
struct jt_lora_Seting_params jt_lora_params = {
        .Freq = 437000000,
        .Power = 20,
        .Bandwidth = 2,
        .Datarate = 8,
        .Coderate = 1,
};
static const FskBandwidth_t FskBandwidths[] = {
        {2600, 0x17},    ///< 占位
        {3100, 0x0F},    ///< 占位
        {3900, 0x07},    ///< 占位
        {5200, 0x16},    ///< 占位
        {6300, 0x0E},    ///< 占位
        {7800, 0x06},    ///< 占位
        {10400, 0x15},   ///< 占位
        {12500, 0x0D},   ///< 占位
        {15600, 0x05},   ///< 占位
        {20800, 0x14},   ///< 占位
        {25000, 0x0C},   ///< 占位
        {31300, 0x04},   ///< 占位
        {41700, 0x13},   ///< 占位
        {50000, 0x0B},   ///< 占位
        {62500, 0x03},   ///< 占位
        {83333, 0x12},   ///< 占位
        {100000, 0x0A},  ///< 占位
        {125000, 0x02},  ///< 占位
        {166700, 0x11},  ///< 占位
        {200000, 0x09},  ///< 占位
        {250000, 0x01},  ///< 占位
        {300000, 0x00},  ///<  Invalid Bandwidth
};                       ///< 预先计算的 FSK 带宽寄存器值
static const RadioRegisters_t RadioRegsInit[] = {
        {MODEM_FSK, REG_LNA, 0x23},
        {MODEM_FSK, REG_RXCONFIG, 0x1E},
        {MODEM_FSK, REG_RSSICONFIG, 0xD2},
        {MODEM_FSK, REG_AFCFEI, 0x01},
        {MODEM_FSK, REG_PREAMBLEDETECT, 0xAA},
        {MODEM_FSK, REG_OSC, 0x07},
        {MODEM_FSK, REG_SYNCCONFIG, 0x12},
        {MODEM_FSK, REG_SYNCVALUE1, 0xC1},
        {MODEM_FSK, REG_SYNCVALUE2, 0x94},
        {MODEM_FSK, REG_SYNCVALUE3, 0xC1},
        {MODEM_FSK, REG_PACKETCONFIG1, 0xD8},
        {MODEM_FSK, REG_FIFOTHRESH, 0x8F},
        {MODEM_FSK, REG_IMAGECAL, 0x02},
        {MODEM_FSK, REG_DIOMAPPING1, 0x00},
        {MODEM_FSK, REG_DIOMAPPING2, 0x30},
        {MODEM_LORA, REG_LR_PAYLOADMAXLENGTH, 0xFF},
};  ///< 无线电硬件寄存器初始化



/// ------------------------------------------------------------------------------------------------------------------------------------
/// 私有函数
/// ------------------------------------------------------------------------------------------------------------------------------------

/// @brief   复制数据到指定地址
/// @param   [in] dst                      目的地址
/// @param   [in] src                      源地址
/// @param   [in] size                     数据大小
static void memcpy1(uint8_t *dst, const uint8_t *src, uint16_t size) {
    while (size--) {
        *dst++ = *src++;
    }
}

/// @brief   返回已知的 FSK 带宽寄存器值
/// @param   [in] bandwidth                以 Hz 为单位的带宽值
/// @return  带宽寄存器值。
static uint8_t _y_lora_get_Fsk_bandwidth_addr(uint32_t bandwidth) {

    for (uint32_t i = 0; i < (sizeof(FskBandwidths) / sizeof(FskBandwidth_t)) - 1; i++) {
        if ((bandwidth >= FskBandwidths[i].bandwidth) && (bandwidth < FskBandwidths[i + 1].bandwidth)) {
            return FskBandwidths[i].RegValue;
        }
    }

    //YLOGE("Fsk bandwidth reg value not found");
    return FskBandwidths[21].RegValue;
}

/// @brief    设置 SX1276 操作模式
/// @param    [in] lora                    lora 句柄
/// @param    [in] opMode                  新的模式
static void _y_lora_set_op_mode(LORA_st *lora, uint8_t opMode) {

    // 断言
    if (lora == NULL) {
        //YLOGE("lora is NULL");
        return;
    }

    y_lora_write(lora, REG_OPMODE, (y_lora_read(lora, REG_OPMODE) & RF_OPMODE_MASK) | opMode);
}

/// @brief    对 LF 和 HF 频段执行 Rx 链校准 必须在重置后立即调用，以便所有寄存器都处于默认值
/// @param    [in] lora                    lora 句柄
static void _y_lora_rx_chain_calibration(LORA_st *lora) {

    // 断言
    if (lora == NULL) {
        //YLOGE("lora is NULL");
        return;
    }

    // save context
    uint8_t  regPaConfigInitVal = y_lora_read(lora, REG_PACONFIG);
    uint32_t initialFreq =
            (uint32_t) ((double) (((uint32_t) y_lora_read(lora, REG_FRFMSB) << 16) | ((uint32_t) y_lora_read(lora, REG_FRFMID) << 8) | ((uint32_t) y_lora_read(lora, REG_FRFLSB)))
                        * (double) LORA_FREQ_STEP);

    // Cut the PA just in case, RFO output, power = -1 dBm
    y_lora_write(lora, REG_PACONFIG, 0x00);

    // Launch Rx chain calibration for LF band
    y_lora_write(lora, REG_IMAGECAL, (y_lora_read(lora, REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_MASK) | RF_IMAGECAL_IMAGECAL_START);
    while ((y_lora_read(lora, REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_RUNNING) {}

    // Sets a Frequency in HF band
    y_lora_set_channel(lora, 868000000);

    // Launch Rx chain calibration for HF band
    y_lora_write(lora, REG_IMAGECAL, (y_lora_read(lora, REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_MASK) | RF_IMAGECAL_IMAGECAL_START);
    while ((y_lora_read(lora, REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_RUNNING) {}

    // Restore context
    y_lora_write(lora, REG_PACONFIG, regPaConfigInitVal);
    y_lora_set_channel(lora, initialFreq);
}

/// @brief    读取 FIFO 的内容
/// @param    [in] lora                    lora 句柄
/// @param    [in] buffer                  数据 buf 指针
/// @param    [in] size                    数据大小
static void _y_lora_read_fifo(LORA_st *lora, uint8_t *buffer, uint8_t size) {
    y_lora_read_buf(lora, 0, buffer, size);
}

/// @brief    将缓冲区内容写入 FIFO
/// @param    [in] lora                    lora 句柄
/// @param    [in] buffer                  数据 buf 指针
/// @param    [in] size                    数据大小
static void _y_lora_write_fifo(LORA_st *lora, uint8_t *buffer, uint8_t size) {
    y_lora_write_buf(lora, 0, buffer, size);
}

/// @brief    设置 tx 超时时间
/// @param    [in] lora                    lora 句柄
/// @param    [in] timeout                 传输超时时间 ms, 0=连续
static void _y_lora_set_tx_tiomeout(LORA_st *lora, uint32_t timeout) {

    // // 断言
    // if (lora == NULL) {
    //     //YLOGE("lora is NULL");
    //     return;
    // }

    // //osTimerStop(lora->rx_timer);

    // switch (lora->setting.Modem) {

    //     case MODEM_FSK: {
    //         // DIO0 = PacketSent
    //         // DIO1 = FifoEmpty
    //         // DIO2 = FifoFull
    //         // DIO3 = FifoEmpty
    //         // DIO4 = LowBat
    //         // DIO5 = ModeReady
    //         y_lora_write(lora,
    //                      REG_DIOMAPPING1,
    //                      (y_lora_read(lora, REG_DIOMAPPING1) & RF_DIOMAPPING1_DIO0_MASK & RF_DIOMAPPING1_DIO1_MASK & RF_DIOMAPPING1_DIO2_MASK) | RF_DIOMAPPING1_DIO1_01);

    //         y_lora_write(lora, REG_DIOMAPPING2, (y_lora_read(lora, REG_DIOMAPPING2) & RF_DIOMAPPING2_DIO4_MASK & RF_DIOMAPPING2_MAP_MASK));
    //         lora->setting.FskPacketHandler.FifoThresh = y_lora_read(lora, REG_FIFOTHRESH) & 0x3F;
    //     } break;

    //     case MODEM_LORA: {
    //         if (lora->setting.LoRa.FreqHopOn == true) {
    //             y_lora_write(lora,
    //                          REG_LR_IRQFLAGSMASK,
    //                          RFLR_IRQFLAGS_RXTIMEOUT | RFLR_IRQFLAGS_RXDONE | RFLR_IRQFLAGS_PAYLOADCRCERROR | RFLR_IRQFLAGS_VALIDHEADER | RFLR_IRQFLAGS_CADDONE
    //                                  | RFLR_IRQFLAGS_CADDETECTED);

    //             // DIO0=TxDone, DIO2=FhssChangeChannel
    //             y_lora_write(lora,
    //                          REG_DIOMAPPING1,
    //                          (y_lora_read(lora, REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK) | RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO2_00);
    //         } else {
    //             y_lora_write(lora,
    //                          REG_LR_IRQFLAGSMASK,
    //                          RFLR_IRQFLAGS_RXTIMEOUT | RFLR_IRQFLAGS_RXDONE | RFLR_IRQFLAGS_PAYLOADCRCERROR | RFLR_IRQFLAGS_VALIDHEADER | RFLR_IRQFLAGS_CADDONE
    //                                  | RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL | RFLR_IRQFLAGS_CADDETECTED);

    //             // DIO0=TxDone
    //             y_lora_write(lora, REG_DIOMAPPING1, (y_lora_read(lora, REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK) | RFLR_DIOMAPPING1_DIO0_01);
    //         }
    //     } break;
    // }

    // lora->setting.State = RF_TX_RUNNING;
    // //osTimerStart(lora->tx_timer, timeout);
    // _y_lora_set_op_mode(lora, RF_OPMODE_TRANSMITTER);
}

/// @brief    设置发送功率
/// @param    [in] lora                    lora 句柄
/// @param    [in] power                   发送功率等级
static void _y_lora_set_tx_power(LORA_st *lora, int8_t power) {

    // 断言
    if (lora == NULL) {
        //YLOGE("lora is NULL");
        return;
    }

    uint8_t paConfig = y_lora_read(lora, REG_PACONFIG);
    uint8_t paDac    = y_lora_read(lora, REG_PADAC);

    if (power > 14) {
        paConfig = (paConfig & RF_PACONFIG_PASELECT_MASK) | RF_PACONFIG_PASELECT_PABOOST;
    } else {
        paConfig = (paConfig & RF_PACONFIG_PASELECT_MASK) | RF_PACONFIG_PASELECT_RFO;
    }

    if ((paConfig & RF_PACONFIG_PASELECT_PABOOST) == RF_PACONFIG_PASELECT_PABOOST) {
        if (power > 17) {
            paDac = (paDac & RF_PADAC_20DBM_MASK) | RF_PADAC_20DBM_ON;
        } else {
            paDac = (paDac & RF_PADAC_20DBM_MASK) | RF_PADAC_20DBM_OFF;
        }
        if ((paDac & RF_PADAC_20DBM_ON) == RF_PADAC_20DBM_ON) {
            if (power < 5) {
                power = 5;
            }
            if (power > 20) {
                power = 20;
            }
            paConfig = (paConfig & RF_PACONFIG_OUTPUTPOWER_MASK) | (uint8_t) ((uint16_t) (power - 5) & 0x0F);
        } else {
            if (power < 2) {
                power = 2;
            }
            if (power > 17) {
                power = 17;
            }
            paConfig = (paConfig & RF_PACONFIG_OUTPUTPOWER_MASK) | (uint8_t) ((uint16_t) (power - 2) & 0x0F);
        }
    } else {
        if (power > 0) {
            if (power > 15) {
                power = 15;
            }
            paConfig = (paConfig & RF_PACONFIG_MAX_POWER_MASK & RF_PACONFIG_OUTPUTPOWER_MASK) | (7 << 4) | (power);
        } else {
            if (power < -4) {
                power = -4;
            }
            paConfig = (paConfig & RF_PACONFIG_MAX_POWER_MASK & RF_PACONFIG_OUTPUTPOWER_MASK) | (0 << 4) | (power + 4);
        }
    }
    y_lora_write(lora, REG_PACONFIG, paConfig);
    y_lora_write(lora, REG_PADAC, paDac);
}

/// @brief    定时器超时回调处理
/// @param    [in] xTimer                  定时器句柄
static void _y_lora_timer_timeout_cb(const void *xTimer) {

    // // 断言
    // LORA_st *lora = pvTimerGetTimerID((TimerHandle_t) xTimer);
    // if (lora == NULL) {
    //     //YLOGE("lora is NULL");
    //     return;
    // }

    // switch (lora->setting.State) {

    //     case RF_RX_RUNNING:
    //         if (lora->setting.Modem == MODEM_FSK) {
    //             lora->setting.FskPacketHandler.PreambleDetected = false;
    //             lora->setting.FskPacketHandler.SyncWordDetected = false;
    //             lora->setting.FskPacketHandler.NbBytes          = 0;
    //             lora->setting.FskPacketHandler.Size             = 0;

    //             // Clear Irqs
    //             y_lora_write(lora, REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI | RF_IRQFLAGS1_PREAMBLEDETECT | RF_IRQFLAGS1_SYNCADDRESSMATCH);
    //             y_lora_write(lora, REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);

    //             if (lora->setting.Fsk.RxContinuous == true) {  // 连续模式重启 Rx 链
    //                 y_lora_write(lora, REG_RXCONFIG, y_lora_read(lora, REG_RXCONFIG) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK);
    //                 //osTimerStart(lora->rx_sync_timer, lora->setting.Fsk.RxSingleTimeout);
    //             } else {
    //                 lora->setting.State = RF_IDLE;
    //                 //osTimerStop(lora->rx_sync_timer);
    //             }
    //         }
    //         if (lora->event.RxTimeout != NULL) {
    //             lora->event.RxTimeout(lora);
    //         }
    //         break;

    //     case RF_TX_RUNNING:
    //         // Tx timeout shouldn't happen.
    //         // But it has been observed that when it happens it is a result of a corrupted SPI transfer
    //         // it depends on the platform design.
    //         //
    //         // The workaround is to put the radio in a known state. Thus, we re-initialize it.

    //         // BEGIN WORKAROUND

    //         // Reset the radio
    //         lora->port.reset();

    //         // Calibrate Rx chain
    //         _y_lora_rx_chain_calibration(lora);

    //         // Initialize radio default values
    //         _y_lora_set_op_mode(lora, RF_OPMODE_SLEEP);

    //         for (uint32_t i = 0; i < sizeof(RadioRegsInit) / sizeof(RadioRegisters_t); i++) {
    //             y_lora_set_modem(lora, RadioRegsInit[i].Modem);
    //             y_lora_write(lora, RadioRegsInit[i].Addr, RadioRegsInit[i].Value);
    //         }
    //         y_lora_set_modem(lora, MODEM_FSK);

    //         // Restore previous network type setting.
    //         y_lora_set_public_network(lora, lora->setting.LoRa.PublicNetwork);
    //         // END WORKAROUND

    //         lora->setting.State = RF_IDLE;
    //         if (lora->event.TxTimeout != NULL) {
    //             lora->event.TxTimeout(lora);
    //         }
    //         break;
    //     default:
    //         break;
    // }
}



/// ------------------------------------------------------------------------------------------------------------------------------------
/// 公有函数
/// ------------------------------------------------------------------------------------------------------------------------------------

/// @brief  打印 y_lora 版本信息
void y_lora_print_version() {
    YLOGI("y_lora module version : V%d.%d.%d", Y_LORA_MAJOR, Y_LORA_MINOR, Y_LORA_PATCH);
}


void Dummy_Callback(){
 ;   
}
void RxDone_Callback(LORA_st *lora, uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr){
    shellPrint(&shell,"Paylod:%s\r\nSize:%d\r\nRssi:%d\r\nSNR:%d\r\n",payload,size,rssi,snr);
}

uint8_t re_code= 0xff;
int Lora_Init(void){
    
    RadioEventCb_t  lora_event =
        {
            .TxDone = Dummy_Callback,
            .TxTimeout = Dummy_Callback,
            .RxDone = RxDone_Callback,
            .RxTimeout =Dummy_Callback,
            .RxError = Dummy_Callback,
            .FhssChangeChannel  = Dummy_Callback,
            .CadDone =Dummy_Callback,
        };
    RadioPortCb_t lora_port ={ 
        .spi_transmit = y_lora_2_spi_transmit,
        .spi_nss = y_lora_2_spi_nss,
        .reset = y_lora_2_reset,
    };
    lora_dev = y_lora_create(&lora_event, &lora_port);

    y_lora_set_channel(lora_dev, jt_lora_params.Freq);

    y_lora_set_modem(lora_dev, MODEM_LORA);

    // re_code = y_lora_read(lora_dev, 0x01);

    // y_lora_set_modem(lora_dev, MODEM_FSK);

    // re_code = y_lora_read(lora_dev, 0x01);

    y_lora_set_tx_config(lora_dev, MODEM_LORA, jt_lora_params.Freq, jt_lora_params.Bandwidth, jt_lora_params.Datarate, jt_lora_params.Coderate);
    y_lora_set_rx_config(lora_dev, MODEM_LORA, jt_lora_params.Bandwidth, jt_lora_params.Datarate, jt_lora_params.Coderate);

    if((y_lora_read(lora_dev, 0x01) & 0X80) != 0x80)
    {
        re_code = y_lora_read(lora_dev, 0x01);
        return 0 ;
    }
    else
        return 1;
}
/// @brief   创建 lora 实例
/// @param   [in] event                    事件回调组
/// @param   [in] port                     底层端口回调组
/// @param   [in] buf_length               接收发送缓冲区大小
LORA_st *y_lora_create(RadioEventCb_t *event, RadioPortCb_t *port) {

    // 断言
    if (event == NULL || port == NULL) {
        //YLOGE("LORA_EVENT_CB_st is NULL or  LORA_PORT_CB_st is NULL, lora_init fail");
        return NULL;
    }

    // 创建 LORA 句柄
    LORA_st *lora = (LORA_st *) malloc(sizeof(LORA_st));  // 开辟 lora 结构体句柄 所需要的空间
    if (lora == NULL) {
        //YLOGE("malloc LORA_st error");
        return NULL;
    } else {
        memset(lora, 0, sizeof(LORA_st));  // 清零
    }

    // 初始化 lora 句柄
    memcpy(&lora->event, event, sizeof(RadioEventCb_t));
    memcpy(&lora->port, port, sizeof(RadioPortCb_t));

    // osTimerDef(lora_tx_timer, _y_lora_timer_timeout_cb);
    // osTimerDef(lora_rx_timer, _y_lora_timer_timeout_cb);
    // osTimerDef(lora_rx_sync_timer, _y_lora_timer_timeout_cb);
    // lora->tx_timer      = osTimerCreate(osTimer(lora_tx_timer), osTimerOnce, (void *) lora);
    // lora->rx_timer      = osTimerCreate(osTimer(lora_rx_timer), osTimerOnce, (void *) lora);
    // lora->rx_sync_timer = osTimerCreate(osTimer(lora_rx_sync_timer), osTimerOnce, (void *) lora);
    // if (lora->tx_timer == NULL || lora->rx_timer == NULL || lora->rx_sync_timer == NULL) {
    //     //YLOGE("malloc lora timer");
    //     goto end;
    // }

    lora->port.reset();                          // 重启 lora
    _y_lora_rx_chain_calibration(lora);          // 重启链
    _y_lora_set_op_mode(lora, RF_OPMODE_SLEEP);  // 设置休眠模式

    for (uint32_t i = 0; i < sizeof(RadioRegsInit) / sizeof(RadioRegisters_t); i++) {
        y_lora_set_modem(lora, RadioRegsInit[i].Modem);
        y_lora_write(lora, RadioRegsInit[i].Addr, RadioRegsInit[i].Value);
    }

    y_lora_set_modem(lora, MODEM_FSK);
    lora->setting.State = RF_IDLE;

    return lora;

// 创建失败
end:
    // if (lora->tx_timer != NULL) {
    //     xTimerDelete(lora->tx_timer, 200);
    // }
    // if (lora->rx_timer != NULL) {
    //     xTimerDelete(lora->rx_timer, 200);
    // }
    // if (lora->rx_sync_timer != NULL) {
    //     xTimerDelete(lora->rx_sync_timer, 200);
    // }
    // if (lora != NULL) {
    //     free(lora);
    // }

    return NULL;
}

/// @brief    销毁 lora 实例
/// @param    [in] lora                    lora 句柄
void y_lora_destroy(LORA_st *lora) {

    // 断言
    // if (lora == NULL) {
    //     //YLOGE("lora is NULL");
    //     return;
    // }

    // if (lora->tx_timer != NULL) {
    //     xTimerDelete(lora->tx_timer, 200);
    // }
    // if (lora->rx_timer != NULL) {
    //     xTimerDelete(lora->rx_timer, 200);
    // }
    // if (lora->rx_sync_timer != NULL) {
    //     xTimerDelete(lora->rx_sync_timer, 200);
    // }
    if (lora != NULL) {
        free(lora);
    }
}

/// @brief   写入多个数据到指定寄存器
/// @param   [in] lora                     lora 句柄
/// @param   [in] addr                     写入地址
/// @param   [in] buffer                   数据指针
/// @param   [in] size                     数据大小
void y_lora_write_buf(LORA_st *lora, uint16_t addr, uint8_t *buffer, uint8_t size) {

    // 断言
    if (lora == NULL) {
        //YLOGE("lora is NULL");
        return;
    }

    lora->port.spi_nss(0);

    lora->port.spi_transmit((uint8_t) (addr | 0x80));
    for (uint8_t i = 0; i < size; i++) {
        lora->port.spi_transmit(buffer[i]);
    }

    lora->port.spi_nss(1);
}

/// @brief   读取指定寄存器的值到 buf 中
/// @param   [in] lora                     lora 句柄
/// @param   [in] addr                      读取地址
/// @param   [in] buffer                    数据指针
/// @param   [in] size                      数据大小
void y_lora_read_buf(LORA_st *lora, uint16_t addr, uint8_t *buffer, uint8_t size) {

    // 断言
    if (lora == NULL) {
        //YLOGE("lora is NULL");
        return;
    }

    lora->port.spi_nss(0);

    lora->port.spi_transmit((uint8_t) (addr & 0x7F));
    for (uint8_t i = 0; i < size; i++) {
        buffer[i] = lora->port.spi_transmit(0);
    }

    lora->port.spi_nss(1);
}

/// @brief   写入单个数据到指定寄存器
/// @param   [in] lora                     lora 句柄
/// @param   [in] addr                     寄存器地址
/// @param   [in] data                     新的寄存器值
void y_lora_write(LORA_st *lora, uint16_t addr, uint8_t data) {
    y_lora_write_buf(lora, addr, &data, 1);
}

/// @brief   读取指定寄存器的值
/// @param   [in] lora                     lora 句柄
/// @param   [in] addr                     寄存器地址
/// @return  寄存器值
uint8_t y_lora_read(LORA_st *lora, uint16_t addr) {
    uint8_t data = 0;
    y_lora_read_buf(lora, addr, &data, 1);
    return data;
}

/// @brief    检查频道在给定时间内是否空闲
/// @param    [in] lora                    lora 句柄
/// @param    [in] modem                   要使用的无线电调制解调器 [0：FSK，1：LoRa]
/// @param    [in] freq                    频道射频频率
/// @param    [in] rssiThresh              RSSI 阈值
/// @param    [in] Timeout                 测量 RSSI 的最长时间
/// @retval   true                         频道空闲
/// @retval   false                        频道不空闲
bool y_lora_is_channel_free(LORA_st *lora, RadioModem_e modem, uint32_t freq, int16_t rssiThresh, uint32_t Timeout) {

    // 断言
    if (lora == NULL) {
        //YLOGE("lora is NULL");
        return false;
    }

    if (y_lora_get_status(lora) != RF_IDLE) {
        return false;
    }

    y_lora_set_modem(lora, modem);
    y_lora_set_channel(lora, freq);
    _y_lora_set_op_mode(lora, RF_OPMODE_RECEIVER);

    HAL_Delay(1);

    bool status = true;
    // todo：fix it， if used
    // uint32_t carrierSenseTime = TimerGetCurrentTime();

    // Perform carrier sense for maxCarrierSenseTime
    // while (TimerGetElapsedTime(carrierSenseTime) < maxCarrierSenseTime) {
    //     int16_t  rssi = SX1276ReadRssi(modem);
    //
    //     if (rssi > rssiThresh) {
    //         status = false;
    //         break;
    //     }
    // }
    y_lora_set_sleep(lora);
    return status;
}

/// @brief    发送数据。准备要发送的数据包并将无线电设置为传输
/// @param    [in] lora                    lora 句柄
/// @param    [in] buffer                  缓冲区指针
/// @param    [in] size                    缓冲区大小
void y_lora_send(LORA_st *lora, uint8_t *buffer, uint8_t size) {

    // 断言
    if (lora == NULL) {
        //YLOGE("lora is NULL");
        return;
    }

    uint32_t txTimeout = 0;
    switch (lora->setting.Modem) {

        case MODEM_FSK: {
            lora->setting.FskPacketHandler.NbBytes = 0;
            lora->setting.FskPacketHandler.Size    = size;

            if (lora->setting.Fsk.FixLen == false) {
                _y_lora_write_fifo(lora, (uint8_t *) &size, 1);
            } else {
                y_lora_write(lora, REG_PAYLOADLENGTH, size);
            }

            if ((size > 0) && (size <= 64)) {
                lora->setting.FskPacketHandler.ChunkSize = size;
            } else {
                memcpy1(lora->buf, buffer, size);
                lora->setting.FskPacketHandler.ChunkSize = 32;
            }

            // Write payload buffer
            _y_lora_write_fifo(lora, buffer, lora->setting.FskPacketHandler.ChunkSize);
            lora->setting.FskPacketHandler.NbBytes += lora->setting.FskPacketHandler.ChunkSize;
            txTimeout = lora->setting.Fsk.TxTimeout;
        } break;

        case MODEM_LORA: {
            if (lora->setting.LoRa.IqInverted == true) {
                y_lora_write(lora,
                             REG_LR_INVERTIQ,
                             ((y_lora_read(lora, REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON));
                y_lora_write(lora, REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON);
            } else {
                y_lora_write(lora,
                             REG_LR_INVERTIQ,
                             ((y_lora_read(lora, REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF));
                y_lora_write(lora, REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF);
            }

            lora->setting.LoRaPacketHandler.Size = size;

            // Initializes the payload size
            y_lora_write(lora, REG_LR_PAYLOADLENGTH, size);

            // Full buffer used for Tx
            y_lora_write(lora, REG_LR_FIFOTXBASEADDR, 0);
            y_lora_write(lora, REG_LR_FIFOADDRPTR, 0);

            // FIFO operations can not take place in Sleep mode
            if ((y_lora_read(lora, REG_OPMODE) & ~RF_OPMODE_MASK) == RF_OPMODE_SLEEP) {
                y_lora_set_stby(lora);
                HAL_Delay(1);
            }
            // Write payload buffer
            _y_lora_write_fifo(lora, buffer, size);
            txTimeout = lora->setting.LoRa.TxTimeout;
        } break;
    }

    _y_lora_set_tx_tiomeout(lora, txTimeout);
}

/// @brief    启动通道活动检测
/// @param    [in] lora                    lora 句柄
void y_lora_start_cad(LORA_st *lora) {

    // 断言
    if (lora == NULL) {
        //YLOGE("lora is NULL");
        return;
    }

    switch (lora->setting.Modem) {
        case MODEM_FSK: {

        } break;
        case MODEM_LORA: {
            y_lora_write(lora,
                         REG_LR_IRQFLAGSMASK,
                         RFLR_IRQFLAGS_RXTIMEOUT | RFLR_IRQFLAGS_RXDONE | RFLR_IRQFLAGS_PAYLOADCRCERROR | RFLR_IRQFLAGS_VALIDHEADER | RFLR_IRQFLAGS_TXDONE
                                 | RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL);

            // DIO3=CADDone
            y_lora_write(lora, REG_DIOMAPPING1, (y_lora_read(lora, REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO3_MASK) | RFLR_DIOMAPPING1_DIO3_00);

            lora->setting.State = RF_CAD;
            _y_lora_set_op_mode(lora, RFLR_OPMODE_CAD);
        } break;
        default:
            break;
    }
}

/// @brief    获取 lora 运行状态
/// @param    [in] lora                    lora 句柄
/// @return   状态值
RadioState_e y_lora_get_status(LORA_st *lora) {

    // 断言
    if (lora == NULL) {
        //YLOGE("lora is NULL");
        return 0;
    }

    return lora->setting.State;
}

/// @brief    根据 RSSI 读数生成 32 位随机值
/// @param    [in] lora                    lora 句柄
/// @return   32位随机值
uint32_t y_lora_get_random(LORA_st *lora) {

    // 断言
    if (lora == NULL) {
        //YLOGE("lora is NULL");
        return 0;
    }

    // 将 LoRa 调制解调器设置为 ON
    y_lora_set_modem(lora, MODEM_LORA);

    // 禁用 LoRa 调制解调器中断
    y_lora_write(lora,
                 REG_LR_IRQFLAGSMASK,
                 RFLR_IRQFLAGS_RXTIMEOUT | RFLR_IRQFLAGS_RXDONE | RFLR_IRQFLAGS_PAYLOADCRCERROR | RFLR_IRQFLAGS_VALIDHEADER | RFLR_IRQFLAGS_TXDONE | RFLR_IRQFLAGS_CADDONE
                         | RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL | RFLR_IRQFLAGS_CADDETECTED);

    // 将无线电设置为连续接收
    _y_lora_set_op_mode(lora, RF_OPMODE_RECEIVER);

    uint32_t rnd = 0;
    for (uint8_t i = 0; i < 32; i++) {
        HAL_Delay(1);
        rnd |= ((uint32_t) y_lora_read(lora, REG_LR_RSSIWIDEBAND) & 0x01) << i;  // 未经过滤的 RSSI 值读取。只接受LSB值
    }

    y_lora_set_sleep(lora);

    return rnd;
}

/// @brief    读取当前 RSSI 值
/// @param    [in] lora                    lora 句柄
/// @param    [in] modem                   要使用的无线电调制解调器
/// @return   当前 RSSI 值 [dBm]
int16_t y_lora_get_rssi(LORA_st *lora, RadioModem_e modem) {

    // 断言
    if (lora == NULL) {
        //YLOGE("lora is NULL");
        return 0;
    }

    int16_t rssi = 0;

    switch (modem) {
        case MODEM_FSK:
            rssi = -(y_lora_read(lora, REG_RSSIVALUE) >> 1);
            break;
        case MODEM_LORA:
            if (lora->setting.Channel > LORA_RF_MID_THRESH) {
                rssi = RSSI_OFFSET_HF + y_lora_read(lora, REG_LR_RSSIVALUE);
            } else {
                rssi = RSSI_OFFSET_LF + y_lora_read(lora, REG_LR_RSSIVALUE);
            }
            break;
        default:
            rssi = -1;
            break;
    }
    return rssi;
}

/// @brief    计算给定有效载荷的空中数据包时间（以毫秒为单位） 只能在调用 set rx config 或 set tx config 后调用
/// @param    [in] lora                    lora 句柄
/// @param    [in] modem                   要使用的无线电调制解调器
/// @param    [in] pktLen                  数据包有效载荷长度
/// @return   给定数据包有效负载长度的计算的 airTime (ms)
uint32_t y_lora_get_time_on_air(LORA_st *lora, RadioModem_e modem, uint8_t pktLen) {

    // 断言
    if (lora == NULL) {
        //YLOGE("lora is NULL");
        return 0;
    }

    uint32_t airTime = 0;
    switch (modem) {

        case MODEM_FSK: {
            airTime = round(
                    (8
                     * (lora->setting.Fsk.PreambleLen + ((y_lora_read(lora, REG_SYNCCONFIG) & ~RF_SYNCCONFIG_SYNCSIZE_MASK) + 1) + ((lora->setting.Fsk.FixLen == 0x01) ? 0.0 : 1.0)
                        + (((y_lora_read(lora, REG_PACKETCONFIG1) & ~RF_PACKETCONFIG1_ADDRSFILTERING_MASK) != 0x00) ? 1.0 : 0) + pktLen
                        + ((lora->setting.Fsk.CrcOn == 0x01) ? 2.0 : 0))
                     / lora->setting.Fsk.Datarate)
                    * 1000);
        } break;

        case MODEM_LORA: {
            double bw = 0.0;
            // REMARK: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
            switch (lora->setting.LoRa.Bandwidth) {
                case 7:  // 125 kHz
                    bw = 125000;
                    break;
                case 8:  // 250 kHz
                    bw = 250000;
                    break;
                case 9:  // 500 kHz
                    bw = 500000;
                    break;
            }

            // Symbol rate : time for one symbol (secs)
            double rs        = bw / (1 << lora->setting.LoRa.Datarate);
            double ts        = 1 / rs;
            // time of preamble
            double tPreamble = (lora->setting.LoRa.PreambleLen + 4.25) * ts;
            // Symbol length of payload and time
            double tmp       = ceil((8 * pktLen - 4 * lora->setting.LoRa.Datarate + 28 + 16 * lora->setting.LoRa.CrcOn - (lora->setting.LoRa.FixLen ? 20 : 0))
                              / (double) (4 * (lora->setting.LoRa.Datarate - ((lora->setting.LoRa.LowDatarateOptimize > 0) ? 2 : 0))))
                         * (lora->setting.LoRa.Coderate + 4);
            double nPayload = 8 + ((tmp > 0) ? tmp : 0);
            double tPayload = nPayload * ts;
            // Time on air
            double tOnAir   = tPreamble + tPayload;
            // return ms secs
            airTime         = floor(tOnAir * 1000 + 0.999);
        } break;
    }
    return airTime;
}

/// @brief   获取无线电从睡眠中醒来所需的时间。[ms]
/// @return  无线电唤醒时间（毫秒）。
uint32_t y_lora_get_wakeup_time() {
    return LORA_TCXO_WAKEUP_TIME + LORA_WAKEUP_TIME;
}

/// @brief    设置模式
/// @param    [in] lora                    lora 句柄
/// @param    [in] modem                   要使用的调制解调器
void y_lora_set_modem(LORA_st *lora, RadioModem_e modem) {

    // 断言
    if (lora == NULL) {
        //YLOGE("lora is NULL");
        return;
    }

    if ((y_lora_read(lora, REG_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_ON) != 0) {
        lora->setting.Modem = MODEM_LORA;
    } else {
        lora->setting.Modem = MODEM_FSK;
    }

    if (lora->setting.Modem == modem) {
        return;
    }

    lora->setting.Modem = modem;
    switch (lora->setting.Modem) {
        default:
        case MODEM_FSK:
            _y_lora_set_op_mode(lora, RF_OPMODE_SLEEP);
            y_lora_write(lora, REG_OPMODE, (y_lora_read(lora, REG_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_MASK) | RFLR_OPMODE_LONGRANGEMODE_OFF);
            y_lora_write(lora, REG_DIOMAPPING1, 0x00);
            y_lora_write(lora, REG_DIOMAPPING2, 0x30);  // DIO5=ModeReady
            break;
        case MODEM_LORA:
            _y_lora_set_op_mode(lora, RF_OPMODE_SLEEP);
            y_lora_write(lora, REG_OPMODE, (y_lora_read(lora, REG_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_MASK) | RFLR_OPMODE_LONGRANGEMODE_ON);
            y_lora_write(lora, REG_DIOMAPPING1, 0x00);
            y_lora_write(lora, REG_DIOMAPPING2, 0x00);
            break;
    }
}

/// @brief    设置通道频率
/// @param    [in] lora                    lora 句柄
/// @param    [in] freq                    频道频率
void y_lora_set_channel(LORA_st *lora, uint32_t freq) {

    // 断言
    if (lora == NULL) {
        //YLOGE("lora is NULL");
        return;
    }

    lora->setting.Channel = freq;
    freq                  = (uint32_t) ((double) freq / (double) LORA_FREQ_STEP);
    y_lora_write(lora, REG_FRFMSB, (uint8_t) ((freq >> 16) & 0xFF));
    y_lora_write(lora, REG_FRFMID, (uint8_t) ((freq >> 8) & 0xFF));
    y_lora_write(lora, REG_FRFLSB, (uint8_t) (freq & 0xFF));
}

/// @brief    设置发送参数
/// @param    [in] lora                    lora 句柄
/// @param    [in] modem                   要使用的无线电调制解调器
/// @param    [in] power                   设置输出功率 [dBm]
/// @param    [in] bandwidth               设置带宽（仅限 LoRa）
/// @param    [in] datarate                设置数据速率
/// @param    [in] coderate                设置编码率（仅限 LoRa）
void y_lora_set_tx_config(LORA_st *lora, RadioModem_e modem, int8_t power, uint32_t bandwidth, uint32_t datarate, uint8_t coderate) {

    // 断言
    if (lora == NULL) {
        //YLOGE("lora is NULL");
        return;
    }

    uint16_t fdev        = LORA_FDEV;
    uint16_t preambleLen = LORA_PREAMBLE_LEN;
    bool     fixLen      = LORA_FIXLEN;
    bool     crcOn       = LORA_CRC_ON;
    bool     freqHopOn   = LORA_FREQ_HOP_ON;
    uint8_t  hopPeriod   = LORA_HOP_PERIOD;
    bool     iqInverted  = LORA_IQ_INVERTED;
    uint32_t timeout     = LORA_TX_TIMEOUT;

    y_lora_set_modem(lora, modem);
    _y_lora_set_tx_power(lora, power);

    switch (modem) {
        case MODEM_FSK: {
            lora->setting.Fsk.Power       = power;
            lora->setting.Fsk.Fdev        = fdev;
            lora->setting.Fsk.Bandwidth   = bandwidth;
            lora->setting.Fsk.Datarate    = datarate;
            lora->setting.Fsk.PreambleLen = preambleLen;
            lora->setting.Fsk.FixLen      = fixLen;
            lora->setting.Fsk.CrcOn       = crcOn;
            lora->setting.Fsk.IqInverted  = iqInverted;
            lora->setting.Fsk.TxTimeout   = timeout;

            fdev                          = (uint16_t) ((double) fdev / (double) LORA_FREQ_STEP);
            y_lora_write(lora, REG_FDEVMSB, (uint8_t) (fdev >> 8));
            y_lora_write(lora, REG_FDEVLSB, (uint8_t) (fdev & 0xFF));

            datarate = (uint16_t) ((double) LORA_XTAL_FREQ / (double) datarate);
            y_lora_write(lora, REG_BITRATEMSB, (uint8_t) (datarate >> 8));
            y_lora_write(lora, REG_BITRATELSB, (uint8_t) (datarate & 0xFF));

            y_lora_write(lora, REG_PREAMBLEMSB, (preambleLen >> 8) & 0x00FF);
            y_lora_write(lora, REG_PREAMBLELSB, preambleLen & 0xFF);

            y_lora_write(lora,
                         REG_PACKETCONFIG1,
                         (y_lora_read(lora, REG_PACKETCONFIG1) & RF_PACKETCONFIG1_CRC_MASK & RF_PACKETCONFIG1_PACKETFORMAT_MASK)
                                 | ((fixLen == 1) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE) | (crcOn << 4));
            y_lora_write(lora, REG_PACKETCONFIG2, (y_lora_read(lora, REG_PACKETCONFIG2) | RF_PACKETCONFIG2_DATAMODE_PACKET));
        } break;
        case MODEM_LORA: {
            lora->setting.LoRa.Power = power;
            if (bandwidth > 2) {
                // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
                while (1)
                    ;
            }
            bandwidth += 7;
            lora->setting.LoRa.Bandwidth   = bandwidth;
            lora->setting.LoRa.Datarate    = datarate;
            lora->setting.LoRa.Coderate    = coderate;
            lora->setting.LoRa.PreambleLen = preambleLen;
            lora->setting.LoRa.FixLen      = fixLen;
            lora->setting.LoRa.FreqHopOn   = freqHopOn;
            lora->setting.LoRa.HopPeriod   = hopPeriod;
            lora->setting.LoRa.CrcOn       = crcOn;
            lora->setting.LoRa.IqInverted  = iqInverted;
            lora->setting.LoRa.TxTimeout   = timeout;

            if (datarate > 12) {
                datarate = 12;
            } else if (datarate < 6) {
                datarate = 6;
            }
            if (((bandwidth == 7) && ((datarate == 11) || (datarate == 12))) || ((bandwidth == 8) && (datarate == 12))) {
                lora->setting.LoRa.LowDatarateOptimize = 0x01;
            } else {
                lora->setting.LoRa.LowDatarateOptimize = 0x00;
            }

            if (lora->setting.LoRa.FreqHopOn == true) {
                y_lora_write(lora, REG_LR_PLLHOP, (y_lora_read(lora, REG_LR_PLLHOP) & RFLR_PLLHOP_FASTHOP_MASK) | RFLR_PLLHOP_FASTHOP_ON);
                y_lora_write(lora, REG_LR_HOPPERIOD, lora->setting.LoRa.HopPeriod);
            }

            y_lora_write(lora,
                         REG_LR_MODEMCONFIG1,
                         (y_lora_read(lora, REG_LR_MODEMCONFIG1) & RFLR_MODEMCONFIG1_BW_MASK & RFLR_MODEMCONFIG1_CODINGRATE_MASK & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK)
                                 | (bandwidth << 4) | (coderate << 1) | fixLen);

            y_lora_write(lora,
                         REG_LR_MODEMCONFIG2,
                         (y_lora_read(lora, REG_LR_MODEMCONFIG2) & RFLR_MODEMCONFIG2_SF_MASK & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK) | (datarate << 4) | (crcOn << 2));

            y_lora_write(lora,
                         REG_LR_MODEMCONFIG3,
                         (y_lora_read(lora, REG_LR_MODEMCONFIG3) & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK) | (lora->setting.LoRa.LowDatarateOptimize << 3));

            y_lora_write(lora, REG_LR_PREAMBLEMSB, (preambleLen >> 8) & 0x00FF);
            y_lora_write(lora, REG_LR_PREAMBLELSB, preambleLen & 0xFF);

            if (datarate == 6) {
                y_lora_write(lora, REG_LR_DETECTOPTIMIZE, (y_lora_read(lora, REG_LR_DETECTOPTIMIZE) & RFLR_DETECTIONOPTIMIZE_MASK) | RFLR_DETECTIONOPTIMIZE_SF6);
                y_lora_write(lora, REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF6);
            } else {
                y_lora_write(lora, REG_LR_DETECTOPTIMIZE, (y_lora_read(lora, REG_LR_DETECTOPTIMIZE) & RFLR_DETECTIONOPTIMIZE_MASK) | RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12);
                y_lora_write(lora, REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF7_TO_SF12);
            }
        } break;
    }
}

/// @brief    设置为连续波传输模式
/// @param    [in] lora                    lora 句柄
/// @param    [in] freq                    频道射频频率
/// @param    [in] power                   设置输出功率 [dBm]
/// @param    [in] time                    传输模式超时 [s]
void y_lora_set_tx_continuous_wave(LORA_st *lora, uint32_t freq, int8_t power, uint16_t time) {

    // 断言
    if (lora == NULL) {
        //YLOGE("lora is NULL");
        return;
    }

    uint32_t timeout = (uint32_t) (time * 1000);

    y_lora_set_channel(lora, freq);

    y_lora_set_tx_config(lora, MODEM_FSK, power, 0, 4800, 0);

    y_lora_write(lora, REG_PACKETCONFIG2, (y_lora_read(lora, REG_PACKETCONFIG2) & RF_PACKETCONFIG2_DATAMODE_MASK));
    // Disable radio interrupts
    y_lora_write(lora, REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_11 | RF_DIOMAPPING1_DIO1_11);
    y_lora_write(lora, REG_DIOMAPPING2, RF_DIOMAPPING2_DIO4_10 | RF_DIOMAPPING2_DIO5_10);

    lora->setting.State = RF_TX_RUNNING;
    ////osTimerStart(lora->tx_timer, timeout);

    _y_lora_set_op_mode(lora, RF_OPMODE_TRANSMITTER);
}

/// @brief    设置接收参数
/// @param    [in] lora                    lora 句柄
/// @param    [in] modem                   要使用的无线电调制解调器
/// @param    [in] bandwidth               设置带宽
/// @param    [in] datarate                设置数据速率
/// @param    [in] coderate                设置编码率(仅限 LoRa)
void y_lora_set_rx_config(LORA_st *lora, RadioModem_e modem, uint32_t bandwidth, uint32_t datarate, uint8_t coderate) {

    // 断言
    if (lora == NULL) {
        //YLOGE("lora is NULL");
        return;
    }

    uint32_t bandwidthAfc = LORA_BANDWIDTH_AFC;  // 设置 AFC 带宽(仅限 FSK
    uint16_t preambleLen  = LORA_PREAMBLE_LEN;   // 设置前导码长度
    uint16_t symbTimeout  = LORA_SYMB_TIMEOUT;   // 设置 RxSingle 超时值
    bool     fixLen       = LORA_FIXLEN;         // 固定长度数据包
    uint8_t  payloadLen   = LORA_PAYLOAD_LEN;    // 固定长度时设置有效负载长度
    bool     crcOn        = LORA_CRC_ON;         // 启用/禁用 CRC
    bool     freqHopOn    = LORA_FREQ_HOP_ON;    // 启用/禁用数据包内跳频
    uint8_t  hopPeriod    = LORA_HOP_PERIOD;     // 每跳之间的符号数
    bool     iqInverted   = LORA_IQ_INVERTED;    // 反转 IQ 信号（仅限 LoRa
    bool     rxContinuous = LORA_RX_CONTINUOUS;  // 将接收设置为连续模式

    y_lora_set_modem(lora, modem);

    switch (modem) {
        case MODEM_FSK: {
            lora->setting.Fsk.Bandwidth       = bandwidth;
            lora->setting.Fsk.Datarate        = datarate;
            lora->setting.Fsk.BandwidthAfc    = bandwidthAfc;
            lora->setting.Fsk.FixLen          = fixLen;
            lora->setting.Fsk.PayloadLen      = payloadLen;
            lora->setting.Fsk.CrcOn           = crcOn;
            lora->setting.Fsk.IqInverted      = iqInverted;
            lora->setting.Fsk.RxContinuous    = rxContinuous;
            lora->setting.Fsk.PreambleLen     = preambleLen;
            lora->setting.Fsk.RxSingleTimeout = (uint32_t) (symbTimeout * ((1.0 / (double) datarate) * 8.0) * 1000);

            datarate                          = (uint16_t) ((double) LORA_XTAL_FREQ / (double) datarate);
            y_lora_write(lora, REG_BITRATEMSB, (uint8_t) (datarate >> 8));
            y_lora_write(lora, REG_BITRATELSB, (uint8_t) (datarate & 0xFF));

            y_lora_write(lora, REG_RXBW, _y_lora_get_Fsk_bandwidth_addr(bandwidth));
            y_lora_write(lora, REG_AFCBW, _y_lora_get_Fsk_bandwidth_addr(bandwidthAfc));

            y_lora_write(lora, REG_PREAMBLEMSB, (uint8_t) ((preambleLen >> 8) & 0xFF));
            y_lora_write(lora, REG_PREAMBLELSB, (uint8_t) (preambleLen & 0xFF));

            if (fixLen == 1) {
                y_lora_write(lora, REG_PAYLOADLENGTH, payloadLen);
            } else {
                y_lora_write(lora, REG_PAYLOADLENGTH, 0xFF);  // Set payload length to the maximum
            }

            y_lora_write(lora,
                         REG_PACKETCONFIG1,
                         (y_lora_read(lora, REG_PACKETCONFIG1) & RF_PACKETCONFIG1_CRC_MASK & RF_PACKETCONFIG1_PACKETFORMAT_MASK)
                                 | ((fixLen == 1) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE) | (crcOn << 4));
            y_lora_write(lora, REG_PACKETCONFIG2, (y_lora_read(lora, REG_PACKETCONFIG2) | RF_PACKETCONFIG2_DATAMODE_PACKET));
        } break;
        case MODEM_LORA: {
            if (bandwidth > 2) {
                // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
                while (1)
                    ;
            }
            bandwidth += 7;
            lora->setting.LoRa.Bandwidth    = bandwidth;
            lora->setting.LoRa.Datarate     = datarate;
            lora->setting.LoRa.Coderate     = coderate;
            lora->setting.LoRa.PreambleLen  = preambleLen;
            lora->setting.LoRa.FixLen       = fixLen;
            lora->setting.LoRa.PayloadLen   = payloadLen;
            lora->setting.LoRa.CrcOn        = crcOn;
            lora->setting.LoRa.FreqHopOn    = freqHopOn;
            lora->setting.LoRa.HopPeriod    = hopPeriod;
            lora->setting.LoRa.IqInverted   = iqInverted;
            lora->setting.LoRa.RxContinuous = rxContinuous;

            if (datarate > 12) {
                datarate = 12;
            } else if (datarate < 6) {
                datarate = 6;
            }

            if (((bandwidth == 7) && ((datarate == 11) || (datarate == 12))) || ((bandwidth == 8) && (datarate == 12))) {
                lora->setting.LoRa.LowDatarateOptimize = 0x01;
            } else {
                lora->setting.LoRa.LowDatarateOptimize = 0x00;
            }

            y_lora_write(lora,
                         REG_LR_MODEMCONFIG1,
                         (y_lora_read(lora, REG_LR_MODEMCONFIG1) & RFLR_MODEMCONFIG1_BW_MASK & RFLR_MODEMCONFIG1_CODINGRATE_MASK & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK)
                                 | (bandwidth << 4) | (coderate << 1) | fixLen);

            y_lora_write(lora,
                         REG_LR_MODEMCONFIG2,
                         (y_lora_read(lora, REG_LR_MODEMCONFIG2) & RFLR_MODEMCONFIG2_SF_MASK & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK)
                                 | (datarate << 4) | (crcOn << 2) | ((symbTimeout >> 8) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK));

            y_lora_write(lora,
                         REG_LR_MODEMCONFIG3,
                         (y_lora_read(lora, REG_LR_MODEMCONFIG3) & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK) | (lora->setting.LoRa.LowDatarateOptimize << 3));

            y_lora_write(lora, REG_LR_SYMBTIMEOUTLSB, (uint8_t) (symbTimeout & 0xFF));

            y_lora_write(lora, REG_LR_PREAMBLEMSB, (uint8_t) ((preambleLen >> 8) & 0xFF));
            y_lora_write(lora, REG_LR_PREAMBLELSB, (uint8_t) (preambleLen & 0xFF));

            if (fixLen == 1) {
                y_lora_write(lora, REG_LR_PAYLOADLENGTH, payloadLen);
            }

            if (lora->setting.LoRa.FreqHopOn == true) {
                y_lora_write(lora, REG_LR_PLLHOP, (y_lora_read(lora, REG_LR_PLLHOP) & RFLR_PLLHOP_FASTHOP_MASK) | RFLR_PLLHOP_FASTHOP_ON);
                y_lora_write(lora, REG_LR_HOPPERIOD, lora->setting.LoRa.HopPeriod);
            }

            if ((bandwidth == 9) && (lora->setting.Channel > LORA_RF_MID_THRESH)) {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                y_lora_write(lora, REG_LR_HIGHBWOPTIMIZE1, 0x02);
                y_lora_write(lora, REG_LR_HIGHBWOPTIMIZE2, 0x64);
            } else if (bandwidth == 9) {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                y_lora_write(lora, REG_LR_HIGHBWOPTIMIZE1, 0x02);
                y_lora_write(lora, REG_LR_HIGHBWOPTIMIZE2, 0x7F);
            } else {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                y_lora_write(lora, REG_LR_HIGHBWOPTIMIZE1, 0x03);
            }

            if (datarate == 6) {
                y_lora_write(lora, REG_LR_DETECTOPTIMIZE, (y_lora_read(lora, REG_LR_DETECTOPTIMIZE) & RFLR_DETECTIONOPTIMIZE_MASK) | RFLR_DETECTIONOPTIMIZE_SF6);
                y_lora_write(lora, REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF6);
            } else {
                y_lora_write(lora, REG_LR_DETECTOPTIMIZE, (y_lora_read(lora, REG_LR_DETECTOPTIMIZE) & RFLR_DETECTIONOPTIMIZE_MASK) | RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12);
                y_lora_write(lora, REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF7_TO_SF12);
            }

            y_lora_set_max_payload_length(lora, MODEM_LORA, 0xFF);
        } break;
    }
}

/// @brief    在给定时间内将无线电设置为接收模式
/// @param    [in] lora                    lora 句柄
/// @param    [in] timeout                 接收时间ms  0=连续
void y_lora_set_rx_time(LORA_st *lora, uint32_t timeout) {

    // 断言
    if (lora == NULL) {
        //YLOGE("lora is NULL");
        return;
    }

    bool rxContinuous = false;
    //osTimerStop(lora->tx_timer);

    switch (lora->setting.Modem) {
        case MODEM_FSK: {
            rxContinuous = lora->setting.Fsk.RxContinuous;
            // DIO0=PayloadReady
            // DIO1=FifoLevel
            // DIO2=SyncAddr
            // DIO3=FifoEmpty
            // DIO4=Preamble
            // DIO5=ModeReady
            y_lora_write(lora,
                         REG_DIOMAPPING1,
                         (y_lora_read(lora, REG_DIOMAPPING1) & RF_DIOMAPPING1_DIO0_MASK & RF_DIOMAPPING1_DIO1_MASK & RF_DIOMAPPING1_DIO2_MASK) | RF_DIOMAPPING1_DIO0_00
                                 | RF_DIOMAPPING1_DIO1_00 | RF_DIOMAPPING1_DIO2_11);

            y_lora_write(lora,
                         REG_DIOMAPPING2,
                         (y_lora_read(lora, REG_DIOMAPPING2) & RF_DIOMAPPING2_DIO4_MASK & RF_DIOMAPPING2_MAP_MASK) | RF_DIOMAPPING2_DIO4_11 | RF_DIOMAPPING2_MAP_PREAMBLEDETECT);

            lora->setting.FskPacketHandler.FifoThresh = y_lora_read(lora, REG_FIFOTHRESH) & 0x3F;

            y_lora_write(lora, REG_RXCONFIG, RF_RXCONFIG_AFCAUTO_ON | RF_RXCONFIG_AGCAUTO_ON | RF_RXCONFIG_RXTRIGER_PREAMBLEDETECT);

            lora->setting.FskPacketHandler.PreambleDetected = false;
            lora->setting.FskPacketHandler.SyncWordDetected = false;
            lora->setting.FskPacketHandler.NbBytes          = 0;
            lora->setting.FskPacketHandler.Size             = 0;
        } break;

        case MODEM_LORA: {
            if (lora->setting.LoRa.IqInverted == true) {
                y_lora_write(lora,
                             REG_LR_INVERTIQ,
                             ((y_lora_read(lora, REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF));
                y_lora_write(lora, REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON);
            } else {
                y_lora_write(lora,
                             REG_LR_INVERTIQ,
                             ((y_lora_read(lora, REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF));
                y_lora_write(lora, REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF);
            }

            // ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal
            if (lora->setting.LoRa.Bandwidth < 9) {
                y_lora_write(lora, REG_LR_DETECTOPTIMIZE, y_lora_read(lora, REG_LR_DETECTOPTIMIZE) & 0x7F);
                y_lora_write(lora, REG_LR_IFFREQ2, 0x00);
                switch (lora->setting.LoRa.Bandwidth) {
                    case 0:  // 7.8 kHz
                        y_lora_write(lora, REG_LR_IFFREQ1, 0x48);
                        y_lora_set_channel(lora, lora->setting.Channel + 7810);
                        break;
                    case 1:  // 10.4 kHz
                        y_lora_write(lora, REG_LR_IFFREQ1, 0x44);
                        y_lora_set_channel(lora, lora->setting.Channel + 10420);
                        break;
                    case 2:  // 15.6 kHz
                        y_lora_write(lora, REG_LR_IFFREQ1, 0x44);
                        y_lora_set_channel(lora, lora->setting.Channel + 15620);
                        break;
                    case 3:  // 20.8 kHz
                        y_lora_write(lora, REG_LR_IFFREQ1, 0x44);
                        y_lora_set_channel(lora, lora->setting.Channel + 20830);
                        break;
                    case 4:  // 31.2 kHz
                        y_lora_write(lora, REG_LR_IFFREQ1, 0x44);
                        y_lora_set_channel(lora, lora->setting.Channel + 31250);
                        break;
                    case 5:  // 41.4 kHz
                        y_lora_write(lora, REG_LR_IFFREQ1, 0x44);
                        y_lora_set_channel(lora, lora->setting.Channel + 41670);
                        break;
                    case 6:  // 62.5 kHz
                        y_lora_write(lora, REG_LR_IFFREQ1, 0x40);
                        break;
                    case 7:  // 125 kHz
                        y_lora_write(lora, REG_LR_IFFREQ1, 0x40);
                        break;
                    case 8:  // 250 kHz
                        y_lora_write(lora, REG_LR_IFFREQ1, 0x40);
                        break;
                }
            } else {
                y_lora_write(lora, REG_LR_DETECTOPTIMIZE, y_lora_read(lora, REG_LR_DETECTOPTIMIZE) | 0x80);
            }

            rxContinuous = lora->setting.LoRa.RxContinuous;

            if (lora->setting.LoRa.FreqHopOn == true) {
                y_lora_write(lora, REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_VALIDHEADER | RFLR_IRQFLAGS_TXDONE | RFLR_IRQFLAGS_CADDONE | RFLR_IRQFLAGS_CADDETECTED);

                // DIO0=RxDone, DIO2=FhssChangeChannel
                y_lora_write(lora,
                             REG_DIOMAPPING1,
                             (y_lora_read(lora, REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK) | RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO2_00);
            } else {
                y_lora_write(lora,
                             REG_LR_IRQFLAGSMASK,
                             RFLR_IRQFLAGS_VALIDHEADER | RFLR_IRQFLAGS_TXDONE | RFLR_IRQFLAGS_CADDONE | RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL | RFLR_IRQFLAGS_CADDETECTED);

                // DIO0=RxDone
                y_lora_write(lora, REG_DIOMAPPING1, (y_lora_read(lora, REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK) | RFLR_DIOMAPPING1_DIO0_00);
            }
            y_lora_write(lora, REG_LR_FIFORXBASEADDR, 0);
            y_lora_write(lora, REG_LR_FIFOADDRPTR, 0);
        } break;
    }

    memset(lora->buf, 0, (size_t) LORA_RX_BUF_SIZE);

    lora->setting.State = RF_RX_RUNNING;
    if (timeout != 0) {
        //osTimerStart(lora->rx_timer, timeout);
    }

    if (lora->setting.Modem == MODEM_FSK) {
        _y_lora_set_op_mode(lora, RF_OPMODE_RECEIVER);
        //osTimerStart(lora->rx_sync_timer, lora->setting.Fsk.RxSingleTimeout);
    } else {
        if (rxContinuous == true) {
            _y_lora_set_op_mode(lora, RFLR_OPMODE_RECEIVER);
        } else {
            _y_lora_set_op_mode(lora, RFLR_OPMODE_RECEIVER_SINGLE);
        }
    }
}

/// @brief   设置最大有效载荷长度
/// @param   [in] lora                     lora 句柄
/// @param   [in] modem                    要使用的无线电调制解调器
/// @param   [in] max                      最大有效负载长度（以字节为单位）
void y_lora_set_max_payload_length(LORA_st *lora, RadioModem_e modem, uint8_t max) {

    // 断言
    if (lora == NULL) {
        //YLOGE("lora is NULL");
        return;
    }

    y_lora_set_modem(lora, modem);
    switch (modem) {
        case MODEM_FSK:
            if (lora->setting.Fsk.FixLen == false) {
                y_lora_write(lora, REG_PAYLOADLENGTH, max);
            }
            break;
        case MODEM_LORA:
            y_lora_write(lora, REG_LR_PAYLOADMAXLENGTH, max);
            break;
    }
}

/// @brief   将网络设置为公共或私有,更新字节同步
/// @param   [in] lora                     lora 句柄
/// @param   [in] enable                   如果为真，则启用公共网络
void y_lora_set_public_network(LORA_st *lora, bool enable) {

    // 断言
    if (lora == NULL) {
        //YLOGE("lora is NULL");
        return;
    }

    y_lora_set_modem(lora, MODEM_LORA);
    lora->setting.LoRa.PublicNetwork = enable;
    if (enable == true) {
        y_lora_write(lora, REG_LR_SYNCWORD, LORA_MAC_PUBLIC_SYNCWORD);  // Change LoRa modem SyncWord
    } else {
        y_lora_write(lora, REG_LR_SYNCWORD, LORA_MAC_PRIVATE_SYNCWORD);  // Change LoRa modem SyncWord
    }
}

/// @brief    设置为睡眠模式
/// @param    [in] lora                    lora 句柄
void y_lora_set_sleep(LORA_st *lora) {

    // 断言
    if (lora == NULL) {
        //YLOGE("lora is NULL");
        return;
    }

    //osTimerStop(lora->rx_timer);
    //osTimerStop(lora->tx_timer);
    //osTimerStop(lora->rx_sync_timer);
    _y_lora_set_op_mode(lora, RF_OPMODE_SLEEP);
    lora->setting.State = RF_IDLE;
}

/// @brief    将无线电设置为待机模式
/// @param    [in] lora                    lora 句柄
void y_lora_set_stby(LORA_st *lora) {

    // 断言
    if (lora == NULL) {
        //YLOGE("lora is NULL");
        return;
    }

    //osTimerStop(lora->rx_timer);
    //osTimerStop(lora->tx_timer);
    //osTimerStop(lora->rx_sync_timer);
    _y_lora_set_op_mode(lora, RF_OPMODE_STANDBY);
    lora->setting.State = RF_IDLE;
}

/// @brief   DIO0中断回调
/// @param   [in] lora                     lora 句柄
void y_lora_io0_irq(LORA_st *lora) {

    // 断言
    if (lora == NULL) {
        //YLOGE("lora is NULL");
        return;
    }

    volatile uint8_t irqFlags = 0;
    switch (lora->setting.State) {

        case RF_RX_RUNNING:
            // //osTimerStop(lora->rx_timer);
            // RxDone interrupt
            switch (lora->setting.Modem) {
                case MODEM_FSK:
                    if (lora->setting.Fsk.CrcOn == true) {
                        irqFlags = y_lora_read(lora, REG_IRQFLAGS2);
                        if ((irqFlags & RF_IRQFLAGS2_CRCOK) != RF_IRQFLAGS2_CRCOK) {
                            // Clear Irqs
                            y_lora_write(lora, REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI | RF_IRQFLAGS1_PREAMBLEDETECT | RF_IRQFLAGS1_SYNCADDRESSMATCH);
                            y_lora_write(lora, REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);

                            //osTimerStop(lora->rx_timer);

                            if (lora->setting.Fsk.RxContinuous == false) {
                                //osTimerStop(lora->rx_sync_timer);
                                lora->setting.State = RF_IDLE;
                            } else {
                                // Continuous mode restart Rx chain
                                y_lora_write(lora, REG_RXCONFIG, y_lora_read(lora, REG_RXCONFIG) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK);
                                //osTimerStart(lora->rx_sync_timer, lora->setting.Fsk.RxSingleTimeout);
                            }

                            if (lora->event.RxError != NULL) {
                                lora->event.RxError(lora);
                            }
                            lora->setting.FskPacketHandler.PreambleDetected = false;
                            lora->setting.FskPacketHandler.SyncWordDetected = false;
                            lora->setting.FskPacketHandler.NbBytes          = 0;
                            lora->setting.FskPacketHandler.Size             = 0;
                            break;
                        }
                    }

                    // Read received packet size
                    if ((lora->setting.FskPacketHandler.Size == 0) && (lora->setting.FskPacketHandler.NbBytes == 0)) {
                        if (lora->setting.Fsk.FixLen == false) {
                            _y_lora_read_fifo(lora, (uint8_t *) &lora->setting.FskPacketHandler.Size, 1);
                        } else {
                            lora->setting.FskPacketHandler.Size = y_lora_read(lora, REG_PAYLOADLENGTH);
                        }
                        _y_lora_read_fifo(lora, lora->buf + lora->setting.FskPacketHandler.NbBytes, lora->setting.FskPacketHandler.Size - lora->setting.FskPacketHandler.NbBytes);
                        lora->setting.FskPacketHandler.NbBytes += (lora->setting.FskPacketHandler.Size - lora->setting.FskPacketHandler.NbBytes);
                    } else {
                        _y_lora_read_fifo(lora, lora->buf + lora->setting.FskPacketHandler.NbBytes, lora->setting.FskPacketHandler.Size - lora->setting.FskPacketHandler.NbBytes);
                        lora->setting.FskPacketHandler.NbBytes += (lora->setting.FskPacketHandler.Size - lora->setting.FskPacketHandler.NbBytes);
                    }
                    //osTimerStop(lora->rx_timer);

                    if (lora->setting.Fsk.RxContinuous == false) {
                        lora->setting.State = RF_IDLE;
                        //osTimerStop(lora->rx_sync_timer);
                    } else {
                        // Continuous mode restart Rx chain
                        y_lora_write(lora, REG_RXCONFIG, y_lora_read(lora, REG_RXCONFIG) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK);
                        //osTimerStart(lora->rx_sync_timer, lora->setting.Fsk.RxSingleTimeout);
                    }

                    if (lora->event.RxDone != NULL) {
                        lora->event.RxDone(lora, lora->buf, lora->setting.FskPacketHandler.Size, lora->setting.FskPacketHandler.RssiValue, 0);
                    }
                    lora->setting.FskPacketHandler.PreambleDetected = false;
                    lora->setting.FskPacketHandler.SyncWordDetected = false;
                    lora->setting.FskPacketHandler.NbBytes          = 0;
                    lora->setting.FskPacketHandler.Size             = 0;
                    break;

                case MODEM_LORA: {
                    // Clear Irq
                    y_lora_write(lora, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE);

                    irqFlags = y_lora_read(lora, REG_LR_IRQFLAGS);
                    if ((irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK) == RFLR_IRQFLAGS_PAYLOADCRCERROR) {
                        // Clear Irq
                        y_lora_write(lora, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR);

                        if (lora->setting.LoRa.RxContinuous == false) {
                            lora->setting.State = RF_IDLE;
                        }
                        //osTimerStop(lora->rx_timer);  // todo： 不管接收数据有没有错误都进入超时回调

                        if (lora->event.RxError != NULL) {
                            lora->event.RxError(lora);
                        }
                        break;
                    }

                    // Returns SNR value [dB] rounded to the nearest integer value
                    lora->setting.LoRaPacketHandler.SnrValue = (((int8_t) y_lora_read(lora, REG_LR_PKTSNRVALUE)) + 2) >> 2;

                    int16_t rssi                             = y_lora_read(lora, REG_LR_PKTRSSIVALUE);
                    if (lora->setting.LoRaPacketHandler.SnrValue < 0) {
                        if (lora->setting.Channel > LORA_RF_MID_THRESH) {
                            lora->setting.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + (rssi >> 4) + lora->setting.LoRaPacketHandler.SnrValue;
                        } else {
                            lora->setting.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + (rssi >> 4) + lora->setting.LoRaPacketHandler.SnrValue;
                        }
                    } else {
                        if (lora->setting.Channel > LORA_RF_MID_THRESH) {
                            lora->setting.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + (rssi >> 4);
                        } else {
                            lora->setting.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + (rssi >> 4);
                        }
                    }

                    lora->setting.LoRaPacketHandler.Size = y_lora_read(lora, REG_LR_RXNBBYTES);
                    y_lora_write(lora, REG_LR_FIFOADDRPTR, y_lora_read(lora, REG_LR_FIFORXCURRENTADDR));
                    _y_lora_read_fifo(lora, lora->buf, lora->setting.LoRaPacketHandler.Size);

                    if (lora->setting.LoRa.RxContinuous == false) {
                        lora->setting.State = RF_IDLE;
                    }
                    //osTimerStop(lora->rx_timer);

                    if (lora->event.RxDone != NULL) {
                        lora->event.RxDone(lora,
                                           lora->buf,
                                           lora->setting.LoRaPacketHandler.Size,
                                           lora->setting.LoRaPacketHandler.RssiValue,
                                           lora->setting.LoRaPacketHandler.SnrValue);
                    }
                } break;
                default:
                    break;
            }
            break;
        case RF_TX_RUNNING:
            //osTimerStop(lora->tx_timer);
            // TxDone interrupt
            switch (lora->setting.Modem) {
                case MODEM_LORA:
                    // Clear Irq
                    y_lora_write(lora, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE);
                    // Intentional fall through
                case MODEM_FSK:
                default:
                    lora->setting.State = RF_IDLE;
                    if (lora->event.TxDone != NULL) {
                        lora->event.TxDone(lora);
                    }
                    break;
            }
            break;
        default:
            break;
    }
}

/// @brief   DIO1中断回调
/// @param   [in] lora                     lora 句柄
void y_lora_io1_irq(LORA_st *lora) {

    // 断言
    if (lora == NULL) {
        //YLOGE("lora is NULL");
        return;
    }

    switch (lora->setting.State) {
        case RF_RX_RUNNING:
            switch (lora->setting.Modem) {
                case MODEM_FSK:
                    // Stop timer
                    //osTimerStop(lora->rx_sync_timer);

                    // FifoLevel interrupt
                    // Read received packet size
                    if ((lora->setting.FskPacketHandler.Size == 0) && (lora->setting.FskPacketHandler.NbBytes == 0)) {
                        if (lora->setting.Fsk.FixLen == false) {
                            _y_lora_read_fifo(lora, (uint8_t *) &lora->setting.FskPacketHandler.Size, 1);
                        } else {
                            lora->setting.FskPacketHandler.Size = y_lora_read(lora, REG_PAYLOADLENGTH);
                        }
                    }

                    // ERRATA 3.1 - PayloadReady Set for 31.25ns if FIFO is Empty
                    //
                    //              When FifoLevel interrupt is used to offload the
                    //              FIFO, the microcontroller should  monitor  both
                    //              PayloadReady  and FifoLevel interrupts, and
                    //              read only (FifoThreshold-1) bytes off the FIFO
                    //              when FifoLevel fires
                    if ((lora->setting.FskPacketHandler.Size - lora->setting.FskPacketHandler.NbBytes) >= lora->setting.FskPacketHandler.FifoThresh) {
                        _y_lora_read_fifo(lora, (lora->buf + lora->setting.FskPacketHandler.NbBytes), lora->setting.FskPacketHandler.FifoThresh - 1);
                        lora->setting.FskPacketHandler.NbBytes += lora->setting.FskPacketHandler.FifoThresh - 1;
                    } else {
                        _y_lora_read_fifo(lora, (lora->buf + lora->setting.FskPacketHandler.NbBytes), lora->setting.FskPacketHandler.Size - lora->setting.FskPacketHandler.NbBytes);
                        lora->setting.FskPacketHandler.NbBytes += (lora->setting.FskPacketHandler.Size - lora->setting.FskPacketHandler.NbBytes);
                    }
                    break;
                case MODEM_LORA:
                    // Sync time out
                    //osTimerStop(lora->rx_timer);
                    // Clear Irq
                    y_lora_write(lora, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXTIMEOUT);
                    lora->setting.State = RF_IDLE;
                    if (lora->event.RxTimeout != NULL) {
                        lora->event.RxTimeout(lora);
                    }
                    break;
                default:
                    break;
            }
            break;
        case RF_TX_RUNNING:
            switch (lora->setting.Modem) {
                case MODEM_FSK:
                    // FifoEmpty interrupt
                    if ((lora->setting.FskPacketHandler.Size - lora->setting.FskPacketHandler.NbBytes) > lora->setting.FskPacketHandler.ChunkSize) {
                        _y_lora_write_fifo(lora, (lora->buf + lora->setting.FskPacketHandler.NbBytes), lora->setting.FskPacketHandler.ChunkSize);
                        lora->setting.FskPacketHandler.NbBytes += lora->setting.FskPacketHandler.ChunkSize;
                    } else {
                        // Write the last chunk of data
                        _y_lora_write_fifo(lora, lora->buf + lora->setting.FskPacketHandler.NbBytes, lora->setting.FskPacketHandler.Size - lora->setting.FskPacketHandler.NbBytes);
                        lora->setting.FskPacketHandler.NbBytes += lora->setting.FskPacketHandler.Size - lora->setting.FskPacketHandler.NbBytes;
                    }
                    break;
                case MODEM_LORA:
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}

/// @brief   DIO2中断回调
/// @param   [in] lora                     lora 句柄
void y_lora_io2_irq(LORA_st *lora) {

    // 断言
    if (lora == NULL) {
        //YLOGE("lora is NULL");
        return;
    }

    switch (lora->setting.State) {
        case RF_RX_RUNNING:
            switch (lora->setting.Modem) {
                case MODEM_FSK:
                    // Checks if DIO4 is connected. If it is not PreambleDetected is set to true.
                    lora->setting.FskPacketHandler.PreambleDetected = true;

                    if ((lora->setting.FskPacketHandler.PreambleDetected == true) && (lora->setting.FskPacketHandler.SyncWordDetected == false)) {
                        //osTimerStop(lora->rx_sync_timer);

                        lora->setting.FskPacketHandler.SyncWordDetected = true;
                        lora->setting.FskPacketHandler.RssiValue        = (int8_t) - (y_lora_read(lora, REG_RSSIVALUE) >> 1);
                        lora->setting.FskPacketHandler.AfcValue =
                                (int32_t) (double) (((uint16_t) y_lora_read(lora, REG_AFCMSB) << 8) | (uint16_t) y_lora_read(lora, REG_AFCLSB)) * (double) LORA_FREQ_STEP;
                        lora->setting.FskPacketHandler.RxGain = (y_lora_read(lora, REG_LNA) >> 5) & 0x07;
                    }
                    break;
                case MODEM_LORA:
                    if (lora->setting.LoRa.FreqHopOn == true) {
                        // Clear Irq
                        y_lora_write(lora, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL);

                        if (lora->event.FhssChangeChannel != NULL) {
                            lora->event.FhssChangeChannel(lora, (y_lora_read(lora, REG_LR_HOPCHANNEL) & RFLR_HOPCHANNEL_CHANNEL_MASK));
                        }
                    }
                    break;
                default:
                    break;
            }
            break;
        case RF_TX_RUNNING:
            switch (lora->setting.Modem) {
                case MODEM_FSK:
                    break;
                case MODEM_LORA:
                    if (lora->setting.LoRa.FreqHopOn == true) {
                        // Clear Irq
                        y_lora_write(lora, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL);

                        if (lora->event.FhssChangeChannel != NULL) {
                            lora->event.FhssChangeChannel(lora, (y_lora_read(lora, REG_LR_HOPCHANNEL) & RFLR_HOPCHANNEL_CHANNEL_MASK));
                        }
                    }
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}

/// @brief   DIO3中断回调
/// @param   [in] lora                     lora 句柄
void y_lora_io3_irq(LORA_st *lora) {

    // 断言
    if (lora == NULL) {
        //YLOGE("lora is NULL");
        return;
    }

    switch (lora->setting.Modem) {
        case MODEM_FSK:
            break;
        case MODEM_LORA:
            if ((y_lora_read(lora, REG_LR_IRQFLAGS) & RFLR_IRQFLAGS_CADDETECTED) == RFLR_IRQFLAGS_CADDETECTED) {
                // Clear Irq
                y_lora_write(lora, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED | RFLR_IRQFLAGS_CADDONE);
                if (lora->event.CadDone != NULL) {
                    lora->event.CadDone(lora, true);
                }
            } else {
                // Clear Irq
                y_lora_write(lora, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDONE);
                if (lora->event.CadDone != NULL) {
                    lora->event.CadDone(lora, false);
                }
            }
            break;
        default:
            break;
    }
}

/// @brief   DIO4中断回调
/// @param   [in] lora                     lora 句柄
void y_lora_io4_irq(LORA_st *lora) {

    // 断言
    if (lora == NULL) {
        //YLOGE("lora is NULL");
        return;
    }

    switch (lora->setting.Modem) {
        case MODEM_FSK: {
            if (lora->setting.FskPacketHandler.PreambleDetected == false) {
                lora->setting.FskPacketHandler.PreambleDetected = true;
            }
        } break;
        case MODEM_LORA:
            break;
        default:
            break;
    }
}

/// @brief   DIO5中断回调
/// @param   [in] lora                     lora 句柄
void y_lora_io5_irq(LORA_st *lora) {

    // 断言
    if (lora == NULL) {
        //YLOGE("lora is NULL");
        return;
    }

    switch (lora->setting.Modem) {
        case MODEM_FSK:
            break;
        case MODEM_LORA:
            break;
        default:
            break;
    }
}



/// ------------------------------------------------------------------------------------------------------------------------------------
/// 底层接口实现          note: 需根据实际修改( 若有多个 lora ,可按以下函数类型增加 )
/// ------------------------------------------------------------------------------------------------------------------------------------

/// @brief   lora spi 发送/接收数据
/// @param   [in] outData                  要发送的数据
/// @return  接收到的数据
uint8_t y_lora_1_spi_transmit(uint8_t outData) {

    uint8_t RxData = 0;

    // while (!LL_SPI_IsActiveFlag_TXE(SPI1)) {};
    // LL_SPI_TransmitData8(SPI1, outData);
    // while (!LL_SPI_IsActiveFlag_TXE(SPI1)) {};
    // while (!LL_SPI_IsActiveFlag_RXNE(SPI1)) {};
    // RxData = LL_SPI_ReceiveData8(SPI1);

    if (HAL_SPI_TransmitReceive(&hspi1, &outData, &RxData, 1, 60) != HAL_OK) {
        return 0;
    }

    return RxData;
}

/// @brief   lora spi 开关
/// @param   [in] on_off                   开/关
void y_lora_1_spi_nss(bool on_off) {

    if (on_off == true) {
        HAL_GPIO_WritePin(LORA1_SPI_NSS_GPIO_Port, LORA1_SPI_NSS_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(LORA1_SPI_NSS_GPIO_Port, LORA1_SPI_NSS_Pin, GPIO_PIN_RESET);
    }
}

/// @brief   lora 重启
void y_lora_1_reset() {

    // 将 RESET 引脚设置为 0
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin   = LORA1_RST_Pin;
    GPIO_InitStructure.Pull  = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_WritePin(LORA1_RST_GPIO_Port, LORA1_RST_Pin, GPIO_PIN_RESET);
    HAL_GPIO_Init(LORA1_RST_GPIO_Port, &GPIO_InitStructure);

    HAL_Delay(1);  // Wait 1 ms

    // 配置 RESET 为输入
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(LORA1_RST_GPIO_Port, &GPIO_InitStructure);

    HAL_Delay(6);  // Wait 6 ms
}

/// @brief   lora spi 发送/接收数据
/// @param   [in] outData                 要发送的数据
/// @return  接收到的数据
uint8_t y_lora_2_spi_transmit(uint8_t outData) {

    uint8_t RxData = 0;

    // while (!LL_SPI_IsActiveFlag_TXE(SPI1)) {};
    // LL_SPI_TransmitData8(SPI1, outData);
    // while (!LL_SPI_IsActiveFlag_TXE(SPI1)) {};
    // while (!LL_SPI_IsActiveFlag_RXNE(SPI1)) {};
    // RxData = LL_SPI_ReceiveData8(SPI1);

    if (HAL_SPI_TransmitReceive(&hspi2, &outData, &RxData, 1, 100) != HAL_OK) {
        return 0;
    }

    return RxData;
}

/// @brief   lora spi 开关
/// @param   [in] on_off                   开/关
void y_lora_2_spi_nss(bool on_off) {

    if (on_off == true) {
        HAL_GPIO_WritePin(LORA2_SPI_NSS_GPIO_Port, LORA2_SPI_NSS_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(LORA2_SPI_NSS_GPIO_Port, LORA2_SPI_NSS_Pin, GPIO_PIN_RESET);
    }
}

/// @brief   lora 重启
void y_lora_2_reset() {

    // 将 RESET 引脚设置为 0
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin   = LORA2_RST_Pin;
    GPIO_InitStructure.Pull  = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_WritePin(LORA2_RST_GPIO_Port, LORA2_RST_Pin, GPIO_PIN_RESET);
    

    HAL_Delay(1);  // Wait 1 ms

    // 配置 RESET 为输入
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(LORA2_RST_GPIO_Port, &GPIO_InitStructure);

    HAL_Delay(6);  // Wait 6 ms
}

// /**
//  * @brief 
//  * 
//  * @param fd 
//  * @param tx_buff 
//  * @param expect_reply 
//  * @param timeout 
//  */
// void LORA_Send(char *tx_buff)
// {
//     uint8_t *tx_data,len=0;
//     len = strlen(tx_buff);

//     tx_data = (char *)malloc((len)*sizeof(uint8_t));
//     memcpy(tx_data,tx_buff,len);

//     y_lora_send(lora_dev,tx_data,len);
   
//     free(tx_data);
//     tx_data = NULL;
//     y_lora_set_rx_time(lora_dev, 0);
// }

// SHELL_EXPORT_CMD(
// SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_DISABLE_RETURN,
// lora_send, LORA_Send, ec2x test);