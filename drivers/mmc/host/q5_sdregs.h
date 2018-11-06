/*
 * (C) Copyright 2011
 * Ricado Ribalda - ricardo.ribalda@gmail.com
 * QTechnology  http://qtec.com/
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef Q5_SD_REGS
#define Q5_SD_REGS

#include "q5_defines.h"

#define QSD_CMD_DATA 0x0

#define QSD_CMD_IF_CONTROL 0x4
#define CLOCK_DIVIDE_MASK MASK(10)
#define CLOCK_DIVIDE_SHIFT 0
#define POWER_ON_MASK MASK(1)
#define POWER_ON_SHIFT 31

#define QSD_CMD_CONTROL 0x8
#define CMD_ENABLE_MASK MASK(1)
#define CMD_ENABLE_SHIFT 0
#define SKIP_CRC_MASK MASK(1)
#define SKIP_CRC_SHIFT 1
#define WAIT_READY_MASK MASK(1)
#define WAIT_READY_SHIFT 2
#define CMD_IRQ_ENA_MASK MASK(1)
#define CMD_IRQ_ENA_SHIFT 3
#define RECV_LEN_MASK MASK(8)
#define RECV_LEN_SHIFT 16

#define QSD_STATUS 0xc
#define CMD_IDLE_MASK MASK(1)
#define CMD_IDLE_SHIFT 0
#define CMD_TX_IDLE_MASK MASK(1)
#define CMD_TX_IDLE_SHIFT 1
#define CMD_RX_IDLE_MASK MASK(1)
#define CMD_RX_IDLE_SHIFT 2
#define CMD_RX_CRC_MASK MASK(1)
#define CMD_RX_CRC_SHIFT 3
#define CMD_RX_EMPTY_MASK MASK(1)
#define CMD_RX_EMPTY_SHIFT 4
#define CMD_TX_FULL_MASK MASK(1)
#define CMD_TX_FULL_SHIFT 5
#define DATA_TX_IDLE_MASK MASK(1)
#define DATA_TX_IDLE_SHIFT 6
#define DATA_RX_IDLE_MASK MASK(1)
#define DATA_RX_IDLE_SHIFT 7
#define DATA_RX_EMPTY_MASK MASK(1)
#define DATA_RX_EMPTY_SHIFT 8
#define DATA_TX_FULL_MASK MASK(1)
#define DATA_TX_FULL_SHIFT 9
#define CARD_DETECTED_MASK MASK(1)
#define CARD_DETECTED_SHIFT 10
#define CARD_PROTECTED_MASK MASK(1)
#define CARD_PROTECTED_SHIFT 11
#define DATA_TX_STATUS_MASK MASK(3)
#define DATA_TX_STATUS_SHIFT 16
#define CMD_IRQ_STATUS_MASK MASK(1)
#define CMD_IRQ_STATUS_SHIFT 19
#define DATA_RX_STATUS_MASK MASK(4)
#define DATA_RX_STATUS_SHIFT 20
#define RX_FIFO_LEN_MASK MASK(4)
#define RX_FIFO_LEN_SHIFT 24
#define TX_FIFO_LEN_MASK MASK(4)
#define TX_FIFO_LEN_SHIFT 28

#define DATA_TX_OK 0x2

#define QSD_DATA_DATA 0x10

#define QSD_DATA_CONTROL 0x14
#define DATA_TX_ENABLE_MASK MASK(1)
#define DATA_TX_ENABLE_SHIFT 0
#define DATA_RX_ENABLE_MASK MASK(1)
#define DATA_RX_ENABLE_SHIFT 1
#define DATA_RX_RST_MASK MASK(1)
#define DATA_RX_RST_SHIFT 2
#define MODE_4BIT_MASK MASK(1)
#define MODE_4BIT_SHIFT 3
#define DATA_RECV_LEN_MASK MASK(16)
#define DATA_RECV_LEN_SHIFT 16

#define QSD_DMA_ADDR 0x18

#define QSD_DMA_LENGTH 0x1c
#define DMA_LENGTH_MASK MASK(16)

#define QSD_DMA_CONTROL 0x20
#define DMA_ENABLE_MASK MASK(1)
#define DMA_ENABLE_SHIFT 0
#define DMA_DIR_MASK MASK(1)
#define DMA_DIR_SHIFT 1
#define DMA_IRQ_ENA_MASK MASK(1)
#define DMA_IRQ_ENA_SHIFT 2

#define QSD_DMA_STATUS 0x24
#define DMA_REMAIN_MASK MASK(16)
#define DMA_REMAIN_SHIFT 0
#define DMA_IRQ_STATUS_MASK MASK(1)
#define DMA_IRQ_STATUS_SHIFT 16
#define DMA_IDLE_MASK MASK(1)
#define DMA_IDLE_SHIFT 17
#define DMA_TX_ERR_MASK MASK(1)
#define DMA_TX_ERR_SHIFT 18
#define DMA_RX_ERR_MASK MASK(1)
#define DMA_RX_ERR_SHIFT 19
#define DMA_BUS_ERR_MASK MASK(1)
#define DMA_BUS_ERR_SHIFT 20


#endif
