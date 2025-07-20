#include "./dev_usart.h"

// #include <stm32f4xx_hal_dma.h>
#include "../Comm/protocol_header.h"
#include "../Services/service_message.h"
#include "../Utils/crc.h"
#include "cmsis_os.h"
#include "string.h"
#include "usart.h"

#define TX_BUFFER_SIZE 8192
#define RX_BUFFER_SIZE 512

extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern osSemaphoreId msgReadyBinarySemHandle;
extern osSemaphoreId transportBinarySemHandle;

static uint8_t tx_buffer[TX_BUFFER_SIZE];
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static volatile uint16_t rx_size = 0;
static volatile uint16_t tx_size = 0;

static volatile uint32_t g_tx_overflow_count = 0;

static volatile uint16_t write_index = 0;
static volatile uint16_t read_index = 0;
static volatile uint8_t transmit_busy = 0;
static uint16_t last_sent_len = 0;

// 获取缓冲区中可用数据长度
static uint16_t get_available_data(void) {
  if (write_index >= read_index) {
    return write_index - read_index;
  } else {
    return TX_BUFFER_SIZE - read_index + write_index;
  }
}

// 获取缓冲区剩余空间
static uint16_t get_free_space(void) {
  return TX_BUFFER_SIZE - get_available_data() - 1;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart->Instance == USART3) {
    if (last_sent_len > 0) {
      read_index = (read_index + last_sent_len) % TX_BUFFER_SIZE;
      last_sent_len = 0;
    }
    transmit_busy = 0;
  }
}

int dev_usart_read() {
  if (HAL_OK !=
      HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buffer, RX_BUFFER_SIZE))
    return -1;
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
  return 0;
}

int dev_usart_init(void) { return dev_usart_read(); }

static int _write(const uint8_t* buf, uint16_t size) {
  if (transmit_busy || size > TX_BUFFER_SIZE) {
    return -1;
  }
  transmit_busy = 1;
  HAL_StatusTypeDef sta = HAL_UART_Transmit_DMA(&huart3, buf, size);

  if (sta != HAL_OK) {
    transmit_busy = 0;
    return -1;
  }
  return 0;
}

void dev_usart_periodic(void) {
  portENTER_CRITICAL();
  if (!transmit_busy && get_available_data() > 0) {
    uint16_t send_len;

    if (write_index > read_index) {
      send_len = write_index - read_index;
    } else {
      send_len = TX_BUFFER_SIZE - read_index;
    }

    if (send_len > 0) {
      last_sent_len = send_len;
      int result = _write(&tx_buffer[read_index], send_len);
      if (result != 0) {
        transmit_busy = 0;
        last_sent_len = 0;
      }
    }
  }
  portEXIT_CRITICAL();
}

int dev_usart_write_queue(const uint8_t* buf, uint16_t len) {
  if (buf == NULL || len == 0) return -1;

  // 如果请求写入的长度大于可用空间，则认为是溢出
  if (len > get_free_space()) {
    g_tx_overflow_count++;  // 累加溢出计数器
    return -1;              // 拒绝写入
  }

  // 创建临界区
  portENTER_CRITICAL();

  uint16_t space_to_end = TX_BUFFER_SIZE - write_index;

  if (len <= space_to_end) {
    memcpy(&tx_buffer[write_index], buf, len);
    write_index = (write_index + len) % TX_BUFFER_SIZE;
  } else {
    memcpy(&tx_buffer[write_index], buf, space_to_end);
    memcpy(&tx_buffer[0], buf + space_to_end, len - space_to_end);
    write_index = len - space_to_end;
  }

  portEXIT_CRITICAL();

  osSemaphoreRelease(transportBinarySemHandle);
  return 0;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size) {
  if (huart->Instance == USART3) {
    rx_size = size;
    osSemaphoreRelease(msgReadyBinarySemHandle);
  }
  dev_usart_read();
}

/// 下面错误处理函数中把奇偶校验错误以及数据溢出错误标志清除
/// 可以直接在调试时打断点查看huart->ErrorCode的值
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
  if (huart->Instance == USART3) {
    uint32_t error_code = huart->ErrorCode;

    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);

    if ((error_code & HAL_UART_ERROR_DMA) != 0) {
      HAL_UART_AbortTransmit_IT(huart);
      transmit_busy = 0;
      last_sent_len = 0;
    }

    if (huart->RxState != HAL_UART_STATE_READY) {
      huart->RxState = HAL_UART_STATE_READY;
    }

    dev_usart_read();
  }
}

void dev_usart_get_rx_buffer(uint8_t* buf, uint16_t* size) {
  if (buf == NULL || size == NULL) return;
  if (rx_size == 0) return;
  *size = rx_size;
  memcpy(buf, rx_buffer, rx_size);
  rx_size = 0;
}

uint32_t dev_usart_get_tx_overflow_count(void) { return g_tx_overflow_count; }