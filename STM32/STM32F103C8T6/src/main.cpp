// ============================================================
//  文件: main.cpp
//  File: main.cpp
//  描述: STM32F103C8T6 转接板固件 — FTDX10 电台数据采集
//  Desc: adapter firmware — FTDX10 radio data acquisition
//  电台 ACC Pinout / Radio ACC connector:
//    Pin4  (MOSI)  → 数据 Data   Pin5  (SCLK) → 时钟 Clock
//    Pin6  (CS)    → 片选 Chip Select
//  协议: SPI 模式1 2MHz 从机 → USB CDC
//  Protocol: SPI mode1 slave 2MHz → USB CDC
//  输出: USB CDC (虚拟串口) via SerialUSB
//  Output: USB CDC (virtual serial) via SerialUSB
//  平台: PlatformIO + STM32duino (Arduino_Core_STM32)
//  Platform: PlatformIO + STM32duino
// ============================================================

#include <Arduino.h>
#include <stdint.h>
#include <USBSerial.h>

// ============================================================
//  引脚定义
//  Pin Definitions
// ============================================================
#define PIN_SPI_CS          PA4
#define PIN_SPI_SCK         PA5
#define PIN_SPI_MOSI        PA7
#define PIN_LED             PC13
#define LED_ON              LOW
#define LED_OFF             HIGH

// ============================================================
//  协议常量
//  Protocol Constants
// ============================================================
#define PACKET_SIZE          4096

// ============================================================
//  全局变量
//  Global Variables
// ============================================================
static uint8_t spi_buffer[PACKET_SIZE];        // SPI 单缓冲 / single SPI buffer
static volatile uint16_t spi_rx_index = 0;     // 当前接收位置 / current RX index
static volatile bool     packet_ready = false;  // 包就绪标志 / frame complete flag
static volatile bool     spi_active   = false;  // SPI 正在接收 / SPI actively receiving

static uint32_t packet_count = 0;
static uint32_t total_bytes  = 0;

// ============================================================
//  SPI1 中断 — RXNE (每收到一个字节)
//  SPI1 ISR — RXNE (one byte per interrupt)
// ============================================================
extern "C" void SPI1_IRQHandler(void) {
  if (SPI1->SR & SPI_SR_RXNE) {
    uint8_t byte = *(volatile uint8_t *)&SPI1->DR;
    if (spi_rx_index < PACKET_SIZE) {
      spi_buffer[spi_rx_index++] = byte;
    }
  }

  // 清除溢出 / clear overrun flag
  if (SPI1->SR & SPI_SR_OVR) {
    (void)SPI1->DR;
    (void)SPI1->SR;
  }
}

// ============================================================
//  CS 引脚中断 — 使用 Arduino attachInterrupt
//  CS Pin ISR — via Arduino attachInterrupt
// ============================================================
static void cs_interrupt_handler(void) {
  bool cs_low = !digitalRead(PIN_SPI_CS);

  if (cs_low) {
    // CS ↓ — SPI 开始发送数据 / radio starts a frame
    spi_rx_index = 0;
    spi_active   = true;

    // 清空残留字节 & 溢出标志 / flush stale bytes & clear overrun
    while (SPI1->SR & SPI_SR_RXNE) (void)SPI1->DR;
    if (SPI1->SR & SPI_SR_OVR) { (void)SPI1->DR; (void)SPI1->SR; }

    // 开 RXNE 中断 / enable RXNE interrupt
    SPI1->CR2 |= SPI_CR2_RXNEIE;

  } else if (spi_active) {
    // CS ↑ — 数据接收完毕 / frame complete
    spi_active = false;
    SPI1->CR2 &= ~SPI_CR2_RXNEIE;   // 关中断 / disable RXNE interrupt
    packet_ready = (spi_rx_index > 0);
  }
}

// ============================================================
//  SPI 从机初始化
//  SPI Slave Initialization
// ============================================================
static void spi_slave_init(void) {
  // ---- 使能时钟 / Enable clocks ----
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;

  // ---- PA4(CS) - 浮空输入 / floating input ----
  GPIOA->CRL &= ~(0xF << (4 * 4));
  GPIOA->CRL |=  (0x4 << (4 * 4));

  // ---- PA5(SCK) - 浮空输入 / floating input ----
  GPIOA->CRL &= ~(0xF << (5 * 4));
  GPIOA->CRL |=  (0x4 << (5 * 4));

  // ---- PA7(MOSI) - 浮空输入 / floating input ----
  GPIOA->CRL &= ~(0xF << (7 * 4));
  GPIOA->CRL |=  (0x4 << (7 * 4));

  // ---- SPI1: 从机, 模式1, 硬件 NSS / slave, mode1, hardware NSS ----
  SPI1->CR1 = 0;
  SPI1->CR1 = SPI_CR1_CPHA;           // 模式1, MSTR=0(从机), SSM=0(硬件NSS) / mode1, slave, hw NSS
  SPI1->CR1 &= ~SPI_CR1_LSBFIRST;     // MSB 先行 / MSB first
  SPI1->CR2 = 0;                      // RXNEIE 由 CS 中断控制 / managed by CS ISR
  SPI1->CR1 |= SPI_CR1_SPE;           // 使能 SPI / enable SPI

  // ---- NVIC ----
  //  USB LP 默认优先级 0 > EXTI4(1), 会延迟 CS 中断导致丢前几个字节
  //  USB LP default priority 0 > EXTI4(1), delays CS ISR causing byte loss
  NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 3);
  NVIC_SetPriority(SPI1_IRQn, 2);
  NVIC_EnableIRQ(SPI1_IRQn);
  NVIC_SetPriority(EXTI4_IRQn, 1);

  // ---- EXTI ----
  pinMode(PIN_SPI_CS, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_SPI_CS), cs_interrupt_handler, CHANGE);

  SerialUSB.printf("[SPI] 从机就绪 | 模式1 | PA4(CS) attachInterrupt | 中断逐字节接收");
  SerialUSB.println();
}

// ============================================================
//  帧封装 (USB CDC)
//  Frame Encapsulation (USB CDC)
// ============================================================
static void send_frame(const uint8_t *data, size_t len) {
  uint8_t xor_sum = 0;
  for (size_t i = 0; i < len; i++) xor_sum ^= data[i];

  uint8_t header[] = {0x66, 0xCC, 0xFF};
  SerialUSB.write(header, 3);
  SerialUSB.write((len >> 8) & 0xFF);
  SerialUSB.write(len & 0xFF);
  SerialUSB.write(xor_sum);

  // 分块发送, 避免长时间阻塞 / send in chunks to avoid long blocking
  size_t offset = 0;
  while (offset < len) {
    size_t chunk = len - offset;
    if (chunk > 64) chunk = 64;
    SerialUSB.write(data + offset, chunk);
    offset += chunk;
  }
}

// ============================================================
//  USB 串口命令
//  USB Serial Commands
// ============================================================
#define CMD_BUF_SIZE  160
static char cmd_buf[CMD_BUF_SIZE];
static uint8_t cmd_idx = 0;

static void handle_usb_commands(void) {
  while (SerialUSB.available() > 0) {
    char c = SerialUSB.read();
    if (c == '\n' || c == '\r') {
      if (cmd_idx > 0) {
        cmd_buf[cmd_idx] = '\0';
        if (strcmp(cmd_buf, "HELP") == 0) {
          SerialUSB.println("[CMD] 可用命令:");
          SerialUSB.println("  HELP  — 显示帮助 / show help");
        } else if (cmd_buf[0] != '\0') {
          SerialUSB.printf("[CMD] 未知: %s\n", cmd_buf);
        }
        cmd_idx = 0;
      }
    } else if (cmd_idx < CMD_BUF_SIZE - 1) {
      cmd_buf[cmd_idx++] = c;
    }
  }
}

// ============================================================
//  初始化
//  Setup / Initialization
// ============================================================
void setup(void) {
  SerialUSB.begin();
  delay(500);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LED_OFF);

  SerialUSB.println();
  SerialUSB.println("╔═══════════════════════════════════╗");
  SerialUSB.println("║  FTDX10 转接板  —  STM32F103C8   ║");
  SerialUSB.println("╚═══════════════════════════════════╝");
  SerialUSB.println("SPI:  PA4(CS)←Pin6 | PA5(SCK)←Pin5 | PA7(MOSI)←Pin4 | 模式1");
  SerialUSB.println("输出: USB CDC (虚拟串口) / Output: USB CDC");
  SerialUSB.println();

  spi_slave_init();

  SerialUSB.println("[OK] 初始化完成, 等待电台 SPI 数据...\n");
  SerialUSB.println("[OK] Init done, waiting for radio SPI data...\n");
}

// ============================================================
//  主循环
//  Main Loop
// ============================================================
void loop(void) {
  handle_usb_commands();

  if (packet_ready) {
    packet_ready = false;

    size_t rx_len = spi_rx_index;
    if (rx_len > 0) {
      packet_count++;
      total_bytes += rx_len;

      send_frame(spi_buffer, rx_len);

      if (rx_len == PACKET_SIZE) {
        static unsigned long last_summary = 0;
        unsigned long now = millis();
        if (now - last_summary > 5000) {
          SerialUSB.printf("[SPI] 包 #%u | %u B | 总: %u KB\n",
                        packet_count, rx_len, total_bytes / 1024);
          last_summary = now;
        }
      }
    }
  }
}
