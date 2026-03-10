# STM32 Quick Reference

## Инициализация периферии

### GPIO

```cpp
// Включение тактирования
RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

// Выход push-pull
GPIOA->MODER &= ~(3 << (pin * 2));     // Очистка
GPIOA->MODER |= (1 << (pin * 2));      // Output mode
GPIOA->OTYPER &= ~(1 << pin);          // Push-pull
GPIOA->OSPEEDR |= (3 << (pin * 2));    // High speed

// Вход с pull-up
GPIOA->MODER &= ~(3 << (pin * 2));     // Input mode
GPIOA->PUPDR |= (1 << (pin * 2));      // Pull-up

// Альтернативная функция
GPIOA->MODER |= (2 << (pin * 2));      // Alternate function
GPIOA->AFR[pin/8] |= (af << ((pin%8)*4));

// Чтение/запись
if (GPIOA->IDR & (1 << pin)) { ... }   // Чтение
GPIOA->BSRR = (1 << pin);              // Set
GPIOA->BSRR = (1 << (pin + 16));       // Reset
GPIOA->ODR ^= (1 << pin);              // Toggle
```

### UART

```cpp
// Включение тактирования
RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

// Настройка скорости (BRR = fPCLK / baud)
USART1->BRR = SystemCoreClock / 115200;

// Конфигурация: 8N1
USART1->CR1 = USART_CR1_TE | USART_CR1_RE;

// Включение
USART1->CR1 |= USART_CR1_UE;

// Передача
while (!(USART1->SR & USART_SR_TXE));
USART1->DR = data;

// Приём
while (!(USART1->SR & USART_SR_RXNE));
data = USART1->DR;
```

### SPI

```cpp
// Включение тактирования
RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

// Настройка: Master, Mode 0, fPCLK/4
SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_0;

// Включение
SPI1->CR1 |= SPI_CR1_SPE;

// Обмен
SPI1->DR = tx_data;
while (!(SPI1->SR & SPI_SR_RXNE));
rx_data = SPI1->DR;
```

### I2C

```cpp
// Включение тактирования
RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

// Сброс
I2C1->CR1 = I2C_CR1_SWRST;
I2C1->CR1 = 0;

// Настройка скорости (100 kHz)
I2C1->CR2 = 16;                    // Freq = 16 MHz
I2C1->CCR = 80;                    // T_high + T_low
I2C1->TRISE = 17;                  // Max rise time

// Включение
I2C1->CR1 |= I2C_CR1_PE;

// Запись
I2C1->CR1 |= I2C_CR1_START;        // START
while (!(I2C1->SR1 & I2C_SR1_SB));
I2C1->DR = (addr << 1);            // Address + W
while (!(I2C1->SR1 & I2C_SR1_ADDR));
(void)I2C1->SR2;                   // Clear ADDR
I2C1->DR = data;                   // Data
while (!(I2C1->SR1 & I2C_SR1_TXE));
I2C1->CR1 |= I2C_CR1_STOP;         // STOP
```

### CAN

```cpp
// Включение тактирования
RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

// Выход из sleep mode
CAN1->MCR &= ~CAN_MCR_SLEEP;
while (CAN1->MSR & CAN_MSR_SLAK);

// Инициализация
CAN1->MCR |= CAN_MCR_INRQ;
while (!(CAN1->MSR & CAN_MSR_INAK));

// Настройка скорости (500 kbps при 36 MHz)
// Prescaler = 4, BS1 = 14, BS2 = 3
CAN1->BTR = (3 << 20) | (14 << 16) | 3;

// Фильтр (приём всех)
CAN1->FMR |= CAN_FMR_FINIT;
CAN1->FA1R = 0;
CAN1->FM1R = 0;
CAN1->FS1R = 1;
CAN1->sFilterRegister[0].FR1 = 0;
CAN1->sFilterRegister[0].FR2 = 0;
CAN1->FA1R = 1;
CAN1->FMR &= ~CAN_FMR_FINIT;

// Запуск
CAN1->MCR &= ~CAN_MCR_INRQ;
while (CAN1->MSR & CAN_MSR_INAK);

// Передача
CAN1->sTxMailBox[0].TIR = (id << 21);  // STD ID
CAN1->sTxMailBox[0].TDTR = dlc;
CAN1->sTxMailBox[0].TDLR = data_low;
CAN1->sTxMailBox[0].TDHR = data_high;
CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;

// Проверка приёма
if (CAN1->RF0R & CAN_RF0R_FMP0) {
    id = CAN1->sFIFOMailBox[0].RIR >> 21;
    dlc = CAN1->sFIFOMailBox[0].RDTR & 0xF;
    data = CAN1->sFIFOMailBox[0].RDLR;
    CAN1->RF0R |= CAN_RF0R_RFOM0;  // Release
}
```

---

## Прерывания

### NVIC

```cpp
// Приоритет (меньше = выше)
NVIC_SetPriority(USART1_IRQn, 1);

// Включение
NVIC_EnableIRQ(USART1_IRQn);

// Выключение
NVIC_DisableIRQ(USART1_IRQn);
```

### UART прерывание

```cpp
// Включение прерывания RX
USART1->CR1 |= USART_CR1_RXNEIE;

// Обработчик
void USART1_IRQHandler(void) {
    if (USART1->SR & USART_SR_RXNE) {
        uint8_t data = USART1->DR;
        // Обработка...
    }
    if (USART1->SR & USART_SR_ORE) {
        USART1->SR &= ~USART_SR_ORE;  // Сброс ошибки
    }
}
```

---

## DMA

### Конфигурация

```cpp
// Включение тактирования
RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

// Настройка потока
DMA2_Stream7->CR = 0;                    // Остановить
while (DMA2_Stream7->CR & DMA_SxCR_EN);  // Ждать

DMA2_Stream7->PAR = (uint32_t)&USART1->DR;  // Periph address
DMA2_Stream7->M0AR = (uint32_t)buffer;      // Memory address
DMA2_Stream7->NDTR = length;                // Number of data

DMA2_Stream7->CR = DMA_SxCR_CHSEL_2 |    // Channel 4
                   DMA_SxCR_MINC |        // Memory increment
                   DMA_SxCR_DIR_0 |       // Memory-to-periph
                   DMA_SxCR_TCIE;         // Transfer complete IRQ

DMA2_Stream7->FCR = DMA_SxFCR_DMDIS |    // Direct mode disable
                    DMA_SxFCR_FTH_0;      // Full threshold

DMA2_Stream7->CR |= DMA_SxCR_EN;         // Запуск
```

---

## Таймеры

### PWM

```cpp
// Включение тактирования
RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

// Базовая настройка (20 kHz)
TIM2->PSC = 83;          // Prescaler: 84MHz / 84 = 1MHz
TIM2->ARR = 49;          // Auto-reload: 1MHz / 50 = 20kHz

// PWM mode 1 на канале 1
TIM2->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;
TIM2->CCER = TIM_CCER_CC1E;

// Duty cycle (50%)
TIM2->CCR1 = 25;

// Запуск
TIM2->CR1 = TIM_CR1_CEN;
```

### Input capture

```cpp
// Настройка таймера
TIM3->PSC = 83;
TIM3->ARR = 0xFFFF;

// Input capture на канале 1
TIM3->CCMR1 = TIM_CCMR1_CC1S_0;  // TI1 mapped to IC1
TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P;  // Falling edge

// Прерывание
TIM3->DIER = TIM_DIER_CC1IE;
NVIC_EnableIRQ(TIM3_IRQn);

TIM3->CR1 = TIM_CR1_CEN;
```

---

## Watchdog

### IWDG

```cpp
// Разблокировка
IWDG->KR = 0x5555;

// Предделитель
IWDG->PR = IWDG_PR_PR_2;  // /256

// Перезагрузка (~26 секунд при LSI=32kHz)
IWDG->RLR = 0x0FFF;

// Запуск
IWDG->KR = 0xCCCC;

// Кик (в основном цикле)
IWDG->KR = 0xAAAA;
```

### WWDG

```cpp
// Включение тактирования
RCC->APB1ENR |= RCC_APB1ENR_WWDGEN;

// Конфигурация (~40 мс окно)
WWDG->CFR = WWDG_CFR_W_6 | WWDG_CFR_W_0 |  // Window = 0x41
            WWDG_CFR_WDGTB_0;               // Prescaler /8

WWDG->CR = WWDG_CR_WDGA | 0x40;  // Enable, counter = 64

// Кик (counter > window)
if (WWDG->CR > 0x41) {
    WWDG->CR = 0x40;
}
```

---

## RCC и Clock

### Смена источника тактирования

```cpp
// HSE (внешний кварц)
RCC->CR |= RCC_CR_HSEON;
while (!(RCC->CR & RCC_CR_HSERDY));

// PLL (HSE * N / M / P)
RCC->PLLCFGR = (8 << 0) |    // M = 8
               (336 << 6) |  // N = 336
               (0 << 16) |   // P = 2
               RCC_PLLCFGR_PLLSRC_HSE;

RCC->CR |= RCC_CR_PLLON;
while (!(RCC->CR & RCC_CR_PLLRDY));

// Переключение
RCC->CFGR |= RCC_CFGR_SW_PLL;
while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
```

---

## АЦП

```cpp
// Включение тактирования
RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

// Конфигурация
ADC1->CR1 = 0;  // 12-bit resolution
ADC1->CR2 = ADC_CR2_ADON;  // Enable

// Калибровка
ADC1->CR2 |= ADC_CR2_CAL;
while (ADC1->CR2 & ADC_CR2_CAL);

// Преобразование
ADC1->SQR3 = channel;  // Channel selection
ADC1->CR2 |= ADC_CR2_SWSTART;
while (!(ADC1->SR & ADC_SR_EOC));
uint16_t result = ADC1->DR;
```

---

## Flash

### Запись

```cpp
// Разблокировка
FLASH->KEYR = 0x45670123;
FLASH->KEYR = 0xCDEF89AB;

// Стирание сектора
FLASH->CR = FLASH_CR_SER | (sector << 3);
FLASH->CR |= FLASH_CR_STRT;
while (FLASH->SR & FLASH_SR_BSY);

// Программирование
FLASH->CR = FLASH_CR_PG;
*addr = data;
while (FLASH->SR & FLASH_SR_BSY);

// Блокировка
FLASH->CR |= FLASH_CR_LOCK;
```
