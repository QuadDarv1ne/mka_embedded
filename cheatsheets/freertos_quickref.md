# FreeRTOS Quick Reference

## Задачи

### Создание и удаление

```cpp
// Создание задачи
TaskHandle_t taskHandle;
xTaskCreate(
    taskFunction,      // Функция задачи
    "TaskName",        // Имя (для отладки)
    256,               // Размер стека (в словах)
    (void*)param,      // Параметр
    2,                 // Приоритет
    &taskHandle        // Дескриптор
);

// Удаление задачи
vTaskDelete(taskHandle);  // или vTaskDelete(NULL) для самоудаления
```

### Функция задачи

```cpp
void taskFunction(void* pvParameters) {
    // Инициализация
    
    for (;;) {
        // Основной код
        
        vTaskDelay(pdMS_TO_TICKS(100));  // Задержка 100 мс
    }
}
```

### Управление

```cpp
// Приостановить
vTaskSuspend(taskHandle);

// Возобновить
vTaskResume(taskHandle);

// Приоритет
UBaseType_t priority = uxTaskPriorityGet(taskHandle);
vTaskPrioritySet(taskHandle, newPriority);

// Информация
TaskStatus_t status;
vTaskGetInfo(taskHandle, &status, pdTRUE, eRunning);
// status.eTaskState: eRunning, eReady, eBlocked, eSuspended, eDeleted

// Список задач
char buffer[1024];
vTaskList(buffer);  // Имя, Состояние, Приоритет, Стек
```

---

## Очереди

### Создание

```cpp
QueueHandle_t queue = xQueueCreate(
    10,           // Длина (кол-во элементов)
    sizeof(Msg)   // Размер элемента
);
```

### Отправка

```cpp
Msg msg = {.id = 1, .data = 100};

// Блокирующая (ждать места)
xQueueSend(queue, &msg, portMAX_DELAY);

// С таймаутом
xQueueSend(queue, &msg, pdMS_TO_TICKS(100));

// Из ISR
BaseType_t higherPriorityTaskWoken = pdFALSE;
xQueueSendFromISR(queue, &msg, &higherPriorityTaskWoken);
portYIELD_FROM_ISR(higherPriorityTaskWoken);

// В начало очереди
xQueueSendToFront(queue, &msg, 0);
```

### Приём

```cpp
Msg msg;

// Блокирующий
xQueueReceive(queue, &msg, portMAX_DELAY);

// С таймаутом
if (xQueueReceive(queue, &msg, pdMS_TO_TICKS(100)) == pdTRUE) {
    // Данные получены
}

// Из ISR
BaseType_t higherPriorityTaskWoken = pdFALSE;
xQueueReceiveFromISR(queue, &msg, &higherPriorityTaskWoken);
portYIELD_FROM_ISR(higherPriorityTaskWoken);

// Пик (без удаления)
xQueuePeek(queue, &msg, 0);
```

### Информация

```cpp
UBaseType_t waiting = uxQueueMessagesWaiting(queue);
UBaseType_t spaces = uxQueueSpacesAvailable(queue);
UBaseType_t items = uxQueueMessagesWaitingFromISR(queue);
```

---

## Семафоры и мьютексы

### Бинарный семафор

```cpp
// Создание
SemaphoreHandle_t sem = xSemaphoreCreateBinary();

// Дать (из задачи)
xSemaphoreGive(sem);

// Дать из ISR
BaseType_t higherPriorityTaskWoken = pdFALSE;
xSemaphoreGiveFromISR(sem, &higherPriorityTaskWoken);
portYIELD_FROM_ISR(higherPriorityTaskWoken);

// Взять (блокирующий)
xSemaphoreTake(sem, portMAX_DELAY);

// Взять с таймаутом
if (xSemaphoreTake(sem, pdMS_TO_TICKS(100)) == pdTRUE) {
    // Семафор получен
}
```

### Считающий семафор

```cpp
// Создание (max = 10, initial = 5)
SemaphoreHandle_t sem = xSemaphoreCreateCounting(10, 5);

// Получить текущий счёт
UBaseType_t count = uxSemaphoreGetCount(sem);
```

### Мьютекс

```cpp
// Создание
SemaphoreHandle_t mutex = xSemaphoreCreateMutex();

// Захват
if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    // Критическая секция
    sharedResource++;
    
    // Освобождение
    xSemaphoreGive(mutex);
}
```

### Рекурсивный мьютекс

```cpp
SemaphoreHandle_t mutex = xSemaphoreCreateRecursiveMutex();

xSemaphoreTakeRecursive(mutex, portMAX_DELAY);
xSemaphoreTakeRecursive(mutex, portMAX_DELAY);  // Можно!

xSemaphoreGiveRecursive(mutex);
xSemaphoreGiveRecursive(mutex);  // Два раза
```

---

## Программные таймеры

### Создание

```cpp
void timerCallback(TimerHandle_t timer) {
    // Действие по таймеру
}

TimerHandle_t timer = xTimerCreate(
    "TimerName",                    // Имя
    pdMS_TO_TICKS(1000),           // Период
    pdTRUE,                         // Auto-reload (pdFALSE = one-shot)
    (void*)param,                   // Parameter
    timerCallback                   // Callback
);
```

### Управление

```cpp
// Запуск
xTimerStart(timer, portMAX_DELAY);

// Останов
xTimerStop(timer, portMAX_DELAY);

// Перезапуск (сброс счётчика)
xTimerReset(timer, portMAX_DELAY);

// Изменить период
xTimerChangePeriod(timer, pdMS_TO_TICKS(500), portMAX_DELAY);

// Из ISR
BaseType_t higherPriorityTaskWoken = pdFALSE;
xTimerStartFromISR(timer, &higherPriorityTaskWoken);
portYIELD_FROM_ISR(higherPriorityTaskWoken);
```

---

## Группы событий

### Создание

```cpp
EventGroupHandle_t events = xEventGroupCreate();
```

### Установка битов

```cpp
#define BIT_0 (1 << 0)
#define BIT_1 (1 << 1)
#define BIT_2 (1 << 2)

// Установить биты
xEventGroupSetBits(events, BIT_0 | BIT_1);

// Из ISR
BaseType_t higherPriorityTaskWoken = pdFALSE;
xEventGroupSetBitsFromISR(events, BIT_0, &higherPriorityTaskWoken);
portYIELD_FROM_ISR(higherPriorityTaskWoken);
```

### Ожидание битов

```cpp
// Ожидание любого бита (OR)
EventBits_t bits = xEventGroupWaitBits(
    events,
    BIT_0 | BIT_1,   // Биты для ожидания
    pdTRUE,          // Очистить при выходе
    pdFALSE,         // Ожидание любого бита (pdTRUE = все биты)
    portMAX_DELAY    // Таймаут
);

// Проверка
if (bits & BIT_0) {
    // BIT_0 установлен
}

// Ожидание всех битов (AND)
bits = xEventGroupWaitBits(
    events,
    BIT_0 | BIT_1,
    pdTRUE,
    pdTRUE,          // Ждать все биты
    portMAX_DELAY
);
```

### Синхронизация

```cpp
// Синхронизация нескольких задач
#define SYNC_BIT_0 (1 << 0)
#define SYNC_BIT_1 (1 << 1)

// Каждая задача вызывает:
xEventGroupSync(
    events,
    SYNC_BIT_0,           // Установить свой бит
    SYNC_BIT_0 | SYNC_BIT_1,  // Ждать все биты
    portMAX_DELAY
);
```

---

## Уведомления задач

### Отправка

```cpp
// Уведомить (инкремент)
xTaskNotifyGive(taskHandle);

// Из ISR
vTaskNotifyGiveFromISR(taskHandle, &higherPriorityTaskWoken);

// Установить значение
xTaskNotify(taskHandle, 0x1234, eSetValueWithOverwrite);

// Установить биты
xTaskNotify(taskHandle, BIT_0, eSetBits);

// Из ISR
xTaskNotifyFromISR(taskHandle, BIT_0, eSetBits, &higherPriorityTaskWoken);
```

### Приём

```cpp
// Ожидание уведомления (с инкрементом)
uint32_t count = ulTaskNotifyTake(
    pdTRUE,            // Очистить при выходе
    portMAX_DELAY
);

// Ожидание с битами
uint32_t value = ulTaskNotifyTake(
    pdFALSE,           // Не очищать (для проверки битов)
    portMAX_DELAY
);

// Полное ожидание
uint32_t value;
xTaskNotifyWait(
    0,                 // Биты не очищать при входе
    0xFFFFFFFF,        // Очистить все биты при выходе
    &value,
    portMAX_DELAY
);
```

---

## Критические секции

```cpp
// Критическая секция (отключает прерывания)
taskENTER_CRITICAL();
// ... код ...
taskEXIT_CRITICAL();

// Из ISR
UBaseType_t intStatus = taskENTER_CRITICAL_FROM_ISR();
// ... код ...
taskEXIT_CRITICAL_FROM_ISR(intStatus);

// Приостановка планировщика
vTaskSuspendAll();
// ... код (без блокирующих вызовов!) ...
xTaskResumeAll();
```

---

## Память

### Кучи

```cpp
// Свободная память
size_t free = xPortGetFreeHeapSize();
size_t minFree = xPortGetMinimumEverFreeHeapSize();

// Выделение (из кучи FreeRTOS)
void* ptr = pvPortMalloc(256);

// Освобождение
vPortFree(ptr);
```

### Стек задачи

```cpp
// Проверка переполнения стека
// В FreeRTOSConfig.h:
#define configCHECK_FOR_STACK_OVERFLOW 2

// Callback (weak, переопределить)
void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
    // Обработка переполнения
    while(1);
}

// Использование стека
UBaseType_t remaining = uxTaskGetStackHighWaterMark(taskHandle);
```

---

## Хуки (Hooks)

```cpp
// В FreeRTOSConfig.h включить:
#define configUSE_IDLE_HOOK 1
#define configUSE_TICK_HOOK 1
#define configUSE_MALLOC_FAILED_HOOK 1

// Idle hook (выполняется в idle задаче)
void vApplicationIdleHook(void) {
    // Можно перевести CPU в режим пониженного потребления
    __WFI();
}

// Tick hook (выполняется каждый системный тик)
void vApplicationTickHook(void) {
    // Небольшие действия
}

// Malloc failed hook
void vApplicationMallocFailedHook(void) {
    while(1);
}
```

---

## Типичные паттерны

### Задача-обработчик очереди

```cpp
void processingTask(void* pvParameters) {
    QueueHandle_t queue = (QueueHandle_t)pvParameters;
    DataItem item;
    
    for (;;) {
        if (xQueueReceive(queue, &item, portMAX_DELAY) == pdTRUE) {
            processItem(&item);
        }
    }
}
```

### Таймер с периодической работой

```cpp
void periodicTask(void* pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(100);
    
    for (;;) {
        // Работа...
        
        // Точное периодическое выполнение
        vTaskDelayUntil(&lastWakeTime, period);
    }
}
```

### Защита ресурса мьютексом

```cpp
SemaphoreHandle_t resourceMutex;

void safeAccess(void) {
    if (xSemaphoreTake(resourceMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Безопасный доступ к ресурсу
        sharedCounter++;
        
        xSemaphoreGive(resourceMutex);
    } else {
        // Таймаут - обработка ошибки
    }
}
```

### Событийная задача

```cpp
EventGroupHandle_t events;
#define EVENT_DATA_READY (1 << 0)
#define EVENT_TIMEOUT    (1 << 1)

void eventTask(void* pvParameters) {
    for (;;) {
        EventBits_t bits = xEventGroupWaitBits(
            events,
            EVENT_DATA_READY | EVENT_TIMEOUT,
            pdTRUE,   // Очистить
            pdFALSE,  // Любой бит
            portMAX_DELAY
        );
        
        if (bits & EVENT_DATA_READY) {
            processNewData();
        }
        if (bits & EVENT_TIMEOUT) {
            handleTimeout();
        }
    }
}
```

---

## Конфигурация (FreeRTOSConfig.h)

```cpp
// Частота системного тика
#define configTICK_RATE_HZ 1000

// Приоритеты прерываний
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 5

// Размеры
#define configMINIMAL_STACK_SIZE 128
#define configTOTAL_HEAP_SIZE (32 * 1024)

// Функции
#define configUSE_MUTEXES 1
#define configUSE_COUNTING_SEMAPHORES 1
#define configUSE_RECURSIVE_MUTEXES 1
#define configUSE_QUEUE_SETS 1
#define configUSE_TIMERS 1

// Хуки
#define configCHECK_FOR_STACK_OVERFLOW 2
#define configUSE_IDLE_HOOK 1
```
