// ================================================================
//  main_full_test.cpp  —  Gas & Flame Detector + Blynk WiFi
//  ESP32 FreeRTOS — Complete Test Suite
//
//  ┌─────────────────────────────────────────────────────────┐
//  │  HOW TO SWITCH PHASES — uncomment ONE #define below     │
//  └─────────────────────────────────────────────────────────┘
//  #define PHASE_NORMAL            ← default baseline
//  #define PHASE3_QUEUE_FULL       ← queue overflow test
//  #define PHASE3_SLOW_CONSUMER    ← producer faster than consumer
//  #define PHASE4_FAULT_GAS_DISC   ← gas sensor disconnected
//  #define PHASE4_FAULT_STUCK_HI   ← flame sensor stuck HIGH
//  #define PHASE4_FAULT_CPU_LOAD   ← artificial CPU overload
//  #define PHASE5_STRESS           ← stress test (run 10 min)
//  #define PHASE6_RACE_DEMO        ← race condition WITHOUT mutex
//  #define PHASE6_RACE_FIXED       ← race condition WITH mutex
//  #define PHASE7_DEADLOCK_DEMO    ← deadlock demonstration
//  #define PHASE7_DEADLOCK_FIXED   ← deadlock fixed
//  #define PHASE8_PRIORITY_INV     ← priority inversion demo
//  #define PHASE9_TIMING           ← WCET / jitter / latency
// ================================================================

#define PHASE9_TIMING 

// ================================================================
//  BLYNK CONFIG
// ================================================================
#define BLYNK_TEMPLATE_ID   "TMPL2KKRxyQR_"
#define BLYNK_TEMPLATE_NAME "gas and flame detect"
#define BLYNK_AUTH_TOKEN    "d3QQPra4Kwk1SF4YJf4YySvcgx9cAIoT"
#define BLYNK_PRINT         Serial

#include <Arduino.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

// ================================================================
//  WiFi CREDENTIALS
// ================================================================
char ssid[] = "iPhone";
char pass[] = "12345678900";

// ================================================================
//  PIN DEFINITIONS
// ================================================================
#define MQ6_AO_PIN      32    // ADC1_CH4
#define FLAME_DO_PIN    19    // Active-HIGH flame sensor
#define BUZZER_PIN      21    // Passive buzzer PWM
#define TIMING_GPIO     22    // Oscilloscope toggle (Phase 9)

// ================================================================
//  PHASE-CONTROLLED CONFIGURATION
// ================================================================
#if defined(PHASE3_QUEUE_FULL)
  #define QUEUE_LENGTH        2
  #define CONSUMER_DELAY_MS   0
#elif defined(PHASE3_SLOW_CONSUMER)
  #define QUEUE_LENGTH        5
  #define CONSUMER_DELAY_MS   600
#elif defined(PHASE5_STRESS)
  #define QUEUE_LENGTH        10
  #define PERIOD_ANALOG_MS    20
  #define PERIOD_DIGITAL_MS   10
  #define PERIOD_LOGGING_MS   200
  #define PERIOD_BLYNK_MS     500
#else
  #define QUEUE_LENGTH        20
#endif

#ifndef PERIOD_ANALOG_MS
  #define PERIOD_ANALOG_MS    100
#endif
#ifndef PERIOD_DIGITAL_MS
  #define PERIOD_DIGITAL_MS   50
#endif
#ifndef PERIOD_LOGGING_MS
  #define PERIOD_LOGGING_MS   1000
#endif
#ifndef PERIOD_BLYNK_MS
  #define PERIOD_BLYNK_MS     1000
#endif
#ifndef CONSUMER_DELAY_MS
  #define CONSUMER_DELAY_MS   0
#endif
#define PERIOD_PROCESSING_MS  200
#define PERIOD_OUTPUT_MS      100

// ================================================================
//  GAS SENSOR THRESHOLDS
// ================================================================
#define GAS_WARMUP_MS       30000
#define GAS_THRESHOLD       2000
#define GAS_INVALID_LOW     50
#define GAS_INVALID_HIGH    4090

// ================================================================
//  BUZZER FREQUENCIES
// ================================================================
#define BUZZER_FREQ_FLAME   2000
#define BUZZER_FREQ_GAS     1000

// ================================================================
//  DATA STRUCTURES
// ================================================================
typedef struct {
  int        gasADC;
  bool       gasValid;
  bool       gasAlert;
  bool       warmingUp;
  TickType_t timestamp;
  int64_t    enqueue_us;
} GasSensorData_t;

typedef struct {
  bool       flameDetected;
  bool       sensorValid;
  TickType_t timestamp;
  int64_t    isr_us;
  int64_t    task_us;
} FlameSensorData_t;

typedef struct {
  bool  triggerBuzzer;
  int   buzzerFreq;
  bool  gasAlert;
  bool  flameAlert;
  bool  gasWarmingUp;
  int   gasADC;
} SystemDecision_t;

// ================================================================
//  RTOS HANDLES
// ================================================================
QueueHandle_t     xGasQueue;
QueueHandle_t     xFlameQueue;
QueueHandle_t     xDecisionQueue;
SemaphoreHandle_t xFlameSemaphore;
SemaphoreHandle_t xSerialMutex;
SemaphoreHandle_t xRaceMutex;
SemaphoreHandle_t xMutexA;
SemaphoreHandle_t xMutexB;
SemaphoreHandle_t xPrioMutex;

// ================================================================
//  DIAGNOSTIC COUNTERS
// ================================================================
volatile int     droppedGasSamples   = 0;
volatile int     droppedFlameSamples = 0;
volatile int     missedDeadlines     = 0;
volatile int64_t isrTimestamp_us     = 0;
volatile int     sharedCounter       = 0;
volatile int     sharedCounterSafe   = 0;

// Phase 9 timing
volatile int64_t wcet_analog_us    = 0;
volatile int64_t wcet_process_us   = 0;
volatile int64_t wcet_output_us    = 0;
volatile int64_t max_sem_lat_us    = 0;
volatile int64_t max_mutex_blk_us  = 0;
volatile int64_t max_queue_lat_us  = 0;
volatile int64_t max_jitter_us     = 0;

// Blynk state change tracking
bool lastBlynkGasAlert   = false;
bool lastBlynkFlameAlert = false;

// ================================================================
//  UNIT-TESTABLE PURE FUNCTIONS
//  (Phase 2 — these run on host with test_unit.cpp)
// ================================================================
bool isGasValid(int adc) {
  return (adc >= GAS_INVALID_LOW && adc <= GAS_INVALID_HIGH);
}

bool isFlameDetected(int pinVal) {
  return (pinVal == HIGH);
}

// Decision: 0=silent, 1=gas(1000Hz), 2=flame(2000Hz)
int decideSystem(bool gasAlert, bool flameAlert) {
  if (flameAlert)      return 2;
  if (gasAlert)        return 1;
  return 0;
}

int convertAdcToPercent(int adc) {
  if (adc < 0)    adc = 0;
  if (adc > 4095) adc = 4095;
  return (adc * 100) / 4095;
}

// ================================================================
//  SAFE SERIAL MACRO
// ================================================================
#define SAFE_PRINT(fmt, ...) do {                                   \
  if (xSemaphoreTake(xSerialMutex, pdMS_TO_TICKS(50)) == pdTRUE) { \
    Serial.printf(fmt, ##__VA_ARGS__);                              \
    xSemaphoreGive(xSerialMutex);                                   \
  }                                                                 \
} while(0)

// ================================================================
//  ISR — Flame Sensor RISING edge
// ================================================================
void IRAM_ATTR flameISR() {
  isrTimestamp_us = esp_timer_get_time();
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(xFlameSemaphore, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

// ================================================================
//  TASK 1 — Analog Sensor Task (Priority 4, 100ms)
//  REQ-01: sampled every 100ms
//  REQ-02: invalid readings flagged
// ================================================================
void vAnalogSensorTask(void *pvParameters) {
  TickType_t xLastWake = xTaskGetTickCount();
  GasSensorData_t data;
  int64_t lastPeriod_us = esp_timer_get_time();

  for (;;) {
    int64_t t_start = esp_timer_get_time();

    // Jitter measurement
    int64_t now_us   = esp_timer_get_time();
    int64_t jitter   = abs((now_us - lastPeriod_us) - (int64_t)(PERIOD_ANALOG_MS * 1000));
    lastPeriod_us    = now_us;
    if (jitter > max_jitter_us) max_jitter_us = jitter;

    #ifdef PHASE9_TIMING
      digitalWrite(TIMING_GPIO, HIGH);
    #endif

    // Read sensor (with fault injection support)
    #ifdef PHASE4_FAULT_GAS_DISC
      data.gasADC = 10;   // simulate disconnected → below GAS_INVALID_LOW
    #else
      data.gasADC = analogRead(MQ6_AO_PIN);
    #endif

    data.timestamp  = xTaskGetTickCount();
    data.enqueue_us = t_start;
    data.warmingUp  = (millis() < GAS_WARMUP_MS);

    // Validate + decide alert
    if (!isGasValid(data.gasADC)) {
      data.gasValid = false;
      data.gasAlert = false;
      SAFE_PRINT("[GAS]  INVALID reading: ADC=%d — sensor fault!\n", data.gasADC);
    } else {
      data.gasValid = true;
      data.gasAlert = (!data.warmingUp) && (data.gasADC >= GAS_THRESHOLD);
    }

    // Phase 4: CPU overload injection
    #ifdef PHASE4_FAULT_CPU_LOAD
      volatile int dummy = 0;
      for (int i = 0; i < 3000000; i++) dummy++;
    #endif

    #ifdef PHASE9_TIMING
      digitalWrite(TIMING_GPIO, LOW);
      int64_t wcet = esp_timer_get_time() - t_start;
      if (wcet > wcet_analog_us) wcet_analog_us = wcet;
    #endif

    // Send to queue (non-blocking)
    if (xQueueSend(xGasQueue, &data, 0) != pdTRUE) {
      droppedGasSamples++;
      SAFE_PRINT("[DROP] Gas sample lost! ADC=%d  Total=%d\n",
                 data.gasADC, droppedGasSamples);
    }

    // Missed deadline check
    if (xTaskGetTickCount() > xLastWake + pdMS_TO_TICKS(PERIOD_ANALOG_MS)) {
      missedDeadlines++;
      SAFE_PRINT("[DL]   Analog task missed deadline! Total=%d\n", missedDeadlines);
    }

    vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(PERIOD_ANALOG_MS));
  }
}

// ================================================================
//  TASK 2 — Digital Sensor Task (Priority 5, 50ms)
//  ISR-driven + polling fallback
//  REQ-03: ISR semaphore synchronisation
// ================================================================
void vDigitalSensorTask(void *pvParameters) {
  FlameSensorData_t data;

  for (;;) {
    // Block on semaphore (ISR wakes) OR timeout = polling fallback
    BaseType_t got = xSemaphoreTake(xFlameSemaphore,
                                    pdMS_TO_TICKS(PERIOD_DIGITAL_MS));

    data.task_us   = esp_timer_get_time();
    data.isr_us    = isrTimestamp_us;
    data.timestamp = xTaskGetTickCount();

    // Semaphore latency (Phase 9)
    if (got == pdTRUE) {
      int64_t latency = data.task_us - data.isr_us;
      if (latency > 0 && latency > max_sem_lat_us)
        max_sem_lat_us = latency;
      #ifdef PHASE9_TIMING
        SAFE_PRINT("[TIMING] ISR→Task semaphore latency: %lld us\n", latency);
      #endif
    }

    // Read pin (with fault injection)
    #ifdef PHASE4_FAULT_STUCK_HI
      data.flameDetected = true;
      data.sensorValid   = true;
      SAFE_PRINT("[FAULT] Flame sensor STUCK HIGH simulated\n");
    #else
      data.flameDetected = (digitalRead(FLAME_DO_PIN) == HIGH);
      data.sensorValid   = true;
    #endif

    if (xQueueSend(xFlameQueue, &data, 0) != pdTRUE) {
      droppedFlameSamples++;
      SAFE_PRINT("[DROP] Flame sample lost! Total=%d\n", droppedFlameSamples);
    }
  }
}

// ================================================================
//  TASK 3 — Communication Task = Blynk WiFi (Priority 2, 1s)
//  Satisfies PDF requirement: "communication-based sensor/interface"
//  Sends data to Blynk cloud + push notifications on state change
//  REQ-09: system shall report alerts over WiFi
// ================================================================
void vBlynkTask(void *pvParameters) {
  SystemDecision_t decision;
  memset(&decision, 0, sizeof(decision));

  for (;;) {
    // Keep Blynk alive
    Blynk.run();

    if (xQueuePeek(xDecisionQueue, &decision, 0) == pdTRUE) {

      // Send virtual pins
      Blynk.virtualWrite(V0, decision.gasAlert   ? 1 : 0);
      Blynk.virtualWrite(V1, decision.flameAlert ? 1 : 0);
      Blynk.virtualWrite(V2, decision.gasADC);
      Blynk.virtualWrite(V3, convertAdcToPercent(decision.gasADC));

      // Push notification only on state CHANGE (no spam)
      if (decision.flameAlert && !lastBlynkFlameAlert) {
        Blynk.logEvent("flame_detected", "DANGER: Flame detected!");
        SAFE_PRINT("[BLYNK] Notification sent: FLAME\n");
      }
      if (decision.gasAlert && !lastBlynkGasAlert) {
        Blynk.logEvent("gas_detected", "WARNING: Gas detected!");
        SAFE_PRINT("[BLYNK] Notification sent: GAS\n");
      }
      if (!decision.flameAlert && lastBlynkFlameAlert) {
        Blynk.logEvent("flame_cleared", "Flame cleared — safe.");
      }
      if (!decision.gasAlert && lastBlynkGasAlert) {
        Blynk.logEvent("gas_cleared", "Gas level normal.");
      }

      lastBlynkFlameAlert = decision.flameAlert;
      lastBlynkGasAlert   = decision.gasAlert;
    }

    vTaskDelay(pdMS_TO_TICKS(PERIOD_BLYNK_MS));
  }
}

// ================================================================
//  TASK 4 — Processing / Decision Task (Priority 3, 200ms)
//  Drains queues, applies decision logic, posts to xDecisionQueue
//  REQ-04: responds within 200ms
// ================================================================
void vProcessingTask(void *pvParameters) {
  GasSensorData_t   gasData;
  FlameSensorData_t flameData;
  SystemDecision_t  decision;

  bool lastGasAlert   = false;
  bool lastFlameAlert = false;
  bool lastWarmup     = true;
  int  lastADC        = 0;

  for (;;) {
    int64_t t_start = esp_timer_get_time();

    // Drain gas queue — keep latest valid reading
    while (xQueueReceive(xGasQueue, &gasData, 0) == pdTRUE) {
      lastWarmup = gasData.warmingUp;
      lastADC    = gasData.gasADC;
      if (gasData.gasValid)
        lastGasAlert = gasData.gasAlert;

      // Queue latency (Phase 9)
      int64_t lat = esp_timer_get_time() - gasData.enqueue_us;
      if (lat > max_queue_lat_us) max_queue_lat_us = lat;
    }

    // Drain flame queue — keep latest
    while (xQueueReceive(xFlameQueue, &flameData, 0) == pdTRUE) {
      if (flameData.sensorValid)
        lastFlameAlert = flameData.flameDetected;
    }

    // Decision logic (uses unit-tested function)
    int result = decideSystem(lastGasAlert, lastFlameAlert);

    decision.gasAlert     = lastGasAlert;
    decision.flameAlert   = lastFlameAlert;
    decision.gasWarmingUp = lastWarmup;
    decision.gasADC       = lastADC;

    if (result == 2) {
      decision.triggerBuzzer = true;
      decision.buzzerFreq    = BUZZER_FREQ_FLAME;
    } else if (result == 1) {
      decision.triggerBuzzer = true;
      decision.buzzerFreq    = BUZZER_FREQ_GAS;
    } else {
      decision.triggerBuzzer = false;
      decision.buzzerFreq    = 0;
    }

    xQueueOverwrite(xDecisionQueue, &decision);

    // WCET
    int64_t wcet = esp_timer_get_time() - t_start;
    if (wcet > wcet_process_us) wcet_process_us = wcet;

    #if defined(PHASE3_SLOW_CONSUMER) && CONSUMER_DELAY_MS > 0
      vTaskDelay(pdMS_TO_TICKS(CONSUMER_DELAY_MS));
    #else
      vTaskDelay(pdMS_TO_TICKS(PERIOD_PROCESSING_MS));
    #endif
  }
}

// ================================================================
//  TASK 5 — Output / Actuator Task (Priority 4, 100ms)
//  Drives passive buzzer via LEDC — updates only on freq change
// ================================================================
void vOutputTask(void *pvParameters) {
  SystemDecision_t decision;
  int currentFreq = -1;

  for (;;) {
    int64_t t_start = esp_timer_get_time();

    if (xQueuePeek(xDecisionQueue, &decision,
                   pdMS_TO_TICKS(PERIOD_OUTPUT_MS)) == pdTRUE) {
      int target = decision.triggerBuzzer ? decision.buzzerFreq : 0;
      if (target != currentFreq) {
        ledcWriteTone(BUZZER_PIN, target);
        currentFreq = target;

        if (target == 0)
          SAFE_PRINT("[OUT]  Buzzer OFF\n");
        else
          SAFE_PRINT("[OUT]  Buzzer ON — %d Hz (%s)\n",
                     target, target == BUZZER_FREQ_FLAME ? "FLAME" : "GAS");
      }
    }

    int64_t wcet = esp_timer_get_time() - t_start;
    if (wcet > wcet_output_us) wcet_output_us = wcet;

    vTaskDelay(pdMS_TO_TICKS(PERIOD_OUTPUT_MS));
  }
}

// ================================================================
//  TASK 6 — Logging / Diagnostic Task (Priority 1, 1s)
//  Lowest priority — full system report every second
//  REQ-10: mutex protects Serial — no garbled output
// ================================================================
void vLoggingTask(void *pvParameters) {
  SystemDecision_t decision;
  memset(&decision, 0, sizeof(decision));
  static uint32_t cycle = 0;
  static uint32_t maxQueueOccupancy = 0;

  for (;;) {
    // Measure mutex blocking time
    int64_t req_t = esp_timer_get_time();
    xSemaphoreTake(xSerialMutex, portMAX_DELAY);
    int64_t blk = esp_timer_get_time() - req_t;
    if (blk > max_mutex_blk_us) max_mutex_blk_us = blk;

    xQueuePeek(xDecisionQueue, &decision, 0);
    cycle++;

    // Track max queue occupancy
    uint32_t gasOcc = QUEUE_LENGTH - uxQueueSpacesAvailable(xGasQueue);
    if (gasOcc > maxQueueOccupancy) maxQueueOccupancy = gasOcc;

    Serial.println("\n=== SYSTEM LOG ===");
    Serial.printf("[TIME]   Uptime: %lu ms  |  Cycle: %lu\n", millis(), cycle);

    // Warmup status
    if (decision.gasWarmingUp) {
      uint32_t remaining = (GAS_WARMUP_MS - millis()) / 1000;
      Serial.printf("[GAS]    WARMING UP — %lu s remaining\n", remaining);
    } else {
      Serial.printf("[GAS]    Alert: %s  |  ADC: %d  |  %%: %d%%\n",
                    decision.gasAlert ? "YES [GAS!]" : "no",
                    decision.gasADC,
                    convertAdcToPercent(decision.gasADC));
    }

    Serial.printf("[FLAME]  Alert: %s\n",
                  decision.flameAlert ? "YES [FIRE!]" : "no");
    Serial.printf("[BUZZER] %s  |  Freq: %d Hz\n",
                  decision.triggerBuzzer ? "ON" : "OFF",
                  decision.buzzerFreq);
    Serial.printf("[QUEUE]  GasQ  used: %2lu/%d  max: %lu  |  FlameQ used: %2lu/%d\n",
                  (uint32_t)(QUEUE_LENGTH - uxQueueSpacesAvailable(xGasQueue)),   QUEUE_LENGTH, maxQueueOccupancy,
                  (uint32_t)(QUEUE_LENGTH - uxQueueSpacesAvailable(xFlameQueue)), QUEUE_LENGTH);
    Serial.printf("[DROP]   Gas: %d  |  Flame: %d  |  MissedDL: %d\n",
                  droppedGasSamples, droppedFlameSamples, missedDeadlines);
    Serial.printf("[HEAP]   Free: %lu bytes\n", esp_get_free_heap_size());
    Serial.printf("[WIFI]   %s\n", WiFi.isConnected() ? "Connected" : "DISCONNECTED");

    // Phase 9 timing report
    #ifdef PHASE9_TIMING
      Serial.println("[TIMING] ─────────────────────────────────");
      Serial.printf("[WCET]   Analog: %lld us  |  Processing: %lld us  |  Output: %lld us\n",
                    wcet_analog_us, wcet_process_us, wcet_output_us);
      Serial.printf("[LAT]    Semaphore: %lld us  |  MutexBlk: %lld us  |  QueueLat: %lld us\n",
                    max_sem_lat_us, max_mutex_blk_us, max_queue_lat_us);
      Serial.printf("[JITTER] MaxJitter: %lld us\n", max_jitter_us);

      // Stack high-water marks (BONUS)
      Serial.println("[STACK]  High-Water Marks:");
      Serial.printf("         AnalogSensor: check via uxTaskGetStackHighWaterMark\n");
    #endif

    Serial.println("==================");
    xSemaphoreGive(xSerialMutex);

    vTaskDelay(pdMS_TO_TICKS(PERIOD_LOGGING_MS));
  }
}

// ================================================================
//  PHASE 6 — Race Condition Demo
//  Two tasks → 10,000 increments each → expected = 20,000
//  WITHOUT mutex: corrupted result (race)
//  WITH mutex:    always 20,000 (correct)
// ================================================================
#if defined(PHASE6_RACE_DEMO) || defined(PHASE6_RACE_FIXED)
void vRaceTask(void *pvParameters) {
  int taskId = (int)(intptr_t)pvParameters;

  for (int i = 0; i < 10000; i++) {
    #ifdef PHASE6_RACE_FIXED
      xSemaphoreTake(xRaceMutex, portMAX_DELAY);
      sharedCounterSafe++;
      xSemaphoreGive(xRaceMutex);
    #else
      // UNSAFE — read-modify-write without protection
      sharedCounter++;
    #endif
  }

  SAFE_PRINT("[RACE] Task %d done.\n", taskId);

  if (taskId == 2) {
    vTaskDelay(pdMS_TO_TICKS(500));  // ensure task 1 also finished
    #ifdef PHASE6_RACE_FIXED
      SAFE_PRINT("[RACE] WITH MUTEX  — Expected: 20000  Got: %d  %s\n",
                 sharedCounterSafe,
                 sharedCounterSafe == 20000 ? "CORRECT ✓" : "WRONG ✗");
    #else
      SAFE_PRINT("[RACE] NO MUTEX    — Expected: 20000  Got: %d  %s\n",
                 sharedCounter,
                 sharedCounter == 20000 ? "(lucky)" : "CORRUPTED ✗ ← race!");
      SAFE_PRINT("[RACE] Why: sharedCounter++ = LOAD + ADD + STORE\n");
      SAFE_PRINT("[RACE] FreeRTOS can preempt between any two instructions.\n");
      SAFE_PRINT("[RACE] Fix: wrap with xSemaphoreTake/Give (PHASE6_RACE_FIXED)\n");
    #endif
  }
  vTaskDelete(NULL);
}
#endif

// ================================================================
//  PHASE 7 — Deadlock Demo
//  Task A: locks MutexA → tries MutexB
//  Task B: locks MutexB → tries MutexA
//  → circular wait → deadlock detected via timeout
//  Fix: always acquire in same order (A then B)
// ================================================================
#if defined(PHASE7_DEADLOCK_DEMO) || defined(PHASE7_DEADLOCK_FIXED)
void vDeadlockTaskA(void *pvParameters) {
  #ifdef PHASE7_DEADLOCK_DEMO
    SAFE_PRINT("[DEADLOCK] Task A: locking MutexA...\n");
    xSemaphoreTake(xMutexA, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(50));
    SAFE_PRINT("[DEADLOCK] Task A: trying MutexB (held by B)... WILL BLOCK\n");
    if (xSemaphoreTake(xMutexB, pdMS_TO_TICKS(3000)) == pdTRUE) {
      xSemaphoreGive(xMutexB);
    } else {
      SAFE_PRINT("[DEADLOCK] Task A: TIMEOUT — DEADLOCK DETECTED!\n");
    }
    xSemaphoreGive(xMutexA);
  #else
    SAFE_PRINT("[FIXED] Task A: ordered lock A→B\n");
    xSemaphoreTake(xMutexA, portMAX_DELAY);
    xSemaphoreTake(xMutexB, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(100));
    xSemaphoreGive(xMutexB);
    xSemaphoreGive(xMutexA);
    SAFE_PRINT("[FIXED] Task A: done — no deadlock ✓\n");
  #endif
  vTaskDelete(NULL);
}

void vDeadlockTaskB(void *pvParameters) {
  #ifdef PHASE7_DEADLOCK_DEMO
    SAFE_PRINT("[DEADLOCK] Task B: locking MutexB...\n");
    xSemaphoreTake(xMutexB, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(50));
    SAFE_PRINT("[DEADLOCK] Task B: trying MutexA (held by A)... WILL BLOCK\n");
    if (xSemaphoreTake(xMutexA, pdMS_TO_TICKS(3000)) == pdTRUE) {
      xSemaphoreGive(xMutexA);
    } else {
      SAFE_PRINT("[DEADLOCK] Task B: TIMEOUT — DEADLOCK DETECTED!\n");
    }
    xSemaphoreGive(xMutexB);
  #else
    vTaskDelay(pdMS_TO_TICKS(20));
    SAFE_PRINT("[FIXED] Task B: ordered lock A→B\n");
    xSemaphoreTake(xMutexA, portMAX_DELAY);
    xSemaphoreTake(xMutexB, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(100));
    xSemaphoreGive(xMutexB);
    xSemaphoreGive(xMutexA);
    SAFE_PRINT("[FIXED] Task B: done — no deadlock ✓\n");
  #endif
  vTaskDelete(NULL);
}
#endif

// ================================================================
//  PHASE 8 — Priority Inversion Demo
//  Low  (pri=1): holds mutex 3s
//  High (pri=3): waits for same mutex
//  Med  (pri=2): burns CPU → blocks Low from releasing → High starves
// ================================================================
#ifdef PHASE8_PRIORITY_INV
void vPrioLow(void *pvParameters) {
  SAFE_PRINT("[PRIOINV] Low (pri=1): taking mutex...\n");
  xSemaphoreTake(xPrioMutex, portMAX_DELAY);
  SAFE_PRINT("[PRIOINV] Low: holding mutex for 3s\n");
  vTaskDelay(pdMS_TO_TICKS(3000));
  xSemaphoreGive(xPrioMutex);
  SAFE_PRINT("[PRIOINV] Low: released mutex\n");
  vTaskDelete(NULL);
}

void vPrioMed(void *pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(100));
  SAFE_PRINT("[PRIOINV] Med (pri=2): burning CPU — blocks Low!\n");
  volatile long x = 0;
  for (long i = 0; i < 50000000L; i++) x++;
  SAFE_PRINT("[PRIOINV] Med: done\n");
  vTaskDelete(NULL);
}

void vPrioHigh(void *pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(50));
  int64_t t = esp_timer_get_time();
  SAFE_PRINT("[PRIOINV] High (pri=3): WAITING for mutex (held by Low)...\n");
  xSemaphoreTake(xPrioMutex, portMAX_DELAY);
  int64_t wait_ms = (esp_timer_get_time() - t) / 1000;
  SAFE_PRINT("[PRIOINV] High: got mutex after %lld ms\n", wait_ms);
  SAFE_PRINT("[PRIOINV] High (pri=3) was delayed by Med (pri=2) — inversion shown!\n");
  SAFE_PRINT("[PRIOINV] Fix: FreeRTOS mutex supports priority inheritance.\n");
  xSemaphoreGive(xPrioMutex);
  vTaskDelete(NULL);
}
#endif

// ================================================================
//  setup()
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  // Phase banner
  Serial.println("\n[BOOT] Gas & Flame Detector — ESP32 FreeRTOS + Blynk");
  Serial.println("[BOOT] ================================================");
  #if   defined(PHASE3_QUEUE_FULL)
    Serial.println("[PHASE] 3A — Queue Full (QUEUE_LENGTH=2)");
  #elif defined(PHASE3_SLOW_CONSUMER)
    Serial.println("[PHASE] 3B — Slow Consumer (600ms delay)");
  #elif defined(PHASE4_FAULT_GAS_DISC)
    Serial.println("[PHASE] 4  — FAULT: Gas Sensor Disconnected");
  #elif defined(PHASE4_FAULT_STUCK_HI)
    Serial.println("[PHASE] 4  — FAULT: Flame Sensor Stuck HIGH");
  #elif defined(PHASE4_FAULT_CPU_LOAD)
    Serial.println("[PHASE] 4  — FAULT: CPU Overload Injection");
  #elif defined(PHASE5_STRESS)
    Serial.println("[PHASE] 5  — STRESS TEST (run 10 minutes)");
  #elif defined(PHASE6_RACE_DEMO)
    Serial.println("[PHASE] 6  — Race Condition WITHOUT mutex (BUG)");
  #elif defined(PHASE6_RACE_FIXED)
    Serial.println("[PHASE] 6  — Race Condition WITH mutex (FIXED)");
  #elif defined(PHASE7_DEADLOCK_DEMO)
    Serial.println("[PHASE] 7  — Deadlock Demonstration");
  #elif defined(PHASE7_DEADLOCK_FIXED)
    Serial.println("[PHASE] 7  — Deadlock Fixed (ordered locking)");
  #elif defined(PHASE8_PRIORITY_INV)
    Serial.println("[PHASE] 8  — Priority Inversion Demo");
  #elif defined(PHASE9_TIMING)
    Serial.println("[PHASE] 9  — Timing: WCET / Jitter / Latency");
  #else
    Serial.println("[PHASE] NORMAL — Baseline Operation");
  #endif

  Serial.printf("[CFG]  QUEUE=%d  ANALOG=%dms  PROC=%dms  BLYNK=%dms\n",
                QUEUE_LENGTH, PERIOD_ANALOG_MS,
                PERIOD_PROCESSING_MS, PERIOD_BLYNK_MS);

  // Pins
  pinMode(MQ6_AO_PIN,   INPUT);
  pinMode(FLAME_DO_PIN, INPUT);
  pinMode(BUZZER_PIN,   OUTPUT);
  #ifdef PHASE9_TIMING
    pinMode(TIMING_GPIO, OUTPUT);
    digitalWrite(TIMING_GPIO, LOW);
  #endif

  // Buzzer init
  ledcAttach(BUZZER_PIN, 2000, 8);
  ledcWriteTone(BUZZER_PIN, 0);

  // WiFi + Blynk
  Serial.println("[BOOT] Connecting WiFi + Blynk...");
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  Serial.println("[BOOT] Blynk connected!");

  // Queues
  xGasQueue      = xQueueCreate(QUEUE_LENGTH, sizeof(GasSensorData_t));
  xFlameQueue    = xQueueCreate(QUEUE_LENGTH, sizeof(FlameSensorData_t));
  xDecisionQueue = xQueueCreate(1,            sizeof(SystemDecision_t));
  configASSERT(xGasQueue);
  configASSERT(xFlameQueue);
  configASSERT(xDecisionQueue);

  // Semaphores + Mutexes
  xFlameSemaphore = xSemaphoreCreateBinary();
  xSerialMutex    = xSemaphoreCreateMutex();
  xRaceMutex      = xSemaphoreCreateMutex();
  xMutexA         = xSemaphoreCreateMutex();
  xMutexB         = xSemaphoreCreateMutex();
  configASSERT(xFlameSemaphore);
  configASSERT(xSerialMutex);
  configASSERT(xRaceMutex);
  configASSERT(xMutexA);
  configASSERT(xMutexB);

  // ISR
  attachInterrupt(digitalPinToInterrupt(FLAME_DO_PIN), flameISR, RISING);

  // Core tasks
  //                              func                  name           stack  param pri  handle
  xTaskCreate(vAnalogSensorTask,  "AnalogSensor",  2048, NULL, 4, NULL);
  xTaskCreate(vDigitalSensorTask, "DigitalSensor", 2048, NULL, 5, NULL);
  xTaskCreate(vBlynkTask,         "BlynkComm",     4096, NULL, 2, NULL);
  xTaskCreate(vProcessingTask,    "Processing",    3072, NULL, 3, NULL);
  xTaskCreate(vOutputTask,        "Output",        2048, NULL, 4, NULL);
  xTaskCreate(vLoggingTask,       "Logging",       3072, NULL, 1, NULL);

  // Phase-specific tasks
  #if defined(PHASE6_RACE_DEMO) || defined(PHASE6_RACE_FIXED)
    sharedCounter = sharedCounterSafe = 0;
    xTaskCreate(vRaceTask, "RaceA", 2048, (void*)1, 3, NULL);
    xTaskCreate(vRaceTask, "RaceB", 2048, (void*)2, 3, NULL);
  #endif

  #if defined(PHASE7_DEADLOCK_DEMO) || defined(PHASE7_DEADLOCK_FIXED)
    xTaskCreate(vDeadlockTaskA, "DLockA", 2048, NULL, 3, NULL);
    xTaskCreate(vDeadlockTaskB, "DLockB", 2048, NULL, 3, NULL);
  #endif

  #ifdef PHASE8_PRIORITY_INV
    xPrioMutex = xSemaphoreCreateMutex();
    xTaskCreate(vPrioLow,  "PrioLow",  2048, NULL, 1, NULL);
    xTaskCreate(vPrioMed,  "PrioMed",  2048, NULL, 2, NULL);
    xTaskCreate(vPrioHigh, "PrioHigh", 2048, NULL, 3, NULL);
  #endif

  Serial.println("[BOOT] All tasks started.");
  Serial.printf("[BOOT] MQ6 warmup: alerts disabled for %d s\n\n",
                GAS_WARMUP_MS / 1000);
}

// ================================================================
//  loop() — unused under FreeRTOS
// ================================================================
void loop() {
  vTaskDelete(NULL);
}