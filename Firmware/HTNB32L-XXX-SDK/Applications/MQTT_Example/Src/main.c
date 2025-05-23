#define USE_USART_DEBUG
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "slpman_qcx212.h"
#include "pad_qcx212.h"
#include "HT_gpio_qcx212.h"
#include "ic_qcx212.h"
#include "HT_ic_qcx212.h"
#define BUTTON_QUEUE_LENGTH 1
#include "HT_adc_qcx212.h"
#include "adc_qcx212.h"
#include "hal_adc.h"
#include <stdio.h>
#include "string.h"
#include "HT_bsp.h"
#include "stdint.h"
#include "qcx212.h"

static uint32_t uart_cntrl = (ARM_USART_MODE_ASYNCHRONOUS | ARM_USART_DATA_BITS_8 | ARM_USART_PARITY_NONE |
                                 ARM_USART_STOP_BITS_1 | ARM_USART_FLOW_CONTROL_NONE);


#define DEMO_ADC_CHANNEL ADC_ChannelAio2 

static volatile uint32_t callback = 0;
static volatile uint32_t user_adc_channel = 0;

volatile bool button_state = false;
QueueHandle_t xFila;
static adc_config_t adcConfig;

static void HT_ADC_ConversionCallback(uint32_t result) {
    //printf("Adc callback: %d", result);
    callback |= DEMO_ADC_CHANNEL;
    user_adc_channel = result;
}

static void  HT_ADC_Init(uint8_t channel) {

    ADC_GetDefaultConfig(&adcConfig);

    adcConfig.channelConfig.aioResDiv = ADC_AioResDivRatio3Over16; 

    ADC_ChannelInit(channel, ADC_UserAPP, &adcConfig, HT_ADC_ConversionCallback);
}



uint32_t contador_ms = 0;
int blinkDelay = 1000; 

extern USART_HandleTypeDef huart1;
static uint8_t rx_buffer = 0;
static char cmdRxBuffer[32];
static int cmdIdx = 0;

//GPIO10 - BUTTON
#define BUTTON_INSTANCE          0                  /**</ Button pin instance. */
#define BUTTON_PIN               10                 /**</ Button pin number. */
#define BUTTON_PAD_ID            25                 /**</ Button Pad ID. */
#define BUTTON_PAD_ALT_FUNC      PAD_MuxAlt0        /**</ Button pin alternate function. */

//GPIO3 - LED
#define LED_INSTANCE             0                  /**</ LED pin instance. */
#define LED_GPIO_PIN             3                  /**</ LED pin number. */
#define LED_PAD_ID               14                 /**</ LED Pad ID. */
#define LED_PAD_ALT_FUNC         PAD_MuxAlt0        /**</ LED pin alternate function. */

//GPIO4 - LED
#define LED_INSTANCE1             0                  /**</ LED pin instance. */
#define LED_GPIO_PIN2             4                  /**</ LED pin number. */
#define LED_PAD_ID2              15                /**</ LED Pad ID. */
#define LED_PAD_ALT_FUNC         PAD_MuxAlt0        /**</ LED pin alternate function. */

//GPIO5 - LED
#define LED_INSTANCE2             0                  /**</ LED pin instance. */
#define LED_GPIO_PIN3             5                  /**</ LED pin number. */
#define LED_PAD_ID3               16                /**</ LED Pad ID. */
#define LED_PAD_ALT_FUNC         PAD_MuxAlt0        /**</ LED pin alternate function. */

#define LED_ON  1                                   /**</ LED on. */
#define LED_OFF 0                                   /**</ LED off. */

QueueHandle_t xButtonQueue;  // Fila para comunicação entre tarefas

static void HT_GPIO_InitButton(void) {
  GPIO_InitType GPIO_InitStruct = {0};

  GPIO_InitStruct.af = PAD_MuxAlt0;
  GPIO_InitStruct.pad_id = BUTTON_PAD_ID;
  GPIO_InitStruct.gpio_pin = BUTTON_PIN;
  GPIO_InitStruct.pin_direction = GPIO_DirectionInput;
  GPIO_InitStruct.pull = PAD_InternalPullUp;
  GPIO_InitStruct.instance = BUTTON_INSTANCE;
  GPIO_InitStruct.exti = GPIO_EXTI_DISABLED;
  GPIO_InitStruct.interrupt_config = GPIO_InterruptFallingEdge;

  HT_GPIO_Init(&GPIO_InitStruct);
}

static void HT_GPIO_InitLed(void) {
  GPIO_InitType GPIO_InitStruct = {0};

  GPIO_InitStruct.af = PAD_MuxAlt0;
  GPIO_InitStruct.pad_id = LED_PAD_ID;
  GPIO_InitStruct.gpio_pin = LED_GPIO_PIN;
  GPIO_InitStruct.pin_direction = GPIO_DirectionOutput;
  GPIO_InitStruct.init_output = 1;
  GPIO_InitStruct.pull = PAD_AutoPull;
  GPIO_InitStruct.instance = LED_INSTANCE;
  GPIO_InitStruct.exti = GPIO_EXTI_DISABLED;

  HT_GPIO_Init(&GPIO_InitStruct);

  GPIO_InitStruct.af = PAD_MuxAlt0;
  GPIO_InitStruct.pad_id = LED_PAD_ID2;
  GPIO_InitStruct.gpio_pin = LED_GPIO_PIN2;
  GPIO_InitStruct.pin_direction = GPIO_DirectionOutput;
  GPIO_InitStruct.init_output = 0;
  GPIO_InitStruct.pull = PAD_AutoPull;
  GPIO_InitStruct.instance = LED_INSTANCE1;
  GPIO_InitStruct.exti = GPIO_EXTI_DISABLED;

  HT_GPIO_Init(&GPIO_InitStruct);

  GPIO_InitStruct.af = PAD_MuxAlt0;
  GPIO_InitStruct.pad_id = LED_PAD_ID3;
  GPIO_InitStruct.gpio_pin = LED_GPIO_PIN3;
  GPIO_InitStruct.pin_direction = GPIO_DirectionOutput;
  GPIO_InitStruct.init_output = 1;
  GPIO_InitStruct.pull = PAD_AutoPull;
  GPIO_InitStruct.instance = LED_INSTANCE2;
  GPIO_InitStruct.exti = GPIO_EXTI_DISABLED;

  HT_GPIO_Init(&GPIO_InitStruct);
}


void Task1(void *pvParameters) {
    int button_value = 1;       // Assume que botão começa solto
    int button_value_last = 1;  // Último estado: solto

    while (1) {
        button_value = HT_GPIO_PinRead(BUTTON_INSTANCE, BUTTON_PIN);

        if (button_value == 0 && button_value_last == 1) {
            // Botão acabou de ser pressionado
            contador_ms = 0;
        }

        if (button_value == 0) {
            // Botão está pressionado
            contador_ms += 10;
        }

        if (button_value == 1 && button_value_last == 0) {
            // Botão acabou de ser solto, imprime e envia para fila
            printf("Botão pressionado por %lu ms\n", contador_ms);

            // Envia tempo para fila (não bloqueia)
            xQueueSend(xButtonQueue, &contador_ms, 0);
        }

        button_value_last = button_value;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void Task2(void *pvParameters) {
    uint32_t pressed_time_ms = 0;

    while (1) {
        if (xQueueReceive(xButtonQueue, &pressed_time_ms, portMAX_DELAY) == pdPASS) {

            HT_GPIO_WritePin(LED_GPIO_PIN, LED_INSTANCE, LED_ON ? LED_OFF : LED_ON); 

            // Mantém desligado enquanto o botão ficou pressionado
            vTaskDelay(pdMS_TO_TICKS(pressed_time_ms));

            // Liga o LED (liga nível baixo)
            HT_GPIO_WritePin(LED_GPIO_PIN, LED_INSTANCE, LED_ON);

            // Aguarda 500 ms com LED ligado
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}
void Task3_UART(void *pvParameters) {
    // inicialização UART1
    HAL_USART_Initialize(NULL, &huart1);
    HAL_USART_PowerControl(ARM_POWER_FULL, &huart1);
    HAL_USART_Control(
        ARM_USART_MODE_ASYNCHRONOUS |
        ARM_USART_DATA_BITS_8       |
        ARM_USART_PARITY_NONE       |
        ARM_USART_STOP_BITS_1       |
        ARM_USART_FLOW_CONTROL_NONE,
        115200, &huart1);

    // envia prompt corretamente
    const char *prompt = "\r\nDigite intervalo LED2 (ms) e \r\n";
    HAL_USART_SendPolling(&huart1,
                         (uint8_t*)prompt,
                         strlen(prompt));

    // limpa buffer de comando
    memset(cmdRxBuffer, 0, sizeof(cmdRxBuffer));
    cmdIdx = 0;

    while (1) {
        HAL_USART_ReceivePolling(&huart1, &rx_buffer, 1);
        if (rx_buffer) {
            // quando receber CR ou LF, processa
            if (rx_buffer == '\r' || rx_buffer == '\n') {
                if (cmdIdx > 0) {
                    cmdRxBuffer[cmdIdx] = '\0';
                    int novo = atoi(cmdRxBuffer);
                    if (novo > 0 && novo < 10000) {
                        blinkDelay = novo;
                        char ok[64];
                        int l = snprintf(ok, sizeof(ok),
                                         "Intervalo atualizado: %u ms\r\n",
                                         blinkDelay);
                        HAL_USART_SendPolling(&huart1,
                                             (uint8_t*)ok, l);
                    } else {
                        const char *err = "ERRO: valor inválido\r\n";
                        HAL_USART_SendPolling(&huart1,
                                             (uint8_t*)err,
                                             strlen(err));
                    }
                }
                // reseta índice mesmo que buffer vazio
                cmdIdx = 0;
                memset(cmdRxBuffer, 0, sizeof(cmdRxBuffer));
            }
            // se vier caractere normal, armazena
            else if (cmdIdx < sizeof(cmdRxBuffer) - 1) {
                cmdRxBuffer[cmdIdx++] = rx_buffer;
            }
            rx_buffer = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void Task4_LED2(void *pvParameters) {
    while (1) {
        HT_GPIO_WritePin(LED_GPIO_PIN2, LED_INSTANCE1, LED_OFF);
        vTaskDelay(pdMS_TO_TICKS(blinkDelay));
        HT_GPIO_WritePin(LED_GPIO_PIN2, LED_INSTANCE1, LED_ON);
        vTaskDelay(pdMS_TO_TICKS(blinkDelay));
    }
}
     void Task5_LDR(void *pvParameters) {
    while (1) {
        callback = 0;
        HT_ADC_StartConversion(DEMO_ADC_CHANNEL, ADC_UserAPP);

        while (callback != DEMO_ADC_CHANNEL) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        uint16_t adcValue = user_adc_channel;
        printf("ADC Value: %d\n", adcValue);

        if (adcValue > 200) {
            HT_GPIO_WritePin(LED_GPIO_PIN3, LED_INSTANCE, LED_OFF);
        } else {
            HT_GPIO_WritePin(LED_GPIO_PIN3, LED_INSTANCE, LED_ON);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void main_entry(void) {
    HT_GPIO_InitButton();
    HT_GPIO_InitLed();

    BSP_CommonInit();

    HT_GPIO_InitButton();                     
    HT_ADC_Init(DEMO_ADC_CHANNEL);  
    HAL_USART_InitPrint(&huart1, GPR_UART1ClkSel_26M, uart_cntrl, 115200);
    printf("FUNFOU \n");


    slpManNormalIOVoltSet(IOVOLT_3_30V);

    #ifdef USE_USART_DEBUG
    HAL_USART_InitPrint(&huart1, GPR_UART1ClkSel_26M,
        ARM_USART_MODE_ASYNCHRONOUS|ARM_USART_DATA_BITS_8|
        ARM_USART_PARITY_NONE|ARM_USART_STOP_BITS_1,
        115200);
    HAL_USART_SendPolling(&huart1,(uint8_t*)"Sistema iniciado\r\n",16);
    #endif
    
    xButtonQueue=xQueueCreate(BUTTON_QUEUE_LENGTH,sizeof(uint32_t));
    xTaskCreate(Task1,"LeBotao",128,NULL,1,NULL);
    xTaskCreate(Task2,"ControleLED1",128,NULL,1,NULL);
    xTaskCreate(Task3_UART,"UART",256,NULL,1,NULL);
    xTaskCreate(Task4_LED2,"LED2Blink",128,NULL,1,NULL);
    xTaskCreate(Task5_LDR, "LDR", 128, NULL, 1, NULL);
    vTaskStartScheduler();
    while(1);
}