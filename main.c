#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "lib/ws2818b.h"
#include "ws2818b.pio.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Definição de macros para o protocolo I2C (SSD1306)
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define SSD1306_ADDRESS 0x3C

#define BTN_B_PIN 6
#define BTN_A_PIN 5
#define BUZZER_PIN 21

#define LED_RED 13
#define LED_GREEN 11
#define LED_BLUE 12

#define JOYSTICK_Y 27 // Representa o volume da chuva
#define JOYSTICK_X 26 // Representa o nível da água

// Define variáveis para debounce dos botões A e B
volatile uint32_t last_time_btn_press = 0;
const uint32_t debounce_delay_ms = 260;

// pwm
uint32_t clock   = 125000000;
uint32_t divider = 0;
uint32_t wrap    = 0;
uint slice_num   = 0;
uint channel_num = 0;

typedef struct {
    uint16_t water_level; // Eixo X (GPIO 26)
    uint16_t rain_volume; // Eixo Y (GPIO 27)
} sensor_data_t;

// Cria a fila para os dados lidos pelo potenciômetro do joystick
QueueHandle_t xQueueSensorData;

// Realiza a inicialização dos botões
void btn_setup(uint gpio);

// Realiza a inicialização do ADC
void adc_setup();

// Realiza a inicialização dos LEDs RGB
void led_rgb_setup(uint gpio);

// Realiza a inicialização do protocolo I2C para comunicação com o display OLED
void i2c_setup(uint baud_in_kilo);

// Cálculo dos paramêtros do PWM para buzzer emitir frequência especificada
void pwm_set_frequency(float frequency);

// Realiza a inicialização do display OLED
void ssd1306_setup(ssd1306_t *ssd_ptr);

// Implementa a tarefa do joystick
void vSensorTask();

// Implementa a tarefa do display OLED
void vDisplayTask();

// Implementa a tarefa do buzzer
void vBuzzerTask();

// Implementa a tarefa dos LEDs
void vLedMatrixTask();

// Implementa a tarefa dos LEDs RGB
void vLEDsRGBTask();

int main() {
    stdio_init_all();

    // Inicilização dos botões
    // btn_setup(BTN_B_PIN);
    // btn_setup(BTN_A_PIN);

    // Inicilização das interrupções para os botões
    // gpio_set_irq_enabled_with_callback(BTN_B_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    // gpio_set_irq_enabled(BTN_A_PIN, GPIO_IRQ_EDGE_FALL, true);

    // Criação das filas para compartilhamento de dados entre as tarefas
    xQueueSensorData = xQueueCreate(5, sizeof(sensor_data_t));

    // Verifica se as filas foram criadas
    if (xQueueSensorData != NULL) {
        // Criação das tarefas
        xTaskCreate(vSensorTask, "Task: Sensores", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
        xTaskCreate(vDisplayTask, "Task: Display", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY, NULL);
        xTaskCreate(vLedMatrixTask, "Task: LEDs Matriz", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
        xTaskCreate(vBuzzerTask, "Task: Buzzer", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
        xTaskCreate(vLEDsRGBTask, "Task: LEDs RGB", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);

        // Chamda do Scheduller de tarefas
        vTaskStartScheduler();
    }
    panic_unsupported();
}

// Realiza a inicialização dos botões
void btn_setup(uint gpio) {
  gpio_init(gpio);
  gpio_set_dir(gpio, GPIO_IN);
  gpio_pull_up(gpio);
}

// Realiza a inicialização do ADC
void adc_setup() {
    adc_gpio_init(JOYSTICK_Y);
    adc_gpio_init(JOYSTICK_X);
    adc_init();
}

// Realiza a inicialização dos LEDs RGB
void led_rgb_setup(uint gpio) {
  gpio_init(gpio);
  gpio_set_dir(gpio, GPIO_OUT);
}

// Realiza a inicialização do protocolo I2C para comunicação com o display OLED
void i2c_setup(uint baud_in_kilo) {
  i2c_init(I2C_PORT, baud_in_kilo * 1000);

  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA);
  gpio_pull_up(I2C_SCL);
}

// Cálculo dos paramêtros do PWM para buzzer emitir frequência especificada
void pwm_set_frequency(float frequency) {
  // Se frequência for menor que zero não executa nada
  if (frequency <= 0.0f) {
    pwm_set_enabled(slice_num, false);
    return;
  }

  // Calcula os valores para o divisor e para o wrap
  divider = clock / (uint32_t)(frequency * 1000);
  wrap    = clock / (divider * (uint32_t)frequency) - 1;

  // Aplica as configurações calculados
  pwm_set_clkdiv_int_frac(slice_num, divider, 0);
  pwm_set_wrap(slice_num, wrap);
  pwm_set_chan_level(slice_num, channel_num, wrap / 2); // Define o Duty cycle de 50%
}

// Realiza a inicialização do display OLED
void ssd1306_setup(ssd1306_t *ssd_ptr) {
  ssd1306_init(ssd_ptr, WIDTH, HEIGHT, false, SSD1306_ADDRESS, I2C_PORT); // Inicializa o display
  ssd1306_config(ssd_ptr);                                                // Configura o display
  ssd1306_send_data(ssd_ptr);                                             // Envia os dados para o display

  // Limpa o display. O display inicia com todos os pixels apagados.
  ssd1306_fill(ssd_ptr, false);
  ssd1306_send_data(ssd_ptr);
}

// Implementa a tarefa do joystick
void vSensorTask() {
    // Inicializa o ADC
    adc_setup();

    // Cria variável para armazenar os dados brutos coletados pelos sensores
    sensor_data_t sensor_data;

    while (true) {
        // Seleciona o canal referente ao eixo X
        adc_select_input(1);
        sensor_data.water_level = (adc_read() * 100) / 4095;

        // Seleciona o canal referente ao eixo Y
        adc_select_input(0);
        sensor_data.rain_volume = (adc_read() * 100) / 4095;

        // Envia o valor do joystick para a fila
        xQueueSend(xQueueSensorData, &sensor_data, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Implementa a tarefa do display OLED
void vDisplayTask() {
    // Inicialização do protocolo I2C com 400Khz
    i2c_setup(400);

    // Inicializa a estrutura do display
    ssd1306_t ssd;
    ssd1306_setup(&ssd);

    // Variáveis para receber os dados das filas
    sensor_data_t sensor_data;
    bool is_alert_mode = true;

    // Referente ao estado display
    bool color = true;
    char buffer[100];

    // Realiza a limpeza do display
    ssd1306_fill(&ssd, !color);

    while (true) {
        // Realiza a limpeza do display e define o layout do display OLED
        ssd1306_fill(&ssd, !color);
        ssd1306_rect(&ssd, 3, 3, 122, 60, color, !color);
        ssd1306_line(&ssd, 3, 25, 123, 25, color);
        ssd1306_draw_string(&ssd, "Estacao", 38, 6);
        ssd1306_draw_string(&ssd, "De Alerta", 31, 14);

        // Verifica se existe algum dado na fila dos sensores. Ele espera um tempo máximo (portMAX_DELAY)
        if (xQueueReceive(xQueueSensorData, &sensor_data, portMAX_DELAY) == pdTRUE) {
            // Exibição no terminal serial
            printf("NIVEL AGUA: %u%%  VOL. CHUVA: %u%%\n", sensor_data.water_level, sensor_data.rain_volume);

            // Verifica se é modo de alerta ou modo normal
            is_alert_mode = sensor_data.rain_volume >= 80 || sensor_data.water_level >= 70;

            if (is_alert_mode) {
                // Monta string para o display
                snprintf(buffer, sizeof(buffer), "NIV AGUA: %u%%", sensor_data.water_level);
                ssd1306_draw_string(&ssd, buffer, 6, 28);

                // Monta string para o display
                snprintf(buffer, sizeof(buffer), "VOL CHUVA: %u%%", sensor_data.rain_volume);
                ssd1306_draw_string(&ssd, buffer, 6, 36);

                ssd1306_draw_string(&ssd, "PROTEJA-SE!", 6, 50);
            }

            if (!is_alert_mode) {
                // Monta string para o display
                snprintf(buffer, sizeof(buffer), "NIV AGUA: %u%%", sensor_data.water_level);
                ssd1306_draw_string(&ssd, buffer, 6, 28);

                // Monta string para o display
                snprintf(buffer, sizeof(buffer), "VOL CHUVA: %u%%", sensor_data.rain_volume);
                ssd1306_draw_string(&ssd, buffer, 6, 36);
            }
        }

        ssd1306_send_data(&ssd);
        vTaskDelay(pdMS_TO_TICKS(60));
    }
}

// Implementa a tarefa do buzzer
void vBuzzerTask() {
  // Configura o pino do buzzer para PWM e obtém as infos do pino
  gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
  slice_num   = pwm_gpio_to_slice_num(BUZZER_PIN);
  channel_num = pwm_gpio_to_channel(BUZZER_PIN);

  // Configuração inicial do PWM
  pwm_config config = pwm_get_default_config();
  pwm_init(slice_num, &config, true);

  // Desliga PWM do pino ligado ao buzzer
  pwm_set_enabled(slice_num, false);

  while (true) {
  }
}

// Implementa a tarefa dos LEDs
void vLedMatrixTask() {
  // Iniciliza a matriz de LEDs, faz a limpeza do buffer e envia para a matriz
  npInit(LED_PIN);
  npClear();
  npWrite();

  while (true) {
  }
}

// Implementa a tarefa dos LEDs RGB (semaforo)
void vLEDsRGBTask() {
    // Iniciliza os LEDs
    led_rgb_setup(LED_RED);
    led_rgb_setup(LED_GREEN);
    led_rgb_setup(LED_BLUE);

    bool is_alert_mode = true;
    sensor_data_t sensor_data;

    while (true) {
        // Verifica se existe algum dado na fila dos sensores. Ele espera um tempo máximo (portMAX_DELAY). Caso exi
        if (xQueueReceive(xQueueSensorData, &sensor_data, portMAX_DELAY) == pdTRUE) {
            // Verifica se é modo de alerta ou modo normal
            is_alert_mode = sensor_data.rain_volume >= 80 || sensor_data.water_level >= 70;

            // Caso esteja em modo de alerta, pisca o LED VERMELHO
            if (is_alert_mode) {
                gpio_put(LED_RED, 1);
                gpio_put(LED_GREEN, 0);
                gpio_put(LED_BLUE, 0);
                vTaskDelay(pdMS_TO_TICKS(500));
                gpio_put(LED_RED, 0);
                gpio_put(LED_GREEN, 0);
                gpio_put(LED_BLUE, 0);
                vTaskDelay(pdMS_TO_TICKS(500));
            }

            // Caso esteja em no modo normal, pisca o LED VERDE
            if (!is_alert_mode) {
                gpio_put(LED_RED, 0);
                gpio_put(LED_GREEN, 1);
                gpio_put(LED_BLUE, 0);
                vTaskDelay(pdMS_TO_TICKS(500));
                gpio_put(LED_RED, 0);
                gpio_put(LED_GREEN, 0);
                gpio_put(LED_BLUE, 0);
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }
    }
}
