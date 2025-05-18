#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
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

#define JOYSTICK_Y 27
#define JOYSTICK_X 26

// Define variáveis para debounce dos botões A e B
volatile uint32_t last_time_btn_press = 0;
const uint32_t debounce_delay_ms = 260;

// define variável global para trocar modo de operação (NORMAL e NOTURNO)
volatile bool is_night_mode = false;

// pwm
uint32_t clock   = 125000000;
uint32_t divider = 0;
uint32_t wrap    = 0;
uint slice_num   = 0;
uint channel_num = 0;

typedef struct {
    uint16_t x_pos;
    uint16_t y_pos;
} joystick_data_t;

QueueHandle_t xQueueJoystickSensor;

// Realiza a inicialização dos botões
void btn_setup(uint gpio);

// Realiza a inicialização dos LEDs RGB
void led_rgb_setup(uint gpio);

// Realiza a inicialização do protocolo I2C para comunicação com o display OLED
void i2c_setup(uint baud_in_kilo);

// Cálculo dos paramêtros do PWM para buzzer emitir frequência especificada
void pwm_set_frequency(float frequency);

// Realiza a inicialização do display OLED
void ssd1306_setup(ssd1306_t *ssd_ptr);

// Implementa a tarefa do joystick
void vJoystickTask();

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
    btn_setup(BTN_B_PIN);
    btn_setup(BTN_A_PIN);

    // Inicilização das interrupções para os botões
    gpio_set_irq_enabled_with_callback(BTN_B_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled(BTN_A_PIN, GPIO_IRQ_EDGE_FALL, true);

    // Criação das tarefas
    xTaskCreate(vJoystickTask, "Task: Joystick", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vDisplayTask, "Task: Display", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vLedMatrixTask, "Task: LEDs Matriz", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vBuzzerTask, "Task: Buzzer", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vLEDsRGBTask, "Task: LEDs RGB", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);

    // Chamda do Scheduller de tarefas
    vTaskStartScheduler();
    panic_unsupported();
}

// Realiza a inicialização dos botões
void btn_setup(uint gpio) {
  gpio_init(gpio);
  gpio_set_dir(gpio, GPIO_IN);
  gpio_pull_up(gpio);
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
void vJoystickTask() {
    
}

// Implementa a tarefa do display OLED
void vDisplayTask() {
  // Inicialização do protocolo I2C com 400Khz
  i2c_setup(400);

  // Inicializa a estrutura do display
  ssd1306_t ssd;
  ssd1306_setup(&ssd);

  bool color = true;

  while (true) {
    // Realiza a limpeza do display e define o layout do display OLED
    ssd1306_fill(&ssd, !color);
    ssd1306_rect(&ssd, 3, 3, 122, 60, color, !color);
    ssd1306_line(&ssd, 3, 15, 123, 15, color);
    ssd1306_line(&ssd, 3, 27, 123, 27, color);
    ssd1306_draw_string(&ssd, "SEMAFORO", 32, 6);

    // Envia os dados armazenados no buffer para o display OLED
    ssd1306_send_data(&ssd);
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

  const TickType_t step = pdMS_TO_TICKS(STEP_MS);
  const uint stepsPer = 2000 / STEP_MS;

  while (true) {
  }
}

// Implementa a tarefa dos LEDs RGB (semaforo)
void vLEDsRGBTask() {
  // Iniciliza os LEDs
  led_rgb_setup(LED_RED);
  led_rgb_setup(LED_GREEN);
  led_rgb_setup(LED_BLUE);

  while (true) {
  }
}
