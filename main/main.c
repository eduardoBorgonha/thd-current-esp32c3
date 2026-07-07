/*
  EEL7030 - Microprocessadores
  Projeto: Calculadora de Grandezas Elétricas - ESP32-C3
  
  Descrição: 
  Firmware para aquisição e processamento digital de sinais de corrente elétrica.
  Implementa um sistema de tempo real (FreeRTOS) para amostragem determinística
  (Timer via Interrupção), cálculo de Componente DC, Valor RMS (True RMS) e 
  análise espectral via Transformada Rápida de Fourier (FFT Radix-2) para 
  cálculo da Distorção Harmônica Total (THD).
  
  Interface: Display LCD 16x2 via protocolo I2C e LEDs de sinalização de estado.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"

// ============================================================================
// DEFINIÇÕES DE HARDWARE E PINAGEM
// ============================================================================
#define BOTAO_PIN 10
#define LED_HEARTBEAT_PIN 8      // Sinalização de atividade do RTOS
#define LED_ALERTA_PIN 3         // Alerta visual de alta distorção harmônica

#define TEMPO_DEBOUNCE_US 250000 // 250 ms de tolerância mecânica para o botão

#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define LCD_ENDERECO 0x27        // Endereço padrão do expansor PCF8574 (I2C)

#define ADC_PINO_CANAL ADC_CHANNEL_0 // Relativo ao GPIO 0 no ESP32-C3

// ============================================================================
// PARÂMETROS DE AQUISIÇÃO E INSTRUMENTAÇÃO
// ============================================================================
#define N_AMOSTRAS 512           // Janela temporal: 4 ciclos da rede (60 Hz)
#define ACS712_SENSIBILIDADE_V_POR_A 0.185f // Fator de sensibilidade do ACS712 (185mV/A)

// Condicionamento de Sinal:
#define FATOR_DIVISOR_TENSAO 1.551f   // Atenuação resistiva de proteção do ADC
#define OFFSET_TENSAO_PINO_ZERO 1.85f // Calibração empírica do ponto de 0A

#define THD_LIMITE_ALERTA_PCT 20.0f   // Limite percentual para acionamento do led

#define PI 3.14159265358979323846

// ============================================================================
// VARIÁVEIS GLOBAIS E RECURSOS COMPARTILHADOS
// ============================================================================
volatile uint8_t flag_tela = 0; // Qualificador 'volatile' previne otimização na ISR
i2c_master_dev_handle_t lcd_handle;

// Memória de Transição (Protegida por Semáforos)
static int raw_adc[N_AMOSTRAS];

// Resultados Matemáticos Consolidados
static float dc_resultado = 0.0f;
static float rms_resultado = 0.0f;
static float thd_resultado = 0.0f;
static int freq_harmonica_sig = 0;
static float amp_harmonica_sig = 0.0f;

// Handles de Drivers do ESP-IDF
gptimer_handle_t timer_handle;
adc_oneshot_unit_handle_t adc_handle;
TaskHandle_t handle_amostragem;

// Semáforos binários para controle de fluxo sequencial e exclusão mútua (Token Ring)
SemaphoreHandle_t sem_processamento;
SemaphoreHandle_t sem_exibicao;
SemaphoreHandle_t sem_inicio_ciclo;

// ============================================================================
// PROCESSAMENTO DIGITAL DE SINAIS (DSP)
// ============================================================================

// Função auxiliar para ordenação dos vetores
void swap(float *a, float *b) {
    float temp = *a;
    *a = *b;
    *b = temp;
}

// Implementação in-place da Transformada Rápida de Fourier (Cooley-Tukey Radix-2)
// Otimizada em C nativo para adequação à arquitetura RISC-V sem FPU de hardware.
void calcular_fft(float *vReal, float *vImag, uint16_t amostras) {
    uint16_t i, j, k, n1, n2, a;
    float c, s, t1, t2;

    // Etapa 1: Reordenação Bit-Reversal
    j = 0;
    n2 = amostras / 2;
    for (i = 1; i < amostras - 1; i++) {
        n1 = n2;
        while (j >= n1) { j = j - n1; n1 = n1 / 2; }
        j = j + n1;
        if (i < j) {
            swap(&vReal[i], &vReal[j]);
            swap(&vImag[i], &vImag[j]);
        }
    }

    // Etapa 2: Borboletas (Butterfly Computation)
    n1 = 0;
    n2 = 1;
    for (i = 0; i < 9; i++) { // log2(512) = 9 estágios
        n1 = n2;
        n2 = n2 + n2;
        a = 0;
        for (j = 0; j < n1; j++) {
            c = cosf(-2.0f * PI * a / amostras);
            s = sinf(-2.0f * PI * a / amostras);
            a += 1 << (9 - i - 1);
            for (k = j; k < amostras; k = k + n2) {
                t1 = c * vReal[k + n1] - s * vImag[k + n1];
                t2 = s * vReal[k + n1] + c * vImag[k + n1];
                vReal[k + n1] = vReal[k] - t1;
                vImag[k + n1] = vImag[k] - t2;
                vReal[k] = vReal[k] + t1;
                vImag[k] = vImag[k] + t2;
            }
        }
    }
}

// ============================================================================
// ROTINAS DE SERVIÇO DE INTERRUPÇÃO (ISRs)
// ============================================================================

// Interrupção externa (Botão). Implementa Debounce Lógico via Software.
static void IRAM_ATTR gpio_isr_botao(void *arg) {
    static uint64_t ultimo_tempo_isr = 0;
    uint64_t tempo_atual = esp_timer_get_time();
    // Rejeita transientes (bouncing) inferiores a 250ms
    if ((tempo_atual - ultimo_tempo_isr) > TEMPO_DEBOUNCE_US) {
        flag_tela = !flag_tela;
        ultimo_tempo_isr = tempo_atual;
    }
}

// Alarme do Temporizador (130 us). 
// NOTA ARQUITETURAL: Não realiza a leitura do ADC internamente para evitar travamentos.
// Emite uma notificação rápida para a *Task* de Amostragem assumir o contexto.
static bool IRAM_ATTR callback_alarme(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    BaseType_t acordou = pdFALSE;
    vTaskNotifyGiveFromISR(handle_amostragem, &acordou);
    portYIELD_FROM_ISR(acordou); // Solicita preempção imediata do Scheduler
    return true;
}

// ============================================================================
// DRIVER I2C E INTERFACE HOMEM-MÁQUINA (LCD)
// ============================================================================

// Empacota e transmite instruções I2C para o expansor de 8-bits PCF8574
// Respeita a multiplexação de 4-bits exigida pelo controlador HD44780
void lcd_send(uint8_t valor, uint8_t modo) {
    uint8_t nibble_alto = valor & 0xF0;
    uint8_t nibble_baixo = (valor << 4) & 0xF0;
    uint8_t dados[4];
    uint8_t controle = modo | 0x08; // 0x08 garante o Backlight ligado
    
    // Geração do pulso de Enable (bit 0x04)
    dados[0] = nibble_alto | controle | 0x04;
    dados[1] = nibble_alto | controle;
    dados[2] = nibble_baixo | controle | 0x04;
    dados[3] = nibble_baixo | controle;
    
    i2c_master_transmit(lcd_handle, dados, 4, -1);
    usleep(2000); // Respeita tempo de latência do display
}

void lcd_cmd(uint8_t cmd)   { lcd_send(cmd, 0); } // Transmite Comando Lógico
void lcd_data(uint8_t dado) { lcd_send(dado, 1); } // Transmite Caractere ASCII

// Centraliza a escrita e preenche lacunas residuais para limpar o buffer da tela
void lcd_escreve_linha(int linha, const char *str) {
    if (linha == 0) lcd_cmd(0x80); else lcd_cmd(0xC0);
    int i = 0;
    while (i < 16 && str[i] != '\0') { lcd_data(str[i]); i++; }
    while (i < 16) { lcd_data(' '); i++; }
}

// ============================================================================
// ROTINAS DE CONFIGURAÇÃO DE HARDWARE
// ============================================================================

void configura_leds(void) {
    uint64_t mask = (1ULL << LED_HEARTBEAT_PIN) | (1ULL << LED_ALERTA_PIN);
    gpio_config_t io_conf = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_HEARTBEAT_PIN, 0);
    gpio_set_level(LED_ALERTA_PIN, 0);
}

void configura_botao(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOTAO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, // Pull-up interno ativado
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE   // Interrupção na borda de descida
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(BOTAO_PIN, gpio_isr_botao, NULL);
}

void configura_i2c_lcd(void) {
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_SCL_PIN,
        .sda_io_num = I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    i2c_new_master_bus(&i2c_bus_config, &bus_handle);

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LCD_ENDERECO,
        .scl_speed_hz = 100000,
    };
    i2c_master_bus_add_device(bus_handle, &dev_cfg, &lcd_handle);

    usleep(50000);
    // Sequência mandatória de inicialização do controlador HD44780
    lcd_cmd(0x33); lcd_cmd(0x32); lcd_cmd(0x28); lcd_cmd(0x0C); lcd_cmd(0x01);
    usleep(50000);
}

void configura_adc(void) {
    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&init_config, &adc_handle);
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12 // Permite leitura da escala total 0 ~ 3.3V
    };
    adc_oneshot_config_channel(adc_handle, ADC_PINO_CANAL, &config);
}

void configura_timer(void) {
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000 // Base temporal de 1 MHz (Tick = 1 us)
    };
    gptimer_new_timer(&timer_config, &timer_handle);

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 130,      // Taxa de Amostragem (Fs) = ~7680 Hz
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true
    };
    gptimer_set_alarm_action(timer_handle, &alarm_config);

    gptimer_event_callbacks_t cbs = { .on_alarm = callback_alarme };
    gptimer_register_event_callbacks(timer_handle, &cbs, NULL);
}

// ============================================================================
// TAREFAS DO SISTEMA (FREERTOS)
// ============================================================================

// Tarefa de Prioridade Máxima: Gerencia exclusivamente as restrições de tempo real
void vTarefaAmostragem(void *pvParameters) {
    while (1) {
        // Aguarda liberação do ciclo
        xSemaphoreTake(sem_inicio_ciclo, portMAX_DELAY);
        
        gptimer_enable(timer_handle);
        gptimer_start(timer_handle);

        for (int i = 0; i < N_AMOSTRAS; i++) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Sincroniza com a ISR do Timer
            adc_oneshot_read(adc_handle, ADC_PINO_CANAL, &raw_adc[i]);
        }

        gptimer_stop(timer_handle);
        gptimer_disable(timer_handle);
        
        // Finaliza aquisição e autoriza início do processamento matemático
        xSemaphoreGive(sem_processamento);
    }
}

// Tarefa de Processamento (DSP): Executa todo o escopo matemático da aplicação
void vTarefaProcessamento(void *pvParameters) {
    // Alocação dinâmica mitigatória para prevenção de Stack Overflow 
    // das matrizes necessárias à Transformada de Fourier
    float *vReal = malloc(N_AMOSTRAS * sizeof(float));
    float *vImag = malloc(N_AMOSTRAS * sizeof(float));
    float corrente_array[N_AMOSTRAS]; 

    while (1) {
        // Aguarda integridade do vetor de amostragem
        xSemaphoreTake(sem_processamento, portMAX_DELAY);

        float soma_corrente = 0.0f;
        float soma_quadrados_true_rms = 0.0f;

        // 1. CONDICIONAMENTO NO DOMÍNIO DO TEMPO
        for (int i = 0; i < N_AMOSTRAS; i++) {
            // Conversão da escala quantizada para grandeza analógica
            float tensao_pino = ((float)raw_adc[i] / 4095.0f) * 3.3f;
            
            // Subtração de Offset no domínio correto para anular erros de calibração
            float tensao_pino_sem_dc = tensao_pino - OFFSET_TENSAO_PINO_ZERO;
            
            // Projeção da tensão na entrada do divisor resistivo
            float tensao_sensor_sem_dc = tensao_pino_sem_dc * FATOR_DIVISOR_TENSAO;
            
            // Aplicação da lei característica do sensor ACS712
            float amostra_corrente = tensao_sensor_sem_dc / ACS712_SENSIBILIDADE_V_POR_A;
            
            corrente_array[i] = amostra_corrente; 
            soma_corrente += amostra_corrente;
            soma_quadrados_true_rms += (amostra_corrente * amostra_corrente); 
        }

        // Extração da Componente Contínua (Nível DC)
        dc_resultado = soma_corrente / N_AMOSTRAS;
        
        // Extração do Valor Eficaz Total (True RMS)
        float rms_bruto = sqrtf(soma_quadrados_true_rms / N_AMOSTRAS);

        // Filtro de Zona Morta (Deadband): Zera correntes fantasmas derivadas de 
        // acúmulo de variância do ruído branco de instrumentação
        if (rms_bruto < 0.1f) {
            dc_resultado = 0.0f;
            rms_resultado = 0.0f;
        } else {
            rms_resultado = rms_bruto;
        }

        // 2. CONDICIONAMENTO NO DOMÍNIO DA FREQUÊNCIA (FFT e THD)
        // Evita execuções pesadas e divisão por zero quando a carga está inativa
        if (rms_resultado > 0.1f) {
            for (int i = 0; i < N_AMOSTRAS; i++) {
                // Remoção do offset DC residual pré-FFT para evitar Vazamento Espectral (Spectral Leakage)
                float ac_puro = corrente_array[i] - dc_resultado; 
                
                // Aplicação da Janela de Hann (Mitigação de descontinuidades nas bordas da amostra)
                float hann = 0.5f * (1.0f - cosf(2.0f * PI * i / (N_AMOSTRAS - 1.0f)));
                vReal[i] = ac_puro * hann;
                vImag[i] = 0.0f;
            }

            calcular_fft(vReal, vImag, N_AMOSTRAS);

            // Resolução = Fs / N = 7680 / 512 = 15 Hz por Bin
            float magnitude[65]; // Avaliação restrita aos 15 primeiros harmônicos (Bin 60 = 900 Hz)
            for(int i = 0; i <= 60; i++) {
                magnitude[i] = sqrtf(vReal[i] * vReal[i] + vImag[i] * vImag[i]);
            }

            // Mapeamento da Frequência Fundamental (60 Hz -> Bin 4)
            float fundamental = magnitude[4];
            
            float soma_harmonica_quadrados = 0.0f;
            float max_harm_mag = 0.0f;
            int max_harm_h = 0;

            // Extração de componentes harmônicos pares e ímpares (2º ao 15º)
            if (fundamental > 0.001f) {
                for (int h = 2; h <= 15; h++) {
                    int bin = h * 4;
                    float mag = magnitude[bin];
                    soma_harmonica_quadrados += (mag * mag);

                    // Captura do Harmônico mais ruidoso (Diagnóstico de Carga)
                    if (mag > max_harm_mag) {
                        max_harm_mag = mag;
                        max_harm_h = h;
                    }
                }

                // Cálculo Final do Perfil de Distorção (THD)
                float v_harmonica = sqrtf(soma_harmonica_quadrados);
                thd_resultado = (v_harmonica / fundamental) * 100.0f;
                amp_harmonica_sig = (max_harm_mag / fundamental) * 100.0f;
                freq_harmonica_sig = max_harm_h * 60;
            } else {
                thd_resultado = 0.0f;
                amp_harmonica_sig = 0.0f;
                freq_harmonica_sig = 0;
            }
        } else {
            // Garante inicialização segura de variáveis relativas à FFT em repouso
            thd_resultado = 0.0f;
            amp_harmonica_sig = 0.0f;
            freq_harmonica_sig = 0;
        }

        // Autoriza atualização da Interface Gráfica
        xSemaphoreGive(sem_exibicao);
    }
}

// Tarefa de Prioridade Mínima: Executa formatações e interface I/O Lenta (I2C)
void vTarefaExibicao(void *pvParameters) {
    xSemaphoreGive(sem_inicio_ciclo); // Inicializa a cadeia de processamento

    while (1) {
        xSemaphoreTake(sem_exibicao, portMAX_DELAY);

        // Atualização Visual - Heartbeat prova a vivacidade do Scheduler (RTOS)
        static uint8_t led_estado = 0;
        led_estado = !led_estado;
        gpio_set_level(LED_HEARTBEAT_PIN, led_estado);

        // Lógica Atuadora Condicional (Alarme Físico)
        if (thd_resultado >= THD_LIMITE_ALERTA_PCT) {
            gpio_set_level(LED_ALERTA_PIN, 1); 
        } else {
            gpio_set_level(LED_ALERTA_PIN, 0); 
        }

        // Construção semântica e transmissão ao LCD 16x2
        char buffer[17];
        if (flag_tela == 0) {
            snprintf(buffer, sizeof(buffer), "Nivel DC: %4.2fA", dc_resultado);
            lcd_escreve_linha(0, buffer);
            
            snprintf(buffer, sizeof(buffer), "RMS AC:   %4.2fA", rms_resultado);
            lcd_escreve_linha(1, buffer);
        } else {
            snprintf(buffer, sizeof(buffer), "THD: %5.1f %%", thd_resultado);
            lcd_escreve_linha(0, buffer);
            
            snprintf(buffer, sizeof(buffer), "Max: %dHz %.1f%%", freq_harmonica_sig, amp_harmonica_sig);
            lcd_escreve_linha(1, buffer);
        }

        // Determina a taxa de atualização natural (Taxa de *Refresh* Visível)
        vTaskDelay(pdMS_TO_TICKS(500));  
        
        // Devolve o controle de contexto para reiniciar um novo ciclo temporal
        xSemaphoreGive(sem_inicio_ciclo);
    }
}

// ============================================================================
// PONTO DE ENTRADA DO SISTEMA (Setup)
// ============================================================================
void app_main(void)
{
    // Desativado preventivamente para não acionar resets de tempo de guarda
    esp_task_wdt_deinit();

    // Inicialização do Hardware
    configura_leds();
    configura_adc();
    configura_timer();
    configura_i2c_lcd();
    configura_botao();

    // Alocação Primitiva RTOS: Máquina de estados baseada em Semáforos
    sem_processamento = xSemaphoreCreateBinary();
    sem_exibicao      = xSemaphoreCreateBinary();
    sem_inicio_ciclo  = xSemaphoreCreateBinary();

    // Despacho das Tarefas ao Escalonador (Atribuição explícita de Prioridades)
    xTaskCreate(vTarefaAmostragem,   "Amostragem",    4096, NULL, tskIDLE_PRIORITY + 3, &handle_amostragem);
    // Prioridade e Stack elevadas em decorrência das exigências da FFT (Alocação, Math.h)
    xTaskCreate(vTarefaProcessamento,"Processamento", 8192, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(vTarefaExibicao,     "Exibicao",      4096, NULL, tskIDLE_PRIORITY + 1, NULL);
}