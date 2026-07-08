/*
  EEL7035 - Microprocessors e & Microcontrollers 
  Universidade Federal de Santa Catarina (UFSC)
  Project: Current Quantities Calculator - ESP32-C3
  Authors: Eduardo and Mateus
  
  Description: 
  Firmware for the acquisition and digital processing of electrical current signals.
  Implements a real-time system using FreeRTOS for deterministic sampling
  (Hardware Timer via Interrupt), calculation of the DC Component, True RMS value, and 
  spectral analysis via Fast Fourier Transform (Cooley-Tukey Radix-2 FFT) to 
  compute the Total Harmonic Distortion of Current (THD-i).
  
  Interface: 16x2 LCD Display via I2C protocol and status indicator LEDs.
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
// HARDWARE DEFINITIONS AND PINOUT
// ============================================================================
#define BUTTON_PIN 10
#define LED_HEARTBEAT_PIN 8      // RTOS activity signaling
#define LED_ALERT_PIN 3          // Visual alert for high harmonic distortion

#define DEBOUNCE_TIME_US 250000  // 250 ms mechanical tolerance for the push-button

#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define LCD_ADDRESS 0x27         // Default address for the PCF8574 I2C expander

#define ADC_CHANNEL_PIN ADC_CHANNEL_0 // Relative to GPIO 0 on ESP32-C3

// ============================================================================
// ACQUISITION AND INSTRUMENTATION PARAMETERS
// ============================================================================
#define NUM_SAMPLES 512          // Time window: 4 full grid cycles (60 Hz)
#define ACS712_SENSITIVITY_V_PER_A 0.185f // ACS712 sensitivity factor (185mV/A)

// Signal Conditioning:
#define VOLTAGE_DIVIDER_FACTOR 1.551f   // Resistive attenuation to protect the ADC
#define ZERO_A_PIN_VOLTAGE_OFFSET 1.85f // Empirical calibration for the 0A point

#define THD_ALERT_LIMIT_PCT 20.0f       // Percentage threshold to trigger the alert LED

#define PI 3.14159265358979323846

// ============================================================================
// GLOBAL VARIABLES AND SHARED RESOURCES
// ============================================================================
volatile uint8_t screen_flag = 0; // 'volatile' prevents compiler optimization in the ISR
i2c_master_dev_handle_t lcd_handle;

// Transition Memory (Protected by Semaphores)
static int raw_adc[NUM_SAMPLES];

// Consolidated Mathematical Results
static float dc_result = 0.0f;
static float rms_result = 0.0f;
static float thd_result = 0.0f;
static int sig_harmonic_freq = 0;
static float sig_harmonic_amp = 0.0f;

// ESP-IDF Driver Handles
gptimer_handle_t timer_handle;
adc_oneshot_unit_handle_t adc_handle;
TaskHandle_t sampling_handle;

// Binary semaphores for sequential flow control and mutual exclusion (Token Ring pattern)
SemaphoreHandle_t sem_processing;
SemaphoreHandle_t sem_display;
SemaphoreHandle_t sem_cycle_start;

// ============================================================================
// DIGITAL SIGNAL PROCESSING (DSP)
// ============================================================================

// Helper function to swap array elements
void swap(float *a, float *b) {
    float temp = *a;
    *a = *b;
    *b = temp;
}

// In-place Fast Fourier Transform (Cooley-Tukey Radix-2)
// Optimized in native C to suit the RISC-V architecture without hardware FPU.
void calculate_fft(float *vReal, float *vImag, uint16_t samples) {
    uint16_t i, j, k, n1, n2, a;
    float c, s, t1, t2;

    // Step 1: Bit-Reversal Reordering
    j = 0;
    n2 = samples / 2;
    for (i = 1; i < samples - 1; i++) {
        n1 = n2;
        while (j >= n1) { j = j - n1; n1 = n1 / 2; }
        j = j + n1;
        if (i < j) {
            swap(&vReal[i], &vReal[j]);
            swap(&vImag[i], &vImag[j]);
        }
    }

    // Step 2: Butterfly Computation
    n1 = 0;
    n2 = 1;
    for (i = 0; i < 9; i++) { // log2(512) = 9 stages
        n1 = n2;
        n2 = n2 + n2;
        a = 0;
        for (j = 0; j < n1; j++) {
            c = cosf(-2.0f * PI * a / samples);
            s = sinf(-2.0f * PI * a / samples);
            a += 1 << (9 - i - 1);
            for (k = j; k < samples; k = k + n2) {
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
// INTERRUPT SERVICE ROUTINES (ISRs)
// ============================================================================

// External interrupt (Button). Implements logical Debounce via Software.
static void IRAM_ATTR gpio_isr_button(void *arg) {
    static uint64_t last_isr_time = 0;
    uint64_t current_time = esp_timer_get_time();
    // Rejects mechanical bouncing under 250ms
    if ((current_time - last_isr_time) > DEBOUNCE_TIME_US) {
        screen_flag = !screen_flag;
        last_isr_time = current_time;
    }
}

// Hardware Timer Alarm (130 us). 
// ARCHITECTURAL NOTE: Does not read the ADC internally to avoid system crashes.
// Emits a fast notification to the SAMPLING TASK to assume the context.
static bool IRAM_ATTR alarm_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    BaseType_t awoken = pdFALSE;
    vTaskNotifyGiveFromISR(sampling_handle, &awoken);
    portYIELD_FROM_ISR(awoken); // Requests immediate preemption from the Scheduler
    return true;
}

// ============================================================================
// I2C DRIVER AND HUMAN-MACHINE INTERFACE (LCD)
// ============================================================================

// Packages and transmits I2C instructions to the 8-bit PCF8574 expander
// Respects the 4-bit multiplexing required by the HD44780 controller
void lcd_send(uint8_t value, uint8_t mode) {
    uint8_t high_nibble = value & 0xF0; // Grabs the 4 most significant bits and clears the rest
    uint8_t low_nibble = (value << 4) & 0xF0; // Pushes the 4 least significant bits to the left
    uint8_t data[4];
    uint8_t control = mode | 0x08; // 0x08 ensures the Backlight remains ON
    
    // Enable pulse generation (bit 0x04)
    data[0] = high_nibble | control | 0x04;
    data[1] = high_nibble | control;
    data[2] = low_nibble | control | 0x04;
    data[3] = low_nibble | control;
    
    i2c_master_transmit(lcd_handle, data, 4, -1);
    usleep(2000); // Respects the physical latency of the display
}

void lcd_cmd(uint8_t cmd)   { lcd_send(cmd, 0); } // Transmits Logic Command
void lcd_data(uint8_t data) { lcd_send(data, 1); } // Transmits ASCII Character

// Centralizes the writing process and fills residual gaps to clear the screen buffer
void lcd_write_line(int line, const char *str) {
    if (line == 0) lcd_cmd(0x80); else lcd_cmd(0xC0);
    int i = 0;
    while (i < 16 && str[i] != '\0') { lcd_data(str[i]); i++; }
    while (i < 16) { lcd_data(' '); i++; } // Fills the rest of the line with blank spaces avoiding the clear screen command
}

// ============================================================================
// HARDWARE CONFIGURATION ROUTINES
// ============================================================================

void config_leds(void) {
    uint64_t mask = (1ULL << LED_HEARTBEAT_PIN) | (1ULL << LED_ALERT_PIN);
    gpio_config_t io_conf = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_HEARTBEAT_PIN, 0);
    gpio_set_level(LED_ALERT_PIN, 0);
}

void config_button(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, // Internal pull-up enabled
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE   // Interrupt on falling edge
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(BUTTON_PIN, gpio_isr_button, NULL);
}

void config_i2c_lcd(void) {
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_SCL_PIN,
        .sda_io_num = I2C_SDA_PIN,
        .glitch_ignore_cnt = 7, // Noise filter
        .flags.enable_internal_pullup = true, 
    };
    i2c_master_bus_handle_t bus_handle;
    i2c_new_master_bus(&i2c_bus_config, &bus_handle);

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LCD_ADDRESS,
        .scl_speed_hz = 100000,
    };
    i2c_master_bus_add_device(bus_handle, &dev_cfg, &lcd_handle);

    usleep(50000);
    // Mandatory initialization sequence for the HD44780 controller
    lcd_cmd(0x33); lcd_cmd(0x32); lcd_cmd(0x28); lcd_cmd(0x0C); lcd_cmd(0x01);
    usleep(50000);
}

void config_adc(void) {
    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&init_config, &adc_handle);
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12 // Allows reading the full 0 ~ 3.3V scale
    };
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_PIN, &config);
}

void config_timer(void) {
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000 // 1 MHz time base (Tick = 1 us)
    };
    gptimer_new_timer(&timer_config, &timer_handle);

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 130,      // Sampling Rate (Fs) = ~7680 Hz, Ts =~ 130us
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true
    };
    gptimer_set_alarm_action(timer_handle, &alarm_config);

    gptimer_event_callbacks_t cbs = { .on_alarm = alarm_callback };
    gptimer_register_event_callbacks(timer_handle, &cbs, NULL);
}

// ============================================================================
// SYSTEM TASKS (FREERTOS)
// ============================================================================

// Highest Priority Task: Manages strict real-time constraints exclusively
void vSamplingTask(void *pvParameters) {
    while (1) {
        // Waits for cycle permission
        xSemaphoreTake(sem_cycle_start, portMAX_DELAY);
        
        gptimer_enable(timer_handle);
        gptimer_start(timer_handle);

        for (int i = 0; i < NUM_SAMPLES; i++) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Synchronizes with the Timer ISR
            adc_oneshot_read(adc_handle, ADC_CHANNEL_PIN, &raw_adc[i]);
        }

        gptimer_stop(timer_handle);
        gptimer_disable(timer_handle);
        
        // Concludes acquisition and authorizes the start of mathematical processing
        xSemaphoreGive(sem_processing);
    }
}

// Processing Task (DSP): Executes the entire mathematical scope of the application
void vProcessingTask(void *pvParameters) {
    // Mitigative dynamic allocation to prevent Stack Overflow 
    // of the arrays required by the Fourier Transform
    float *vReal = malloc(NUM_SAMPLES * sizeof(float));
    float *vImag = malloc(NUM_SAMPLES * sizeof(float));
    float current_array[NUM_SAMPLES]; 

    while (1) {
        // Waits for sampling array integrity
        xSemaphoreTake(sem_processing, portMAX_DELAY);

        float sum_current = 0.0f;
        float sum_squares_true_rms = 0.0f;

        // 1. TIME DOMAIN CONDITIONING
        for (int i = 0; i < NUM_SAMPLES; i++) {
            // Conversion from quantized scale to analog magnitude
            float pin_voltage = ((float)raw_adc[i] / 4095.0f) * 3.3f;
            
            // Offset subtraction in the correct domain to nullify calibration errors
            float pin_voltage_no_dc = pin_voltage - ZERO_A_PIN_VOLTAGE_OFFSET;
            
            // Voltage projection at the input of the resistive divider
            float sensor_voltage_no_dc = pin_voltage_no_dc * VOLTAGE_DIVIDER_FACTOR;
            
            // Application of the ACS712 sensor's characteristic law
            float current_sample = sensor_voltage_no_dc / ACS712_SENSITIVITY_V_PER_A;
            
            current_array[i] = current_sample; 
            sum_current += current_sample;
            sum_squares_true_rms += (current_sample * current_sample); 
        }

        // Direct Current Component Extraction (DC Level)
        dc_result = sum_current / NUM_SAMPLES;
        
        // Total Effective Value Extraction (True RMS)
        float raw_rms = sqrtf(sum_squares_true_rms / NUM_SAMPLES);

        // Deadband Filter: Zeroes out phantom currents derived from 
        // instrumentation white noise variance accumulation
        if (raw_rms < 0.1f) {
            dc_result = 0.0f;
            rms_result = 0.0f;
        } else {
            rms_result = raw_rms;
        }

        // 2. FREQUENCY DOMAIN CONDITIONING (FFT and THD)
        // Avoids heavy executions and division by zero when the load is inactive
        if (rms_result > 0.1f) {
            for (int i = 0; i < NUM_SAMPLES; i++) {
                // Pre-FFT residual DC offset removal to prevent Spectral Leakage
                float pure_ac = current_array[i] - dc_result; 
                
                // Hann Window Application (Mitigation of discontinuities at sample edges)
                float hann = 0.5f * (1.0f - cosf(2.0f * PI * i / (NUM_SAMPLES - 1.0f)));
                vReal[i] = pure_ac * hann;
                vImag[i] = 0.0f;
            }

            calculate_fft(vReal, vImag, NUM_SAMPLES);

            // Resolution = Fs / N = 7680 / 512 = 15 Hz per Bin
            float magnitude[65]; // Evaluation restricted to the first 15 harmonics (Bin 60 = 900 Hz)
            for(int i = 0; i <= 60; i++) {
                magnitude[i] = sqrtf(vReal[i] * vReal[i] + vImag[i] * vImag[i]);
            }

            // Fundamental Frequency Mapping (60 Hz -> Bin 4)
            float fundamental = magnitude[4];
            
            float sum_harmonic_squares = 0.0f;
            float max_harm_mag = 0.0f;
            int max_harm_h = 0;

            // Extraction of even and odd harmonic components (2nd to 15th)
            if (fundamental > 0.001f) {
                for (int h = 2; h <= 15; h++) {
                    int bin = h * 4;
                    float mag = magnitude[bin];
                    sum_harmonic_squares += (mag * mag);

                    // Capture of the noisiest Harmonic (Load Diagnostics)
                    if (mag > max_harm_mag) {
                        max_harm_mag = mag;
                        max_harm_h = h;
                    }
                }

                // Final Calculation of the Distortion Profile (THD)
                float v_harmonic = sqrtf(sum_harmonic_squares);
                thd_result = (v_harmonic / fundamental) * 100.0f;
                sig_harmonic_amp = (max_harm_mag / fundamental) * 100.0f;
                sig_harmonic_freq = max_harm_h * 60;
            } else {
                thd_result = 0.0f;
                sig_harmonic_amp = 0.0f;
                sig_harmonic_freq = 0;
            }
        } else {
            // Ensures safe initialization of FFT-related variables at rest
            thd_result = 0.0f;
            sig_harmonic_amp = 0.0f;
            sig_harmonic_freq = 0;
        }

        // Authorizes Graphical Interface update
        xSemaphoreGive(sem_display);
    }
}

// Lowest Priority Task: Executes formatting and slow I/O interface (I2C)
void vDisplayTask(void *pvParameters) {
    xSemaphoreGive(sem_cycle_start); // Initializes the processing chain

    while (1) {
        xSemaphoreTake(sem_display, portMAX_DELAY);

        // Visual Update - Heartbeat proves the Scheduler's (RTOS) vivacity
        static uint8_t led_status = 0;
        led_status = !led_status;
        gpio_set_level(LED_HEARTBEAT_PIN, led_status);

        // Conditional Actuator Logic (Physical Alarm)
        if (thd_result >= THD_ALERT_LIMIT_PCT) {
            gpio_set_level(LED_ALERT_PIN, 1); 
        } else {
            gpio_set_level(LED_ALERT_PIN, 0); 
        }

        // Semantic construction and transmission to the 16x2 LCD
        char buffer[17];
        if (screen_flag == 0) {
            snprintf(buffer, sizeof(buffer), "DC Level: %4.2fA", dc_result);
            lcd_write_line(0, buffer);
            
            snprintf(buffer, sizeof(buffer), "AC RMS:   %4.2fA", rms_result);
            lcd_write_line(1, buffer);
        } else {
            snprintf(buffer, sizeof(buffer), "THD: %5.1f %%", thd_result);
            lcd_write_line(0, buffer);
            
            snprintf(buffer, sizeof(buffer), "Max: %dHz %.1f%%", sig_harmonic_freq, sig_harmonic_amp);
            lcd_write_line(1, buffer);
        }

        // Determines the natural refresh rate (Visible Refresh Rate)
        vTaskDelay(pdMS_TO_TICKS(500));  
        
        // Returns context control to restart a new time cycle
        xSemaphoreGive(sem_cycle_start);
    }
}

// ============================================================================
// SYSTEM ENTRY POINT (Setup)
// ============================================================================
void app_main(void)
{
    // Preventively deactivated so as not to trigger guard time resets
    esp_task_wdt_deinit();

    // Hardware Initialization
    config_leds();
    config_adc();
    config_timer();
    config_i2c_lcd();
    config_button();

    // Primitive RTOS Allocation: Semaphore-based state machine
    sem_processing  = xSemaphoreCreateBinary();
    sem_display     = xSemaphoreCreateBinary();
    sem_cycle_start = xSemaphoreCreateBinary();

    // Dispatching Tasks to the Scheduler (Explicit Priority Assignment)
    xTaskCreate(vSamplingTask,   "Sampling",   4096, NULL, tskIDLE_PRIORITY + 3, &sampling_handle);
    // Elevated Priority and Stack due to FFT demands (Allocation, Math.h)
    xTaskCreate(vProcessingTask, "Processing", 8192, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(vDisplayTask,    "Display",    4096, NULL, tskIDLE_PRIORITY + 1, NULL);
}