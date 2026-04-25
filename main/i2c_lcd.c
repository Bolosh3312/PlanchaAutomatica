/**
 * @file i2c_lcd.c
 * @brief Implementación del driver LCD 16x2 con adaptador I2C
 * 
 * El adaptador I2C (PCF8574) convierte 8 bits paralelos del LCD a 2 cables I2C.
 * La comunicación se hace en modo 4 bits: cada byte se envía en dos nibbles
 * (4 bits superiores primero, luego 4 bits inferiores).
 * 
 * Bits del byte enviado al PCF8574:
 *   Bit 7-4: Datos (D7-D4 del LCD)
 *   Bit 3: Backlight (1=encendido)
 *   Bit 2: Enable (pulso para confirmar dato)
 *   Bit 1: Read/Write (siempre 0, solo escribimos)
 *   Bit 0: RS (0=comando, 1=dato/caracter)
 * 
 * Proyecto: Plancha automática de vapor — Proyecto Mecatrónico II
 */

#include "i2c_lcd.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "LCD";

#define I2C_PORT  I2C_NUM_0   /* Puerto I2C del ESP32 (tiene 2: NUM_0 y NUM_1) */
#define BACKLIGHT 0x08         /* Bit 3: mantener backlight encendido */
#define EN        0x04         /* Bit 2: Enable del LCD */
#define RS        0x01         /* Bit 0: Register Select (0=comando, 1=dato) */

/**
 * @brief Envía un byte al PCF8574 por I2C
 * @param data Byte a enviar (contiene datos + señales de control)
 */
static esp_err_t lcd_write_byte(uint8_t data) {
    return i2c_master_write_to_device(I2C_PORT, LCD_ADDR, &data, 1, pdMS_TO_TICKS(100));
}

/**
 * @brief Genera un pulso en el pin Enable del LCD para confirmar un dato
 * @param data Byte con los datos ya posicionados en los bits superiores
 * 
 * El LCD captura los datos en el flanco de bajada del Enable
 */
static void lcd_pulse_enable(uint8_t data) {
    lcd_write_byte(data | EN | BACKLIGHT);   /* Enable HIGH + datos */
    esp_rom_delay_us(1);
    lcd_write_byte((data & ~EN) | BACKLIGHT); /* Enable LOW: LCD captura */
    esp_rom_delay_us(50);
}

/**
 * @brief Envía un nibble (4 bits) al LCD
 * @param nibble Datos en los 4 bits superiores del byte
 * @param mode 0=comando, RS=dato/caracter
 */
static void lcd_send_nibble(uint8_t nibble, uint8_t mode) {
    uint8_t data = (nibble & 0xF0) | mode | BACKLIGHT;
    lcd_pulse_enable(data);
}

/**
 * @brief Envía un byte completo al LCD en dos nibbles
 * @param byte Byte a enviar
 * @param mode 0=comando, RS=dato/caracter
 */
static void lcd_send_byte(uint8_t byte, uint8_t mode) {
    lcd_send_nibble(byte & 0xF0, mode);         /* Nibble alto primero */
    lcd_send_nibble((byte << 4) & 0xF0, mode);  /* Nibble bajo después */
}

/**
 * @brief Envía un comando al LCD (RS=0)
 * @param cmd Código del comando
 * @note Los comandos 0x01 (clear) y 0x02 (home) necesitan 2ms de espera
 */
static void lcd_command(uint8_t cmd) {
    lcd_send_byte(cmd, 0);
    if (cmd < 4) vTaskDelay(pdMS_TO_TICKS(2)); /* Clear y Home son lentos */
}

esp_err_t lcd_init(void) {
    /* Configurar el bus I2C del ESP32 como maestro */
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = LCD_SDA_GPIO,           /* GPIO21 para datos */
        .scl_io_num = LCD_SCL_GPIO,           /* GPIO22 para reloj */
        .sda_pullup_en = GPIO_PULLUP_ENABLE,  /* Pull-ups internos */
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,           /* 100kHz estándar */
    };
    esp_err_t err = i2c_param_config(I2C_PORT, &conf);
    if (err != ESP_OK) { ESP_LOGE(TAG, "Error config I2C"); return err; }

    err = i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) { ESP_LOGE(TAG, "Error install I2C"); return err; }

    /* Secuencia de inicialización del LCD según datasheet HD44780 */
    vTaskDelay(pdMS_TO_TICKS(50));             /* Esperar que el LCD se estabilice */
    lcd_send_nibble(0x30, 0); vTaskDelay(pdMS_TO_TICKS(5));   /* Function set 8-bit #1 */
    lcd_send_nibble(0x30, 0); vTaskDelay(pdMS_TO_TICKS(5));   /* Function set 8-bit #2 */
    lcd_send_nibble(0x30, 0); vTaskDelay(pdMS_TO_TICKS(2));   /* Function set 8-bit #3 */
    lcd_send_nibble(0x20, 0); vTaskDelay(pdMS_TO_TICKS(2));   /* Cambiar a modo 4-bit */

    lcd_command(0x28);  /* Function set: 4-bit, 2 líneas, fuente 5x8 */
    lcd_command(0x0C);  /* Display ON, cursor OFF, blink OFF */
    lcd_command(0x06);  /* Entry mode: incrementar cursor, no shift */
    lcd_command(0x01);  /* Clear display */
    vTaskDelay(pdMS_TO_TICKS(2));

    ESP_LOGI(TAG, "LCD inicializado en 0x%02X", LCD_ADDR);
    return ESP_OK;
}

void lcd_clear(void) {
    lcd_command(0x01);  /* Comando clear display */
    vTaskDelay(pdMS_TO_TICKS(2));
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
    /* Direcciones de inicio de cada línea en la memoria del LCD:
     * Línea 0: dirección 0x00 a 0x0F
     * Línea 1: dirección 0x40 a 0x4F */
    uint8_t offsets[] = {0x00, 0x40};
    if (row > 1) row = 1;
    if (col > 15) col = 15;
    lcd_command(0x80 | (offsets[row] + col));  /* Set DDRAM address */
}

void lcd_print(const char *str) {
    /* Envía cada carácter como dato (RS=1) */
    while (*str) {
        lcd_send_byte(*str++, RS);
    }
}
