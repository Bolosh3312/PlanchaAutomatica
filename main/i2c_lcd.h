/**
 * @file i2c_lcd.h
 * @brief Driver para pantalla LCD 16x2 con adaptador I2C (PCF8574)
 * 
 * La pantalla LCD muestra información del sistema en dos líneas de 16 caracteres:
 * - Línea 1: Temperatura actual y duty cycle del PID
 * - Línea 2: Estado actual del sistema (Reposo, Precalent., Vapor, etc.)
 * 
 * Hardware esperado:
 *   - SDA del módulo I2C → GPIO21
 *   - SCL del módulo I2C → GPIO22
 *   - VCC → 5V (necesario para el backlight y contraste)
 *   - GND → GND
 *   - Dirección I2C: 0x27 (default del PCF8574)
 * 
 * Proyecto: Plancha automática de vapor — Proyecto Mecatrónico II
 */

#ifndef I2C_LCD_H
#define I2C_LCD_H

#include "esp_err.h"

#define LCD_ADDR        0x27    /* Dirección I2C del módulo PCF8574 */
#define LCD_SDA_GPIO    21      /* Pin de datos I2C */
#define LCD_SCL_GPIO    22      /* Pin de reloj I2C */

/**
 * @brief Inicializa el bus I2C y la pantalla LCD en modo 4 bits
 * @return ESP_OK si la inicialización fue exitosa
 * @note Llamar UNA SOLA VEZ al inicio del programa
 */
esp_err_t lcd_init(void);

/**
 * @brief Limpia toda la pantalla y regresa el cursor a la posición (0,0)
 */
void lcd_clear(void);

/**
 * @brief Posiciona el cursor en una fila y columna específicas
 * @param row Fila (0 = primera línea, 1 = segunda línea)
 * @param col Columna (0 a 15)
 */
void lcd_set_cursor(uint8_t row, uint8_t col);

/**
 * @brief Imprime una cadena de texto en la posición actual del cursor
 * @param str Cadena de texto a imprimir (máximo 16 caracteres por línea)
 */
void lcd_print(const char *str);

#endif
