/**
 * @file ds18b20.h
 * @brief Driver para sensor de temperatura DS18B20
 * 
 * El DS18B20 es un sensor digital que mide temperatura de -55°C a +125°C
 * con precisión de ±0.5°C. Se comunica por protocolo OneWire usando un
 * solo cable de datos.
 * 
 * Hardware esperado:
 *   - Cable amarillo (data) del DS18B20 → GPIO4
 *   - Cable rojo (VDD) → 3.3V
 *   - Cable negro (GND) → GND
 *   - Resistencia pull-up 4.7kΩ entre GPIO4 y 3.3V
 * 
 * Proyecto: Plancha automática de vapor — Proyecto Mecatrónico II
 */

#ifndef DS18B20_H
#define DS18B20_H

#include "esp_err.h"
#include <stdbool.h>

/* Pin de datos del sensor DS18B20 */
#define DS18B20_GPIO 4

/**
 * @brief Inicializa el GPIO en modo open-drain y verifica que el sensor responde
 * @return ESP_OK si el sensor fue detectado, ESP_FAIL si no responde
 * @note Llamar UNA SOLA VEZ al inicio del programa en app_main()
 */
esp_err_t ds18b20_init(void);

/**
 * @brief Lee la temperatura actual del sensor
 * @param temp_out Puntero donde se guarda la temperatura en °C
 * @return true si la lectura fue exitosa, false si hubo error de comunicación o CRC
 * @note Esta función tarda ~750ms por la conversión interna del sensor
 */
bool ds18b20_read_temp(float *temp_out);

#endif
