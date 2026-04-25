/**
 * @file ds18b20.c
 * @brief Implementación del driver DS18B20 para ESP32 (ESP-IDF)
 * 
 * Protocolo OneWire implementado por bit-banging:
 * - GPIO en modo open-drain (INPUT_OUTPUT_OD): configurado UNA vez
 * - Para poner línea en LOW:  gpio_set_level(pin, 0) → tira a GND
 * - Para LIBERAR la línea:    gpio_set_level(pin, 1) → pull-up la sube a 3.3V
 * - Interrupciones deshabilitadas durante cada bit para proteger el timing
 * - Delays con esp_rom_delay_us() (busy-wait, funciona con interrupts off)
 * 
 * Validado contra punto de hielo (0°C): error medido +0.12°C
 * 
 * Proyecto: Plancha automática de vapor — Proyecto Mecatrónico II
 */

#include "ds18b20.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DS18B20";

/* ================================================================== */
/*  CAPA BAJA: Operaciones OneWire a nivel de bit                     */
/* ================================================================== */

/**
 * @brief Envía pulso de reset y espera respuesta de presencia del sensor
 * @return true si el sensor respondió, false si no hay sensor conectado
 * 
 * Secuencia: maestro tira la línea 480µs → libera → espera 70µs → 
 * lee (si está en LOW, el sensor está ahí)
 */
static bool onewire_reset(void) {
    bool presence = false;

    portDISABLE_INTERRUPTS();
    gpio_set_level(DS18B20_GPIO, 0);    /* Tira la línea a GND */
    esp_rom_delay_us(480);               /* Mantiene por 480µs (pulso de reset) */
    gpio_set_level(DS18B20_GPIO, 1);    /* Libera la línea (pull-up la sube) */
    esp_rom_delay_us(70);                /* Espera a que el sensor responda */
    presence = (gpio_get_level(DS18B20_GPIO) == 0); /* Si está en LOW, hay sensor */
    portENABLE_INTERRUPTS();

    esp_rom_delay_us(410);               /* Espera que termine el slot de reset */
    return presence;
}

/**
 * @brief Escribe un bit en el bus OneWire
 * @param bit Valor del bit a escribir (0 o 1)
 * 
 * Bit 1: pull LOW breve (6µs), luego liberar (54µs)
 * Bit 0: pull LOW largo (60µs), luego liberar (2µs)
 */
static void onewire_write_bit(uint8_t bit) {
    portDISABLE_INTERRUPTS();
    if (bit & 1) {
        /* Escribir 1: pulso corto bajo, luego alto */
        gpio_set_level(DS18B20_GPIO, 0);
        esp_rom_delay_us(6);
        gpio_set_level(DS18B20_GPIO, 1);
        esp_rom_delay_us(54);
    } else {
        /* Escribir 0: pulso largo bajo */
        gpio_set_level(DS18B20_GPIO, 0);
        esp_rom_delay_us(60);
        gpio_set_level(DS18B20_GPIO, 1);
        esp_rom_delay_us(2);
    }
    portENABLE_INTERRUPTS();
}

/**
 * @brief Lee un bit del bus OneWire
 * @return 0 o 1 según lo que el sensor puso en la línea
 * 
 * Secuencia: maestro tira LOW 6µs → libera → espera 9µs → lee el nivel
 */
static uint8_t onewire_read_bit(void) {
    uint8_t bit = 0;

    portDISABLE_INTERRUPTS();
    gpio_set_level(DS18B20_GPIO, 0);    /* Inicia slot de lectura */
    esp_rom_delay_us(6);
    gpio_set_level(DS18B20_GPIO, 1);    /* Libera para que el sensor ponga su dato */
    esp_rom_delay_us(9);
    bit = gpio_get_level(DS18B20_GPIO) ? 1 : 0;  /* Lee lo que puso el sensor */
    esp_rom_delay_us(45);                /* Espera fin del slot */
    portENABLE_INTERRUPTS();

    return bit;
}

/**
 * @brief Escribe un byte completo (8 bits, LSB primero)
 * @param data Byte a enviar al sensor
 */
static void onewire_write_byte(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        onewire_write_bit(data & 0x01);  /* Envía bit menos significativo primero */
        data >>= 1;                       /* Desplaza al siguiente bit */
    }
}

/**
 * @brief Lee un byte completo (8 bits, LSB primero)
 * @return Byte leído del sensor
 */
static uint8_t onewire_read_byte(void) {
    uint8_t data = 0;
    for (int i = 0; i < 8; i++) {
        data >>= 1;                       /* Hace espacio para el nuevo bit */
        if (onewire_read_bit()) {
            data |= 0x80;                 /* Pone 1 en el bit más significativo */
        }
    }
    return data;
}

/* ================================================================== */
/*  CAPA MEDIA: Verificación CRC8                                     */
/* ================================================================== */

/**
 * @brief Calcula CRC8 según el polinomio de Dallas/Maxim
 * @param data Buffer de datos a verificar
 * @param len Longitud del buffer
 * @return CRC8 calculado
 * 
 * Se usa para verificar que los datos del sensor llegaron sin errores.
 * Si el CRC calculado de los primeros 8 bytes coincide con el byte 9
 * (que es el CRC enviado por el sensor), la lectura es válida.
 */
static uint8_t crc8(const uint8_t *data, int len) {
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        uint8_t byte = data[i];
        for (int j = 0; j < 8; j++) {
            uint8_t mix = (crc ^ byte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;  /* Polinomio Dallas/Maxim */
            byte >>= 1;
        }
    }
    return crc;
}

/* ================================================================== */
/*  CAPA ALTA: API pública                                            */
/* ================================================================== */

esp_err_t ds18b20_init(void) {
    /* Configurar GPIO en modo open-drain UNA SOLA VEZ.
     * Open-drain permite leer y escribir con el mismo pin:
     * - set_level(0) → tira a GND (activa, el ESP32 drena corriente)
     * - set_level(1) → alta impedancia (la pull-up externa de 4.7kΩ sube a 3.3V) */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DS18B20_GPIO),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,  /* Open-drain bidireccional */
        .pull_up_en = GPIO_PULLUP_DISABLE,  /* Usamos pull-up EXTERNO de 4.7kΩ */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(DS18B20_GPIO, 1);  /* Liberar línea al inicio */

    /* Verificar que el sensor responde al pulso de reset */
    if (!onewire_reset()) {
        ESP_LOGE(TAG, "DS18B20 NO detectado en GPIO%d", DS18B20_GPIO);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "DS18B20 detectado en GPIO%d", DS18B20_GPIO);
    return ESP_OK;
}

bool ds18b20_read_temp(float *temp_out) {
    if (!temp_out) return false;

    /* PASO 1: Enviar comando de conversión de temperatura */
    if (!onewire_reset()) return false;
    onewire_write_byte(0xCC);  /* Skip ROM: solo hay un sensor en el bus */
    onewire_write_byte(0x44);  /* Convert T: inicia la medición interna */

    /* Esperar 750ms: tiempo de conversión a resolución de 12 bits.
     * Usamos vTaskDelay para no bloquear otros tasks de FreeRTOS */
    vTaskDelay(pdMS_TO_TICKS(750));

    /* PASO 2: Leer el scratchpad (9 bytes de memoria del sensor) */
    if (!onewire_reset()) return false;
    onewire_write_byte(0xCC);  /* Skip ROM */
    onewire_write_byte(0xBE);  /* Read Scratchpad: pide los datos */

    uint8_t sp[9];
    for (int i = 0; i < 9; i++) {
        sp[i] = onewire_read_byte();  /* Lee los 9 bytes del scratchpad */
    }

    /* PASO 3: Verificar integridad de datos con CRC */
    if (crc8(sp, 8) != sp[8]) {
        ESP_LOGW(TAG, "CRC incorrecto");  /* Datos corruptos, descartar */
        return false;
    }

    /* PASO 4: Convertir bytes crudos a temperatura en °C
     * Los bytes 0 y 1 contienen la temperatura en formato de 16 bits con signo.
     * Resolución de 12 bits: cada unidad = 0.0625°C, por eso dividimos entre 16 */
    int16_t raw = (int16_t)((sp[1] << 8) | sp[0]);
    float temp = (float)raw / 16.0f;

    /* Verificar que la lectura está dentro del rango válido del sensor */
    if (temp < -55.0f || temp > 125.0f) return false;

    *temp_out = temp;
    return true;
}
