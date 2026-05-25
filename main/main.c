/**
 * @file main.c
 * @brief Sistema de control principal — Plancha automática de vapor
 * 
 * Este archivo implementa la máquina de estados finitos (FSM) que controla
 * todo el sistema de la plancha de vapor. La FSM coordina:
 * - Sensor DS18B20: monitoreo de temperatura de la cámara
 * - SSR-40DA: control de encendido/apagado de la caldera (1350W/110V)
 * - Ventilador 92mm: enfriamiento de la cámara
 * - LCD 16x2: visualización de estado y temperatura
 * - Botón: inicio/paro del ciclo por el usuario
 * - PID: control proporcional de temperatura (time-proportional)
 * 
 * Ciclo de operación:
 * Reposo → Precalentamiento → Vapor alta → Vapor media → Vapor baja → 
 * Enfriamiento (con ventilador) → Fin (seguro para abrir)
 * 
 * Protecciones de seguridad:
 * - Corte automático a 95°C (software)
 * - Termostato KSD301 integrado en caldera (hardware)
 * - Fusible cerámico 15A
 * - Watchdog de sensor (3 lecturas fallidas → corte)
 * - Paro manual con botón → enfriamiento automático
 * 
 * Altitud: Zapopan ~1566m, punto de ebullición ≈ 95°C
 * 
 * Proyecto: Plancha automática de vapor — Proyecto Mecatrónico II
 * Universidad Panamericana, Zapopan
 * Alumnos: Rodrigo Ojeda Moreno, Juan Manuel Vega Guzmán
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "ds18b20.h"
#include "i2c_lcd.h"
#include "pid.h"

static const char *TAG = "PLANCHA";

/* ================================================================== */
/*  CONFIGURACIÓN DE PINES GPIO                                       */
/* ================================================================== */

#define SSR_GPIO    13   /* Control del SSR-40DA (salida, activa la caldera) */
#define FAN_GPIO    14   /* Control del MOSFET IRLZ44N (salida, activa ventilador) */
#define BUTTON_GPIO 15   /* Botón de inicio/paro (entrada, activo en LOW) */

/* ================================================================== */
/*  UMBRALES DE TEMPERATURA (en °C)                                   */
/* ================================================================== */

#define SAFETY_MAX_TEMP     95.0f   /* Corte de seguridad absoluto (≈ ebullición en Zapopan) */
#define SAFETY_MIN_TEMP     -10.0f  /* Lectura bajo este valor = sensor desconectado */
#define SENSOR_FAIL_MAX     3       /* Lecturas fallidas consecutivas antes de corte */
#define MAX_DUTY            0.90f   /* Duty máximo del SSR (90%, nunca 100% por seguridad) */
#define PID_PERIOD_MS       2000    /* Período del lazo de control: 2 segundos */

/* Setpoints de temperatura para cada estado de vapor */
#define TEMP_PRECALENT      65.0f   /* Temperatura de transición a vapor activo */
#define TEMP_VAPOR_ALTA     80.0f   /* Máxima generación de vapor */
#define TEMP_VAPOR_MEDIA    70.0f   /* Vapor moderado */
#define TEMP_VAPOR_BAJA     60.0f   /* Vapor suave, fase final */
#define TEMP_SAFE_OPEN      40.0f   /* Seguro para que el usuario abra la cámara */

/* Duración de cada fase de vapor en segundos */
#define TIME_VAPOR_ALTA_S   180     /* 3 minutos en vapor alta */
#define TIME_VAPOR_MEDIA_S  180     /* 3 minutos en vapor media */
#define TIME_VAPOR_BAJA_S   120     /* 2 minutos en vapor baja */

/* ================================================================== */
/*  GANANCIAS DEL PID (ajustar experimentalmente con la caldera real) */
/* ================================================================== */

#define PID_KP  0.05f    /* Proporcional: respuesta al error */
#define PID_KI  0.003f   /* Integral: elimina error estacionario */
#define PID_KD  0.8f     /* Derivativo: frena cambios bruscos */

/* ================================================================== */
/*  ESTADOS DE LA MÁQUINA DE ESTADOS FINITOS (FSM)                    */
/* ================================================================== */

typedef enum {
    STATE_REPOSO,             /* Sistema apagado, esperando botón */
    STATE_PRECALENTAMIENTO,   /* Caldera encendida, subiendo temperatura a 65°C */
    STATE_VAPOR_ALTA,         /* PID mantiene 80°C, máxima acción de vapor */
    STATE_VAPOR_MEDIA,        /* PID mantiene 70°C, vapor moderado */
    STATE_VAPOR_BAJA,         /* PID mantiene 60°C, vapor suave */
    STATE_ENFRIAMIENTO,       /* Caldera OFF, ventilador ON hasta temp segura */
    STATE_FIN,                /* Ciclo terminado, seguro para abrir */
    STATE_ERROR               /* Fallo detectado, todo apagado excepto ventilador */
} system_state_t;

/* Nombres de estados para mostrar en LCD y serial (máx 16 caracteres) */
static const char *state_names[] = {
    "Reposo", "Precalent.", "Vapor alta",
    "Vapor media", "Vapor baja", "Enfriando",
    "Fin ciclo", "ERROR"
};

/* ================================================================== */
/*  VARIABLES GLOBALES                                                */
/* ================================================================== */

static system_state_t current_state = STATE_REPOSO;  /* Estado actual de la FSM */
static int state_seconds = 0;          /* Segundos transcurridos en el estado actual */
static int sensor_fail_count = 0;      /* Contador de lecturas fallidas consecutivas */
static float current_temp = 0.0f;      /* Última temperatura válida leída */
static float current_duty = 0.0f;      /* Duty cycle actual del SSR (0.0 a 1.0) */
static pid_controller_t pid;           /* Instancia del controlador PID */
static volatile bool button_flag = false; /* Flag de interrupción del botón */

/* ================================================================== */
/*  INTERRUPCIÓN DEL BOTÓN                                            */
/* ================================================================== */

/**
 * @brief ISR (Interrupt Service Routine) del botón
 * 
 * Se ejecuta instantáneamente cuando se detecta un flanco de bajada
 * en GPIO15 (botón presionado). Solo levanta un flag que el loop
 * principal revisa. No se puede hacer lógica compleja aquí dentro.
 * 
 * IRAM_ATTR: la función se almacena en RAM para ejecución rápida
 */
static void IRAM_ATTR button_isr(void *arg) {
    button_flag = true;
}

/* ================================================================== */
/*  FUNCIONES DE CONTROL DE ACTUADORES                                */
/* ================================================================== */

/** @brief Enciende o apaga el SSR (caldera) */
static void ssr_set(bool on) { gpio_set_level(SSR_GPIO, on ? 1 : 0); }

/** @brief Enciende o apaga el ventilador vía MOSFET */
static void fan_set(bool on) { gpio_set_level(FAN_GPIO, on ? 1 : 0); }

/** @brief Apaga todo: SSR, ventilador y resetea duty */
static void all_off(void) {
    ssr_set(false);
    fan_set(false);
    current_duty = 0.0f;
}

/* ================================================================== */
/*  FUNCIONES DE CAMBIO DE ESTADO Y SEGURIDAD                        */
/* ================================================================== */

/**
 * @brief Cambia el estado de la FSM y resetea el contador de tiempo
 * @param new_state Nuevo estado al que transicionar
 */
static void change_state(system_state_t new_state) {
    system_state_t old = current_state;
    current_state = new_state;
    state_seconds = 0;
    ESP_LOGI(TAG, "Estado: %s -> %s (Temp: %.1fC)",
             state_names[old], state_names[new_state], current_temp);
}

/**
 * @brief Ejecuta un paro de emergencia
 * @param reason Texto descriptivo de la causa (se imprime por serial)
 * 
 * Apaga la caldera inmediatamente, resetea el PID y entra en estado ERROR.
 * El ventilador se enciende en el estado ERROR para enfriar.
 */
static void emergency_stop(const char *reason) {
    ssr_set(false);          /* Caldera OFF inmediato */
    current_duty = 0.0f;
    pid_reset(&pid);
    change_state(STATE_ERROR);
    ESP_LOGE(TAG, "EMERGENCY STOP: %s", reason);
}

/**
 * @brief Verifica condiciones de seguridad del sensor y temperatura
 * @param temp Temperatura leída
 * @param read_ok Si la lectura del sensor fue exitosa
 * @return true si es seguro continuar, false si se debe detener
 * 
 * Verifica tres condiciones:
 * 1. Que el sensor respondió (si no, incrementa contador de fallos)
 * 2. Que la lectura está en rango válido (-10 a 125°C)
 * 3. Que no se excedió el límite de seguridad (95°C)
 */
static bool safety_check(float temp, bool read_ok) {
    /* Verificar comunicación con sensor */
    if (!read_ok) {
        sensor_fail_count++;
        ESP_LOGW(TAG, "Sensor fail %d/%d", sensor_fail_count, SENSOR_FAIL_MAX);
        if (sensor_fail_count >= SENSOR_FAIL_MAX) {
            emergency_stop("Sensor no responde");
            return false;
        }
        ssr_set(false);      /* Por precaución, apagar caldera si falla lectura */
        current_duty = 0.0f;
        return false;
    }
    sensor_fail_count = 0;  /* Reset contador si lectura exitosa */

    /* Verificar rango válido del sensor */
    if (temp < SAFETY_MIN_TEMP || temp > 125.0f) {
        emergency_stop("Lectura fuera de rango");
        return false;
    }

    /* Verificar límite máximo de temperatura */
    if (temp >= SAFETY_MAX_TEMP) {
        emergency_stop("Sobretemperatura");
        return false;
    }

    return true;
}

/* ================================================================== */
/*  CONTROL TIME-PROPORTIONAL DEL SSR                                 */
/* ================================================================== */

/**
 * @brief Aplica el duty cycle calculado por el PID al SSR
 * @param duty Fracción de tiempo ON (0.0 a 0.90)
 * 
 * El SSR es un switch ON/OFF (no regula voltaje). Para controlar
 * la potencia, usamos "time-proportional control":
 * - Período total = 2 segundos (PID_PERIOD_MS)
 * - Si duty = 0.6: SSR ON 1.2s, OFF 0.8s
 * - Si duty = 0.3: SSR ON 0.6s, OFF 1.4s
 * 
 * Mínimo pulso ON: 50ms (para que el SSR zero-cross alcance a conmutar)
 */
static void apply_ssr_duty(float duty) {
    if (duty > MAX_DUTY) duty = MAX_DUTY;
    current_duty = duty;

    uint32_t on_ms = (uint32_t)(duty * PID_PERIOD_MS);
    uint32_t off_ms = PID_PERIOD_MS - on_ms;

    /* Pulsos menores a 50ms no son confiables con SSR zero-cross */
    if (on_ms < 50) { on_ms = 0; off_ms = PID_PERIOD_MS; }

    /* Fase ON: caldera encendida */
    if (on_ms > 0) {
        ssr_set(true);
        vTaskDelay(pdMS_TO_TICKS(on_ms));
    }
    /* Fase OFF: caldera apagada */
    if (off_ms > 0) {
        ssr_set(false);
        vTaskDelay(pdMS_TO_TICKS(off_ms));
    }
}

/* ================================================================== */
/*  ACTUALIZACIÓN DE PANTALLA LCD                                     */
/* ================================================================== */

/**
 * @brief Actualiza la información mostrada en el LCD
 * @param temp Temperatura actual
 * @param temp_ok Si la lectura fue válida
 * 
 * Línea 1: "T:XX.XC D:XX%"  (temperatura y duty cycle)
 * Línea 2: nombre del estado actual
 */
static void update_lcd(float temp, bool temp_ok) {
    char line1[17], line2[17];

    if (temp_ok) {
        snprintf(line1, sizeof(line1), "T:%.1fC D:%.0f%%  ", temp, current_duty * 100.0f);
    } else {
        snprintf(line1, sizeof(line1), "Temp: ERROR     ");
    }
    snprintf(line2, sizeof(line2), "%-16s", state_names[current_state]);

    lcd_set_cursor(0, 0);
    lcd_print(line1);
    lcd_set_cursor(1, 0);
    lcd_print(line2);
}

/* ================================================================== */
/*  FUNCIÓN PRINCIPAL (app_main)                                      */
/* ================================================================== */

void app_main(void) {
    ESP_LOGI(TAG, "=== Plancha de Vapor — Sistema completo ===");

    /* --- Configurar GPIOs de salida (SSR y ventilador) --- */
    gpio_config_t out_conf = {
        .pin_bit_mask = (1ULL << SSR_GPIO) | (1ULL << FAN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,  /* Pull-down: OFF por defecto */
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&out_conf);
    all_off();  /* Asegurar que todo arranca apagado */

    /* --- Configurar GPIO de entrada (botón) con interrupción --- */
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,      /* Pull-up interno: HIGH cuando no presionado */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,        /* Dispara en flanco de bajada (al presionar) */
    };
    gpio_config(&btn_conf);
    gpio_install_isr_service(0);                         /* Habilitar servicio de interrupciones */
    gpio_isr_handler_add(BUTTON_GPIO, button_isr, NULL); /* Registrar ISR del botón */

    /* --- Inicializar sensor de temperatura --- */
    esp_err_t ret = ds18b20_init();
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Error DS18B20"); return; }

    /* --- Inicializar pantalla LCD --- */
    ret = lcd_init();
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Error LCD"); return; }

    /* Pantalla de bienvenida */
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Plancha Vapor");
    lcd_set_cursor(1, 0);
    lcd_print("Iniciando...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* --- Inicializar controlador PID --- */
    float dt = (float)PID_PERIOD_MS / 1000.0f;  /* dt = 2.0 segundos */
    pid_init(&pid, PID_KP, PID_KI, PID_KD, dt);

    /* Descartar primera lectura del sensor (puede dar 85°C por power-on reset) */
    float temp_descarte;
    ds18b20_read_temp(&temp_descarte);

    ESP_LOGI(TAG, "Sistema listo. Presiona boton para iniciar.");

    /* ============================================================== */
    /*  LOOP PRINCIPAL — Se ejecuta indefinidamente cada 2 segundos   */
    /* ============================================================== */

    while (1) {
        /* --- Leer temperatura del sensor --- */
        float temp = 0.0f;
        bool temp_ok = ds18b20_read_temp(&temp);
        if (temp_ok) current_temp = temp;

        /* --- Verificar seguridad (excepto en estado ERROR) --- */
        if (current_state != STATE_ERROR) {
            if (!safety_check(temp, temp_ok)) {
                update_lcd(current_temp, temp_ok);
                vTaskDelay(pdMS_TO_TICKS(PID_PERIOD_MS));
                continue;  /* Saltar el resto del loop si hay fallo */
            }
        }

        /* --- Enviar datos por serial para graficado en tiempo real --- */
        /* Formato: DATA,timestamp_ms,temp,setpoint,duty,estado,P,I,D   */
        /* El script Python (plot_plancha.py) filtra líneas con "DATA,"  */
        printf("DATA,%lu,%.2f,%.1f,%.4f,%s,%.4f,%.4f,%.4f\n",
            (unsigned long)(xTaskGetTickCount() * portTICK_PERIOD_MS),
            current_temp,
            pid.setpoint,
            current_duty,
            state_names[current_state],
            pid.p_term,
            pid.i_term,
            pid.d_term);

        /* --- Procesar botón (si fue presionado) --- */
        if (button_flag) {
            button_flag = false;
            vTaskDelay(pdMS_TO_TICKS(200));  /* Debounce: esperar 200ms */

            if (current_state == STATE_REPOSO || current_state == STATE_FIN) {
                /* En reposo o fin: INICIAR nuevo ciclo */
                pid_reset(&pid);
                pid_set_setpoint(&pid, TEMP_PRECALENT);
                change_state(STATE_PRECALENTAMIENTO);
            } else if (current_state == STATE_ERROR) {
                /* En error: RESETEAR si la temperatura es segura */
                if (current_temp < SAFETY_MAX_TEMP - 10.0f) {
                    all_off();
                    pid_reset(&pid);
                    change_state(STATE_REPOSO);
                    ESP_LOGI(TAG, "Error reseteado");
                }
            } else {
                /* En cualquier otro estado: PARO MANUAL */
                ssr_set(false);       /* Apagar caldera inmediatamente */
                current_duty = 0.0f;
                pid_reset(&pid);
                ESP_LOGW(TAG, "PARO MANUAL — enfriando");

                /* Si está caliente, enfriar con ventilador */
                if (current_temp > TEMP_SAFE_OPEN) {
                    fan_set(true);
                    change_state(STATE_ENFRIAMIENTO);
                } else {
                    all_off();
                    change_state(STATE_FIN);
                }
            }
        }

        /* --- Ejecutar lógica del estado actual --- */
        switch (current_state) {

            case STATE_REPOSO:
                /* Todo apagado, mostrar temperatura, esperar botón */
                all_off();
                update_lcd(current_temp, temp_ok);
                vTaskDelay(pdMS_TO_TICKS(PID_PERIOD_MS));
                break;

            case STATE_PRECALENTAMIENTO: {
                /* PID controla SSR para llevar temperatura a 65°C */
                float duty = pid_compute(&pid, current_temp);
                ESP_LOGI(TAG, "PRECAL T=%.1fC SP=%.1fC D=%.0f%%",
                         current_temp, TEMP_PRECALENT, duty * 100.0f);
                update_lcd(current_temp, temp_ok);
                apply_ssr_duty(duty);

                /* Transición: cuando llega a 63°C (setpoint - 2°C de margen) */
                if (current_temp >= TEMP_PRECALENT - 2.0f) {
                    pid_set_setpoint(&pid, TEMP_VAPOR_ALTA);
                    change_state(STATE_VAPOR_ALTA);
                }
                break;
            }

            case STATE_VAPOR_ALTA: {
                /* PID mantiene 80°C por 3 minutos — máxima generación de vapor */
                float duty = pid_compute(&pid, current_temp);
                ESP_LOGI(TAG, "ALTA T=%.1fC SP=%.1fC D=%.0f%% t=%ds",
                         current_temp, TEMP_VAPOR_ALTA, duty * 100.0f, state_seconds);
                update_lcd(current_temp, temp_ok);
                apply_ssr_duty(duty);
                state_seconds += PID_PERIOD_MS / 1000;

                if (state_seconds >= TIME_VAPOR_ALTA_S) {
                    pid_set_setpoint(&pid, TEMP_VAPOR_MEDIA);
                    change_state(STATE_VAPOR_MEDIA);
                }
                break;
            }

            case STATE_VAPOR_MEDIA: {
                /* PID mantiene 70°C por 3 minutos — vapor moderado */
                float duty = pid_compute(&pid, current_temp);
                ESP_LOGI(TAG, "MEDIA T=%.1fC SP=%.1fC D=%.0f%% t=%ds",
                         current_temp, TEMP_VAPOR_MEDIA, duty * 100.0f, state_seconds);
                update_lcd(current_temp, temp_ok);
                apply_ssr_duty(duty);
                state_seconds += PID_PERIOD_MS / 1000;

                if (state_seconds >= TIME_VAPOR_MEDIA_S) {
                    pid_set_setpoint(&pid, TEMP_VAPOR_BAJA);
                    change_state(STATE_VAPOR_BAJA);
                }
                break;
            }

            case STATE_VAPOR_BAJA: {
                /* PID mantiene 60°C por 2 minutos — vapor suave, última fase */
                float duty = pid_compute(&pid, current_temp);
                ESP_LOGI(TAG, "BAJA T=%.1fC SP=%.1fC D=%.0f%% t=%ds",
                         current_temp, TEMP_VAPOR_BAJA, duty * 100.0f, state_seconds);
                update_lcd(current_temp, temp_ok);
                apply_ssr_duty(duty);
                state_seconds += PID_PERIOD_MS / 1000;

                /* Al terminar, apagar caldera y encender ventilador */
                if (state_seconds >= TIME_VAPOR_BAJA_S) {
                    ssr_set(false);
                    pid_reset(&pid);
                    fan_set(true);    /* Ventilador ON inmediato al terminar vapor */
                    change_state(STATE_ENFRIAMIENTO);
                }
                break;
            }

            case STATE_ENFRIAMIENTO:
                /* Caldera OFF, ventilador ON hasta temperatura segura (40°C) */
                ssr_set(false);
                fan_set(true);
                current_duty = 0.0f;
                ESP_LOGI(TAG, "ENFRIANDO T=%.1fC", current_temp);
                update_lcd(current_temp, temp_ok);
                vTaskDelay(pdMS_TO_TICKS(PID_PERIOD_MS));

                if (current_temp < TEMP_SAFE_OPEN) {
                    fan_set(false);
                    change_state(STATE_FIN);
                }
                break;

            case STATE_FIN:
                /* Todo apagado, mostrar "Seguro abrir" hasta que presionen botón */
                all_off();
                lcd_set_cursor(0, 0);
                lcd_print("Seguro abrir    ");
                lcd_set_cursor(1, 0);
                {
                    char fin_line[17];
                    snprintf(fin_line, sizeof(fin_line), "T:%.1fC Fin     ", current_temp);
                    lcd_print(fin_line);
                }
                vTaskDelay(pdMS_TO_TICKS(PID_PERIOD_MS));
                break;

            case STATE_ERROR:
                /* Caldera OFF, ventilador ON para enfriar.
                 * Se sale de este estado presionando botón cuando temp < 85°C */
                ssr_set(false);
                fan_set(true);      /* Enfriar aunque sea error */
                current_duty = 0.0f;
                update_lcd(current_temp, temp_ok);
                vTaskDelay(pdMS_TO_TICKS(PID_PERIOD_MS));
                break;
        }
    }
}
