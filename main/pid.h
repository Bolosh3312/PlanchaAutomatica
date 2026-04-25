/**
 * @file pid.h
 * @brief Controlador PID genérico con anti-windup
 * 
 * Este módulo implementa un controlador PID (Proporcional-Integral-Derivativo)
 * diseñado para el control térmico de la caldera de vapor.
 * 
 * Características:
 * - Derivada sobre medición (no sobre error) para evitar picos al cambiar setpoint
 * - Anti-windup por clamping del término integral
 * - Salida normalizada entre 0.0 y 1.0 (duty cycle para control time-proportional)
 * 
 * Proyecto: Plancha automática de vapor — Proyecto Mecatrónico II
 * Universidad Panamericana, Zapopan
 */

#ifndef PID_H
#define PID_H

/**
 * @brief Estructura que contiene todos los parámetros y estado del controlador PID
 */
typedef struct {
    /* Ganancias del controlador */
    float kp;               /* Ganancia proporcional: respuesta al error actual */
    float ki;               /* Ganancia integral: elimina error estacionario */
    float kd;               /* Ganancia derivativa: frena cambios bruscos */

    /* Referencia */
    float setpoint;          /* Temperatura objetivo en °C */

    /* Estado interno del controlador */
    float integral;          /* Acumulador del término integral */
    float prev_measurement;  /* Última medición, para calcular derivada */

    /* Límites anti-windup (evita que el integral se acumule sin control) */
    float integral_min;      /* Límite inferior del integral (-1.0) */
    float integral_max;      /* Límite superior del integral (1.0) */

    /* Límites de salida */
    float output_min;        /* Salida mínima (0.0 = SSR siempre apagado) */
    float output_max;        /* Salida máxima (1.0 = SSR siempre encendido) */

    /* Configuración temporal */
    float dt;                /* Período de muestreo en segundos */
    int first_run;           /* Flag para inicializar la derivada en la primera ejecución */
} pid_controller_t;

/**
 * @brief Inicializa el controlador PID con ganancias y período dados
 * @param pid Puntero a la estructura PID
 * @param kp Ganancia proporcional
 * @param ki Ganancia integral
 * @param kd Ganancia derivativa
 * @param dt Período de muestreo en segundos (ej: 2.0 para ciclo de 2s)
 */
void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float dt);

/**
 * @brief Ejecuta un paso del controlador PID
 * @param pid Puntero a la estructura PID
 * @param measurement Temperatura actual leída del sensor en °C
 * @return Duty cycle entre 0.0 y 1.0 (porcentaje de tiempo que el SSR debe estar ON)
 */
float pid_compute(pid_controller_t *pid, float measurement);

/**
 * @brief Cambia la temperatura objetivo del controlador
 * @param pid Puntero a la estructura PID
 * @param setpoint Nueva temperatura objetivo en °C
 * @note Si el cambio es mayor a 2°C, resetea el integral para evitar overshoot
 */
void pid_set_setpoint(pid_controller_t *pid, float setpoint);

/**
 * @brief Resetea el estado interno del PID (integral y derivada)
 * @param pid Puntero a la estructura PID
 * @note Llamar al cambiar de estado en la FSM o después de un paro de emergencia
 */
void pid_reset(pid_controller_t *pid);

#endif
