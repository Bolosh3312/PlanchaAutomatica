/**
 * @file pid.c
 * @brief Implementación del controlador PID
 * 
 * El PID calcula cuánto tiempo debe estar encendido el SSR (duty cycle)
 * basándose en la diferencia entre la temperatura deseada y la actual.
 * 
 * Ejemplo: si el setpoint es 80°C y la temperatura actual es 60°C,
 * el PID dará un duty alto (cercano a 1.0) para calentar rápido.
 * Conforme se acerca a 80°C, el duty baja para no pasarse.
 * 
 * Proyecto: Plancha automática de vapor — Proyecto Mecatrónico II
 */

#include "pid.h"

/**
 * @brief Limita un valor entre un mínimo y un máximo
 * @param value Valor a limitar
 * @param min Límite inferior
 * @param max Límite superior
 * @return Valor limitado
 */
static float clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float dt) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->first_run = 1;

    /* Límites del integral: evitan que se acumule más allá de lo que
       la salida puede usar (anti-windup) */
    pid->integral_min = -1.0f;
    pid->integral_max = 1.0f;

    /* Salida = duty cycle del SSR: 0% a 100% */
    pid->output_min = 0.0f;
    pid->output_max = 1.0f;
}

float pid_compute(pid_controller_t *pid, float measurement) {
    /* Error = diferencia entre lo que queremos y lo que tenemos */
    float error = pid->setpoint - measurement;

    /* Término Proporcional: reacción directa al error actual
       Error grande → corrección grande */
    float p_term = pid->kp * error;

    /* Término Integral: acumula error con el tiempo para eliminar
       el error estacionario (cuando la temp se queda un poco abajo del setpoint) */
    pid->integral += pid->ki * error * pid->dt;
    pid->integral = clamp(pid->integral, pid->integral_min, pid->integral_max);
    float i_term = pid->integral;

    /* Término Derivativo: frena cambios bruscos de temperatura
       Se calcula sobre la MEDICIÓN (no sobre el error) para evitar
       picos cuando se cambia el setpoint.
       Signo negativo: si la temperatura sube rápido, reduce la salida */
    float d_term = 0.0f;
    if (!pid->first_run) {
        float d_measurement = (measurement - pid->prev_measurement) / pid->dt;
        d_term = -pid->kd * d_measurement;
    } else {
        pid->first_run = 0;
    }
    pid->prev_measurement = measurement;

    /* Salida final: suma de los tres términos, limitada entre 0 y 1 */
    float output = p_term + i_term + d_term;
    return clamp(output, pid->output_min, pid->output_max);
}

void pid_set_setpoint(pid_controller_t *pid, float setpoint) {
    /* Si el cambio de setpoint es grande (>2°C), resetear el integral
       para evitar overshoot por acumulación del estado anterior */
    float diff = setpoint - pid->setpoint;
    if (diff > 2.0f || diff < -2.0f) {
        pid->integral = 0.0f;
    }
    pid->setpoint = setpoint;
}

void pid_reset(pid_controller_t *pid) {
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->first_run = 1;
}
