#pragma once

#include <array>
#include <cstdint>

#include "hagie_state.h"


/*
 * ============================================================
 * CONFIGURACIÓN HIDRÁULICA HAGIE
 * ============================================================
 *
 * Esta estructura contiene parámetros que describen
 * el comportamiento hidráulico de la máquina.
 *
 * Hay dos grupos:
 *
 * 1. Parámetros físicos/comunes:
 *    pueden utilizarse tanto en SIMULACIÓN como en REAL.
 *
 * 2. Parámetros exclusivos del simulador:
 *    solamente modelan artificialmente la respuesta.
 *
 * IMPORTANTE:
 *
 * En modo REAL esta estructura nunca genera alturas.
 * La altura real siempre proviene del encoder físico.
 */
struct HydraulicConfiguration
{
    /*
     * ========================================================
     * CONFIGURACIÓN POR CUERPO
     * ========================================================
     */
    struct BodyConfiguration
    {
        /*
         * Velocidad máxima física estimada.
         */
        double max_up_speed_mm_s =
            80.0;

        double max_down_speed_mm_s =
            100.0;


        /*
         * Error dentro del cual consideramos
         * que el cuerpo está correctamente posicionado.
         */
        double deadband_mm =
            5.0;


        /*
         * Peso/prioridad hidráulica.
         *
         * 1.0 = normal.
         *
         * Más adelante permitirá priorizar
         * determinados cuerpos cuando falte caudal.
         */
        double priority =
            1.0;
    };


    std::array<
        BodyConfiguration,
        HagieState::BODY_COUNT
    > bodies;


    /*
     * ========================================================
     * CONFIGURACIÓN GENERAL DEL SISTEMA HIDRÁULICO
     * ========================================================
     */


    /*
     * Capacidad hidráulica disponible.
     *
     * 100 % = capacidad nominal.
     *
     * Nos permitirá simular y posteriormente gestionar
     * la pérdida de rendimiento cuando varios cuerpos
     * demandan caudal simultáneamente.
     */
    double hydraulic_capacity_percent =
        100.0;


    /*
     * A partir de este error consideramos
     * que el cuerpo está solicitando
     * un movimiento hidráulico grande.
     */
    double large_movement_threshold_mm =
        100.0;


    /*
     * Penalización adicional cuando existen
     * varias demandas grandes simultáneas.
     *
     * Ejemplo:
     *
     * 0.30 = 30 %
     */
    double simultaneous_demand_penalty =
        0.30;


    /*
     * Cantidad máxima deseada de cuerpos
     * con demanda hidráulica alta simultánea.
     *
     * No significa necesariamente bloquear los demás.
     * El HydraulicDemandManager decidirá posteriormente
     * cómo repartir el caudal.
     */
    int max_high_demand_bodies =
        2;


    /*
     * ========================================================
     * PARÁMETROS EXCLUSIVOS DE SIMULACIÓN
     * ========================================================
     */
    struct SimulationConfiguration
    {
        /*
         * Retardo entre la orden y el comienzo
         * del movimiento.
         */
        uint32_t response_delay_ms =
            150;


        /*
         * Inercia artificial.
         *
         * 0.0 = respuesta instantánea dentro
         *       del límite de velocidad.
         *
         * Valores mayores producen una transición
         * más progresiva.
         */
        double inertia =
            0.15;


        /*
         * Ruido opcional de encoder simulado.
         *
         * Inicialmente queda en cero.
         */
        double encoder_noise_mm =
            0.0;


        /*
         * Permite activar/desactivar la simulación
         * de pérdida de caudal compartido.
         */
        bool shared_flow_enabled =
            true;
    };


    SimulationConfiguration simulation;
};
