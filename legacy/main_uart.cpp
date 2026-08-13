#include <iostream>
#include <chrono>
#include <thread>
#include <array>

#include "stm32canbusif.h"
#include "height_control.h"
#include "valve_test.h"

// PRUEBA DE GIT - VERSION 2

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cout
            << "Uso: "
            << argv[0]
            << " /dev/ttyACM0"
            << std::endl;

        return 1;
    }

    try
    {
        // ----------------------------------------------------
        // Crear interfaz con STM32
        // ----------------------------------------------------

        stm32canbus_serialif stm32(
            argv[1],
            115200
        );

        /*
         * Consignas de altura producidas por la IA.
         * Más adelante la nube 3D actualizará este objeto.
         */
        height_control heightControl;

        std::array<bool, height_control::BODY_COUNT>
            targetWasActive{};

        


        // ----------------------------------------------------
        // Encoder 'E'
        // ----------------------------------------------------

        std::array<uint16_t, 6> test_heights =
        {
            200,
            500,
            500,
            500,
            900,
            500
        };

        valve_test_set_targets(
            heightControl,
            test_heights
        );

        stm32.set_encoder_callback(
            [](const stm32canbus_serialif::encoder_state& state)
            {
                std::cout << "ENCODERS: ";

                for (std::size_t i = 0;
                     i < state.height_mm.size();
                     ++i)
                {
                    std::cout
                        << state.height_mm[i]
                        << " mm";

                    if (i + 1 < state.height_mm.size())
                    {
                        std::cout << " | ";
                    }
                }

                std::cout << std::endl;
            }
        );


        // ----------------------------------------------------
        // Válvulas 'F'
        // ----------------------------------------------------

        stm32.set_valve_callback(
            [](const stm32canbus_serialif::valve_state& state)
            {
                std::cout << "VALVULAS: ";

                for (std::size_t i = 0;
                     i < state.command.size();
                     ++i)
                {
                    std::cout << state.command[i];

                    if (i + 1 < state.command.size())
                    {
                        std::cout << " | ";
                    }
                }

                std::cout << std::endl;
            }
        );


        // ----------------------------------------------------
        // Diagnóstico 'G'
        // ----------------------------------------------------

        stm32.set_diagnostic_callback(
            [](const stm32canbus_serialif::diagnostic_state& state)
            {
                std::cout
                    << "DIAGNOSTICO: dropped="
                    << state.axiomatic_rx_dropped

                    << " uart_errors="
                    << state.uart_error_count

                    << " tx_queue_dropped="
                    << state.jetson_tx_queue_dropped

                    << " tx_dma_errors="
                    << state.jetson_tx_dma_errors

                    << " jetson="
                    << state.jetson_connection_ok

                    << " modulos="
                    << static_cast<int>(
                        state.axiomatic_modules
                    )

                    << " system_faults=0x"
                    << std::hex
                    << state.system_faults

                    << " body_faults=[0x"
                    << state.body_faults[0]

                    << ",0x"
                    << state.body_faults[1]

                    << ",0x"
                    << state.body_faults[2]

                    << ",0x"
                    << state.body_faults[3]

                    << ",0x"
                    << state.body_faults[4]

                    << ",0x"
                    << state.body_faults[5]

                    << "]"
                    << std::dec
                    << std::endl;
            }
        );


        // ----------------------------------------------------
        // Estado STM32 'H'
        // ----------------------------------------------------

        stm32.set_stm32_state_callback(
            [](const stm32canbus_serialif::stm32_state& state)
            {
                std::cout
                    << "STM32: uptime="
                    << state.uptime_ticks

                    << " jetson="
                    << state.jetson_connection_ok

                    << " encoders="
                    << static_cast<int>(
                        state.encoder_count
                    )

                    << " cuerpos="
                    << static_cast<int>(
                        state.body_count
                    )

                    << " axiomatic="
                    << static_cast<int>(
                        state.axiomatic_modules
                    )

                    << std::endl;
            }
        );


        // ----------------------------------------------------
        // IMU 'I'
        // ----------------------------------------------------

        stm32.set_imu_callback(
            [](const stm32canbus_serialif::imu_state& state)
            {
                std::cout
                    << "IMU: valid="
                    << state.valid

                    << " roll="
                    << state.roll_deg

                    << " pitch="
                    << state.pitch_deg

                    << " gravity="
                    << state.gravity_deg

                    << " gyro=("
                    << state.gyro_roll_dps << ", "
                    << state.gyro_pitch_dps << ", "
                    << state.gyro_yaw_dps << ")"

                    << " accel=("
                    << state.accel_x_mps2 << ", "
                    << state.accel_y_mps2 << ", "
                    << state.accel_z_mps2 << ")"

                    << std::endl;
            }
        );


        // ----------------------------------------------------
        // Arrancar comunicación
        // ----------------------------------------------------

        stm32.start();

        std::cout
            << "HAGIE iniciado."
            << std::endl;

        std::cout
            << "Puerto: "
            << argv[1]
            << " @ 115200"
            << std::endl;


        // ----------------------------------------------------
        // Heartbeat periódico
        // ----------------------------------------------------

        auto lastHeartbeat =
            std::chrono::steady_clock::now();

        


        while (true)
        {
            auto now =
                std::chrono::steady_clock::now();

            auto dt =
                std::chrono::duration_cast<
                    std::chrono::milliseconds>(
                        now - lastHeartbeat
                    ).count();

            if (dt > 150)
            {
                std::cout
                    << "!!! HEARTBEAT ATRASADO: "
                    << dt
                    << " ms !!!"
                    << std::endl;
            }

            lastHeartbeat = now;


            // ------------------------------------------------
            // Heartbeat
            // ------------------------------------------------

            stm32.send_heartbeat();
                     


            // ------------------------------------------------
            // Consignas de altura
            // ------------------------------------------------

            /*
             * Enviar a la STM32 las alturas objetivo
             * que actualmente sean válidas.
             *
             * Todo el TX serie se realiza desde este mismo hilo:
             * heartbeat + consignas.
             */
            for (std::size_t body = 0;
                 body < height_control::BODY_COUNT;
                 ++body)
            {
                
                               
                bool targetActive =
                    heightControl.has_target(
                        static_cast<uint8_t>(body)
                    );

                if (targetActive)
                {
                    /*
                     * Hay una consigna reciente de la IA.
                     */
                    stm32.set_target_height(
                        static_cast<uint8_t>(body),
                        heightControl.get_target(
                            static_cast<uint8_t>(body)
                        )
                    );
                }
                else if (targetWasActive[body])
                {
                    /*
                     * La consigna estaba activa y venció.
                     *
                     * Detener ese cuerpo.
                     */
                    stm32.set_valve_command(
                        static_cast<uint8_t>(body),
                        0
                    );
                }

                targetWasActive[body] =
                    targetActive;
            }


            std::this_thread::sleep_for(
                std::chrono::milliseconds(100)
            );
        }
    }
    catch (const std::exception& e)
    {
        std::cerr
            << "ERROR: "
            << e.what()
            << std::endl;

        return 1;
    }

    return 0;
}