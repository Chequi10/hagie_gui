#ifndef HEIGHT_TARGET_CONTROLLER_H
#define HEIGHT_TARGET_CONTROLLER_H


#include <array>
#include <cstddef>
#include <cstdint>


class HeightTargetController
{
public:

    static constexpr std::size_t BODY_COUNT = 6;


    struct BodyConfiguration
    {
        /*
         * Diferencia entre la altura detectada
         * por visión y la posición deseada
         * del cuerpo.
         *
         * Por ahora queda en 0.
         *
         * Más adelante definiremos físicamente
         * cuánto debe quedar el cabezal respecto
         * de la planta/panoja.
         */
        int32_t offset_mm = 0;


        /*
         * Límites permitidos para el cuerpo.
         */
        uint16_t min_height_mm = 0;

        uint16_t max_height_mm = 2000;
    };


    struct TargetResult
    {
        uint16_t target_mm = 0;

        bool valid = false;
    };


    HeightTargetController();


    /*
     * Configuración individual por cuerpo.
     */
    void setBodyConfiguration(
        std::size_t body,
        const BodyConfiguration& configuration
    );


    BodyConfiguration getBodyConfiguration(
        std::size_t body
    ) const;


    /*
     * Calcular objetivo de un cuerpo.
     *
     * vision_height_mm:
     * altura entregada por vision/.
     *
     * vision_valid:
     * indica si la medición 3D es válida.
     */
    TargetResult calculateTarget(
        std::size_t body,
        uint16_t vision_height_mm,
        bool vision_valid
    ) const;


private:

    std::array<
        BodyConfiguration,
        BODY_COUNT
    > bodyConfiguration {};
};


#endif