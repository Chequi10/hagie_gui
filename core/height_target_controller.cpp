#include "height_target_controller.h"


HeightTargetController::HeightTargetController()
{
    /*
     * Configuración inicial provisoria.
     *
     * Offset = 0:
     * objetivo calculado = altura de visión.
     *
     * Después definiremos el offset real
     * según la geometría mecánica del Hagie.
     */
    for (std::size_t body = 0;
         body < BODY_COUNT;
         ++body)
    {
        bodyConfiguration[body].offset_mm =
            0;


        bodyConfiguration[body].min_height_mm =
            0;


        bodyConfiguration[body].max_height_mm =
            2000;
    }
}


void HeightTargetController::setBodyConfiguration(
    std::size_t body,
    const BodyConfiguration& configuration)
{
    if (body >= BODY_COUNT)
    {
        return;
    }


    /*
     * No aceptar límites inválidos.
     */
    if (configuration.min_height_mm >=
        configuration.max_height_mm)
    {
        return;
    }


    bodyConfiguration[body] =
        configuration;
}


HeightTargetController::BodyConfiguration
HeightTargetController::getBodyConfiguration(
    std::size_t body) const
{
    if (body >= BODY_COUNT)
    {
        return BodyConfiguration {};
    }


    return bodyConfiguration[body];
}


HeightTargetController::TargetResult
HeightTargetController::calculateTarget(
    std::size_t body,
    uint16_t vision_height_mm,
    bool vision_valid) const
{
    TargetResult result;


    /*
     * Cuerpo inválido.
     */
    if (body >= BODY_COUNT)
    {
        return result;
    }


    /*
     * Si visión no tiene una medición válida,
     * NO generamos ninguna consigna.
     */
    if (!vision_valid)
    {
        return result;
    }


    const BodyConfiguration& config =
        bodyConfiguration[body];


    /*
     * Aplicar offset usando int32_t para
     * permitir offsets negativos.
     */
    int32_t calculatedTarget =
        static_cast<int32_t>(
            vision_height_mm
        )
        +
        config.offset_mm;


    /*
     * Limitar al mínimo configurado.
     */
    if (calculatedTarget <
        static_cast<int32_t>(
            config.min_height_mm
        ))
    {
        calculatedTarget =
            config.min_height_mm;
    }


    /*
     * Limitar al máximo configurado.
     */
    if (calculatedTarget >
        static_cast<int32_t>(
            config.max_height_mm
        ))
    {
        calculatedTarget =
            config.max_height_mm;
    }


    result.target_mm =
        static_cast<uint16_t>(
            calculatedTarget
        );


    result.valid =
        true;


    return result;
}