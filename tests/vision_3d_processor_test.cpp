#include <iostream>

#include "vision/vision_3d_processor.h"


int main()
{
    Vision3DProcessor processor;

    Vision3DProcessor::PointCloud cloud;


    /*
     * ========================================================
     * NUBE ARTIFICIAL
     * ========================================================
     *
     * Las regiones actuales son:
     *
     * Cuerpo 1: -3 .. -2
     * Cuerpo 2: -2 .. -1
     * Cuerpo 3: -1 ..  0
     * Cuerpo 4:  0 ..  1
     * Cuerpo 5:  1 ..  2
     * Cuerpo 6:  2 ..  3
     *
     * Z está expresado en metros.
     */


    /*
     * Cuerpo 1 -> 400 mm
     */
    cloud.push_back(
        {-2.5f, 0.0f, 0.40f}
    );


    /*
     * Cuerpo 2 -> 500 mm
     */
    cloud.push_back(
        {-1.5f, 0.0f, 0.50f}
    );


    /*
     * Cuerpo 3 -> 600 mm
     */
    cloud.push_back(
        {-0.5f, 0.0f, 0.60f}
    );


    /*
     * Cuerpo 4 -> 700 mm
     */
    cloud.push_back(
        {0.5f, 0.0f, 0.70f}
    );


    /*
     * Cuerpo 5 -> 800 mm
     */
    cloud.push_back(
        {1.5f, 0.0f, 0.80f}
    );


    /*
     * Cuerpo 6 -> 900 mm
     */
    cloud.push_back(
        {2.5f, 0.0f, 0.90f}
    );


    /*
     * Punto extra dentro del cuerpo 3.
     *
     * Como el algoritmo actual toma max Z,
     * debería seguir quedándose con 600 mm.
     */
    cloud.push_back(
        {-0.4f, 0.0f, 0.42f}
    );


    /*
     * Punto fuera de todas las regiones.
     *
     * Debe ignorarse.
     */
    cloud.push_back(
        {10.0f, 0.0f, 5.0f}
    );


    VisionHeightSource::VisionResult result =
        processor.processPointCloud(
            cloud
        );


    std::cout
        << "Resultado Vision3DProcessor"
        << std::endl;


    for (std::size_t body = 0;
         body < Vision3DProcessor::BODY_COUNT;
         ++body)
    {
        std::cout
            << "Cuerpo "
            << (body + 1)
            << ": ";

        if (!result.bodies[body].valid)
        {
            std::cout
                << "NO VALIDO"
                << std::endl;

            continue;
        }

        std::cout
            << result.bodies[body].height_mm
            << " mm"
            << " | timestamp="
            << result.bodies[body].timestamp_ms
            << std::endl;
    }


    return 0;
}