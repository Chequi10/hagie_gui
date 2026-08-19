#include <iostream>
#include <cmath>

#include "vision/vision_3d_processor.h"


static void printResult(
    const VisionHeightSource::VisionResult& result)
{
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
}


int main()
{
    /*
     * ========================================================
     * TEST 1
     * EJES NORMALES:
     *
     * X = lateral
     * Y = longitudinal
     * Z = vertical
     * ========================================================
     */
    {
        Vision3DProcessor processor;

        Vision3DProcessor::PointCloud cloud;


        cloud.push_back(
            {-2.5f, 0.0f, 0.40f}
        );

        cloud.push_back(
            {-1.5f, 0.0f, 0.50f}
        );

        cloud.push_back(
            {-0.5f, 0.0f, 0.60f}
        );

        cloud.push_back(
            {0.5f, 0.0f, 0.70f}
        );

        cloud.push_back(
            {1.5f, 0.0f, 0.80f}
        );

        cloud.push_back(
            {2.5f, 0.0f, 0.90f}
        );


        /*
         * Punto extra dentro del cuerpo 3.
         *
         * Como por ahora usamos max Z,
         * debe seguir dando 600 mm.
         */
        cloud.push_back(
            {-0.4f, 0.0f, 0.42f}
        );


        /*
         * Punto fuera de todas las regiones.
         */
        cloud.push_back(
            {10.0f, 0.0f, 5.0f}
        );


        VisionHeightSource::VisionResult result =
            processor.processPointCloud(
                cloud
            );


        std::cout
            << std::endl
            << "========================================"
            << std::endl
            << "TEST 1 - EJES NORMALES"
            << std::endl
            << "X=lateral Y=longitudinal Z=vertical"
            << std::endl
            << "========================================"
            << std::endl;


        printResult(
            result
        );
    }


    /*
     * ========================================================
     * TEST 2
     * REMAPEO DE EJES
     *
     * X de entrada -> lateral
     * Z de entrada -> longitudinal
     * Y de entrada -> vertical
     *
     * Es decir:
     *
     * lateral      = X
     * longitudinal = Z
     * vertical     = Y
     * ========================================================
     */
    {
        Vision3DProcessor processor;


        Vision3DProcessor::AxisMapping mapping;

        mapping.lateral =
            Vision3DProcessor::Axis::X;

        mapping.longitudinal =
            Vision3DProcessor::Axis::Z;

        mapping.vertical =
            Vision3DProcessor::Axis::Y;


        mapping.lateral_sign =
            1;

        mapping.longitudinal_sign =
            1;

        mapping.vertical_sign =
            1;


        processor.setAxisMapping(
            mapping
        );


        Vision3DProcessor::PointCloud cloud;


        /*
         * OJO:
         *
         * Ahora la ALTURA está en Y,
         * no en Z.
         */


        /*
         * Cuerpo 1 -> 400 mm
         */
        cloud.push_back(
            {-2.5f, 0.40f, 0.0f}
        );


        /*
         * Cuerpo 2 -> 500 mm
         */
        cloud.push_back(
            {-1.5f, 0.50f, 0.0f}
        );


        /*
         * Cuerpo 3 -> 600 mm
         */
        cloud.push_back(
            {-0.5f, 0.60f, 0.0f}
        );


        /*
         * Cuerpo 4 -> 700 mm
         */
        cloud.push_back(
            {0.5f, 0.70f, 0.0f}
        );


        /*
         * Cuerpo 5 -> 800 mm
         */
        cloud.push_back(
            {1.5f, 0.80f, 0.0f}
        );


        /*
         * Cuerpo 6 -> 900 mm
         */
        cloud.push_back(
            {2.5f, 0.90f, 0.0f}
        );


        VisionHeightSource::VisionResult result =
            processor.processPointCloud(
                cloud
            );

        Vision3DProcessor processorImu;

        Vision3DProcessor::Orientation orientation;

        orientation.roll_deg = 0.0f;
        orientation.pitch_deg = 0.0f;
        orientation.valid = true;

        processorImu.setOrientation(
            orientation
        );    


        std::cout
            << std::endl
            << "========================================"
            << std::endl
            << "TEST 2 - EJES REMAPEADOS"
            << std::endl
            << "X=lateral Z=longitudinal Y=vertical"
            << std::endl
            << "========================================"
            << std::endl;


        std::cout
    << std::endl
    << "========================================"
    << std::endl
    << "TEST 3 - IMU"
    << std::endl
    << "========================================"
    << std::endl;


    /*
    * Caso A:
    * IMU válida pero sin inclinación.
    */
    Vision3DProcessor processorImuZero;

    Vision3DProcessor::Orientation orientationZero;

    orientationZero.roll_deg =
        0.0f;

    orientationZero.pitch_deg =
        0.0f;

    orientationZero.valid =
        true;

    processorImuZero.setOrientation(
        orientationZero
    );


    Vision3DProcessor::PointCloud cloudImuZero;

    cloudImuZero.push_back(
        {-2.5f, 0.0f, 0.40f}
    );

    VisionHeightSource::VisionResult resultImuZero =
        processorImuZero.processPointCloud(
            cloudImuZero
        );


    std::cout
        << "IMU 0 deg -> Cuerpo 1: "
        << resultImuZero.bodies[0].height_mm
        << " mm"
        << std::endl;


    /*
    * Caso B:
    * Roll de +10 grados.
    */
    Vision3DProcessor processorImuRoll;

    Vision3DProcessor::Orientation orientationRoll;

    orientationRoll.roll_deg =
        10.0f;

    orientationRoll.pitch_deg =
        0.0f;

    orientationRoll.valid =
        true;

    processorImuRoll.setOrientation(
        orientationRoll
    );


    Vision3DProcessor::PointCloud cloudImuRoll;

    cloudImuRoll.push_back(
        {-2.5f, 0.0f, 0.40f}
    );

    VisionHeightSource::VisionResult resultImuRoll =
        processorImuRoll.processPointCloud(
            cloudImuRoll
        );


    std::cout
        << "IMU roll +10 deg -> Cuerpo 1: ";

    if (
        resultImuRoll
            .bodies[0]
            .valid
    )
    {
        std::cout
            << resultImuRoll
                .bodies[0]
                .height_mm
            << " mm";
    }
    else
    {
        std::cout
            << "NO VALIDO";
    }

    std::cout
        << std::endl;   
        
    std::cout
    << std::endl
    << "========================================"
    << std::endl
    << "TEST 4 - CORRECCION FISICA IMU"
    << std::endl
    << "========================================"
    << std::endl;


    /*
    * Punto original nivelado.
    *
    * Cuerpo 1:
    *
    * x = -2.5 m
    * z = 0.4 m
    */
    Vision3DProcessor::Point3D originalPoint {
        -2.5f,
        0.0f,
        0.40f
    };


    /*
    * Simular que la máquina/cámara quedó inclinada
    * +10 grados.
    *
    * Lo hacemos manualmente para generar
    * un punto "medido" inclinado.
    */
    constexpr float DEG_TO_RAD_TEST =
        3.14159265358979323846f / 180.0f;

    float testAngle =
        10.0f * DEG_TO_RAD_TEST;

    float c =
        std::cos(testAngle);

    float s =
        std::sin(testAngle);


    Vision3DProcessor::Point3D tiltedPoint;

    tiltedPoint.x =
        c * originalPoint.x +
        s * originalPoint.z;

    tiltedPoint.y =
        originalPoint.y;

    tiltedPoint.z =
        -s * originalPoint.x +
        c * originalPoint.z;


    /*
    * Ahora el procesador recibe el punto inclinado.
    *
    * Para desinclinarlo usamos IMU = -10 grados.
    */
    Vision3DProcessor processorPhysicalTest;

    Vision3DProcessor::Orientation correctionOrientation;

    correctionOrientation.roll_deg =
        10.0f;

    correctionOrientation.pitch_deg =
        0.0f;

    correctionOrientation.valid =
        true;

    processorPhysicalTest.setOrientation(
        correctionOrientation
    );


    Vision3DProcessor::PointCloud physicalCloud;

    physicalCloud.push_back(
        tiltedPoint
    );


    VisionHeightSource::VisionResult physicalResult =
        processorPhysicalTest.processPointCloud(
            physicalCloud
        );


    std::cout
        << "Altura original esperada: 400 mm"
        << std::endl;

    std::cout
        << "Altura recuperada: ";

    if (physicalResult.bodies[0].valid)
    {
        std::cout
            << physicalResult.bodies[0].height_mm
            << " mm";
    }
    else
    {
        std::cout
            << "NO VALIDO";
    }

    std::cout
        << std::endl;    

    std::cout
    << std::endl
    << "========================================"
    << std::endl
    << "TEST 5 - ASIGNACION POR CAMARA"
    << std::endl
    << "========================================"
    << std::endl;


    Vision3DProcessor processorCamera;


    /*
    * Configurar CAMARA 0
    * para atender solamente:
    *
    * Cuerpo 1
    * Cuerpo 2
    */
    Vision3DProcessor::CameraConfig camera0;

    camera0.enabled =
        true;

    camera0.body_enabled[0] =
        true;

    camera0.body_enabled[1] =
        true;


    processorCamera.setCameraConfig(
        0,
        camera0
    );


    /*
    * Nube con puntos para los 6 cuerpos.
    */
    Vision3DProcessor::PointCloud cameraCloud;

    cameraCloud.push_back(
        {-2.5f, 0.0f, 0.40f}
    );

    cameraCloud.push_back(
        {-1.5f, 0.0f, 0.50f}
    );

    cameraCloud.push_back(
        {-0.5f, 0.0f, 0.60f}
    );

    cameraCloud.push_back(
        {0.5f, 0.0f, 0.70f}
    );

    cameraCloud.push_back(
        {1.5f, 0.0f, 0.80f}
    );

    cameraCloud.push_back(
        {2.5f, 0.0f, 0.90f}
    );


    /*
    * Procesar específicamente como CAMARA 0.
    */
    VisionHeightSource::VisionResult cameraResult =
        processorCamera.processPointCloud(
            0,
            cameraCloud
        );


    for (std::size_t body = 0;
        body < Vision3DProcessor::BODY_COUNT;
        ++body)
    {
        std::cout
            << "Cuerpo "
            << (body + 1)
            << ": ";

        if (cameraResult.bodies[body].valid)
        {
            std::cout
                << cameraResult
                    .bodies[body]
                    .height_mm
                << " mm";
        }
        else
        {
            std::cout
                << "NO VALIDO";
        }

        std::cout
            << std::endl;
    }

        
        
        std::cout
        << std::endl
        << "========================================"
        << std::endl
        << "TEST 6 - TRES CAMARAS"
        << std::endl
        << "========================================"
        << std::endl;


    Vision3DProcessor processorThreeCameras;


    /*
    * ========================================================
    * CAMARA 0 -> CUERPOS 1 Y 2
    * ========================================================
    */
    Vision3DProcessor::CameraConfig cameraConfig0;

    cameraConfig0.enabled = true;

    cameraConfig0.body_enabled[0] = true;
    cameraConfig0.body_enabled[1] = true;

    processorThreeCameras.setCameraConfig(
        0,
        cameraConfig0
    );


    /*
    * ========================================================
    * CAMARA 1 -> CUERPOS 3 Y 4
    * ========================================================
    */
    Vision3DProcessor::CameraConfig cameraConfig1;

    cameraConfig1.enabled = true;

    cameraConfig1.body_enabled[2] = true;
    cameraConfig1.body_enabled[3] = true;

    processorThreeCameras.setCameraConfig(
        1,
        cameraConfig1
    );


    /*
    * ========================================================
    * CAMARA 2 -> CUERPOS 5 Y 6
    * ========================================================
    */
    Vision3DProcessor::CameraConfig cameraConfig2;

    cameraConfig2.enabled = true;

    cameraConfig2.body_enabled[4] = true;
    cameraConfig2.body_enabled[5] = true;

    processorThreeCameras.setCameraConfig(
        2,
        cameraConfig2
    );


    /*
    * ========================================================
    * NUBE CAMARA 0
    * ========================================================
    */
    Vision3DProcessor::PointCloud cloudCamera0;

    cloudCamera0.push_back(
        {-2.5f, 0.0f, 0.40f}
    );

    cloudCamera0.push_back(
        {-1.5f, 0.0f, 0.50f}
    );


    /*
    * ========================================================
    * NUBE CAMARA 1
    * ========================================================
    */
    Vision3DProcessor::PointCloud cloudCamera1;

    cloudCamera1.push_back(
        {-0.5f, 0.0f, 0.60f}
    );

    cloudCamera1.push_back(
        {0.5f, 0.0f, 0.70f}
    );


    /*
    * ========================================================
    * NUBE CAMARA 2
    * ========================================================
    */
    Vision3DProcessor::PointCloud cloudCamera2;

    cloudCamera2.push_back(
        {1.5f, 0.0f, 0.80f}
    );

    cloudCamera2.push_back(
        {2.5f, 0.0f, 0.90f}
    );


    /*
    * Procesar cada cámara independientemente.
    */
    VisionHeightSource::VisionResult resultCamera0 =
        processorThreeCameras.processPointCloud(
            0,
            cloudCamera0
        );

    VisionHeightSource::VisionResult resultCamera1 =
        processorThreeCameras.processPointCloud(
            1,
            cloudCamera1
        );

    VisionHeightSource::VisionResult resultCamera2 =
        processorThreeCameras.processPointCloud(
            2,
            cloudCamera2
        );


    /*
    * ========================================================
    * UNIFICAR LOS RESULTADOS
    * ========================================================
    *
    * Por ahora lo hacemos explícitamente en el test.
    *
    * Después esta tarea será responsabilidad de una función
    * del sistema de visión.
    */
    VisionHeightSource::VisionResult combinedResult;


    for (std::size_t body = 0;
        body < Vision3DProcessor::BODY_COUNT;
        ++body)
    {
        if (resultCamera0.bodies[body].valid)
        {
            combinedResult.bodies[body] =
                resultCamera0.bodies[body];
        }

        if (resultCamera1.bodies[body].valid)
        {
            combinedResult.bodies[body] =
                resultCamera1.bodies[body];
        }

        if (resultCamera2.bodies[body].valid)
        {
            combinedResult.bodies[body] =
                resultCamera2.bodies[body];
        }
    }


    /*
    * Mostrar resultado final de los seis cuerpos.
    */
    for (std::size_t body = 0;
        body < Vision3DProcessor::BODY_COUNT;
        ++body)
    {
        std::cout
            << "Cuerpo "
            << (body + 1)
            << ": ";

        if (combinedResult.bodies[body].valid)
        {
            std::cout
                << combinedResult
                    .bodies[body]
                    .height_mm
                << " mm";
        }
        else
        {
            std::cout
                << "NO VALIDO";
        }

        std::cout
            << std::endl;
    }
        



    std::cout
        << std::endl
        << "========================================"
        << std::endl
        << "TEST 7 - POSICION FISICA DE CAMARAS"
        << std::endl
        << "========================================"
        << std::endl;


    Vision3DProcessor processorCameraPosition;


    /*
    * ========================================================
    * CAMARA 0
    * ========================================================
    *
    * Punto local:
    *
    * x = 0
    *
    * Posición física cámara:
    *
    * x = -2500 mm
    *
    * Resultado esperado:
    *
    * x máquina = -2.5 m
    *
    * Eso cae en CUERPO 1.
    */
    Vision3DProcessor::CameraConfig positionCamera0;

    positionCamera0.enabled =
        true;

    positionCamera0.body_enabled[0] =
        true;

    positionCamera0.geometry.position_x_mm =
        -2500.0f;

    processorCameraPosition.setCameraConfig(
        0,
        positionCamera0
    );


    /*
    * ========================================================
    * CAMARA 1
    * ========================================================
    *
    * También recibe un punto local x = 0.
    *
    * Pero la cámara está montada en:
    *
    * x = +500 mm
    *
    * Resultado esperado:
    *
    * x máquina = +0.5 m
    *
    * Eso cae en CUERPO 4.
    */
    Vision3DProcessor::CameraConfig positionCamera1;

    positionCamera1.enabled =
        true;

    positionCamera1.body_enabled[3] =
        true;

    positionCamera1.geometry.position_x_mm =
        500.0f;

    processorCameraPosition.setCameraConfig(
        1,
        positionCamera1
    );


    /*
    * ========================================================
    * NUBES LOCALES
    * ========================================================
    *
    * Las dos cámaras ven exactamente
    * el mismo punto local:
    *
    * x = 0
    *
    * pero gracias a su posición física
    * terminan en zonas distintas de la máquina.
    */
    Vision3DProcessor::PointCloud positionCloud0;

    positionCloud0.push_back(
        {0.0f, 0.0f, 0.40f}
    );


    Vision3DProcessor::PointCloud positionCloud1;

    positionCloud1.push_back(
        {0.0f, 0.0f, 0.70f}
    );


    /*
    * Procesar cada cámara con
    * su geometría propia.
    */
    VisionHeightSource::VisionResult positionResult0 =
        processorCameraPosition.processPointCloud(
            0,
            positionCloud0
        );

    VisionHeightSource::VisionResult positionResult1 =
        processorCameraPosition.processPointCloud(
            1,
            positionCloud1
        );


    std::cout
        << "Camara 0 -> ";

    if (positionResult0.bodies[0].valid)
    {
        std::cout
            << "Cuerpo 1: "
            << positionResult0
                .bodies[0]
                .height_mm
            << " mm";
    }
    else
    {
        std::cout
            << "Cuerpo 1: NO VALIDO";
    }

    std::cout
        << std::endl;


    std::cout
        << "Camara 1 -> ";

    if (positionResult1.bodies[3].valid)
    {
        std::cout
            << "Cuerpo 4: "
            << positionResult1
                .bodies[3]
                .height_mm
            << " mm";
    }
    else
    {
        std::cout
            << "Cuerpo 4: NO VALIDO";
    }

    std::cout
        << std::endl;


    std::cout
    << std::endl
    << "========================================"
    << std::endl
    << "TEST 8 - P95 CON OUTLIER"
    << std::endl
    << "========================================"
    << std::endl;


    Vision3DProcessor processorP95;


    /*
    * Usamos el CUERPO 1.
    *
    * Región actual:
    *
    * x entre -3.0 y -2.0 m
    */
    Vision3DProcessor::PointCloud p95Cloud;


    /*
    * Muchos puntos normales alrededor
    * de 400 ... 500 mm.
    */
    p95Cloud.push_back({-2.5f, 0.0f, 0.400f});
    p95Cloud.push_back({-2.5f, 0.0f, 0.405f});
    p95Cloud.push_back({-2.5f, 0.0f, 0.410f});
    p95Cloud.push_back({-2.5f, 0.0f, 0.415f});
    p95Cloud.push_back({-2.5f, 0.0f, 0.420f});
    p95Cloud.push_back({-2.5f, 0.0f, 0.425f});
    p95Cloud.push_back({-2.5f, 0.0f, 0.430f});
    p95Cloud.push_back({-2.5f, 0.0f, 0.435f});
    p95Cloud.push_back({-2.5f, 0.0f, 0.440f});
    p95Cloud.push_back({-2.5f, 0.0f, 0.445f});

    p95Cloud.push_back({-2.5f, 0.0f, 0.450f});
    p95Cloud.push_back({-2.5f, 0.0f, 0.455f});
    p95Cloud.push_back({-2.5f, 0.0f, 0.460f});
    p95Cloud.push_back({-2.5f, 0.0f, 0.465f});
    p95Cloud.push_back({-2.5f, 0.0f, 0.470f});
    p95Cloud.push_back({-2.5f, 0.0f, 0.475f});
    p95Cloud.push_back({-2.5f, 0.0f, 0.480f});
    p95Cloud.push_back({-2.5f, 0.0f, 0.485f});
    p95Cloud.push_back({-2.5f, 0.0f, 0.490f});
    p95Cloud.push_back({-2.5f, 0.0f, 0.495f});


    /*
    * OUTLIER:
    *
    * un punto claramente erróneo a 2 metros.
    *
    * Con maxZ obtendríamos 2000 mm.
    */
    p95Cloud.push_back(
        {-2.5f, 0.0f, 2.000f}
    );


    VisionHeightSource::VisionResult p95Result =
        processorP95.processPointCloud(
            p95Cloud
        );


    std::cout
        << "Cantidad total de puntos: "
        << p95Cloud.size()
        << std::endl;

    std::cout
        << "Maximo artificial: 2000 mm"
        << std::endl;

    std::cout
        << "Resultado P95 -> Cuerpo 1: ";

    if (p95Result.bodies[0].valid)
    {
        std::cout
            << p95Result
                .bodies[0]
                .height_mm
            << " mm";
    }
    else
    {
        std::cout
            << "NO VALIDO";
    }

    std::cout
        << std::endl;



    std::cout
    << std::endl
    << "========================================"
    << std::endl
    << "TEST 9 - CANTIDAD MINIMA DE PUNTOS"
    << std::endl
    << "========================================"
    << std::endl;


    Vision3DProcessor processorMinPoints;


    /*
    * Configurar solamente el CUERPO 1
    * para exigir al menos 5 puntos.
    */
    Vision3DProcessor::BodyRegion body1Region =
        processorMinPoints.getBodyRegion(
            0
        );

    body1Region.min_points =
        5;

    processorMinPoints.setBodyRegion(
        0,
        body1Region
    );


    /*
    * ========================================================
    * CASO A
    *
    * Solo 4 puntos.
    *
    * Debe resultar NO VALIDO.
    * ========================================================
    */
    Vision3DProcessor::PointCloud cloudFourPoints;

    cloudFourPoints.push_back(
        {-2.5f, 0.0f, 0.400f}
    );

    cloudFourPoints.push_back(
        {-2.5f, 0.0f, 0.410f}
    );

    cloudFourPoints.push_back(
        {-2.5f, 0.0f, 0.420f}
    );

    cloudFourPoints.push_back(
        {-2.5f, 0.0f, 0.430f}
    );


    VisionHeightSource::VisionResult resultFourPoints =
        processorMinPoints.processPointCloud(
            cloudFourPoints
        );


    std::cout
        << "4 puntos -> Cuerpo 1: ";

    if (resultFourPoints.bodies[0].valid)
    {
        std::cout
            << resultFourPoints
                .bodies[0]
                .height_mm
            << " mm";
    }
    else
    {
        std::cout
            << "NO VALIDO";
    }

    std::cout
        << std::endl;


    /*
    * ========================================================
    * CASO B
    *
    * Agregamos un quinto punto.
    *
    * Ahora debe resultar VALIDO.
    * ========================================================
    */
    Vision3DProcessor::PointCloud cloudFivePoints =
        cloudFourPoints;

    cloudFivePoints.push_back(
        {-2.5f, 0.0f, 0.440f}
    );


    VisionHeightSource::VisionResult resultFivePoints =
        processorMinPoints.processPointCloud(
            cloudFivePoints
        );


    std::cout
        << "5 puntos -> Cuerpo 1: ";

    if (resultFivePoints.bodies[0].valid)
    {
        std::cout
            << resultFivePoints
                .bodies[0]
                .height_mm
            << " mm";
    }
    else
    {
        std::cout
            << "NO VALIDO";
    }

    std::cout
        << std::endl;
        
        
    std::cout
    << std::endl
    << "========================================"
    << std::endl
    << "TEST 10 - FILTROS DE PUNTOS INVALIDOS"
    << std::endl
    << "========================================"
    << std::endl;


    Vision3DProcessor processorInvalidPoints;


    /*
    * Vamos a exigir 3 puntos válidos
    * para el CUERPO 1.
    */
    Vision3DProcessor::BodyRegion bodyRegion10 =
        processorInvalidPoints.getBodyRegion(
            0
        );

    bodyRegion10.min_points =
        3;

    processorInvalidPoints.setBodyRegion(
        0,
        bodyRegion10
    );


    /*
    * Nube de prueba.
    */
    Vision3DProcessor::PointCloud invalidCloud;


    /*
    * 3 puntos válidos.
    */
    invalidCloud.push_back(
        {-2.5f, 0.0f, 0.410f}
    );

    invalidCloud.push_back(
        {-2.5f, 0.0f, 0.420f}
    );

    invalidCloud.push_back(
        {-2.5f, 0.0f, 0.430f}
    );


    /*
    * NaN.
    */
    invalidCloud.push_back(
        {
            -2.5f,
            0.0f,
            std::numeric_limits<float>::quiet_NaN()
        }
    );


    /*
    * Infinito.
    */
    invalidCloud.push_back(
        {
            -2.5f,
            0.0f,
            std::numeric_limits<float>::infinity()
        }
    );


    /*
    * Punto fuera de la región X del cuerpo 1.
    */
    invalidCloud.push_back(
        {
            2.5f,
            0.0f,
            0.900f
        }
    );


    /*
    * Punto demasiado alto.
    *
    * max_z actual = 5 m.
    */
    invalidCloud.push_back(
        {
            -2.5f,
            0.0f,
            8.0f
        }
    );


    /*
    * Punto fuera del rango longitudinal.
    *
    * max_y actual = +10 m.
    */
    invalidCloud.push_back(
        {
            -2.5f,
            20.0f,
            0.500f
        }
    );


    VisionHeightSource::VisionResult invalidResult =
        processorInvalidPoints.processPointCloud(
            invalidCloud
        );


    std::cout
        << "Cantidad total de puntos: "
        << invalidCloud.size()
        << std::endl;

    std::cout
        << "Puntos validos esperados: 3"
        << std::endl;

    std::cout
        << "Resultado -> Cuerpo 1: ";

    if (invalidResult.bodies[0].valid)
    {
        std::cout
            << invalidResult
                .bodies[0]
                .height_mm
            << " mm";
    }
    else
    {
        std::cout
            << "NO VALIDO";
    }

    std::cout
        << std::endl;


    std::cout
    << std::endl
    << "========================================"
    << std::endl
    << "TEST 11 - FILTRO TEMPORAL EMA"
    << std::endl
    << "========================================"
    << std::endl;


    Vision3DProcessor processorTemporal;


    /*
    * Configurar filtro temporal.
    */
    Vision3DProcessor::TemporalFilterConfig filterConfig;

    filterConfig.enabled =
    true;

    filterConfig.alpha =
    0.25f;

    processorTemporal.setTemporalFilterConfig(
    filterConfig
    );


    /*
    * Secuencia de alturas con ruido.
    */
    std::array<uint16_t, 5> temporalHeights =
    {
    800,
    815,
    790,
    825,
    805
    };


    for (uint16_t heightMm : temporalHeights)
    {
    Vision3DProcessor::PointCloud temporalCloud;

    /*
        * Punto dentro del CUERPO 1.
        *
        * Convertimos mm -> metros.
        */
    temporalCloud.push_back(
        {
            -2.5f,
            0.0f,
            static_cast<float>(heightMm) / 1000.0f
        }
    );


    VisionHeightSource::VisionResult temporalResult =
        processorTemporal.processPointCloud(
            temporalCloud
        );


    std::cout
        << "Entrada: "
        << heightMm
        << " mm"
        << " -> Filtrada: ";


    if (temporalResult.bodies[0].valid)
    {
        std::cout
            << temporalResult
                    .bodies[0]
                    .height_mm
            << " mm";
    }
    else
    {
        std::cout
            << "NO VALIDO";
    }


    std::cout
        << std::endl;
    }    


    std::cout
    << std::endl
    << "========================================"
    << std::endl
    << "TEST 12 - TIMEOUT DE MEDICION"
    << std::endl
    << "========================================"
    << std::endl;


    Vision3DProcessor processorTimeout;


    VisionHeightSource::BodyVisionResult freshResult;

    freshResult.height_mm =
        800;

    freshResult.valid =
        true;

    freshResult.timestamp_ms =
        1000;


    /*
    * Caso A:
    * ahora = 1300 ms
    *
    * edad = 300 ms
    *
    * timeout = 500 ms
    *
    * Debe ser válida.
    */
    bool fresh =
        processorTimeout.isResultFresh(
            freshResult,
            1300,
            500
        );


    std::cout
        << "Edad 300 ms -> "
        << (fresh ? "VALIDO" : "NO VALIDO")
        << std::endl;


    /*
    * Caso B:
    * ahora = 1700 ms
    *
    * edad = 700 ms
    *
    * timeout = 500 ms
    *
    * Debe ser inválida.
    */
    bool stale =
        processorTimeout.isResultFresh(
            freshResult,
            1700,
            500
        );


    std::cout
        << "Edad 700 ms -> "
        << (stale ? "VALIDO" : "NO VALIDO")
        << std::endl;


        std::cout
    << std::endl
    << "========================================"
    << std::endl
    << "TEST 13 - MULTICAMARA CON UNA CAMARA VENCIDA"
    << std::endl
    << "========================================"
    << std::endl;


    Vision3DProcessor processorMerge;


    /*
    * Cámara 0:
    * cuerpos 1 y 2 válidos y frescos.
    */
    VisionHeightSource::VisionResult mergeCamera0;

    mergeCamera0.bodies[0].height_mm = 400;
    mergeCamera0.bodies[0].valid = true;
    mergeCamera0.bodies[0].timestamp_ms = 1000;

    mergeCamera0.bodies[1].height_mm = 500;
    mergeCamera0.bodies[1].valid = true;
    mergeCamera0.bodies[1].timestamp_ms = 1000;


    /*
    * Cámara 1:
    * cuerpos 3 y 4 válidos,
    * pero con timestamp viejo.
    */
    VisionHeightSource::VisionResult mergeCamera1;

    mergeCamera1.bodies[2].height_mm = 600;
    mergeCamera1.bodies[2].valid = true;
    mergeCamera1.bodies[2].timestamp_ms = 200;

    mergeCamera1.bodies[3].height_mm = 700;
    mergeCamera1.bodies[3].valid = true;
    mergeCamera1.bodies[3].timestamp_ms = 200;


    /*
    * Cámara 2:
    * cuerpos 5 y 6 válidos y frescos.
    */
    VisionHeightSource::VisionResult mergeCamera2;

    mergeCamera2.bodies[4].height_mm = 800;
    mergeCamera2.bodies[4].valid = true;
    mergeCamera2.bodies[4].timestamp_ms = 1000;

    mergeCamera2.bodies[5].height_mm = 900;
    mergeCamera2.bodies[5].valid = true;
    mergeCamera2.bodies[5].timestamp_ms = 1000;


    /*
    * Agrupar las tres cámaras.
    */
    std::array<
        VisionHeightSource::VisionResult,
        Vision3DProcessor::CAMERA_COUNT
    > mergeInputs =
    {
        mergeCamera0,
        mergeCamera1,
        mergeCamera2
    };


    /*
    * Momento actual:
    *
    * now = 1200 ms
    *
    * timeout = 500 ms
    *
    * Cámara 0:
    * edad = 200 ms -> válida
    *
    * Cámara 1:
    * edad = 1000 ms -> vencida
    *
    * Cámara 2:
    * edad = 200 ms -> válida
    */
    VisionHeightSource::VisionResult mergedResult =
        processorMerge.mergeCameraResults(
            mergeInputs,
            1200,
            500
        );


    for (std::size_t body = 0;
        body < Vision3DProcessor::BODY_COUNT;
        ++body)
    {
        std::cout
            << "Cuerpo "
            << (body + 1)
            << ": ";

        if (mergedResult.bodies[body].valid)
        {
            std::cout
                << mergedResult.bodies[body].height_mm
                << " mm";
        }
        else
        {
            std::cout
                << "NO VALIDO";
        }

        std::cout
            << std::endl;
    }

        printResult(
            result
        );
    }


    return 0;
}