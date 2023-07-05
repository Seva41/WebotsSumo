// Se importan librerías necesarias para la ejecución del código
#include <webots/camera.h>
#include <webots/range_finder.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <stdio.h>

#define TIME_STEP 32

int main(int argc, char **argv)
{
    // Se definen e inicializan dispositivos a utilizar
    wb_robot_init();
    WbDeviceTag M1_left = wb_robot_get_device("MotorIzq");
    WbDeviceTag M2_right = wb_robot_get_device("MotorDer");
    WbDeviceTag camera = wb_robot_get_device("Camara");
    wb_camera_enable(camera, TIME_STEP);
    wb_camera_recognition_enable(camera, TIME_STEP);
    WbDeviceTag range_finder = wb_robot_get_device("Rangefinder");
    wb_range_finder_enable(range_finder, TIME_STEP);
    int img_width = wb_camera_get_width(camera);
    int img_height = wb_camera_get_height(camera);
    int rf_width = wb_range_finder_get_width(range_finder);
    int rf_height = wb_range_finder_get_height(range_finder);

    // Inicialización de variables usadas
    float distance;
    int contador = 0;
    int i, j, x, y;
    int fuera_arena = 0;
    int FromX = 0;
    int ToToX = 0;
    int FromY = 0;
    int ToToY = 0;

    // Se definen los valores iniciales para los sensores de posición y la velocidad de los motores.
    // Luego se inicializa el programa a correr hasta que la variable pausarelproceso sea 0.
    wb_motor_set_position(M1_left, INFINITY);
    wb_motor_set_position(M2_right, INFINITY);
    wb_motor_set_velocity(M1_left, 0.0);
    wb_motor_set_velocity(M2_right, 0.0);
    while (wb_robot_step(TIME_STEP) != -1 && fuera_arena == 0)
    {
        contador = contador + 1;

        // Se obtiene la imagen de la cámara y se cuentan la cantidad de objetos que hay en el cuadro capturado.
        // Luego se obtiene la captura del telémetro y se inicializan variables para calcular distancias.
        wb_camera_get_image(camera);
        const float *image = wb_range_finder_get_range_image(range_finder);

        // Se analiza cada pixel de la captura del telémetro para medir la distancia en cada uno de ellos.
        for (i = 0; i < rf_width; i++)
        {
            for (j = 0; j < rf_height; j++)
            {
                distance = wb_range_finder_image_get_depth(image, rf_width, i, j);
            }
        }
        

        // Si hay al menos un objeto en la captura de la cámara, se ejecuta el código que identifica cada objeto y retorna su posición y tamaño en el cuadro
        if (CantDeObjetos > 0) //NO IMPLEMENTADO AUN
        {
            const WbCameraRecognitionObject *Objs = wb_camera_recognition_get_objects(camera);
            printf("| Img%dx%d ", img_width, img_height);
            for (int i = 0; i < CantDeObjetos; i++)
            {
                // Al identificarse un objeto, lo cuenta e imprime su ID
                printf(" Obj(%d)=>", i + 1);
                // Se identifica el objeto según su nombre de modelo y se imprime
                printf(" %s", Objs[i].model);
                // Se obtiene el ID del objeto y se imprime
                printf(" Id:%d", Objs[i].id);
                // Se imprime la posición del objeto en el cuadro de la cámara
                printf(" position=%dx%d",
                       Objs[i].position_on_image[0],
                       Objs[i].position_on_image[1]);
                FromX = Objs[i].position_on_image[0] - Objs[i].size_on_image[0] / 2;
                ToToX = Objs[i].position_on_image[0] + Objs[i].size_on_image[0] / 2;
                // Se imprime el tamaño que ocupa el objeto en el cuadro de la cámara
                printf(" Size=%dx%d",
                       Objs[i].size_on_image[0],
                       Objs[i].size_on_image[1]);
                FromY = Objs[i].position_on_image[1] - Objs[i].size_on_image[1] / 2;
                ToToY = Objs[i].position_on_image[1] + Objs[i].size_on_image[1] / 2;
            }
            printf("\n");
        }
        else
        {
            printf("\n");
        }

        // Se manejas los motores según el paso del programa. Mientras sea menor que 600 pasos, se mueve con velocidades especificadas.
        // Si es mayor o igual a 236, se detienen. En el paso 237, se pausa todo el proceso.
        wb_motor_set_velocity(M1_left, -2);
        wb_motor_set_velocity(M2_right, -2);
        // En el paso 237 se imprime la información RGB que posee la imagen de la cámara en ese cuadro

        const unsigned char *image = wb_camera_get_image(camera);
        for (x = FromX; x < ToToX + 1; x++)
        {
            for (y = FromY; y < ToToY + 1; y++)
            {
                int r = wb_camera_image_get_red(image, img_width, x, y);
                int g = wb_camera_image_get_green(image, img_width, x, y);
                int b = wb_camera_image_get_blue(image, img_width, x, y);
                printf("%d %d R= %d, G= %d, B= %d \n", x, y, r, g, b);
            }
        }


        for (i = FromX; i < ToToX + 1; i++)
        {
            for (j = FromY; j < ToToY + 1; j++)
            {
                distance = wb_range_finder_image_get_depth(image, rf_width, i, j);
                printf(" %d %d %f\n", i, j, distance);
            }
        }

    }
    wb_robot_cleanup();
    return 0;
}