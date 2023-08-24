// Se importan librerías necesarias para la ejecución del código
#include <webots/camera.h>
#include <webots/range_finder.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <stdio.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#define TIME_STEP 32
#define WB_CHANNEL_BROADCAST -1

int main(int argc, char **argv)
{
    // Se definen e inicializan dispositivos a utilizar
    wb_robot_init();
    WbDeviceTag M_left = wb_robot_get_device("MotorIzq");
    WbDeviceTag M_right = wb_robot_get_device("MotorDer");
    WbDeviceTag pos_der = wb_robot_get_device("SensorPosDer");
    WbDeviceTag pos_izq = wb_robot_get_device("SensorPosIzq");
    WbDeviceTag camera = wb_robot_get_device("Camara");
    WbDeviceTag camera_end = wb_robot_get_device("CamaraFin");
    WbDeviceTag dist_izq = wb_robot_get_device("SensorDistIzq");
    WbDeviceTag dist_der = wb_robot_get_device("SensorDistDer");
    WbDeviceTag dist_fro = wb_robot_get_device("SensorDistFro");
    WbDeviceTag range_finder = wb_robot_get_device("Rangefinder");
    WbDeviceTag emitter = wb_robot_get_device("emisor");
    WbDeviceTag receiver = wb_robot_get_device("receptor");
    
    wb_camera_enable(camera, TIME_STEP);
    wb_camera_recognition_enable(camera, TIME_STEP);
    wb_range_finder_enable(range_finder, TIME_STEP);
    wb_emitter_set_channel(emitter, 1); //Setea canal 1
    wb_receiver_enable(receiver, TIME_STEP);
    
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
    int id_obj = 0;

    //faltan sensores de posicion
    wb_motor_set_position(M_left, INFINITY);
    wb_motor_set_position(M_right, INFINITY);
    wb_motor_set_velocity(M_left, 0.0);
    wb_motor_set_velocity(M_right, 0.0);
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
        
        if (CantDeObjetos > 0)
        {
            const WbCameraRecognitionObject *Objs = wb_camera_recognition_get_objects(camera);
            for (int i = 0; i < CantDeObjetos; i++)
            {
                id_obj++; //Se identifica y suma objeto
                
                //DEBUG - se quitara print del final - se utilizara para seguir objeto
                printf("Posicion=%dx%d",
                       Objs[i].position_on_image[0],
                       Objs[i].position_on_image[1]);
                FromX = Objs[i].position_on_image[0] - Objs[i].size_on_image[0] / 2;
                ToToX = Objs[i].position_on_image[0] + Objs[i].size_on_image[0] / 2;
                printf("Tamano=%dx%d",
                       Objs[i].size_on_image[0],
                       Objs[i].size_on_image[1]);
                FromY = Objs[i].position_on_image[1] - Objs[i].size_on_image[1] / 2;
                ToToY = Objs[i].position_on_image[1] + Objs[i].size_on_image[1] / 2;
            }
            
        }
        /*
        else
        {
            //rotar en el lugar
        }
        */
        wb_motor_set_velocity(M_left, 0);
        wb_motor_set_velocity(M_right, 0);

        const unsigned char *image2 = wb_camera_get_image(camera);
        for (x = FromX; x < ToToX + 1; x++)
        {
            for (y = FromY; y < ToToY + 1; y++)
            {
                //Actuar según imagen
            }
        }


        for (i = FromX; i < ToToX + 1; i++)
        {
            for (j = FromY; j < ToToY + 1; j++)
            {
                distance = wb_range_finder_image_get_depth(image2, rf_width, i, j);
            }
        }
        
        //Falta implementar manejo de brazos y PID de movimiento

    }
    wb_robot_cleanup();
    return 0;
}