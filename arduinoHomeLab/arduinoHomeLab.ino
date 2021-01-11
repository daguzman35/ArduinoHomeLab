/*
 * Arduino Home Lab
 * Hecho por: David Guzmán
 * Bogotá, Colombia
 * 2021
 * 
 * Descripción:
 *  Generador de pulsos cuadrados (x2) y osciloscopio (x2) con Arduino.
 *  Diseñado para usar con 'serial plotter' de Arduino IDE.
 *  Resolución kHz.
 *  Probado en Arduino Uno. 
 * 
 * Instrucciones de uso:
 *  Debe instalar las siguientes bibliotecas (libraries):
 *    TimerOne                versión 1.1.0
 *    avdweb_AnalogReadFast   versión 1.0.0
 *  Para instalar bibliotecas: https://www.arduino.cc/en/guide/libraries
 *    
 *  Modifique los parámetros en la sección 'parámetros de usuario'.
 *  Evite alterar otras partes del código.
 *  
 *  En el Arduino, conecte las señales a medir en los puertos A0 y A1.
 *  El disparo del osciloscopio está configurado con el puerto A0.
 *  Use 'plotter serial' o 'monitor serial' para obtener los datos.
 *  
 *  Configuración por defecto:
 *    comunicación serial:        115200 baud
 *    frecuencia de muestreo:     10kHz (T=100us)
 *    número de muestras:         501
 *    umbral disparo A0:          int 50 = aprox. 250mV
 *    tiempo refresco datos:      5 segundos
 *    
 * 
 * Historial de versiones:
 * 
 * v21.1    11 de enero de 2021
 * Novedades:
 *  Osciloscopio:
 *  Resolución temporal de mediciones mejorada: 50us (20kHz).
 *  Mediciones usan interrupciones, mejorando confiabilidad.
 *  Mide 2 canales análogos; antes medía 3.
 *  Almacena en RAM todos los datos medidos. Cuando llena memoria, los imprime.
 *  Imprime valor de tiempo, donde t=0 es el inicio de la medición.
 *  Encabezados de columna de datos incluidos.
 *  Sistema de umbral (threshold) implementado, emulando disparo de osciloscopios reales.
 *  Generador:
 *  Para cada generador, solo se debe dar el periodo. Sistema asume duty cycle=50%.
 * 
 * v20.2    6 de julio de 2020
 *  Mide 3 canales análogos, en voltios. Rango: 0V a 5V. Precisión 10 bits: 5mV.
 *  Genera señales cuadradas periódicas de amplitud 5V, con duración especificada por el usuario.
 *  Resolución temporal de mediciones: 10ms (100Hz).
 *  Resolución temporal de generador: 1ms (1kHz).
 *  Imprime datos medidos por puerto serial.
 *  Puede usarse con algún monitor serial para guardar en archivo de texto plano los datos.
 *  Ideal para usarse con 'serial plotter' de Arduino.
 *   
 */

#include <TimerOne.h>               //para interrupciones en microsegundos
#include <avdweb_AnalogReadFast.h>  //para lecturas más eficientes de puerto análogo
//Para 2 canales análogos, sin esta biblioteca el mejor periodo de muestreo es de 300us (3.3kHz)
//                         con esta biblioteca el mejor periodo de muestreo es de 50us (20kHz)

///////////////////////////////
//    Parámetros del usuario
///////////////////////////////
//   para osciloscipio
//   1) Escoger la tasa (baudRate) de comunicación de puerto serial. 
//      ATENCIÓN: usar esta tasa en monitor serial y serial plotter.
unsigned long mi_baud_rate = 115200;  //en baudios, valores válidos: {9600,19200,38400,57600,115200,230400}
//   2) Escoger periodo de muestreo.
unsigned long periodo_muestreo_us = 100;  //en microsegundos, cada cuánto se hará una lectura de señales
                                          //valor recomendado (us): 100
//   3) Umbral (threshold) canal A0:
int umbral_analogo_valor = 50; //cuando la entrada análoga A0 supera este valor, el sistema inicia mediciones.
                               //valor entre 0 y 1023. Recomendado: 50.
//   4) Tasa de refresco
int tiempo_refresco_pantalla_ms = 5000; //tiempo en ms en el cual el sistema deja de medir datos
//   5) tamaño memoria local
const int NUM_DATOS = 501;  // cantidad de datos a medir. Tener presente la capacidad de la memoria SRAM. 
                            // SRAM disponible en Arduino UNO: 2kB.
                            // 501 datos llena exactamente la gráfica de 'plotter serial' de Arduino IDE.
//
//   para generadores
int periodo_generador_D12_ms = 6;   //periodo señal entregada en pin digital D12, en milisegundos
int periodo_generador_D13_ms = 10;  //periodo señal entregada en pin digital D13, en milisegundos
///////////////////////////////



////////////////////////////////////////
///// No modificar código desde aquí
////////////////////////////////////////

typedef struct {  //creado para almacenar de forma eficiente lectura de 2 canales de datos análogos de 10bits, en 3 bytes
  byte a0_lsb;    //byte menos significativo (LSB) de A0
  byte a1_lsb;    //byte menos significativo (LSB) de A1
  byte a0a1_msb;  //bits más significativos (9 y 10) de A0 y A1 respectivamente
} tipo_dato3bytes;


/////////////////////////////////
///Variables usadas por el código
int t_on_generadorD12_ms = 1000;
int t_off_generadorD12_ms = 1000;
int t_on_generadorD13_ms = 1000;
int t_off_generadorD13_ms = 1000;
unsigned long lectura_previa_ms = 0;       // tiempo de la última ejecución de milis
unsigned long crono_gen13=0;               //para almacenar la hora de crono generador 13
unsigned long crono_gen12=0;               //para almacenar la hora de crono generador 12
boolean debo_leer_adcs = false;           //determina si se debe hacer lectura de entradas análogas
boolean debo_imprimir_en_serial = false;  //determina si se debe imprimir el resultado en puerto serial
tipo_dato3bytes datos[NUM_DATOS];         //arreglo de medidas de los puertos A0 y A1
int i_dato=0;
unsigned long tiempo_medidas_inicial_us = 0; //usada para determinar el periodo real de muestreo, usando pocos bytes de memoria
unsigned long tiempo_medidas_final_us = 0; //usada para determinar el periodo real de muestreo, usando pocos bytes de memoria
/////////////////////////////////

void setup() {
  // configura temporizador rápido, en microsegundos. 
  // Se usa biblioteca TimerOne.h
  Timer1.initialize(periodo_muestreo_us);
  Timer1.attachInterrupt(leerADCs); //hace llamado a la función a ejecutar de forma periódica  
  
  // inicia comunicación serial
  Serial.begin(mi_baud_rate);
  
  // configura pines digitales de salida
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  debo_leer_adcs = true;
  debo_imprimir_en_serial = false;

  // configura generadores de pulsos cuadrados
  //  salida D13
  if (periodo_generador_D13_ms <= 1){ //si el periodo es menor a 2, o si es negativo
    //asignar periodo 2ms
    periodo_generador_D13_ms = 2;
    t_on_generadorD13_ms = 1;
    t_off_generadorD13_ms = 1;
  }
  else{
    //si periodo es par, dura el mismo tiempo on y off
    //si periodo es impar, dura 1ms extra off que on
    t_on_generadorD13_ms = int(1.0*periodo_generador_D13_ms/2);
    t_off_generadorD13_ms = int(1.0*periodo_generador_D13_ms/2) + periodo_generador_D13_ms%2;
  }
  
  //  salida D12
  if (periodo_generador_D12_ms <= 1){ //si el periodo es menor a 2, o si es negativo
    //asignar periodo 2ms
    periodo_generador_D12_ms = 2;
    t_on_generadorD12_ms = 1;
    t_off_generadorD12_ms = 1;
  }
  else{
    //si periodo es par, dura el mismo tiempo on y off
    //si periodo es impar, dura 1ms extra off que on
    t_on_generadorD12_ms = int(1.0*periodo_generador_D12_ms/2);
    t_off_generadorD12_ms = int(1.0*periodo_generador_D12_ms/2) + periodo_generador_D12_ms%2;
  }
  
}

void loop() {

  generadorCuadrada(13,t_on_generadorD13_ms,t_off_generadorD13_ms,&crono_gen13); //pin D13 = LED en Arduino UNO
  generadorCuadrada(12,t_on_generadorD12_ms,t_off_generadorD12_ms,&crono_gen12);

  if (debo_imprimir_en_serial == true){
    imprimirEnSerial(); //función para imprimir resultado de datos medidos en puerto serial
  }
  else{
    //  //nada, en espera
  }  
  
}


void leerADCs(void)
{
  //Lee 2 puertos análogos (A0,A1) de forma rápida. 
  //Almacena datos en memoria RAM.
  //Creada en enero de 2021

  if (debo_imprimir_en_serial == false){
    //Solo ejecutar esta función (leerADCs) si no está imprimiendo datos pasados en serial.
    //Si está imprimiendo, no debe leer puertos análogos ni registrar hora actual

    unsigned long hora = micros();  //lo primero a hacer en esta función debe ser tomar la hora
    //int dato_anterior = datosA0[i_dato];
    int dato_anterior_a0 = getA0FromStructDato(datos[i_dato]);
  
    //**Opción 1: Usando analogRead
    //datosA0[i_dato] = analogRead(0);    //AnalogRead tarda 118us en ejecutarse.
    //datosA0[i_dato] = analogRead(1);    //AnalogRead tarda 118us en ejecutarse.
  
    //**Opción 2: Usando analogReadFast (biblioteca avdweb_AnalogReadFast.h)
    int valorA0 = analogReadFast(A0); // se supone tarda 20us en AVR 
    int valorA1 = analogReadFast(A1); // se supone tarda 20us en AVR 
    datos[i_dato] = convertirA0A1enTipoDato3Bytes(valorA0,valorA1); //guarda valores en arreglo de estructura de 3bytes

    if (debo_leer_adcs == true){
      i_dato++; //aumento indice para próxima lectura
      if (tiempo_medidas_inicial_us == 0){
        tiempo_medidas_inicial_us = hora; //si no se ha medido la hora inicial, tomarla
      }
      if (i_dato >= NUM_DATOS){
        tiempo_medidas_final_us = hora; //si acabó, tomar la hora final
        debo_leer_adcs = false; //deshabilita lecturas futuras
        debo_imprimir_en_serial = true;
        i_dato = 0; //reinicia contador para que próxima lectura inicie el arreglo
      }
    }
    else {  //si no debo_leer_adcs
      if (debo_imprimir_en_serial == false){
        //y si no está imprimiendo datos en serial

        if ((dato_anterior_a0 < umbral_analogo_valor) and (valorA0 >= umbral_analogo_valor)) {
          //si hay flanco de subida, pasando por umbral,
          debo_leer_adcs = true;              //iniciar toma de datos
          i_dato = 1;                         //asegurarse inicie en 1 con el siguiente dato
          tiempo_medidas_inicial_us = hora;   //importante: limpiar tiempos de medida anterior
          tiempo_medidas_final_us = hora;     //importante: limpiar tiempos de medida anterior
        }
      }
    }    
  }  
}

float deEnteroAVoltaje(int valor){
  // Convierte número entero de 0 a 1023 en un voltaje, siendo 5V=1023.
  // Creada en julio de 2020
  return float(valor)*5/1023; 
}
  
void generadorCuadrada(int pin_digital, int t_on_ms, int t_off_ms, unsigned long *crono_local){
  // Genera una señal cuadrada periodica de amplitud 5V, con las
  // duraciones de encendido y apagado especificadas.
  // Creada en julio de 2020

  unsigned long actual_millis = millis(); //lee hora actual
  unsigned long siguiente_on=0;
  unsigned long siguiente_off=0;

  siguiente_off = *crono_local + t_on_ms;    //apagar cuando hayan pasado t_on_ms
  siguiente_on = *crono_local + t_on_ms + t_off_ms;  //prender cuando hayan pasado t_on_ms+t_off_ms

  if(actual_millis > siguiente_on) { //si es hora de encender
    digitalWrite(pin_digital,HIGH);  //encender
    *crono_local = siguiente_on;     //actualiza hora de último encendido
    while(actual_millis > (*crono_local + t_on_ms + t_off_ms))
    {
      //si el reloj quedó atrasado, actualizar hasta que la diferencia no supere un periodo (t_on+t_off)
      *crono_local = *crono_local + t_on_ms + t_off_ms; //si el reloj quedó atrasado, actualizar hasta 
    }
  }
  else if(actual_millis > siguiente_off) { //si es hora de apagar
    digitalWrite(pin_digital,LOW); //apagar
  }  
  
}

void imprimirEnSerial(void){
  // Imprime los datos almacenados en el arreglo datos[] por puerto serial
  // Para usar con monitor serial y plotter serial
  //    1er dato: tiempo en s,    2o dato: medida A0 en V     3er dato: medida A1 en V
  // Creada en enero de 2021

  unsigned long delta_real_us = (tiempo_medidas_final_us-tiempo_medidas_inicial_us);
  float periodo_real_us = 1.0*delta_real_us;
  periodo_real_us = periodo_real_us /(NUM_DATOS-1);
  
  // calcula valor asociado de voltaje: 1023=5V
  int valorA0 = 0;
  int valorA1 = 0;
  float voltajeA0 = 0; 
  float voltajeA1 = 0;  
  float tiempo_s = 0;
  int k=0;
  
  //Verificación periodo muestreo
  float error_periodo_muestreo=0;
  error_periodo_muestreo = (1.0*periodo_muestreo_us - periodo_real_us);
  if (error_periodo_muestreo < 0){
  //valor absoluto
  error_periodo_muestreo = error_periodo_muestreo *(-1);
  }
  error_periodo_muestreo = error_periodo_muestreo / periodo_muestreo_us;
  if (error_periodo_muestreo > 0.02){  //si el error es mayor al 2%
    //Mensaje de advertencia 
    Serial.print(F("Advertencia: las mediciones de tiempo no son confiables.\n"));
    Serial.print(F("             Puede deberse a pedir un periodo de muestreo muy bajo.\n"));
    Serial.print(F("             Se recomienda usar un periodo de muestreo mayor o igual a 50us.\n"));
    Serial.print(F("             Periodo de muestreo pedido (us): "));
    Serial.print(periodo_muestreo_us);
    Serial.print(F("\n             Periodo de muestreo estimado (us): "));
    Serial.println(periodo_real_us);
  }

  //Encabezados
  Serial.print(F("tiempo(s)"));
  Serial.print(F("\t"));
  Serial.print(F("A0(V)"));
  Serial.print(F("\t"));
  Serial.print(F("A1(V)"));
  Serial.print(F("\n"));

  //Imprime en puerto serial los valores leídos en voltios, con resolución 5mV.
  for (k=0;k<NUM_DATOS;k++) {
    
    tiempo_s = 1.0*periodo_real_us;
    tiempo_s = tiempo_s*k;
    tiempo_s = tiempo_s/1000000;    
    valorA0 = getA0FromStructDato(datos[k]);
    valorA1 = getA1FromStructDato(datos[k]);
    voltajeA0 = deEnteroAVoltaje(valorA0);
    voltajeA1 = deEnteroAVoltaje(valorA1);
    
    Serial.print(tiempo_s,6);//el '6' indica 6 cifras decimales
    Serial.print(F("\t"));       //valores deben separarse con \t para 'serial plotter'
    Serial.print(voltajeA0,3); //el '3' indica 3 cifras decimales
    Serial.print(F("\t"));       //valores deben separarse con \t para 'serial plotter'
    Serial.print(voltajeA1,3);
    Serial.print(F("\n"));       //la última lectura debe terminar con un enter para 'serial plotter'
  }
  for (k=NUM_DATOS+1;k<500;k++){
    //imprimir línea en blanco para completar la gráfica de plotter serial, quien siempre grafica 500 datos
    Serial.println(F("-1\t-1\t-1")); 
  }

  //espera un tiempo antes de seguir ejecutando. Para mantener en pantalla última secuencia medida.
  delay(tiempo_refresco_pantalla_ms); 
  debo_imprimir_en_serial = false;
}

//Funciones conversión tipo_dato3bytes a enteros y viceversa
tipo_dato3bytes convertirA0A1enTipoDato3Bytes(int vA0, int vA1){
  //recibe 2 enteros, y los convierte en una variable del tipo estructura tipo_dato3bytes 
  //solo almacena los 12 bits menos significativos de cada entero
  //creada en enero de 2021
  tipo_dato3bytes mi_dato;
  mi_dato.a0_lsb = (vA0 & 0xff); //LSB
  mi_dato.a1_lsb = (vA1 & 0xff); //LSB
  mi_dato.a0a1_msb = (((vA0 >> 8) & 0x0f) | ((vA1 >> 4) & 0xf0));  //bits A1_12, A1_11, A1_10, A1_9, A0_12, A0_11, A0_10, A0_9
  return mi_dato;
}

int getA0FromStructDato(tipo_dato3bytes dato){
  //recibe una variable del tipo estructura tipo_dato3bytes, y extrae el valor entero de A0
  //creada en enero de 2021
  return ( dato.a0_lsb | ((dato.a0a1_msb & 0x0f)<<8) );
}

int getA1FromStructDato(tipo_dato3bytes dato){
  //recibe una variable del tipo estructura tipo_dato3bytes, y extrae el valor entero de A0
  //creada en enero de 2021
  return ( dato.a1_lsb | ((dato.a0a1_msb & 0xf0)<<4) );
}
