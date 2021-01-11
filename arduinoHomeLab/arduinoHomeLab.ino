/*
 * Arduino Home Lab
 * Hecho por: David Guzmán
 * 
 * v21.1    10 de enero de 2021
 * 
 * Objetivo: mejorar osciloscopio a 10kHz. Posiblemente bajando la lectura a 2 canales.
 * 
 * v20.2    6 de julio de 2020
 * 
 * Osciloscopio lento (T_muestreo>10ms):
 *   Hace lecturas de 3 puertos análogos de entrada, e imprime en puerto serial los valores medidos en voltios. 
 *   Ideal para usarse con 'serial plotter' de Arduino.
 *   También puede usarse con algún monitor serial para guardar en archivo de texto plano los datos.
 * 
 * Generadores de señales:
 *   Produce señales cuadradas periódicas de amplitud 5V, con duración especificada por el usuario.
 *   
 * 
 * Parámetros configurables:
 *   - periodo de muestreo, en ms. Se recomienda sea mayor a 10ms.
 *   - miBaudRate. Tasa de comunicación de puerto serial. Se debe escoger este mismo valor en monitor serial o en 'serial plotter'.
 *   
 *   Para tener periodo de muestreo de 1ms, se recomienda usar baud rate de 115200 o superior
 */

#include <TimerOne.h>               //para interrupciones en microsegundos
#include <avdweb_AnalogReadFast.h>  //para lecturas más eficientes de puerto análogo
//Para 2 canales análogos:
//sin esta librería el mejor periodo de muestreo es de 300us (3.3kHz)
//con esta librería el mejor periodo de muestreo es de 100us (10kHz)

///////////////////////////////
//    Parámetros del usuario
//
//



//   para osciloscipio
//   1) Escoger la tasa (baudRate) de comunicación de puerto serial. ATENCIÓN: usar esta tasa en monitor serial y serial plotter.
unsigned long miBaudRate = 115200;  //en baudios, valores válidos: {9600,19200,38400,57600,115200,230400}
//   2) Escoger periodo de muestreo.
unsigned long periodo_muestreo_us = 50;  //en microsegundos, cada cuánto se hará una lectura de señales
//   3) Umbral (threshold) canal A0:
int umbral_analogo_valor = 50; //cuando la entrada análoga supera este valor, el sistema inicia mediciones
//   4) Tasa de refresco
int tiempo_refresco_pantalla_ms = 5000; //tiempo en ms en el cual el sistema deja de medir datos
//
//   para generadores

int periodo_generador_D13_ms = 6;
int periodo_generador_D12_ms = 100;

/*int t_on_generadorD13_ms = 10;   //tiempo que dura el pulso de salida en D13 en encendido (5V)
int t_off_generadorD13_ms = 10;  //tiempo que dura el pulso de salida en D13 en apagado (0V)
int t_on_generadorD12_ms = 50;    //tiempo que dura el pulso de salida en D12 en encendido (5V)
int t_off_generadorD12_ms = 50;   //tiempo que dura el pulso de salida en D12 en apagado (0V)*/

int t_on_generadorD13_ms = int(1.0*periodo_generador_D13_ms/2);
int t_off_generadorD13_ms = int(1.0*periodo_generador_D13_ms/2);
int t_on_generadorD12_ms = int(1.0*periodo_generador_D12_ms/2);
int t_off_generadorD12_ms = int(1.0*periodo_generador_D12_ms/2);


//  para tamaño memoria local
const int NUM_DATOS = 401;  // cantidad de datos a medir. Tener presente la capacidad de la memoria SRAM. SRAM disponible en Arduino UNO: 2kB.

///////////////////////////////


/////////////////////////////////
///Variables usadas por el código
int entradaA0 = 0;        // lectura de entrada análoga A0
int entradaA1 = 0;        // lectura de entrada análoga A1
int entradaA2 = 0;        // lectura de entrada análoga A2
float voltajeA0 = 0;       // valor en voltios de entrada análoga A0
float voltajeA1 = 0;       // valor en voltios de entrada análoga A0
float voltajeA2 = 0;       // valor en voltios de entrada análoga A0
unsigned long lecturaPrevia_ms = 0;        // tiempo de la última ejecución de milis
unsigned long cronoGen13=0; //para almacenar la hora de crono generador 13
unsigned long cronoGen12=0; //para almacenar la hora de crono generador 12
boolean debo_leer_adcs = false;  //determina si se debe hacer lectura de entradas análogas
boolean debo_imprimir_en_serial = false;  //determina si se debe imprimir el resultado en puerto serial
int datosA0[NUM_DATOS];   //arreglo de medidas de puerto análogo A0
int datosA1[NUM_DATOS];   //arreglo de medidas de puerto análogo A1
int i_dato=0;
unsigned long tiempo_medidas_inicial_us = 0; //usada para determinar el periodo real de muestreo, usando pocos bytes de memoria
unsigned long tiempo_medidas_final_us = 0; //usada para determinar el periodo real de muestreo, usando pocos bytes de memoria


/////////////////////////////////

void setup() {
  // configura temporizador rápido, en microsegundos. Se usa librería TimerOne.h
  Timer1.initialize(periodo_muestreo_us);
  Timer1.attachInterrupt(leerADCs); //hace llamado a la función a ejecutar de forma periodica  
  // inicia comunicación serial
  Serial.begin(miBaudRate);
  // configura pines digitales de salida
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  debo_leer_adcs = true;
  debo_imprimir_en_serial = false;
}



void loop() {

  //unsigned long lecturaActual_ms = millis(); //lee hora actual

  generadorCuadrada(13,t_on_generadorD13_ms,t_off_generadorD13_ms,&cronoGen13); //pin D13 = LED en Arduino UNO
  generadorCuadrada(12,t_on_generadorD12_ms,t_off_generadorD12_ms,&cronoGen12);

  if (debo_imprimir_en_serial == true){
    imprimirEnSerial(); //función para imprimir resultado de datos medidos en puerto serial
  }
  else{
    delay(1); //libera algo de presión. Temporal. Quitarlo.
  }

  

  //// Lectura ADC, versión 20.2
  //    Basada en millis() y en lectura análoga normal (analogRead)
  //    Si es hora: lee, e imprime en serial
  /*if(lecturaActual_ms - lecturaPrevia_ms > periodoMuestreo_ms) {
    // guarda el valor actual de millis, para la siguiente lectura
    lecturaPrevia_ms = lecturaPrevia_ms + periodoMuestreo_ms;

    // lectura de entradas análogas 0,1,2:
    entradaA0 = analogRead(0);
    entradaA1 = analogRead(1);
    entradaA2 = analogRead(2);
    // calcula valor asociado de voltaje: 1023=5V
    voltajeA0 = deEnteroAVoltaje(entradaA0);
    voltajeA1 = deEnteroAVoltaje(entradaA1);
    voltajeA2 = deEnteroAVoltaje(entradaA2);

    //Imprime en puerto serial los valores leídos en voltios, con resolución 5mV.
    Serial.print(voltajeA0,3); //el '3' indica 3 cifras decimales
    Serial.print("\t");       //valores deben separarse con \t para 'serial plotter'
    Serial.print(voltajeA1,3);
    Serial.print("\t");       //valores deben separarse con \t para 'serial plotter'
    Serial.println(voltajeA2,3);//la última lectura debe terminar con un enter para 'serial plotter'
   
  }*/
  
}


void leerADCs(void)
{
  //Lee 2 puertos análogos de forma rápida. 
  //Almacena datos en memoria RAM.
  //Función creada en enero de 2021

  if (debo_imprimir_en_serial == false){
    //Solo ejecutar esta función (leerADCs) si no está imprimiendo datos pasados en serial.
    //Si está imprimiendo, no debe leer puertos análogos ni registrar hora actual

    unsigned long hora = micros();  //lo primero a hacer en esta función debe ser tomar la hora
    int dato_anterior = datosA0[i_dato];
  
    //**Opción 1: Usando analogRead
    //datosA0[i_dato] = analogRead(0);    //AnalogRead tarda 118us en ejecutarse.
    //datosA0[i_dato] = analogRead(1);    //AnalogRead tarda 118us en ejecutarse.
  
    //**Opción 2: Usando analogReadFast (librería avdweb_AnalogReadFast.h)
    datosA0[i_dato] = analogReadFast(A0); // se supone que tarda 20us en AVR 
    datosA1[i_dato] = analogReadFast(A1); // se supone que tarda 20us en AVR 
    
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
        
        if ((dato_anterior < umbral_analogo_valor) and (datosA0[i_dato] >= umbral_analogo_valor)) {
          //si hay flanco de subida, pasando por umbral,
          /*Serial.println("Hay flanco de subida en A0, inicio toma de datos");
          Serial.println("dato anterior:");
          Serial.println(dato_anterior);
          Serial.println("nuevo dato:");
          Serial.println(datosA0[i_dato]);*/
          debo_leer_adcs = true;  //iniciar toma de datos
          i_dato = 1; //asegurarse inicie en 1 con el siguiente dato
          tiempo_medidas_inicial_us = hora;  //importante: limpiar tiempos de medida anterior
          tiempo_medidas_final_us = hora;    //importante: limpiar tiempos de medida anterior
        }
      }
    }

    
  }

  
}

float deEnteroAVoltaje(int valor){
  //convierte número entero de 0 a 1023 en un voltaje, siendo 5V=1023.
  return float(valor)*5/1023; 
}
  
void generadorCuadrada(int pinDigital, int t_on_ms, int t_off_ms, unsigned long *cronoLocal){
  //genera una señal cuadrada periodica de amplitud 5V, con las
  //duraciones de encendido y apagado especificadas.

  unsigned long actualMillis = millis(); //lee hora actual
  unsigned long siguiente_on=0;
  unsigned long siguiente_off=0;

  siguiente_off = *cronoLocal + t_on_ms;    //apagar cuando hayan pasado t_on_ms
  siguiente_on = *cronoLocal + t_on_ms + t_off_ms;  //prender cuando hayan pasado t_on_ms+t_off_ms

  if(actualMillis > siguiente_on) { //si es hora de encender
    digitalWrite(pinDigital,HIGH);  //encender
    *cronoLocal = siguiente_on;     //actualiza hora de último encendido
    while(actualMillis > (*cronoLocal + t_on_ms + t_off_ms))
    {
      //si el reloj quedó atrasado, actualizar hasta que la diferencia no supere un periodo (t_on+t_off)
      *cronoLocal = *cronoLocal + t_on_ms + t_off_ms; //si el reloj quedó atrasado, actualizar hasta 
    }
  }
  else if(actualMillis > siguiente_off) { //si es hora de apagar
    digitalWrite(pinDigital,LOW); //apagar
  }  
  
}




void imprimirEnSerial(void){
  ////Versión limpia, para usar con monitor serial y plotter serial
  //    1er dato: tiempo en s,    2o dato: medida A0 en V     3er dato: medida A1 en V

  //unsigned long periodo_real_us =(tiempo_medidas_final_us-tiempo_medidas_inicial_us)/(NUM_DATOS-1);
  /*float periodo_real_s = 1.0*periodo_real_us;
  periodo_real_s = periodo_real_s / 1000000;*/

  unsigned long delta_real_us = (tiempo_medidas_final_us-tiempo_medidas_inicial_us);
  float periodo_real_us = 1.0*delta_real_us;
  periodo_real_us = periodo_real_us /(NUM_DATOS-1);
  
 // calcula valor asociado de voltaje: 1023=5V
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

    //Serial.print("periodo_real_us\t");
    //Serial.println(periodo_real_us);
  //Imprime en puerto serial los valores leídos en voltios, con resolución 5mV.
  for (k=0;k<NUM_DATOS;k++) {
    tiempo_s = 1.0*periodo_real_us;
    tiempo_s = tiempo_s*k;
    tiempo_s = tiempo_s/1000000;    
    voltajeA0 = deEnteroAVoltaje(datosA0[k]);
    voltajeA1 = deEnteroAVoltaje(datosA1[k]);
    Serial.print(tiempo_s,6);//el '6' indica 6 cifras decimales
    Serial.print(F("\t"));       //valores deben separarse con \t para 'serial plotter'
    Serial.print(voltajeA0,3); //el '3' indica 3 cifras decimales
    Serial.print(F("\t"));       //valores deben separarse con \t para 'serial plotter'
    Serial.print(voltajeA1,3);
    Serial.print(F("\n"));       //la última lectura debe terminar con un enter para 'serial plotter'
  }
  for (k=NUM_DATOS+1;k<500;k++){
    //imprimir linea en blanco para completar la gráfica de plotter serial, quien siempre grafica 500 datos
    Serial.println(F("-1\t-1\t-1")); 
  }
  
  ///Versión depuración:
  /*Serial.print("ImprimirEnSerial\n");

  Serial.print("Los datos guardados\n");

    Serial.print("Hora inicial (us)=");
    Serial.print(tiempo_medidas_inicial_us);
    Serial.print("\tHora final (us)=");
    Serial.print(tiempo_medidas_final_us);
    Serial.print("\nTiempo total (us)=");
    Serial.print(tiempo_medidas_final_us-tiempo_medidas_inicial_us);
    Serial.print("\nPeriodo real (us)=");
    
    Serial.print(periodo_real);
    Serial.print("\n");
    
    for (int k=0;k<NUM_DATOS;k++) {
      Serial.print("datosA0[");
      Serial.print(k);
      Serial.print("]=\t");
      Serial.print(datosA0[k]);
      Serial.print("\tdatosA1[");
      Serial.print(k);
      Serial.print("]=\t");
      Serial.print(datosA1[k]);
      Serial.print("\n");
    }
    */

  delay(tiempo_refresco_pantalla_ms);
  debo_imprimir_en_serial = false;
}
