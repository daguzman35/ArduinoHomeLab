/*
 * Arduino Home Lab
 * Hecho por: David Guzmán
 * 6 de julio de 2020
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

///////////////////////////////
//    Parámetros del usuario
//
//   para osciloscipio
//   1) Escoger la tasa (baudRate) de comunicación de puerto serial. ATENCIÓN: usar esta tasa en monitor serial y serial plotter.
unsigned long miBaudRate = 115200;  //en baudios, valores válidos: {9600,19200,38400,57600,115200,230400}
//   2) Escoger periodo de muestreo.
unsigned long periodoMuestreo_ms = 1;    // intervalo en el que lee entradas (milisegundos)=periodo de muestreo.
//
//   para generadores
int t_on_generadorD13_ms = 500;   //tiempo que dura el pulso de salida en D13 en encendido (5V)
int t_off_generadorD13_ms = 500;  //tiempo que dura el pulso de salida en D13 en apagado (0V)
int t_on_generadorD12_ms = 50;    //tiempo que dura el pulso de salida en D12 en encendido (5V)
int t_off_generadorD12_ms = 50;   //tiempo que dura el pulso de salida en D12 en apagado (0V)
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
/////////////////////////////////

void setup() {
  // inicia comunicación serial
  Serial.begin(miBaudRate);
  // configura pines digitales de salida
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
}

void loop() {

  unsigned long lecturaActual_ms = millis(); //lee hora actual

  generadorCuadrada(13,t_on_generadorD13_ms,t_off_generadorD13_ms,&cronoGen13); //pin D13 = LED en Arduino UNO
  generadorCuadrada(12,t_on_generadorD12_ms,t_off_generadorD12_ms,&cronoGen12);

  if(lecturaActual_ms - lecturaPrevia_ms > periodoMuestreo_ms) {
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
