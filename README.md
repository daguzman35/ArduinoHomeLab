# ArduinoHomeLab
Arduino-based oscilloscope and square signal generator.

Generador de pulsos cuadrados (x2) y osciloscopio (x2) con Arduino.

Diseñado para usar con 'serial plotter' de Arduino IDE.
Resolución kHz.
Probado en Arduino Uno. 

## Instrucciones de uso
 Debe instalar las siguientes bibliotecas (libraries):
 * TimerOne                versión 1.1.0
 * avdweb_AnalogReadFast   versión 1.0.0
 
 Para instalar bibliotecas: https://www.arduino.cc/en/guide/libraries
 
 Modifique los parámetros en la sección 'parámetros de usuario'.
 Evite alterar otras partes del código.
 
Configuración por defecto:

| Parámetro         | Valor           |
| ------------- | -------------|
| comunicación serial   | 115200 baud |
| frecuencia de muestreo     | 10kHz (T=100us)      |
| número de muestras | 501      |
|umbral disparo A0          | int 50 = aprox. 250mV |
 |tiempo refresco datos      | 5 segundos |

 
## Historial de versiones
 
 ### v21.1
 11 de enero de 2021
 
 Mejoras significativas en osciloscopio.
  
 Osciloscopio:
 *  Resolución temporal de mediciones mejorada: 50us (20kHz).
 *  Mediciones usan interrupciones, mejorando confiabilidad.
 *  Mide 2 canales análogos; antes medía 3.
 *  Almacena en RAM todos los datos medidos. Cuando llena memoria, los imprime.
 *  Imprime valor de tiempo, donde t=0 es el inicio de la medición.
 *  Encabezados de columna de datos incluidos.
 *  Sistema de umbral (threshold) implementado, emulando disparo de osciloscopios reales.
 
 Generador:
 *  Para cada generador, solo se debe dar el periodo. Sistema asume duty cycle=50%.
 
 ### v20.2
 6 de julio de 2020
 
 Primera versión pública.
 
 *  Mide 3 canales análogos, en voltios. Rango: 0V a 5V. Precisión 10 bits: 5mV.
 *  Genera señales cuadradas periódicas de amplitud 5V, con duración especificada por el usuario.
 *  Resolución temporal de mediciones: 10ms (100Hz).
 *  Resolución temporal de generador: 1ms (1kHz).
 *  Imprime datos medidos por puerto serial.
 *  Puede usarse con algún monitor serial para guardar en archivo de texto plano los datos.
 *  Ideal para usarse con 'serial plotter' de Arduino.
