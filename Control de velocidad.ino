//CÓDIGO LEONARDO
#include "PID_v1.h"
#include "ModbusMaster.h"  // Load the (modified) library for modbus communication command codes. Kindly install at our website.

#define MAX485_DE 8        // Define DE Pin to Arduino pin. Connect DE Pin of Max485 converter module to Pin 2 (default) Arduino board
#define MAX485_RE 8        // Define RE Pin to Arduino pin. Connect RE Pin of Max485 converter module to Pin 3 (default) Arduino board

// These DE anwr RE pins can be any other Digital Pins to be activated during transmission and reception process.
static int bombaSlaveAddr = 9;  // Declare the address of device (meter) in term of 8 bits. You can change to 0x02 etc if you have more than 1 meter.    
// Declare your external shunt value. Default is 100A, replace to "0x0001" if using 50A shunt, 0x0002 is for 200A, 0x0003 is for 300A
//static int NewshuntAddr = 3;  
uint16_t ad_reg=0x008;

int velocidadesPosibles[] = {25, 50, 75, 100};  // Ajusta según tus necesidades
int numVelocidades = sizeof(velocidadesPosibles) / sizeof(velocidadesPosibles[0]);

//variables para el flujometro
volatile int NumPulsos=0; //variable para la cantidad de pulsos recibidos
int PinSensor = 2;    //Sensor conectado en el pin 2
bool flag_modbus = false;

ModbusMaster node;                     /* activate modbus master codes*/
unsigned long startMillis;             /* start counting time for LCD Display */
unsigned long currentMillis;           /* current counting time for LCD Display */
unsigned long oldTime;
const unsigned long period = 1000;     // refresh every X seconds (in seconds) in LED Display. Default 1000 = 1 second
uint16_t bombaSpeed = 0;
float frecuencia=0;
float caudal_L_m=0;
float caudal_L_h=0;



float caudalDeseado=30;
float errorMaximo=5;
bool flagCalibracion=0;
int caudal_calibracion;
double calibracion=0.0;

//PID
double setpoint = 200;  // Velocidad deseada (ajústala según tus necesidades)
double input;
double output;
double Kp = 2, Ki = 5, Kd = 10;  // Ajusta estos valores según la respuesta de tu sistema

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600); /* To assign communication port to communicate with meter. with 2 stop bits (refer to manual)*/

  startMillis = millis();                 /* Start counting time for run code */
  node.begin(bombaSlaveAddr,Serial);             /* Define and start the Modbus RTU communication. Communication to specific slave address and which Serial port */
  
  pinMode(MAX485_RE, OUTPUT);             /* Define RE Pin as Signal Output for RS485 converter. Output pin means Arduino command the pin signal to go high or low so that signal is received by the converter*/
  pinMode(MAX485_DE, OUTPUT);             /* Define DE Pin as Signal Output for RS485 converter. Output pin means Arduino command the pin signal to go high or low so that signal is received by the converter*/
  digitalWrite(MAX485_RE, 0);             /* Arduino create output signal for pin RE as LOW (no output)*/
  digitalWrite(MAX485_DE, 0);             /* Arduino create output signal for pin DE as LOW (no output)*/
                                          // both pins no output means the converter is in communication signal receiving mode
  node.preTransmission(preTransmission);  // Callbacks allow us to configure the RS485 transceiver correctly
  node.postTransmission(postTransmission);

  

  delay(1000); /* after everything done, wait for 1 second */
  oldTime=millis();
  pinMode(PinSensor, INPUT);
  attachInterrupt(0, ContarPulsos, RISING);

  // Inicializar el PID
  pid.SetMode(AUTOMATIC);
}

void loop(){
 
  unsigned long currentTime = millis();


  // Calculate flow rate every second
  if (currentTime - oldTime > 1000) {
    detachInterrupt(0);
    frecuencia = NumPulsos;
    // Calculate flow rate
    float fc = fCorreccion(frecuencia, flagCalibracion);
    caudal_L_m=frecuencia/fc; //calculamos el caudal en L/m
    caudal_L_h=caudal_L_m*60; //calculamos el caudal en L/h 
    //Serial.print(fc);

    // Reset pulse count and time
    
    NumPulsos = 0;
    oldTime = currentTime;

    attachInterrupt(0, ContarPulsos, RISING);

    
    // Print results
    Serial.print ("Frecuencia de Pulsos: "); 
    Serial.print (frecuencia,0); 
    Serial.print (" Hz \tCaudal: "); 
    Serial.print (caudal_L_m,3); 
    Serial.print (" L/m\t"); 
    Serial.print (caudal_L_h,3); 
    Serial.println ("L/h"); 
  }

  // Actualizar la entrada del PID
  input = caudal_L_h;

  // Calcular la salida del PID
  pid.Compute();
  //Serial.println(output);
  

  currentMillis = millis();                  /* count time for program run every second (by default)*/
  if (currentMillis - startMillis >= period){ /* for every x seconds, run the codes below*/
    uint8_t result;
                                  /* Declare variable "result" as 8 bits */
    result=node.readHoldingRegisters(ad_reg, 1); /* read the register for the speed (information) of the PZEM-014 / 016 starting 0x0000 (voltage information) kindly refer to manual)*/
    if (result == node.ku8MBSuccess){              /* If there is a response */

      bombaSpeed = node.getResponseBuffer(0);
      Serial.println();
      Serial.println("La velocidad de la bomba es: " );
      Serial.println(bombaSpeed);

      uint16_t output16=output;
      bombaSpeed=controlarBomba(output16);
     
       
    }else{
      Serial.println("No sale nada ");
      Serial.println(result);
    }
    
    startMillis = currentMillis; /* Set the starting point again for next counting time */
  }
  

  //delay(1000);
}

uint16_t controlarBomba(uint16_t nuevaVelocidad){
    uint16_t velocidad = node.writeSingleRegister(0x0008, nuevaVelocidad);
    return velocidad;
}

float fCorreccion(float frecuencia, bool flagCalibracion){
  float caudalDeseado = 30.0;
  float errorMaximo = 5.0;
  float conversion = 98.0;
  if(frecuencia!=0.0){
    if(abs((frecuencia/conversion)*60-caudalDeseado)>=errorMaximo){
      flagCalibracion = 1;
    }

    if(flagCalibracion==1){
      caudal_calibracion = 25;
      conversion=frecuencia/caudal_calibracion;
      flagCalibracion=0;
    }
  }
    //caudal_L_h=frecuencia*conversion;
    return conversion;
}
void ContarPulsos ()
{ 
  NumPulsos++;  //incrementamos la variable de pulsos
} 


void preTransmission() /* transmission program when triggered*/
{
  digitalWrite(MAX485_RE, 1); /* put RE Pin to high*/
  digitalWrite(MAX485_DE, 1); /* put DE Pin to high*/
  delay(1);                   // When both RE and DE Pin are high, converter is allow to transmit communication
}

void postTransmission() /* Reception program when triggered*/
{
  delay(3);                   // When both RE and DE Pin are low, converter is allow to receive communication
  digitalWrite(MAX485_RE, 0); /* put RE Pin to low*/
  digitalWrite(MAX485_DE, 0); /* put DE Pin to low*/
}
