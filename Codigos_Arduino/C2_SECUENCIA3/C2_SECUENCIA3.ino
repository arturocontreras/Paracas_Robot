#include <NewPing.h>
int datas[10];
int XT; //Tipo de tarea
int carga;
float medida_u1;
float medida_u2;
int flag1;
unsigned long tiempo_inicio;
int dutty_bomba = 0;

#define PC Serial
#define C1 Serial1
#define SCADA Serial2

#define VA1 2 // Electroválvula 1, tipo salida a relé //IZQUIERDA
#define VA2 3 // Electroválvula 2, tipo salida a relé //DERECHA
#define MB  6 // Relé de la motobomba


//Tareas
#define fumigacion  1
#define adquisicion 2
#define transporte  3
#define MAX_CARGA   100
#define MIN_LEVEL_TANQUE   10
#define REF_U1   100
#define REF_U2   100
#define TIEMPO_PARADA   10000 //En milisegundos

//Sensores de ultrasonido
#define trig1 41
#define echo1 43
#define trig2 45
#define echo2 47
#define trig3 49
#define echo3 51
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing tanque(trig1, echo1, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing arbol_der(trig2, echo2, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing arbol_izq(trig3, echo3, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

//Sensores Analógicos
#define S1 A1 
#define S2 A2 
#define S3 A3 
#define S4 A4 
#define transmisor_presion A14
#define corriente_total A10
#define voltaje_total A9
#define humedad_suelo A12
#define ph_suelo A13
#define radiacion_solar A11
//Sensores I2C y digitales
#define dht11 53

//Valor de las variables
float hum_amb, temp_amb,rad_sol, hum_sue,i_tot,v_tot,nivel_tank;
//Entradas y salidas extras
#define Out1 23 
#define Out2 25 
#define Out3 27 
#define Out4 29 
#define In1  31 
#define In2  33 
#define In3  35 
#define In4  37 

//Recepciones Seriales
int ibyte,val;
int data[20];
int lata,latb,latc,latd,lona,lonb,lonc,lond,yaw,obs;
// variables
int buttonState = 0;        

//** Tiempos envio serial
long t_ant = 0;
//
void setup() {
  // initialize the LED pin as an output:
  pinMode(VA1, OUTPUT);      
  pinMode(VA2, OUTPUT);   
  pinMode(MB, OUTPUT);   
  
  digitalWrite(VA1, LOW);
  digitalWrite(VA2, LOW);
  
  PC.begin(115200);
  C1.begin(115200); //Comunicación con C1
  SCADA.begin(115200); //Comunicación con SCADA
  Serial.println("Reset");
  
}

void loop(){
   
  delay(5);                      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  control_presion();
  //activacion_electrovalvulas();
  
  if((millis()-t_ant)>=5000){
  read_sensors();}
  
  Serial.print(TOVOLTAJE(analogRead(voltaje_total)));
  Serial.print("    ");
  Serial.println(TOCORRIENTE(analogRead(corriente_total)));

  
  recepcion_C1();
  
  analogWrite(MB,dutty_bomba);
  
  if(Serial.available()>0){
      char ref=Serial.read();
      //Serial.println(ref);
      if(ref == '3'){
        digitalWrite(VA1, LOW);
        // dutty_bomba = 0;

      }
      if(ref == '4'){
        digitalWrite(VA1, HIGH);
         //dutty_bomba = 80;

      }
      else if(ref == '5'){
        digitalWrite(VA2, LOW);
       //digitalWrite(VA2, HIGH);
       // dutty_bomba = 100;
      }
      
      else if(ref == '6'){
        digitalWrite(VA2, HIGH);
        //dutty_bomba = 140;
      }
      else if(ref == '0'){
        dutty_bomba = 0;
       // Serial.println("dutty 150");
      }
      else if(ref == '1'){
        dutty_bomba = 150;
      }
      else if(ref == '2'){
        dutty_bomba = 255;
      }
  }

}

void enviar_datos(){
 //long temp1 = micros();
  
  Serial2.print('y');
  Serial2.print((char)hum_amb);
  Serial2.print((char)temp_amb);
  Serial2.print((char)rad_sol);
  Serial2.print((char)hum_sue);

  
//  Serial2.print('x');
//  Serial2.print((char)10);
//  Serial2.print((char)11);
//  Serial2.print((char)12);
//  Serial2.print((char)13);
//  Serial2.print((char)14);
//  Serial2.print((char)15);
//  Serial2.print((char)16);
//  Serial2.print((char)17);
//  Serial2.print((char)18);
//  Serial2.print((char)19);
//  Serial2.print((char)20);
//  Serial2.print((char)21);
//  Serial2.print((char)22);
//  Serial2.print((char)23);
//  Serial2.print((char)24);
//  Serial2.print((char)25);
//  Serial2.print((char)26);
  
 //Serial.println(micros()-temp1);
}

void init_sensors(){}
boolean llego_DP(){} 
void colocar_sensor(){}
void recoger_sensor(){} 

void read_sensors(){
  //Se van a tomar 4 muestras que se van a promediar
  hum_amb  = read_hum_amb();
  //Serial.print("Hum Amb: ");
  //Serial.println(hum_amb);
  
  temp_amb = read_temp_amb();
  //Serial.print("Temp : ");
  //Serial.println(temp_amb);
  
  rad_sol = analogRead(radiacion_solar);
  rad_sol = map(rad_sol,0,1023*4/5,0,12);
  //Serial.print("Rad : ");
  //Serial.println(rad_sol);
  
  hum_sue = analogRead(humedad_suelo);
  hum_sue = map(hum_sue,0,1023,0,100);
  //Serial.print("Hum Suelo : ");
  //Serial.println(hum_sue);
  
  //i_tot = analogRead(corriente_total);
  //i_tot = map(i_tot,0,1023,0,100);
  //i_tot = TOCORRIENTE(i_tot);
  //Serial.print("I total : ");
  //Serial.println(i_tot);
  
  //v_tot = analogRead(voltaje_total);
  //v_tot = map(v_tot,0,1023,0,100);
  //Serial.print("V total : ");
  //Serial.println(v_tot);
  //Trabajaremos con interrupción timer 1
  enviar_datos();
  
  t_ant = millis();//Se actualiza el tiempo anterior
 
}

void activacion_electrovalvulas(){
  medida_u1 = medida1();

//  if(medida_u1 <= REF_U1 && medida_u1>40) //detecta el arbol en el lado1?
//  {
//   digitalWrite(VA1, HIGH);  
//   Serial.println("VA1 - ON");
//  }
//  else
//  {
//   digitalWrite(VA1, LOW);
//   Serial.println("VA1 - OFF");
//  }
  
  medida_u2 = medida2();
  
//  if(medida_u2 <= REF_U2 && medida_u2>40) //detecta el arbol en el lado1?
//  {
//   digitalWrite(VA2, HIGH);  
//   Serial.println("VA2 - ON");
//  }  
//  else
//  {
//   digitalWrite(VA2, LOW);
//   Serial.println("VA2 - OFF");
//  }  
//  
}

void control_presion(){
}

float medida1(){
  unsigned int uS = arbol_izq.ping(); // Send ping, get ping time in microseconds (uS).
  Serial.print("Ping1: ");
  Serial.print(uS / US_ROUNDTRIP_CM); // Convert ping time to distance in cm and print result (0 = outside set distance range)
  Serial.print("cm  ");
  return uS / US_ROUNDTRIP_CM;
}

float medida2(){
  unsigned int uS = arbol_der.ping(); // Send ping, get ping time in microseconds (uS).
  Serial.print("Ping2: ");
  Serial.print(uS / US_ROUNDTRIP_CM); // Convert ping time to distance in cm and print result (0 = outside set distance range)
  Serial.println("cm");
  return uS / US_ROUNDTRIP_CM;
}

float nivel_tanque(){
  unsigned int uS = tanque.ping(); // Send ping, get ping time in microseconds (uS).
  return uS / US_ROUNDTRIP_CM;
}

void recepcion_C1(){
  
  if(C1.available()>0){
      val=C1.read();
      if(val=='x'){
        while(C1.available()==0){};
          int i=0;
          for(i=0;i<11;i=i+1){
           data[i]=C1.read();
            while(C1.available()==0){};
            // && (char)data[i]!='y'
           }
           flag1 = true;
             //Asignación de variables, solo para que se comprenda mejor:
       lata = data[0];
       latb = data[1];
       latc = data[2];
       latd = data[3];
       lona = data[4];
       lonb = data[5];
       lonc = data[6];
       lond = data[7];
       yaw = data[8];
       obs = data[9];
      }
     else if(val=='y'){
            //  Serial.println((int)val);   
       while(C1.available()==0){};
       int comando =C1.read();
       Serial.println(comando);   
       switch(comando){
          case 1:
            dutty_bomba = 255;
            Serial.println("Bomba on"); 
          break;
          case 2:
            dutty_bomba = 0;
          break;
          case 3:
            digitalWrite(VA1, HIGH);
          break;
          case 4:
            digitalWrite(VA1, LOW);
          break;
          case 5:
            digitalWrite(VA2, HIGH);
          break;
          case 6:
            digitalWrite(VA2, LOW);
          break;
       }
     }
   
   
    if(flag1){
//     for (int j=0;j<11;j++){    
//     Serial.print(data[j]);
//     Serial.print(" ");       
//     }
     Serial2.print('x');
     Serial2.print((char)lata);
     Serial2.print((char)latb);
     Serial2.print((char)latc);
     Serial2.print((char)latd);
     Serial2.print((char)lona);
     Serial2.print((char)lonb);
     Serial2.print((char)lonc);
     Serial2.print((char)lond);
     Serial2.print((char)yaw);
     Serial2.print((char)obs);
     //Serial2.print((char)i_tot);
     Serial2.print((char)TOCORRIENTE(analogRead(corriente_total)));
     Serial2.print((char)TOVOLTAJE(analogRead(voltaje_total)));
     //Serial2.print((char)v_tot);
     Serial2.print((char)nivel_tank);
     flag1 = false; 
     
    }  
         
  }
}

float TOVOLTAJE(int digital){
 return   digital*0.15299;
}

float TOCORRIENTE(int digital){
 return   (510-digital)*0.2443;
}
