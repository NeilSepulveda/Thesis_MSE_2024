//Librerias
  #include <SoftwareSerial.h>   //Comunicación Serial
  #include <math.h> //Libreria de matemática

//Definiendo comunicación serial ESP8266(NodeMCU)
  const byte rxPin = 2;
  const byte txPin = 3;
  SoftwareSerial SerialIoT =  SoftwareSerial(rxPin, txPin);

//Constantes
  #define pi 3.1415926535897    //Valor de PI 
  #define Kv 0.6842619745845    //Constante de calibración Sensor de voltaje AC "(1*350)/511.5"
  #define Ki 0.0977517106549    //Constante de calibración Sensor de corriente AC "(1*50)/511.5"

//Valores de reseteo
  const int PIN_RESET = 12;

//Valores timepo selecionable
  const int Time_1 = 4;
  const int Time_2 = 5;
  const int Time_3 = 6;
  const int Time_4 = 7;
  const int Time_5 = 8;

//Pines analógicos
  int Voltage_Sensor = A0;     //Sensor de voltaje AC
  int Current_Sensor = A1;     //Sensor de corriente AC
  int Zero = A2;               //Sensor de tensión DC (valor de referncia cero)

//Valores para computo en Serie de Fourier
  float w0 = 100*pi;              //Frecuencia de referencia en [rad/s]
  float Ts = 0.0002;              //Tiempo de muetsreo en segundos
  float T_Ts = 200;               //Tiempo de muetsreo en microsegundos
  float DT = 0.02;                //Usado para el cálculo de armónicos "2/L", Ts = 0.0002[s] & L = 0.02[s] 

//Vectores para gusrdar datos
  float I[102], V[102];  

//Armónicos                                                               
  float Va_1 = 0, Vb_1 = 0, Va_3 = 0, Vb_3 = 0, VA_1 = 0, VB_1 = 0, VA_3 = 0, VB_3 = 0;     //Valores para el cálculo de armónicos de voltaje
  float Ia_1 = 0, Ib_1 = 0, Ia_3 = 0, Ib_3 = 0, IA_1 = 0, IB_1 = 0, IA_3 = 0, IB_3 = 0;     //Valores para el cálculo de armónicos de corriente
  float teta_V = 0, teta_I = 0;

//Variables de Tiempo de funciones
  unsigned long Cont_micros = 0;        //Contador de tiempo de microsegundos
  unsigned long T_Vdc =       100000;   //Tiempo de referencia DC  
  unsigned long T_Vsave =     120000;   //Tiempo de para guardar datos de voltaje
  unsigned long T_Isave =     140000;   //Tiempo de para guardar datos de corriente
  unsigned long T_Vcal =      400000;   //Tiempo cálculo de variables de voltaje
  unsigned long T_Ical =      600000;   //Tiempo cálculo de variables de corriente
  unsigned long T_Pcal =      800000;   //Tiempo cálculo de variables de potencia
  unsigned long T_Final =    1000000;   //Tiempo final

//Vriables de tiempo IoT
  unsigned long Cont_IoT = 0;     //Tiempo de envío de datos IoT
  unsigned long IoT_add = 0;      //Tiempo seleccionado de envío de datos IoT

//Valores de entrada
  float Vn = 0;         //Señal de voltaje AC
  float In = 0;         //Señal de corriente AC
  float Vdc = 0;        //Señal de refernecia cero
  float Vdc_ref = 0;    //Señal de refernecia cero filtrada

//Potencia
  float P_med;          //Valor medio de energía
  
//Frecuencia
  float k1= 0.0020910, k2= -1.9940000, k3 = 0.9979000;          //Cosntantes filtro digital pasabanda IIR
  float Vf[3];
  float Vd = 0;
  float V1 = 0;
  float V2 = 0;
  float Vf_RMS = 0;
  float Vd_RMS = 0;

//Variable de conteo
  int N = 0;
  int M = 0;
  int K = 0;

//Variables para cálculo (valores finales)
  float Voltage = 0;
  float Voltage_1 = 0;
  float Voltage_3 = 0;
  float Voltage_THD = 0;
  float Current = 0;
  float Current_1 = 0;
  float Current_3 = 0;
  float Current_THD = 0;
  float Power_P = 0;
  float Power_Q = 0;
  float Power_S = 0;
  float FP = 0;
  float Frequency = 0;



//Funciones principales .........................................................



void setup() {  
  Serial.begin(115200);         //Inicializamos la comunicación serial
  SerialIoT.begin(115200);      //Inicializamos la comunicación serial ESP8266(NodeMCU)
  analogReference(EXTERNAL);    //Referencia de voltaje paradel pin AREF
  Inicial_pines();              //Inicialización de pines
  Select_time();                //Selección tiempo de envío de datos a puerto IoT
  Inicial_reset();              //Inicializando variables para resetar
}

void loop() {
  Referencia_zero();
  Guardar();
  Calculo_voltage();
  Calculo_current();
  Calculo_potencia();
  Imprimir();
  if( millis()>= 3600000 ){Reset();}
  if( FP == 0 ){Reset();}
  
}



//Funciones de cálculo .........................................................



//Filtro media móvil para tensión DC
void Referencia_zero(){
  Vdc = analogRead(Zero);
  while( micros()<T_Vdc ){Vdc_ref = Filtro_MM(0.1, Vdc, Vdc_ref);}
  T_Vdc = T_Vdc + 1000000;
}

//Cálculo para variables de voltaje
void Guardar(){
  //Guardando datos de voltaje
  N = 0;
  Cont_micros = micros();
  while( micros()< T_Vsave ){ 
    if( micros()>= Cont_micros ){
      V[N] = analogRead(Voltage_Sensor);  //Lectura del sensor de voltaje AC
      Cont_micros = Cont_micros + T_Ts;
      N++; 
    }
  }
  T_Vsave = T_Vsave + 1000000;

  //Guardando datos de corriente
  M = 0;
  Cont_micros = micros();
  while( micros()<T_Isave ){ 
    if( micros()>=Cont_micros ){
      I[M] = analogRead(Current_Sensor);  //Lectura del sensor de corriente AC
      Cont_micros = Cont_micros + T_Ts;
      M++; 
    }
  }
  T_Isave = T_Isave + 1000000;
  delayMicroseconds(500);
}

void Calculo_voltage(){
  //Definiedo valor
  for( int i=0 ; i<N ; i++ ){V[i] = V[i] - Vdc_ref; V[i] = Kv*V[i];}
  
  //Cálculo RMS
  Voltage = 0;
  for( int i=0 ; i<N ; i++ ){Voltage = Voltage + V[i]*V[i];}
  Voltage = sqrt(Voltage/N);
  delayMicroseconds(500);
  
  //Cálculo valor fundamental 
  VA_1=0; VB_1=0;
  for( int i=0 ; i<N ; i++ ){
        Va_1 = V[i]*cos(Ts*w0*i);                
        Vb_1 = V[i]*sin(Ts*w0*i);
        VA_1 = VA_1 + Va_1; VB_1 = VB_1 + Vb_1;
  }
  VA_1 = DT*VA_1; VB_1 = DT*VB_1;
  Voltage_1 = VA_1*VA_1 + VB_1*VB_1;
  Voltage_1 = sqrt(Voltage_1/2);
  delayMicroseconds(500);
  if( VA_1==0 ){
    if( VB_1==0 ){teta_V = 0;}
    else{if( VB_1>0 ){teta_V = pi/2;}else{teta_V = -pi/2;}}
  }else{teta_V = VB_1/VA_1; teta_V = atan(teta_V);}
  delayMicroseconds(500);

  //Cálculo tercer armónico 
  VA_3=0; VB_3=0;
  for( int i=0 ; i<N ; i++ ){
        Va_3 = V[i]*cos(3*Ts*w0*i);                                                      
        Vb_3 = V[i]*sin(3*Ts*w0*i);
        VA_3 = VA_3 + Va_3; VB_3 = VB_3 + Vb_3;
  }
  VA_3 = DT*VA_3; VB_3 = DT*VB_3;
  Voltage_3 = VA_3*VA_3 + VB_3*VB_3;
  Voltage_3 = sqrt(Voltage_3/2);
  delayMicroseconds(500);

  //Cálculo THD
  Voltage_THD = Voltage*Voltage - Voltage_1*Voltage_1;
  if( Voltage_THD>0 && Voltage_1!=0 ){Voltage_THD = sqrt(Voltage_THD); Voltage_THD = (Voltage_THD/Voltage_1)*100;}
  else{Voltage_THD = 0;}
  delayMicroseconds(500);

 //Calculo frecuencia 
  Vf[0]=0; Vf[1]=0; Vf[2]=0;
  Vf_RMS = 0; Vd_RMS = 0; K=0; V1=0; V2=0;  
  while( K<=101 ){
    for( int i=0 ; i<N ; i++ ){
      Vf[0] = Filtro_IIR(Vf[1], Vf[2], V1, V2);
      if( K>100 ){
        Vd = (Vf[0]-Vf[1])/Ts;
        Vd_RMS = Vd_RMS + Vd*Vd;
        Vf_RMS = Vf_RMS + Vf[0]*Vf[0];
      }
      Vf[2] = Vf[1];
      Vf[1] = Vf[0];
      V2 = V1;
      V1 = V[i];
    }
    K++;
  }
  delayMicroseconds(500);
  if(Vf_RMS>10){
    Frequency = Vd_RMS/Vf_RMS;
    Frequency = sqrt(Frequency);
    Frequency = Frequency/(2*pi);
  }else{Frequency = 0;}
  
  delayMicroseconds(500);
  while (micros() < T_Vcal){delayMicroseconds(200);}
  T_Vcal = T_Vcal + 1000000;
}

void Calculo_current(){
  //Definiedo valor
  for( int i=0 ; i<M ; i++ ){I[i] = I[i] - Vdc_ref; I[i] = Ki*I[i];}
  
  //Cálculo RMS
  Current = 0;
  for( int i=0 ; i<M ; i++ ){Current = Current + I[i]*I[i];}
  Current = sqrt(Current/M);
  delayMicroseconds(500);
  
  //Cálculo valor fundamental 
  IA_1=0; IB_1=0;
  for( int i=0 ; i<M ; i++ ){
        Ia_1 = I[i]*cos((Ts*w0*i)-teta_V);                
        Ib_1 = I[i]*sin((Ts*w0*i)-teta_V);
        IA_1 = IA_1 + Ia_1; IB_1 = IB_1 + Ib_1;
  }
  IA_1 = DT*IA_1; IB_1 = DT*IB_1;
  Current_1 = IA_1*IA_1 + IB_1*IB_1;
  Current_1 = sqrt(Current_1/2);
  delayMicroseconds(500);
  if( IA_1==0 ){
    if( IB_1==0 ){teta_I = 0;}
    else{if( IB_1>0 ){teta_I = pi/2;}else{teta_I = -pi/2;}}
  }else{teta_I = IB_1/IA_1; teta_I = atan(teta_I);}
  delayMicroseconds(500);

  //Cálculo tercer armónico 
  IA_3=0; IB_3=0;
  for( int i=0 ; i<M ; i++ ){
        Ia_3 = I[i]*cos(3*Ts*w0*i);                                                      
        Ib_3 = I[i]*sin(3*Ts*w0*i);
        IA_3 = IA_3 + Ia_3; IB_3 = IB_3 + Ib_3;
  }
  IA_3 = DT*IA_3; IB_3 = DT*IB_3;
  Current_3 = IA_3*IA_3 + IB_3*IB_3;
  Current_3 = sqrt(Current_3/2);
  delay(1);

  //Cálculo THD
  Current_THD = Current*Current - Current_1*Current_1;
  if( Current_THD>0 && Current_1!=0 ){Current_THD = sqrt(Current_THD); Current_THD = (Current_THD/Current_1)*100;}
  else{Current_THD = 0;}

  delayMicroseconds(500);
  while (micros() < T_Ical){delayMicroseconds(200);}
  T_Ical = T_Ical + 1000000;
}

void Calculo_potencia(){
  if(M<N){N=M;}
  Power_P = 0;
  for( int i=0 ; i<N ; i++ ){Power_P = Power_P + V[i]*I[i];}
  Power_P = Power_P/N;
  Power_S = Voltage*Current;
  delayMicroseconds(500);
  Power_Q = Power_S*Power_S - Power_P*Power_P;
  if( Power_Q>0 ){Power_Q = sqrt(Power_Q);}
  else{Power_Q = 0;}
  if( Power_S==0 ){FP = 0;}
  else{FP = Power_P/Power_S; FP = abs(FP);}
  if(teta_I<0 && FP<0.99){FP=-1*FP;}
  
  delayMicroseconds(500);
  while ( micros()<T_Pcal ){delayMicroseconds(200);}
  T_Pcal = T_Pcal + 1000000;
}



//Funciones para envio de datos....................................................



//Función cambio de tiempo
void Select_time(){
  IoT_add = 5000000;
  int Status_1 = digitalRead(Time_1); delay(1);
  int Status_2 = digitalRead(Time_2); delay(1);
  int Status_3 = digitalRead(Time_3); delay(1);
  int Status_4 = digitalRead(Time_4); delay(1);
  int Status_5 = digitalRead(Time_5); delay(1);
  delay(5);
  
  if(Status_1 == HIGH){IoT_add = 10000000; delay(1);}
  if(Status_2 == HIGH){IoT_add = 15000000; delay(1);}
  if(Status_3 == HIGH){IoT_add = 20000000; delay(1);}
  if(Status_4 == HIGH){IoT_add = 25000000; delay(1);}
  if(Status_5 == HIGH){IoT_add = 30000000; delay(1);}
}

//Función para comunicación serial
void Imprimir(){                    
  Serial_RxTx();
  delay(1);
  if(micros()>= Cont_IoT){
    Enviar_IoT();
    Cont_IoT = Cont_IoT + IoT_add;
    Select_time();;
    delay(1);
  }
  while ( micros()<=T_Final ){delayMicroseconds(200);}
  T_Final = T_Final + 1000000;
}



//Filtros digitales........................



//Filtro de Media Movil
float Filtro_MM(float Valor_1, float Valor_2, float Valor_3){
  float Resultado;
    Resultado = Valor_1*Valor_2+(1-Valor_1)*Valor_3;
  return Resultado;
}

//Filtro digital pasabanda
float Filtro_IIR(float Valor_1, float Valor_2, float Valor_3, float Valor_4){
  float Resultado;
    Resultado = -1*(k2* Valor_1 + k3*Valor_2) + k1*(Valor_3 - Valor_4 );
  return Resultado;
}



//Funciones de inicio y reseteo.............................................


void Inicial_reset(){
  digitalWrite(PIN_RESET, HIGH);
  pinMode(PIN_RESET, OUTPUT);  
}

void Inicial_pines(){
  pinMode(Time_1, INPUT);
  pinMode(Time_2, INPUT);
  pinMode(Time_3, INPUT);
  pinMode(Time_4, INPUT);
  pinMode(Time_5, INPUT);
}

void Reset(){ 
  delay(10);
  digitalWrite(PIN_RESET, LOW);
}


//Envio datos...............................................................



//Enviar comunicación serial
void Serial_RxTx(){
  Serial.print("{\"Voltage\":");
  Serial.print(Voltage,2);
  Serial.print(",\"Voltage_1\":");
  Serial.print(Voltage_1,2);
  Serial.print(",\"Voltage_3\":");
  Serial.print(Voltage_3,2);
  Serial.print(",\"Voltage_THD\":");
  Serial.print(Voltage_THD,2);
  Serial.print(",\"Current\":");
  Serial.print(Current,2);
  Serial.print(",\"Current_1\":");
  Serial.print(Current_1,2);
  Serial.print(",\"Current_3\":");
  Serial.print(Current_3,2);
  Serial.print(",\"Current_THD\":");
  Serial.print(Current_THD,2);
  Serial.print(",\"Power_P\":");
  Serial.print(Power_P,2);
  Serial.print(",\"Power_Q\":");
  Serial.print(Power_Q,2);
  Serial.print(",\"Power_S\":");
  Serial.print(Power_S,2);
  Serial.print(",\"FP\":");
  Serial.print(FP,2);
  Serial.print(",\"Frequency\":");
  Serial.print(Frequency,4);
  Serial.println("}");
}

//Enviar datos a ESP3286
void Enviar_IoT(){
  SerialIoT.print(Voltage,2);
  SerialIoT.print(";");
  SerialIoT.print(Voltage_1,2);
  SerialIoT.print(";");
  SerialIoT.print(Voltage_3,2);
  SerialIoT.print(";");
  SerialIoT.print(Voltage_THD,2);
  SerialIoT.print(";");
  SerialIoT.print(Current,2);
  SerialIoT.print(";");
  SerialIoT.print(Current_1,2);
  SerialIoT.print(";");
  SerialIoT.print(Current_3,2);
  SerialIoT.print(";");
  SerialIoT.print(Current_THD,2);
  SerialIoT.print(";");
  SerialIoT.print(Power_P,2);
  SerialIoT.print(";");
  SerialIoT.print(Power_Q,2);
  SerialIoT.print(";");
  SerialIoT.print(Power_S,2);
  SerialIoT.print(";");
  SerialIoT.print(FP,2);
  SerialIoT.print(";");
  SerialIoT.println(Frequency,4);
}
