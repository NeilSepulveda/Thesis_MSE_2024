//Librerias
  #include <ESP8266WiFi.h>
  #include <DNSServer.h>
  #include <ESP8266WebServer.h >
  #include <WiFiManager.h>
  #include <PubSubClient.h>          
  #include <SD.h>    

//Definiendo conceptos para el envio de datos a la Tarjeta SD
  int SSpin = 15;   //D8 pertenece al 15
  File archivo;     //Objeto archivo del tipo File
  int N = 0;        //Conteo para conectar SD
  int M = 0;        //Conteo para conectar MQTT

//Definición de variables led WiFi
  const int Button_D0 = 16; // D0 pertenece al 16
  const int Led_D2 = 4;     // D2 pertenece al 4

// MQTT Broker
  const char *mqtt_broker = "broker.emqx.io"; // broker address
  const char *topic = "Electric_Meter/All"; // define topic 
  const char *mqtt_username = "UCSC"; // username for authentication
  const char *mqtt_password = "MSE_2023"; // password for authentication
  const int mqtt_port = 1883; // port of MQTT over TCP

//Variables
  String Voltage="";
  String Voltage_1="";
  String Voltage_3="";
  String Voltage_THD="";
  String Current="";
  String Current_1="";
  String Current_3="";
  String Current_THD="";
  String Power_P="";
  String Power_Q="";
  String Power_S="";
  String FP="";
  String Frequency="";
  String cadena ="";
  int K = 1;
  char datos [100];

//Variables para la conexión y envio de mensajes MQTT
  WiFiClient espClient;
  PubSubClient client(espClient);

void callback(char *topic, byte *payload, unsigned int length) {}



//Funciones conexión WiFi .........................................................



void WiFi_Inicial_Conection(){
  int WiFiState = digitalRead(Button_D0);
  if(WiFiState == HIGH){
    delay(5000);
    int WiFiState = digitalRead(Button_D0);
    if(WiFiState == HIGH){WiFi_Conection();}
  }
  delay(10);
}

void WiFi_Conection(){
  delay(20);
  digitalWrite(Led_D2, HIGH);
  delay(20);
  WiFiManager wifiManager;                                  // Creamos una instancia de la clase WiFiManager
  wifiManager.autoConnect("Electric-Meter");                // Cremos AP y portal cautivo
  digitalWrite(Led_D2, LOW);
  delay(20);
}

void WiFi_Status(){
  delay(10);
  if (WiFi.status()!= WL_CONNECTED){
    digitalWrite(Led_D2, HIGH);
    delay(500);
    digitalWrite(Led_D2, LOW);
  }
  delay(10);
}



//Funciones conexión MQTT .........................................................



void EnvioDatos_MQTT(){
  if (WiFi.status()== WL_CONNECTED){
  delay(10);
  if (!client.connected()) {re_connect();}
    client.loop();
    sprintf(datos,"{\"Voltage\":%s,\"Voltage_1\":%s,\"Voltage_3\":%s,\"Voltage_THD\":%s,\"Current\":%s,\"Current_1\":%s,\"Current_3\":%s,\"Current_THD\":%s,\"Power_P\":%s,\"Power_Q\":%s,\"Power_S\":%s,\"FP\":%s,\"Frequency\":%s}", Voltage, Voltage_1, Voltage_3, Voltage_THD, Current, Current_1, Current_3, Current_THD, Power_P, Power_Q, Power_S, FP, Frequency);
    client.publish(topic,datos);
  }
}

void re_connect() {
  int M = 0;
  while (!client.connected() && M<=300) {
    if (client.connect("arduinoClient")) {client.subscribe(topic);} 
    else {delay(10);}
    M++;
  }
}



//Funciones secundarias .........................................................



void Iniciar_SD(){
  while(!SD.begin(SSpin) && N<=300){
    delay(10);
    N++;
  }
  if(SD.begin(SSpin)){
    archivo = SD.open("datos.txt", FILE_WRITE);    
    archivo.close();
  }
  N=0;  
}

void SerialEvent(){
   cadena = Serial.readStringUntil('\n');           //Lectura de caracteres
   cadena = cadena.substring(0, cadena.length()-1);
   AsignarValor();
   Guardar_SD();
   EnvioDatos_MQTT();
   reinicio();
}

void AsignarValor(){
  int n = cadena.length();
    for(int i=0; i<n; i++){
      if(cadena[i] == ';'){K++;}
      else{  
        if(K == 1){Voltage += cadena[i];}
        if(K == 2){Voltage_1 += cadena[i];}
        if(K == 3){Voltage_3 += cadena[i];}
        if(K == 4){Voltage_THD += cadena[i];}
        if(K == 5){Current += cadena[i];}
        if(K == 6){Current_1 += cadena[i];}
        if(K == 7){Current_3 += cadena[i];}
        if(K == 8){Current_THD += cadena[i];}
        if(K == 9){Power_P += cadena[i];}
        if(K == 10){Power_Q += cadena[i];}
        if(K == 11){Power_S += cadena[i];}
        if(K == 12){FP += cadena[i];}
        if(K == 13){Frequency += cadena[i];}
      }
    }
  delay(10);
}

void reinicio(){
  Voltage="";
  Voltage_1="";
  Voltage_3="";
  Voltage_THD="";
  Current="";
  Current_1="";
  Current_3="";
  Current_THD="";
  Power_P="";
  Power_Q="";
  Power_S="";
  FP="";
  Frequency="";
  cadena="";
  K=1;
  delay(100);
}

void Guardar_SD(){
  archivo = SD.open("datos.txt", FILE_WRITE);
  if (archivo) {
    archivo.print(Voltage);
    archivo.print(";");
    archivo.print(Voltage_1);
    archivo.print(";");
    archivo.print(Voltage_3);
    archivo.print(";");
    archivo.print(Voltage_THD);
    archivo.print(";");
    archivo.print(Current);
    archivo.print(";");
    archivo.print(Current_1);
    archivo.print(";");
    archivo.print(Current_3);
    archivo.print(";");
    archivo.print(Current_THD);
    archivo.print(";");
    archivo.print(Power_P);
    archivo.print(";");
    archivo.print(Power_Q);
    archivo.print(";");
    archivo.print(Power_S);
    archivo.print(";");
    archivo.print(FP);
    archivo.print(";");
    archivo.println(Frequency);
    archivo.close();
  }
  delay(10);
}



//Funciones secundarias .........................................................



void setup() {
  Serial.begin(115200);
  pinMode(Button_D0, INPUT);
  pinMode(Led_D2, OUTPUT);
  WiFi_Conection();
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  delay(10);
  Iniciar_SD();                   
  delay(10);
}

void loop() {
  if(Serial.available()>0){SerialEvent();}
  delay(10);
  WiFi_Inicial_Conection();
  WiFi_Status();
}
