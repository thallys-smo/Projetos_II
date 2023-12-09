#include <Arduino.h>

// Definindo os parametros do PWM

const int freq = 50;
const int canal_A = 0;
const int resolucao = 12;
 
const int pin_Atuacao_A = 14;
const int Leitura_A = 27;
 
int leitura = 0;
int ciclo_A = 0;

void motor(int leitura){ // Função que varia o pwm no motor

  ciclo_A = map(leitura, 0, 4095, 205, 410); 
  
  ledcWrite(canal_A, ciclo_A);
}

void setup()
{
  pinMode(pin_Atuacao_A, OUTPUT);
 
  ledcSetup(canal_A, freq, resolucao);
 
  ledcAttachPin(pin_Atuacao_A, canal_A);
 
  ledcWrite(canal_A, ciclo_A);
 
}

void loop() {
     
    leitura = analogRead(Leitura_A); // Leitura do potenciometro -> ideia é que isso recebe o sinal de controle do PID
    motor(leitura); // Função motor recebendo o sinal de leitura do pid
}

