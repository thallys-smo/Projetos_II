float d_bola;         // variável de processo
float k = 5;          // Ganho de Referência
float h = 0.1;        // Tempo de Amostragem 
float ti = 50;        // Tempo de Integração

/* Para a bola de Isopor:
  - Kp = 4, Ki = 0.7 e Kd = 2
  Para a bola de Ping-pong:
  - Kp = 6.5, Ki = 1.8 e Kd = 4
  Jeito antigo de definições das constantes:
  - Kp = k*(1+h/(2*ti)), Ki = k/ti e Kd =  k/20    */

float kp = 6.5;       // Ganho Proporcional
float ki = 2.4;       // Ganho Integral 
float kd = 4.7;       // Ganho Diferencial
float p = 0;          // Variável Proporcional
float i = 0;          // Variável Integrativa
float d = 0;          // Variável Deferenciativa
float e = 0;          // Erro
float e_anterior = 0; // Erro Anterior

// Definindo os parametros do PWM do Motor brushless

const int freq = 50;         // Frequência do PWM de 50 Hz
const int canal_A = 0;       // Usando o canal 0 da Esp32
const int resolucao = 12;    // Resolução de 12 bits, 2 elevado a 12, 4096 posições 
const int pin_Atuacao_A = 5; // Pino input ESC
int valor_pwm = 0;           // Input do AD
int ciclo_A = 0;             // Valor do cilo do PWM

// Definindo os pinos dos sensores de distancia
//const int trigger_mao = 33; //Trigger do sensor da mão
//const int echo_mao = 35;    //Echo do sensor da mão
const int trigger_bola = 32;  //Trigger do sensor da bola
const int echo_bola = 34;     //Echo do sensor da bola

// Tempo da onda do Ultrassonico
const double l_tubo = 40;                       // Comprimento do tubo em cm
const int t_max = (int) (l_tubo * 3.5 / 0.034); // Tempo Máximo de leitura do sensor

void setup(){

   // Inicializando a comunicação serial
  Serial.begin(9600); // Taxa de 9600

  // Inicializando os parametros do motor brushless
  pinMode(pin_Atuacao_A, OUTPUT);
  ledcSetup(canal_A, freq, resolucao);
  ledcAttachPin(pin_Atuacao_A, canal_A); 
  ledcWrite(canal_A, ciclo_A);

  // Calibrando o motor -> Parado durante 1s  
  delay(2000);
  Serial.println("Calibrando Motor");
  delay(2000);
  motor(0, 205, 300);
  delay(5000);
  Serial.println("Motor Calibrado -> Iniciando a operação");
  delay(5000);

  // Inicializando os pinos dos sensores de distancias ultrassonicos
  pinMode(trigger_bola, OUTPUT);
  pinMode(echo_bola, INPUT);
  //pinMode(trigger_mao, OUTPUT);
  //pinMode(echo_mao, INPUT); 

}

void loop(){  

  // setpoint  - valor entre 0 e 40 cm de tubo
  float setpoint = 25;  

  // Distancia da bola
  digitalWrite(trigger_bola, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger_bola, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger_bola, LOW);
  unsigned long t_bola = pulseIn(echo_bola, HIGH, t_max);// Lê o pino echo do sensor que retorna o tempo em microsegundos
  // Calculo da distancia da bola
  d_bola = t_bola * 0.034 / 2;

  // Erro
  e_anterior = e;
  e = d_bola - setpoint;
  
  // Definição do Ganho Proporcional
  p = kp*e; 
  // Definição do Ganho Integral
  i = i +(ki*h*e);
  // Definição do ganho Derivativo
  d = kd*(e-e_anterior);
  
  // Definição da Ação de controle
  float u = p + i + d;

  /* Para a bola de Isopor:
  - Min_Pwm = 245 e Max_Pwm = 270
  Para a bola de Ping-pong:
  - Min_Pwm = 280 e Max_Pwm = 320   */

  // Ação de Controle Aplicada
  u= motor(u, 287, 330); // 
  
  // Imprimindo as coisas 
  imprimindo(setpoint, l_tubo - d_bola, e, p, i, d, u);  
}

float PID(float e, float d_bola, float setpoint, float kp, float ki, float kd, float p, float i, float d){
  // Erro
  e_anterior = e;
  e = d_bola - setpoint;
  
  // Definição do Ganho Proporcional
  p = kp*e; 
  // Definição do Ganho Integral
  i = i +(ki*h*e);
  // Definição do ganho Derivativo
  d = kd*(e-e_anterior);
  
  // Definição da Ação de controle
  float u = p + i + d;

  return u;
}

float motor(float u, float min_pwm, float max_pwm){

  // Ação de Controle Aplicada
  u = constrain(map(u, 0, 100, min_pwm, max_pwm), min_pwm, max_pwm);    
  ledcWrite(canal_A, u);
  return u;
}
// 
void imprimindo(float setpoint, float d_bola, float e, float p, float i, float d, float u){

  //Serial.print("Setpoint: ");
  Serial.print(setpoint);
  Serial.print(" ");

  //Serial.print("Distância da bola: ");
  Serial.print(d_bola);
  Serial.print(" ");
  
  //Serial.print("Erro: ");
  Serial.print(e);
  Serial.print(" ");

  //Serial.print("Novo Kp: ");
  Serial.print(p);
  Serial.print(" ");

  //Serial.print("Novo Ki: ");
  Serial.print(i);
  Serial.print(" ");

  //Serial.print("Novo Kd: ");
  Serial.print(d);
  Serial.print(" ");

  //Serial.print("Ação de controle (PID): ");
  Serial.println(u);
  Serial.print(" ");

  delay(1000);

  Serial.print("\n");
  
}