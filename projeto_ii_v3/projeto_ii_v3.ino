float y;                   // variável de processo
float setpoint = 19;       // setpoint  - valor entre 0 e 40 cm de tubo
float k = 5;              // ganho de referência
float h = 0.1;             // tempo de amostragem 
float ti = 50;             // tempo de integração
float kp = 6.5; //4; //k*(1+h/(2*ti)); // ganho proporcional
float ki = 1.8;//0.7;  //k/ti;           //ganho integral 
float kd = 4;//2; // k/20;
float p = 0;
float i = 0;
float d = 0;
float e = 0;
float e_anterior = 0; 

// Definindo os parametros do PWM do Motor brushless

const int freq = 50;
const int canal_A = 0;
const int resolucao = 12; 
const int pin_Atuacao_A = 5; // pino input ESC
int valor_pwm = 0; // input do AD
int ciclo_A = 0; // valor do cilo do PWM


// Definindo os pinos dos sensores de distancia
//const int trigger_mao = 33; //Trigger do sensor da mão
//const int echo_mao = 35; //Echo do sensor da mão
const int trigger_bola = 32; //Trigger do sensor da bola
const int echo_bola = 34; //Echo do sensor da bola

const double l_tubo = 40; //Comprimento do tubo em cm
const int t_max = (int) (l_tubo * 3.5 / 0.034); //Tempo máximo de leitura do sensor

void setup(){

   // Inicializando a comunicação serial
  Serial.begin(9600); // Taxa de 9600

  // Inicializando os parametros do motor brushless
  pinMode(pin_Atuacao_A, OUTPUT);
  ledcSetup(canal_A, freq, resolucao);
  ledcAttachPin(pin_Atuacao_A, canal_A); 
  ledcWrite(canal_A, ciclo_A);

  // Calibrando o motor -> Parado durante 1s  
  Serial.println("Calibrando");
  delay(2000);
  float u = map(0, 0, 100, 205, 300);   
  ledcWrite(canal_A, u);
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
  // Distancia da bola
  digitalWrite(trigger_bola, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger_bola, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger_bola, LOW);
  unsigned long t_bola = pulseIn(echo_bola, HIGH, t_max);// Lê o pino echo do sensor que retorna o tempo em microsegundos
  // Calculo da distancia da bola
  y = t_bola * 0.034 / 2;

  // Erro
  e_anterior = e;
  e = y - setpoint;
  
  // Definição do Ganho Proporcional
  p = kp*e; 
  // Definição do Ganho Integral
  i = i +(ki*h*e);
  // Definição do ganho Derivativo
  d = kd*(e-e_anterior);
  
  // Definição da Ação de controle
  float u = p + i + d;

  // Ação de Controle Aplicada
  u = constrain(map(u, 0, 100, 280, 320), 280, 320);    //245, 270 isopor
  ledcWrite(canal_A, u);

  // Imprimindo as coisas
  imprimindo(y, e, p, i, d, u);

  
}

void imprimindo(float y, float e, float p, float i, float d, float u){

  Serial.print("Distância da bola: ");
  Serial.println(y);

  Serial.print("Erro: ");
  Serial.println(e);

  Serial.print("Novo Kp: ");
  Serial.println(p);

  Serial.print("Novo Ki: ");
  Serial.println(i);

  Serial.print("Novo Kd: ");
  Serial.println(d);

  Serial.print("Ação de controle (PID): ");
  Serial.println(u);

  delay(1000);

  Serial.print("\n");

}

