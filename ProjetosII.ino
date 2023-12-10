// Definindo os pinos
const int trigger_mao = 33; //Trigger do sensor da mão
const int echo_mao = 35; //Echo do sensor da mão
const int trigger_bola  = 32; //Trigger do sensor da bola
const int echo_bola     = 34; //Echo do sensor da bola


// Variáveis
unsigned long t_mao;
double d_mao;
unsigned long t_bola;
double d_bola;
double ref = 50; //posição da mão
double pos_bola = 0; //posição da bola no tubo

//Aspectos geométricos
const double lacuna_sup = 1; //Lacuna superior em cm
const double lacuna_inf = 10; //Lacuna inferior em cm
const double l_tubo = 40; //Comprimento do tubo em cm
#define REF_BASE 70;
double ref_anterior = REF_BASE; // Referência inicial desejada em % do tubo
const double dm_bola = 4; //Diâmetro da bola em cm
const int t_max = (int) (l_tubo * 3.5 / 0.034); //Tempo máximo de leitura do sensor

//Controle
double controleP = 0;
double controleI = 0;
double controleD = 0;
double controle = 0;
double erro = 0;
double erro_ant = 0;
double pos_init_bola = 0; //Posição inicial da bola
double init_controlI = 0; //Condição inicial para o integrador
double pos_ant_bola = l_tubo + lacuna_sup - dm_bola / 2; //Usado no controle derivativo
double pos_mao[5]; // armazena posições da mão para calcular a média

//Constantes do controle PID
const double t_amos = 0.0625; //tempo de amostragem

const double ta_I = 0.4 * 2.0;                           
const double ta_D = 0.02 * 0.77; 
const double Kpa = 0.7 * 3;                            
const double Kia = Kpa / ta_I;
const double Kda = Kpa * ta_D;

const double tb_I = 0.50 * 3.2;
const double tb_D = 0.012 * 2.2; 
const double Kpb = 0.35 * 2.0; 
const double Kib = Kpb / tb_I;
const double Kdb = Kpb * tb_D;

const double tc_I = 0.5 * 3.2; 
const double tc_D = 0.014 * 1.1;
const double Kpc = 0.45 * 1.65;
const double Kic = Kpc / tc_I;
const double Kdc = Kpc * tc_D;

//flags
int flag = 0; //Contador
bool flag_erro_mao = 0; //Erros na aquisição da referência da mão
bool flag_erro_bola = 0; //Erros na leitura da posição da bola
int flag_mao = 0; //Usado para o cálculo da média das posições da mão
int flag_inst=0; //Instabilidade
char area = 'a'; //areas 'a', 'b' ou 'c' - Divide o tubo em 3 regiões
//Limites das regiões
const int limiteAB = 18; //Limite entre as regiões A e B dada em % do comprimento do tubo
const int limiteBC = 64; //Limite entre as regiões B e C dada em % do comprimento do tubo

void setup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(trigger_bola, OUTPUT);
  pinMode(echo_bola, INPUT);
  pinMode(trigger_mao, OUTPUT);
  pinMode(echo_mao, INPUT);
  pinMode(pino_motorA, OUTPUT);
  pinMode(pino_motorB, OUTPUT);
  Serial.begin(9600); // Starts the serial communication
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

}



void loop() {

  if (flag_inst == 1) { //Reinício em caso de instabilidade

    delay(8);             
    flag_inst=0;
    init_controlI = 0;
    ref_anterior = REF_BASE;
    pos_init_bola = 0;
    flag = 0;
    flag_mao = 0;
    area = 'a';
  }

  if (flag_inst == 0)
  {


    
    //____________Mão___________________
    digitalWrite(trigger_mao, LOW);
    delayMicroseconds(2);
    digitalWrite(trigger_mao, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger_mao, LOW);
    t_mao = pulseIn(echo_mao, HIGH, t_max);// Lê o pino echo do sensor que retorna o tempo em microsegundos
    // Calculando a distancia
    d_mao = t_mao * 0.034 / 2;
    if (d_mao == 0) //Erro na leitura
    {
      flag_erro_mao = true;
      t_mao = t_max;
    }
    digitalWrite(trigger_bola, LOW); 
    delay((int)(30 - t_mao * 0.001));
    pos_mao[flag_mao] = d_mao - lacuna_inf; //Converte o valor no sistema de referência
  


    //_____________Bola_________________
    digitalWrite(trigger_bola, LOW);
    delayMicroseconds(2);
    digitalWrite(trigger_bola, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger_bola, LOW);
    t_bola = pulseIn(echo_bola, HIGH, t_max);// Lê o pino echo do sensor que retorna o tempo em microsegundos
    // Calculo da distancia
    d_bola = t_bola * 0.034 / 2;
    
    if (d_bola == 0) //Erro na leitura
    {
      flag_erro_bola = true;
      t_bola = t_max;
    }
    else flag_erro_bola = false;
    pos_bola = (l_tubo + lacuna_sup) - (d_bola + dm_bola / 2); // Converte o valor no sistema de referência
    delay((int)(25 - t_bola * 0.001));
    
    


    //Converte os valores em % do comprimento do tubo
    pos_mao[flag_mao] = pos_mao[flag_mao] * (100 / l_tubo);
    pos_bola = pos_bola * (100 / l_tubo);
    
    flag_mao = flag_mao + 1;
    if (flag_mao == 3) //Calcula a media de 3 valores de referência
    {
      ref = (pos_mao[0] + pos_mao[1] + pos_mao[2]) / 3;
      if ((ref < 21) || (ref > 77) || flag_erro_mao) ref = ref_anterior;
      flag_erro_mao = false;
      ref_anterior = ref;
      flag_mao = 0; //Reinicia o contador
    }
    
    if ((pos_bola < 0) || (pos_bola > 100) || flag_erro_bola) pos_bola = pos_ant_bola;

    
    
    //Solução para transição entre regiões do tubo
    if (pos_bola > limiteAB && pos_bola <= limiteBC && pos_init_bola <= limiteAB) //transição da area A para B
    {
      init_controlI = controle - Kpb * (ref - pos_bola) - Kib * (ref - pos_bola) * t_amos - Kdb * (-pos_bola + pos_ant_bola) / t_amos; 
      area = 'b';
    }
    else if (pos_bola > limiteBC && pos_init_bola > limiteAB && pos_init_bola <= limiteBC) //transição da area B para C
    {
      init_controlI = controle - Kpc * (ref - pos_bola) - Kic * (ref - pos_bola) * t_amos - Kdc * (-pos_bola + pos_ant_bola) / t_amos; 
      area = 'c';
    }
    else if (pos_bola > limiteAB && pos_bola <= limiteBC && pos_init_bola > limiteBC) //transição da area C para B
    {
      init_controlI = controle - Kpb * (ref - pos_bola) - Kib * (ref - pos_bola) * t_amos - Kdb * (-pos_bola + pos_ant_bola) / t_amos; 
      area = 'b';
    }
    else if (pos_bola <= limiteAB && pos_init_bola > limiteAB && pos_init_bola <= limiteBC) //transição da area B para A
    {
      init_controlI = controle - Kpa * (ref - pos_bola) - Kia * (ref - pos_bola) * t_amos - Kda * (-pos_bola + pos_ant_bola) / t_amos; 
      area = 'a';
    }
    else if (pos_bola > limiteBC && pos_init_bola <= limiteAB) //transição da area A para C
    {
      init_controlI = controle - Kpc*(ref-pos_bola) - Kic*(ref-pos_bola)*t_amos - Kdc*(-pos_bola+pos_ant_bola)/t_amos; 
      area = 'c';
    }

    else if (pos_bola <= limiteAB && pos_init_bola > limiteBC) //transição da area C para A
    {
      init_controlI = controle - Kpa*(ref-pos_bola) - Kia*(ref-pos_bola)*t_amos - Kda*(-pos_bola+pos_ant_bola)/t_amos; 
      area = 'a';
    }

    PID(area); //Calcula os parâmetros de controle
    if (pos_bola>85) //Posição instável que o sensor tem problemas na leitura
    {
      motor(pino_motorA,0); //Desliga o motor
      motor(pino_motorB,0); //Desliga o motor
      flag_inst=1;
    }
    else
    { 
      motor(pino_motorA, controle); //Controla a alimentação do motor
      motor(pino_motorB, controle); //Controla a alimentação do motor
      pos_init_bola = pos_bola;
      flag = flag + 1;
    }

    
  }
  Serial.print(pos_bola);
  Serial.print(" - ");
  Serial.println(ref);
  if (flag_inst == 1)
  {
    ref_anterior = REF_BASE;
    flag = 0;
  }
  delay(5);

}

//**************************************************************************************************************************************************


void motor (int pino_motor , double valor) {
  //Aplica o valor pwm desejado no motor
  //Recebe como entrada o pino do motor e o valor em %.

 // alterando map 0-100

  double pwm = (int)((valor / 100) * 255); //Converte para um intervalo de 0-255
  analogWrite(pino_motor, pwm);
}



void PID (char area) {
  //Calcula os valores para o controle PID

  double Kp, Ki, Kd;
  if (area == 'a') //Area a
  {
    Kp = Kpa;
    Ki = Kia;
    Kd = Kda;
  }
  else if (area == 'b') //Area b
  {
    Kp = Kpb;
    Ki = Kib;
    Kd = Kdb;
  }
  else  if (area == 'c') //Area c
  {
    Kp = Kpc;
    Ki = Kic;
    Kd = Kdc;
  }

  erro = ref - pos_bola; //Calculo do erro
  controleP = Kp * erro; //Controle proporcional
  controleI = init_controlI + Ki * erro * t_amos; //Controle integral
  controleD = Kd * (-pos_bola + pos_ant_bola) / t_amos; //Controle derivativo

  //Controle total
  controle = controleP + controleI + controleD;
  if (controle > 100) {//Saturação
    controle = 100;
    controleI = init_controlI;
  }
  else if (controle < 0) {//Saturação
    controle = 0;
    controleI = init_controlI;
  }
  init_controlI = controleI;
  erro_ant = erro;
}
