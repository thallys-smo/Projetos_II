#include <NewPing.h>

#define TRIGGER_PIN_BOLINHA  12
#define ECHO_PIN_BOLINHA     14
#define TRIGGER_PIN_MAO      16
#define ECHO_PIN_MAO         17
#define ESC_PIN              13  // Pino de controle da ESC

NewPing sensorBolinha(TRIGGER_PIN_BOLINHA, ECHO_PIN_BOLINHA, 200);
NewPing sensorMao(TRIGGER_PIN_MAO, ECHO_PIN_MAO, 200);

int setpoint = 20;  // Altura desejada da bolinha

// Parâmetros PID
double kp = 2.0;  // Ganho proporcional
double ki = 0.1;  // Ganho integral
double kd = 0.5;  // Ganho derivativo

double integral = 0;
double lastError = 0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  int distanceBolinha = sensorBolinha.ping_cm();  // Mede a distância da bolinha ao sensor ultrassônico
  int distanceMao = sensorMao.ping_cm();  // Mede a distância da mão ao sensor ultrassônico

  if (distanceBolinha > 0 && distanceMao > 0) {
    int error = setpoint - distanceBolinha;  // Calcula o erro (diferença entre a altura desejada e a medida)

    // Calcula as componentes PID
    double proportional = kp * error;
    integral += ki * error;
    double derivative = kd * (error - lastError);

    // Calcula a saída do PID
    double output = proportional + integral + derivative;

    // Ajusta a velocidade do motor proporcionalmente à saída do PID
    int escValue = constrain(map(output, 0, 100, 205, 300), 205, 300);

    // Envie o sinal de controle para a ESC
    analogWrite(ESC_PIN, escValue);

    Serial.print("Distance Bolinha: ");
    Serial.print(distanceBolinha);
    Serial.print("cm, Distance Mao: ");
    Serial.print(distanceMao);
    Serial.print("cm, Error: ");
    Serial.print(error);
    Serial.print(", ESC Pulse Width: ");
    Serial.println(escValue);

    // Atualiza o valor do erro anterior para o próximo ciclo
    lastError = error;
  }

  delay(50);  // Aguarda um curto período para evitar leituras muito frequentes
}