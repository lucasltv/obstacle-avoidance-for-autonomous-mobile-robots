 /*
 * CONTROLES NO TECLADO (VIA BT):
 * W = INCREMENTA SETPOINT VELOCIDADE LINEAR
 * S = DECREMENTA SETPOINT VELOCIDADE LINEAR
 * BARRA DE ESPAÇO = ZERA SETPOINT VELOCIDADE LINEAR
 * D = INCREMENTA SETPOINT VELOCIDADE ANGULAR (SENTIDO 1)
 * A = INCREMENTA SETPOINT VELOCIDADE ANGULAR (SENTIDO 2)
 * E = VELOCIDADE ANGULAR MÁXIMA (SENTIDO 1)
 * Q = VELOCIDADE ANGULAR MÁXIMA (SENTIDO 1)
 * R = ZERA VELOCIDADE ANGULAR
 * X = +SETPOINT Xd (COORDENADA X DA POSICAO FINAL)
 * x = -SETPOINT Xd (COORDENADA X DA POSICAO FINAL)
 * Y = +SETPOINT Yd (COORDENADA Y DA POSICAO FINAL)
 * y = -SETPOINT Yd (COORDENADA Y DA POSICAO FINAL)
 * v = modo automático
 * V = modo manual
*/

int xd = 300; //coordenada x (em cm)
int yd = 300; //coordenada y (em cm)

//Controlador PID
const int KiLinear = 15;
const int KiAngular = 1;
const int KpLinear = 1;

//Variáveis controlador de posição
const int kw = 850; //ganho angular
const float kvl = 1; //ganho linear

#define PWM_MAX 120
#define PWM_MIN 50

int Ponderacao[7];
int anguloponderado;
float ErroPosicaoX;
float ErroPosicaoY;
float mod_rho;
volatile float odometro = 0;
volatile float odometro_esq = 0; //odometro esq
volatile float odometro_dir = 0; //odometro dir
volatile float delta_odo, odometro_anterior = 0;
long int VelocidadeAngularanterior;
long int variacaoX,variacaoY;
float teta;
float Alfa;
float Rho;

float angulodesvio;
float x = 0;       //posicao atual x
float y = 0;       //posicao atual y
//Coordenadas de destino:
#define xd_max 300 //limite para xd (em cm)
#define yd_max 300 //limite para yd (em cm)
#define passo_destino 1 //passo que incrementa o setpoint a cada pressionamento da tecla


//Portas:
#define encoder_dir 3 //Pino encoder direito
#define encoder_esq 2 //Pino encoder esquerdo
#define pwm_dir1 10 // pino PWM1 roda direita
#define pwm_dir2 9 // pino PWM2 roda direita
#define pwm_esq1 7 // pino PWM1 roda esquerda
#define pwm_esq2 6 // pino PWM2 roda esquerda
#define pwmservo 11 //pino de PWM do TIMER 1 (servo ultrassom)
#define echo 21              //Pino Echo
#define trigger 20           //Pino de trigger do ultrassonico


//Constantes:
const float distancia_entre_rodas = 14.4; //distancia entre rodas = 14,4
#define passo_vl 1 //passo que incrementa ou decrementa setpoint da velocidade linear
#define limite_vl 100   //limite de velocidade linear
#define passo_w 1 //passo que incrementa ou decrementa setpoint da velocidade linear
//#define limite_w 100   //limite de velocidade angular
#define vl_max 100 //velocidade linear maxima
#define vl_max 100 //velocidade linear maxima
#define w_max 500 //velocidade linear maxima
#define parado 0  //codigo para flag do sentido atual
#define frente 1  //codigo para flag do sentido atual
#define re 2      //codigo para flag do sentido atualr
#define manual 0
#define automatico 1
#define dist_max 30 //distancia maxima para definir obstaculo ou nao


//Nomenclatura das teclas de controle
#define tecla_espaco 32 //entrada teclado barra de espaço
#define tecla_a 97 //entrada teclado
#define tecla_b 98 //entrada teclado
#define tecla_d 100 //entrada teclado
#define tecla_e 101 //entrada teclado
#define tecla_w 119 //entrada teclado
#define tecla_s 115 //entrada teclado
#define tecla_q 113 //entrada teclado
#define tecla_r 114 //entrada teclado
#define tecla_v 118 //entrada teclado
#define tecla_V 86 //entrada teclado
#define tecla_X 88 //entrada teclado (X) maiusculo
#define tecla_x 120 //entrada teclado (x) minusculo
#define tecla_Y 89 //entrada teclado (Y) maiusculo
#define tecla_y 121 //entrada teclado (y) minusculo

//Variáveis medição de distancia
signed int estado_ultrassom = 0; //Estado inicial da maquina de estados do ultrassom (graus para angulação do ultrassom)
signed int angulo_sensor = 0; //Direção que aponta o sensor ultrassom
//criar vetor distancia

//Variáveis
byte serial_input = 0; //registrador para armazenar o que vem na serial
byte sentido = parado; //flag do sentido: 0=parado, 1=frente , 2=tras

//Variáveis controlador baixo nível (velocidade)
float sp_vl,sp_w; //setpoints vl e w
float IntegralDireita = 0;
float IntegralEsquerda = 0;
float IntegralW = 0;
float ErroDireita;
float ErroDireitaAnterior = 0;
int DireitaPWM = 0;
float ErroEsquerda;
float ErroEsquerdaAnterior = 0;
int EsquerdaPWM = 0;
float ErroW = 0; //erro velocidade angular
float AnteriorErroW = 0;
float AuxIntegralDireita;
float AuxIntegralEsquerda;

//Variáveis determinação da velocidade
float w_anterior,w,vl = 0;   //velocidade linear (cm/s) e angular (VP)
volatile float ve = 0;   //velocidade roda esquerda (cm/s)
volatile float vd = 0;   //velocidade roda direita   (cm/s)
//volatile float  odometro = 0; //acumulador da distancia percorrida
//volatile int odometro_esq = 0; //odometro esq
//volatile int odometro_dir = 0; //odometro dir
volatile int periodo_esq, last_periodo_esq = 0;
volatile int  periodo_dir, last_periodo_dir = 0;

//Variáveis média móvel (filtro de realimentação da velocidade)
//const int n = 1; //Atenção: Ajustes para baixa rotação!!
//volatile int media_ve = 0;
//volatile int media_vd = 0;
//volatile long total_esq = 0;
//volatile long total_dir = 0;
//int readings_esq[n];
//int readings_dir[n];

//Variáveis de controle do PWM (SERVO MOTOR ultrassom)
//byte ajuste_fino = centro;
//byte ajuste_grosso = 5; // posicao do servo
//volatile byte passo_angulo = 5; //passo para acres/decrec MANUAL do angulo da direcao via teclado (mínimo = 1). Quanto menor, mais preciso e menos rápido. Quanto maior, menos preciso e mais rápido.

//Variáveis do controle do ultrassonico:
#define centro 255 //ajuste do centro da direção (0º)
byte estado_sonar = 0; //maq estados sonar (inicia no zero)
int last_estado = 1;
int prox_estado = 2;
volatile float pulso = 0;             //Tempo do echo em nivel alto (proporcional à distancia)
volatile float dist_sensor = 0;       //Distancia sensor
int distancias[7] = {0,0,0,0,0,0,0} ; //vetor que armazena as distancias dos respectivos angulos
byte timer_scan = 0; //contador do scan
byte tempo_estab = 10; //tempo de estabilização do servo motor
int modo = manual;
int timeout_echo = 0;
int novo_echo = 0;
int periodo_scan = 40; //periodo de mudança do angulo de leitura


//=======fim declaração variáveis=======INICIO FUNCOES

//Função que imprime variáveis na serial (debug)
void imprime_variaveis()
{
    Serial2.println(" ");
    Serial2.print("Rho: ");
    Serial2.println(Rho*180/3.1415);
    Serial2.print("Mod de Rho: ");
    Serial2.println(mod_rho);
    Serial2.print("Teta:");
    Serial2.println(teta*180/3.1415);
    Serial2.print("Alfa: ");
    Serial2.println(Alfa*180/3.1415);
    Serial2.print("Setpoint W: ");
    Serial2.println(sp_w);
    Serial2.print("Setpoint VL: ");
    Serial2.println(sp_vl);
    Serial2.print("X atual: ");
    Serial2.println(x);
    Serial2.print("Y atual: ");
    Serial2.println(y);
}

void ControladorDePosicao(void)
{
   odometro += ((odometro_dir + odometro_esq)/2 * 1.04); //atualiza odometro
   odometro_esq = 0;
   odometro_dir = 0;

   delta_odo = odometro - odometro_anterior;
   odometro_anterior = odometro;
   x += delta_odo*cos(teta);
   y += delta_odo*sin(teta);

   ErroPosicaoX = xd - x;
   ErroPosicaoY = yd - y;
   Rho = atan(ErroPosicaoY/ErroPosicaoX);

   mod_rho = sqrt(pow(ErroPosicaoY, 2) + pow(ErroPosicaoX, 2));

   Alfa = Rho - teta;

   if(abs(xd-x)<1 && abs(yd-y)<1) //chegou perto, pára
   {
    sentido = parado;
    modo = manual;
   }

   //Alteração dos setpoints das velocidades:
   //sp_w = (Alfa + desvio*k); ///dado em mRad/s
   sp_w = Alfa*kw; ///dado em mRad/s
   sp_vl = mod_rho*cos(Alfa)*kvl;

//   for(int l=0 ; l<7 ; l++)
//   {
//    Ponderacao[l] = 30 - distancias[l];
//   }
//   anguloponderado = 2*Ponderacao[0] + 4*Ponderacao[1] + 9*Ponderacao[2] - 9*Ponderacao[4] - 4*Ponderacao[5] - 2*Ponderacao[6];
//   if (anguloponderado>=0)
//    anguloponderado = anguloponderado + 15*Ponderacao[3]; //Valor do Angulo ponderado vai de 0 a 900
//   else
//    anguloponderado = anguloponderado - 15*Ponderacao[3];
//
//   //definicao do angulo de desvio do obstaculo:
//   angulodesvio = anguloponderado/572,95; //Angulo maximo de 90= (Angulo ponderado/10)*pi/180. angulo em rad
}




//Função que pára carrinho
void freia(){
  sentido = parado;
  sp_w = 0; //zera sp w
  sp_vl = 0; //zera sp vl
  Serial2.println("Parado!");
  angulo_sensor = 0;
  delay(300);

  //debug
//  odometro_esq=0;
//  odometro_dir=0;

}


//Rotina de interrupcao externa (encoder roda esquerda)
void ISR_enc_esquerdo()
{ //Rotina executada a cada transição positiva no pino de entrada 3 (encoder esquerdo)
  TCNT5 = 35100;                //Reseta watchdog timer5 (carro parado)
  //TCNT5 = 62480; //50ms - Reseta watchdog timer5 
  periodo_esq = millis() - last_periodo_esq; //Determina periodo da interrupção
  //if(periodo_esq < 0) ve=vd;    //Verifica se valor deu negativo (estouro do timer). RARO
  //else
  ve = 1037 / periodo_esq; //Calculo da velocidade roda esquerda.
  last_periodo_esq = millis();  //Armazena valor do instante para ser usado na próxima interrupção
//  tira_media_ve();              //Calcula média móvel
  odometro_esq++;               //Atualiza contador do odometro da roda esquerda
//  Serial2.println(odometro_esq);  //debug:
//OBS.: diametro da roda = 6,6cm (marcacoes no encoder = 20) - cada buraco no encoder = 1,036cm que a roda percorre
}

//void tira_media_ve() //Média móvel velocidade esquerda
//{
//  total_esq -= readings_esq[0]; //Subtrai do total o valor mais antigo antes do mesmo ser descartado.
//  //Libera espaco para o novo dado:
//  for (int i=0; i < (n-1) ; i++) readings_esq[i] = readings_esq[i+1];
//  readings_esq[n-1] = ve; //Salva valor mais novo da velocidade instantanea
//  total_esq += readings_esq[n-1]; //Acumula valor no total
//  media_ve = total_esq / n; //Determina a média
//}

//Interrupcao externa encoder direito
void ISR_enc_direito()
{
  //TCNT5 = 35100; //5000ms - Reseta watchdog timer5 
  periodo_dir = millis() - last_periodo_dir;
  //if(periodo_dir < 0) vd=ve; //verifica se valor deu negativo (transicao do timer) - se ocorreu, usa a referencia da outra roda
  //else
  vd = 1037 / periodo_dir;
  last_periodo_dir = millis(); 
//  tira_media_vd(); //calcula média móvel
  odometro_dir++;
//  Serial2.println(odometro_dir);  //debug:
  //OBS.: diametro da roda = 6,6cm (marcacoes no encoder = 20) - cada buraco no encoder = 1,036cm que a roda percorre
}

//void tira_media_vd()
//{
//  //Média móvel velocidade esquerdo:
//  total_dir -= readings_dir[0]; //Deleta valor mais antigo
//  for (int j=0; j < (n-1) ; j++) readings_dir[j] = readings_dir[j+1]; //deslocamento do vetor e libera espaco para o novo
//  readings_dir[n-1] = vd; //Salva valor mais novo
//  total_dir += readings_dir[n-1]; //Acumula valor no total
//  media_vd = total_dir / n; //determina a média  
//}

//Rotina de interrupção para o echo ultrassom:
void ISR_echo()
{
  if (PIND & (1 << PIND0))  // D0 (pino 21) mudou de LOW para HIGH;
  {
    pulso = micros();
  }
  else
  { // D0 mudou de HIGH para LOW;
    /* Mede quanto tempo o pino de echo ficou no estado alto, ou seja,
       o tempo de propagação da onda. */
    pulso = micros() - pulso;
    dist_sensor = (pulso * 0.01715); //distancia medida pelo ultrassom (em cm)
    if(dist_sensor > dist_max) dist_sensor = 0;
    novo_echo = 1;
   
//    for(int i=0; i<=6; i++)
//    {
//      Serial2.print(distancias[i]);
//      if(i==6)
//      {
//        Serial2.println("");
//        break;
//      }
//      Serial2.print(" - ");
//      //delay(1000);
//    }
  }
 
}

//SETUP
void setup() {
  noInterrupts();           //Desabilita interrupcoes para configuracaos dos timers
 
  pinMode(pwm_dir1, OUTPUT);
  pinMode(pwm_dir2, OUTPUT);
  pinMode(pwm_esq1, OUTPUT);
  pinMode(pwm_esq2, OUTPUT);
  pinMode(pwmservo, OUTPUT);
  pinMode(encoder_dir, INPUT_PULLUP);
  pinMode(encoder_esq, INPUT_PULLUP);
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);
 
 
  //Configura seriais
  Serial.begin(115200); //USB ARDUINO
  Serial2.begin(19200); //HC-06 bluetooth
  
  //Configuração do TIMER 1: PWM do SERVO (ultrassom)
  TIMSK1 |= (1 << TOIE1);   //Habilita interrupçoes por overflow no timer1 (caso queira usar um timer de 20ms)
  TCCR1A = 0b10101000; //Seta prescaler para timer1 (PWM do SERVO)
  TCCR1B = 0b00010010;
  //Configura frequencia to TIMER1 para 50Hz (Especificacao do servo motor)
  ICR1H = 0b01001100; //Input capture register
  ICR1L = 0b00100000;
  //Ajuste do dutycicle do SERVO (OUTPUT COMPARE):
  OCR1AH = 5;   //ajuste grosso do duty cicle (servo) - posicao inicial das rodas (90 graus = 1,5ms)
  OCR1AL = centro; //ajuste fino do duty, iniciando no centro (baseado no sistema de direcao do carrinho)
//  /*OBS: Referências do SERVO e não do sistema de direção!
//    1,0ms (-90º):   OCR1AH=3 e OCR1AL=240
//    1,5ms (0º):     OCR1AH=5 e OCR1AL=220
//    2,0ms (90º):    OCR1AH=7 e OCR1AL=220
//  */

//Configuração do TIMER 2: CLOCK do PWM do motor DC direito (TIMER 2)
  TCCR2B = TCCR0B & 0b11111000 | 0x01; /*Configura frequencia do pwm para 31,250KHz. Referência: http://playground.arduino.cc/Main/TimerPWMCheatsheet timer 2 controla pin 10 e 9)*/
 
  //Configuração do TIMER3 (controlador PI de velocidade) (ajustar para 5ms-patrick)
  TCCR3A = 0;         //Limpa registradores do timer3
  TCCR3B = 0;
  TCCR3B |= (0 << CS30);   
  TCCR3B |= (0 << CS31); //prescaler = 1024
  TCCR3B |= (1 << CS32);    //Equivale o estouro a cada 32,768ms (65536 * 1/(16Mhz/prescaler))
  TIMSK3 |= (1 << TOIE3);   //habilita interrupcao

//Configuração do TIMER 4: PWM do motor DC esquerdo (TIMER 4)
  TCCR4B = TCCR0B & 0b11111000 | 0x01; /*Configura frequencia do pwm para 31,250KHz. Referência: http://playground.arduino.cc/Main/TimerPWMCheatsheet timer 2 controla pin 10 e 9)*/
 
  //Configuração do TIMER 5: Watchdog timer do estouro de velocidade (Checa se o carro esta parado!)
  TCCR5A = 0;
  TCCR5B = 0;
  TCCR5B |= (0 << CS30);   
  TCCR5B |= (0 << CS31); //prescaler = 256 (pagina 157 do datasheet atmel 2560)
  TCCR5B |= (1 << CS32);
  TIMSK5 |= (1 << TOIE5);   //Habilita interrupçoes por overflow no timer5
  //Referencias para timers: http://playground.arduino.cc/Code/Timer1
  //http://www.embarcados.com.br/timers-do-atmega328-no-arduino/
 
  ///Habilita interrupção externa no pino do sensor ultrassonico:
  attachInterrupt(digitalPinToInterrupt(echo), ISR_echo, CHANGE);

  //Habilita interrupção extena para mudança encoders 
  attachInterrupt(digitalPinToInterrupt(encoder_dir), ISR_enc_direito, RISING);  
  attachInterrupt(digitalPinToInterrupt(encoder_esq), ISR_enc_esquerdo, RISING);
 
 
  interrupts();        //Habilita todas as interrupções após o SETUP dos timers

  analogWrite(pwm_dir1, 0); //inicia parado
  analogWrite(pwm_dir2, 0);
  analogWrite(pwm_esq1, 0);
  analogWrite(pwm_esq2, 0);
 
  Serial2.println("Reset!");
 
  
}//====================Fim do SETUP==================
//======================ROTINAS DE INTERRUPÇÕES============


//Interrupção do timer1 (20ms) - rotina do trigger ultrassom
ISR(TIMER1_OVF_vect)
{
  //Testando, estava no timer4
//  digitalWrite(trigger, LOW);  //Inicia pino em nível baixo
//  delayMicroseconds(2);        //Aguarda 2 us
//  digitalWrite(trigger, HIGH); //Seta o pino trigger (22) em nível alto
//  delayMicroseconds(12);       //Mantém em nível ALTO por um pouco mais de 10us (exigência do sensor)
//  digitalWrite(trigger, LOW);  //Encerra solicitação voltando o pino para nível BAIXO.
 
  timer_scan++;
  timeout_echo++;

 
 
 
} //fim ISR timer 1

//Rotina de interrupção do CONTROLADOR PID: Estouro do TIMER3
ISR(TIMER3_OVF_vect) //Executa a cada 50ms.
{

  vl = (vd + ve)/2; //calcula velocidade linear
  w = (vd - ve) / distancia_entre_rodas;    //calcula velocidade angular

 
  teta += (1.1)*(w + w_anterior)/(400); //Integral angular (taxa de 5ms) (1,1 para calibração)
  w_anterior = w;
 
  if(modo == automatico) ControladorDePosicao();
  //ControladorDePosicao();

  ErroW = (sp_w - w);
  IntegralW = (AnteriorErroW + ErroW)*KiAngular/2; //Integral angular (taxa de 5ms)
  AnteriorErroW = ErroW;

  //calculo controles individuais das rodas:
  ErroDireita = (abs(sp_vl) - vd) + IntegralW; //Já soma ao erro da roda direita, a integral da veloc angular
  AuxIntegralDireita = IntegralDireita + (ErroDireita + ErroDireitaAnterior) / (400); //taxa de 5ms
  //AuxIntegralDireita = IntegralDireita + (ErroDireita + ErroDireitaAnterior) / (40); //taxa de 50ms
  ErroDireitaAnterior = ErroDireita;
 
  DireitaPWM = (ErroDireita*KpLinear) + (AuxIntegralDireita*KiLinear);
 
  if((DireitaPWM < PWM_MAX) && (DireitaPWM > PWM_MIN)) //pergunta se está saturado
  {
    IntegralDireita = AuxIntegralDireita; //nao ta saturando, entao assume o valor da integral
  }
  else  //ja sabe q esta saturado
  {
    if ((DireitaPWM >= PWM_MAX)&&((AuxIntegralDireita - IntegralDireita) < 0)) //saturado em cima com tendendia para baixo
      IntegralDireita = AuxIntegralDireita;  
    if (DireitaPWM >= PWM_MAX)
      DireitaPWM = PWM_MAX;
   
    if ((DireitaPWM <= PWM_MIN)&&((AuxIntegralDireita - IntegralDireita) > 0)) //saturado em cima com tendendia para baixo
      IntegralDireita = AuxIntegralDireita;  
        if (DireitaPWM <= PWM_MIN)
      DireitaPWM = PWM_MIN;      
  }
   
    ErroEsquerda = abs(sp_vl) - ve - IntegralW; //Já soma ao erro da roda esquerda, a integral da veloc angular
    //AuxIntegralEsquerda = IntegralEsquerda + (ErroEsquerda + ErroEsquerdaAnterior) / 2;
    AuxIntegralEsquerda = IntegralEsquerda + (ErroEsquerda + ErroEsquerdaAnterior) / (400); //taxa de 5ms
    //AuxIntegralEsquerda = IntegralEsquerda + (ErroEsquerda + ErroEsquerdaAnterior) / (40); //taxa de 50ms
    ErroEsquerdaAnterior = ErroEsquerda;
 

    EsquerdaPWM = (ErroEsquerda*KpLinear) + (AuxIntegralEsquerda*KiLinear); //deslocamento para 1000
 
  if((EsquerdaPWM < PWM_MAX) && (EsquerdaPWM > PWM_MIN)){ //pergunta se está saturado
    IntegralEsquerda = AuxIntegralEsquerda; //nao ta saturando, entao assume o valor da integral
  }
    else { //ja sabe q esta saturado
    if ((EsquerdaPWM >= PWM_MAX)&&((AuxIntegralEsquerda - IntegralEsquerda) < 0)) //saturado em cima com tendendia para baixo
      IntegralEsquerda = AuxIntegralEsquerda;  
        if (EsquerdaPWM >= PWM_MAX)
      EsquerdaPWM = PWM_MAX;
   
    if ((EsquerdaPWM <= PWM_MIN)&&((AuxIntegralEsquerda - IntegralEsquerda) > 0)) //saturado embaixo com tendendia para cima
      IntegralEsquerda = AuxIntegralEsquerda;  
        if (EsquerdaPWM <= PWM_MIN)
      EsquerdaPWM = PWM_MIN;       
  }
     
    //Executa saida pwm
    switch (sentido)
      {
        case frente:
          analogWrite(pwm_dir1, DireitaPWM);
          analogWrite(pwm_dir2, 0);
          analogWrite(pwm_esq1, EsquerdaPWM);
          analogWrite(pwm_esq2, 0);
        break;     
       
        case re:
          analogWrite(pwm_dir1, 0);
          analogWrite(pwm_dir2, DireitaPWM);
          analogWrite(pwm_esq1, 0);
          analogWrite(pwm_esq2, EsquerdaPWM);
        break;
 
        default:
          analogWrite(pwm_dir1, 0);
          analogWrite(pwm_dir2, 0);
          analogWrite(pwm_esq1, 0);
          analogWrite(pwm_esq2, 0);
      }


    TCNT3 = 65230; //5ms
    Serial.println(millis());
    //TCNT3 = 49972; //~250ms
    //TCNT3 = 62485; //50ms
    //TCNT3 = 56381; // 150ms de estouro -
    //TCNT3 = 53100; //~200ms
    //TCNT3 = 59275; //100ms
    //TCNT3 = 41122; //400ms
    //TCNT3 = 35018; //500ms
    //58000 ~120ms
    //41122 ~400ms
    //44174 ~350ms
    //TCNT3 = 49972; //~250ms (melhor ajuste!)


    //debug
  //Serial.print(sp_vl);
  //Serial.print(" , ");
//  Serial.print(ve);
//  Serial.print(" , ");
  //Serial.println(vl);
} //FIM ROTINA INTERRUPCAO TIMER3 (controlador PID);

/* ISR(TIMER4_OVF_vect) //Interrupção por estouro do TIMER 4 (Trigger dos ultrassonicos e pisca-pisca)
{ //interrupcao a cada 262ms

   
 
}//fim isr timer 4 */

     
ISR(TIMER5_OVF_vect)
{ //Watchdog timer para estouro da velocidade (só acontece quando o carro está parado)
    vl=0;         //Zera valor da velocidade linear (realimentação para o controlador PID)
    //media_ve=0;   //Zera média velocidade roda esquerda
    //media_vd=0;   //Zera média velocidade roda direita
    //total_esq=0;  //Zera total da média esq
    //total_dir=0;  //Zera total da média dir
    ve=0;         //Zera velocidade instantanea esq
    vd=0;         //Zera velocidade instantanea dir
    //memset(readings_esq, 0, sizeof(readings_esq));    //Zera vetor das médias esq
    //memset(readings_dir, 0, sizeof(readings_dir));    //Zera vetor das médias dir
}



void trigger_sonar()
{
  digitalWrite(trigger, LOW);  //Inicia pino em nível baixo
  delayMicroseconds(2);        //Aguarda 2 us
  digitalWrite(trigger, HIGH); //Seta o pino trigger (22) em nível alto
  delayMicroseconds(12);       //Mantém em nível ALTO por um pouco mais de 10us (exigência do sensor)
  digitalWrite(trigger, LOW);  //Encerra solicitação voltando o pino para nível BAIXO.
}

//Função que controla o sentido do carrinho pelo motor DC
void ajustesetpointVl(int entrada_teclado)
{
    switch (entrada_teclado) //Interpreta o que foi digitado no teclado
    {
      case tecla_w: //incrementa VL
        if(sp_vl < vl_max)
          {
            sp_vl += passo_vl;
          }
          else
          {
            sp_vl = vl_max; //limite de velocidade
          }
      break;
 
      case tecla_s: //decrementa VL
    if(sp_vl > -vl_max)
          {
            sp_vl -= passo_vl;
          }
          else
          {
            sp_vl -= vl_max;
          }
      break;
    }
    if(sp_vl > 0) sentido = frente;
    else if (sp_vl < 0) sentido = re;
    else freia();
   
    Serial2.print("SetPoint de VL: ");
    Serial2.println(sp_vl);   
} //Fim da função de controle de velocidade linear

//Ajuste do setpoint da posicao Xd (destino
void ajustesetpointXd(int dado_x)
{
  switch (dado_x)
    {
      case tecla_X: //X maiusculo
        if(xd < xd_max) //máximo do espaço
          {
            xd += passo_destino; //incrementa xd
          }
          else
          {
            xd = xd_max;
          }       
      break;
     
      case tecla_x: //x minusculo
         if(xd <= 0)
          {
            xd = 0; //limite inferior (ponto zero da abscissa)
          }
          else
          {
            xd -= passo_destino; //decrementa xd
          } 
      break;
    }
    Serial2.print("Xd: ");
    Serial2.println(xd);
}

void ajustesetpointYd(int dado_y) //Ajuste do setpoint da posicao Yd (destino
{
  switch (dado_y)
    {
      case tecla_Y: //Y maiusculo
        if(yd < yd_max) //máximo do espaço em Y
          {
            yd += passo_destino; //incrementa yd
          }
          else
          {
            yd = yd_max;
          }       
      break;
     
      case tecla_y: //y minusculo
         if(yd <= 0)
          {
            yd = 0; //limite inferior (ponto zero da ordenada)
          }
          else
          {
            yd -= passo_destino; //decrementa yd
          } 
      break;
    }
    Serial2.print("Yd: ");
    Serial2.println(yd);
}


void ajustesetpointW(int dado_w) //função que ajusta sp velocidade angular (W)
{   
    switch (dado_w) //Interpreta o que foi digitado no teclado
    {
        case tecla_a: //virar a direita
                if(sp_w < w_max) //se módulo de sp_w esta dentro dos limites
                {
                    sp_w += passo_w; //incrementa sp velocidade angular
                }
                else
                {
                    sp_w = w_max; //limite de velocidade angular (positiva)
                }
        break;
       
        case tecla_d: //virar para esq
            if(sp_w > -w_max) //se módulo de sp_w esta dentro dos limites
                  {
                    sp_w -= passo_w; //decrementa sp velocidade angular
                  }
                  else
                  {
                    sp_w = -w_max; //limite de velocidade angular (negativa)
                  }
        break;      
       
        case tecla_q:
            sp_w = w_max; //velocidade angular máxima (positiva)
        break;
       
        case tecla_e:
            sp_w = -w_max; //velocidade angular máxima (negativa)
        break;     
       
        case tecla_r: //reset velocidade angular
            sp_w = 0;
      odometro = 0;
      teta = 0;
    break;
   
    }
    Serial2.print("Setpoint de W: ");
    Serial2.println(sp_w);
}

//Loop principal
void loop()
{
  //Leitura de dados da serial: 
  if (Serial.available()>0 || Serial2.available()>0) //Entrada vinda na serial
  {
    serial_input = Serial2.read();
    if(serial_input == tecla_w || serial_input == tecla_s) ajustesetpointVl(serial_input); //ajuste setpoint velocidade linear
    if(serial_input == tecla_X || serial_input == tecla_x) ajustesetpointXd(serial_input); //Ajuste do setpoint da posicao Xd (destino
    if(serial_input == tecla_Y || serial_input == tecla_y) ajustesetpointYd(serial_input); //Ajuste do setpoint da posicao Yd (destino)
    if (serial_input == tecla_espaco) freia(); //zera sp_vl e sp_w
    if(serial_input == tecla_d || serial_input == tecla_a || serial_input == tecla_r || serial_input == tecla_q || serial_input == tecla_e) ajustesetpointW(serial_input); //ajusta set point w
    if(serial_input == tecla_b) imprime_variaveis(); //debbug variáveis na serial
    if(serial_input == tecla_v)
    {
      modo = automatico;
      odometro=0;
      odometro_dir = 0;
      odometro_esq = 0;
      x=0;
      y=0;
      teta = 0;
      sentido = frente;
      Serial2.println("Modo automatico iniciado.");
    }
    if(serial_input == tecla_V)
    {
      modo = manual;
      Serial2.println("Modo manual");
      freia();
      sp_vl=0;
      sentido = parado;
    }
   
  }//FIM SERIAL AVAILABLE

 
  if(modo == automatico) //maquina de estados angulação do ultrassom (servo)
  //if(true) //debug
  {
    switch (estado_sonar)
    {
     
      case 0: //aguarda echo
          trigger_sonar();
          //Serial2.println(estado_sonar);
          //Serial2.println(novo_echo);
          if(timeout_echo > 5 && novo_echo == 0)
          {
            distancias[last_estado-1] = 0;
            estado_sonar = prox_estado;
          }
          else if(novo_echo == 1)
          {
            distancias[last_estado-1] = dist_sensor;
            estado_sonar = prox_estado;
          }
          timer_scan = 0;
      break;
     
      case 1: //90 graus
        OCR1AH=9; //90 graus
        OCR1AL=230; //90 graus   
        last_estado = 1;
        prox_estado = 2;
        if(timer_scan >= tempo_estab)
        {
          trigger_sonar();
          estado_sonar = 0;
          timeout_echo = 0;
          novo_echo = 0;
        }
      break;

      case 2: //63 graus
        OCR1AH=8;
        OCR1AL=128;    
        if(last_estado == 3)
        {
          prox_estado = 1;
        }
        else prox_estado = 3;
        if(timer_scan >= tempo_estab)
        {
          trigger_sonar();
          estado_sonar = 0;
          timeout_echo = 0;
          novo_echo = 0;       
          last_estado = 2;
        }
      break;
 
      case 3: //27 graus
        OCR1AH=7;   
        OCR1AL=128;      
        if(last_estado == 2)
        {
          prox_estado = 4;
        }
        else prox_estado = 2;       
        if(timer_scan >= tempo_estab)
        {
          trigger_sonar();
          estado_sonar = 0;
          timeout_echo = 0;
          novo_echo = 0;
          last_estado = 3;
        }
      break;
 
      case 4: //0 graus
        OCR1AH=5;
        OCR1AL=centro;
        if(last_estado == 3)
        {
          prox_estado = 5;
        }
        else prox_estado = 3;
        if(timer_scan >= tempo_estab)
        {
          trigger_sonar();
          estado_sonar = 0;
          timeout_echo = 0;
          novo_echo = 0;       
          last_estado = 4;
        }
      break;
 
      case 5: //-27 graus
        OCR1AH=4;
        OCR1AL=128;
        if(last_estado == 4)
        {
          prox_estado = 6;
        }
        else prox_estado = 4;
        if(timer_scan >= tempo_estab)
        {
          trigger_sonar();
          estado_sonar = 0;
          timeout_echo = 0;
          novo_echo = 0;       
          last_estado = 5;
        }
      break;
 
      case 6: //-63 graus
        OCR1AH=3;
        OCR1AL=128;
        if(last_estado == 5)
        {
          prox_estado = 7;
        }
        else prox_estado = 5;
        if(timer_scan >= tempo_estab)
        {
          trigger_sonar();
          estado_sonar = 0;
          timeout_echo = 0;
          novo_echo = 0;       
          last_estado = 6;
        }
      break;
     
      case 7: //-90 graus
        OCR1AH=2;
        OCR1AL=128;
        last_estado = 7;
        prox_estado = 5;
        if(timer_scan >= tempo_estab)
        {
          trigger_sonar();
          estado_sonar = 0;
          timeout_echo = 0;
          novo_echo = 0;
        }
      break;
    }//fim switch
  }
}//================FIM DO LOOP PRINCIPAL========================
//===========================================FUNÇÕES DO PROGRAMA=========================
