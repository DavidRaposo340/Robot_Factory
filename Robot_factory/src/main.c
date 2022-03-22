#include <avr/io.h>
#include <avr/interrupt.h>
#include "serial_printf.h"
#include <avr/eeprom.h>

#define T1BOTTOM 65536-31250   //Note: Mode 0 
//queremos 125ms => (125ms*16Mhz)/prescaler(64)=31250

#define ON_STOP_FRENTE 0   //STOP_FRENTE-> ligado se 1, desligado se !1 

uint8_t state_FSM0, next_state_FSM0, state_FSM1, next_state_FSM1, state_FSM2, next_state_FSM2, state_FSM3, next_state_FSM3, state_FSM10, next_state_FSM10;
uint8_t sw_1, sw_prev_1;


volatile uint16_t T1, Tshow;  //timers
volatile uint8_t dutycycle_direita, dutycycle_esquerda;  //pwm
volatile uint8_t BT;  //valor vindo do bluetooth

volatile uint16_t sensor_IR_1, sensor_IR_2, sensor_IR_3, sensor_IR_4, sensor_IR_5;  //vetor dos sensores
volatile uint8_t canal;  //posiçao do vetor dos sensores




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Timers
ISR(TIMER1_OVF_vect)
{
  TCNT1 = T1BOTTOM; //reload TC1

  if(Tshow) Tshow--;
  if(T1) T1--;
  
  if (serial_receive_ready()) {       // Test if there is serial data to be read
       BT = serial_receive();   // Read serial data
  }
}

void tc1_init(void) {
  cli();
  TCCR1B = 0;          // Stop TC1
  TIFR1 = (7<<TOV1)    // Clear pending intr
        | (1<<ICF1);
  TCCR1A = 0;          // Mode zero
  TCNT1 = T1BOTTOM;    // Load BOTTOM value
  TIMSK1 = (1<<TOIE1); // Enable Ovf intrpt
  TCCR1B &= ~(1 << CS12);   // Start TC1 (TP=64) ->>> prescaler64= 011 = 3  -> tambem se poderia escrever TCCR1B = 3; 
  TCCR1B |= (1 << CS11);
  TCCR1B |= (1 << CS10);
  sei();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


 void EEPROM_write(unsigned int uiAddress, uint8_t ucData)
  {
      /* Wait for completion of previous write */
      while(EECR & (1<<EEPE))
      ;
      /* Set up address and Data Registers */
        EEAR = uiAddress;
        EEDR = ucData;
      /* Write logical one to EEMPE */
        EECR |= (1<<EEMPE);
      /* Start eeprom write by setting EEPE */
        EECR |= (1<<EEPE);
  }

  uint8_t EEPROM_read(unsigned int uiAddress)
  {
      /* Wait for completion of previous write */
        while(EECR & (1<<EEPE))
      ;
      /* Set up address register */
        EEAR = uiAddress;
      /* Start eeprom read by writing EERE */
      EECR |= (1<<EERE);
      /* Return data from Data Register */
      return EEDR;
  }


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ISR(TIMER2_OVF_vect)
{
    OCR2A= (dutycycle_direita/100.0)*255; //converte para valor de 0-255
    OCR2B= (dutycycle_esquerda/100.0)*255; //converte para valor de 0-255
}


void motor_direita(uint8_t velocidade, uint8_t frente )
{ 
  //velocidade valor de 0 a 100
  // frente=1 para motor andar para frente, frente=0 para motor andar para tras
  
  if (frente) //anda para frente
  {
    PORTB |= (1 << 4);  //ain2 on
    PORTB &= ~(1 << 2);  //ain1 off
  }
  else //anda para tras
  {
    PORTB &= ~(1 << 4);  //ain2 off
    PORTB |= (1 << 2);  //ain1 on
  }

  if (velocidade==0)
  {
      PORTB &= ~(1 << 2);  //ain1 off
      PORTB &= ~(1 << 4);  //ain2 off
  }

  dutycycle_direita=velocidade;
  //OCR2A= (velocidade/100.0)*255; //converte para valor de 0-255

  return;
}

void motor_esquerda(uint8_t velocidade, uint8_t frente )
{ 
  //velocidade valor de 0 a 100
  // frente=1 para motor andar para frente, frente=0 para motor andar para tras
  
  if (frente) //anda para frente
  {
    PORTD |= (1 << 4);  //bin2 on
    PORTD &= ~(1 << 2);  //bin1 off
  }
  else //anda para tras
  {
    PORTD &= ~(1 << 4);  //bin2 off
    PORTD |= (1 << 2);  //bin1 on
  }

  if (velocidade==0)
  {
      PORTD &= ~(1 << 2);  //bin1 off
      PORTD &= ~(1 << 4);  //bin2 off
  }

  dutycycle_esquerda=velocidade;  
  //OCR2B= (velocidade/100.0)*255; //converte para valor de 0-255

  return;
}

void frente( uint8_t on){
  if (on){
     motor_esquerda(70, 1);
     motor_direita(70, 1);
  }
  else return;
}

void tras( uint8_t on){
  if (on){
     motor_esquerda(70, 0);
     motor_direita(70, 0);
  }
  else return;
}

void vira_dir( uint8_t on){
  if (on){
     motor_esquerda(70, 1);
     motor_direita(0, 1);
  }
  else return;
}

void vira_esq( uint8_t on){
  if (on){
     motor_esquerda(0, 1);
     motor_direita(70, 1);
  }
  else return;
}

void vira_dir_tras( uint8_t on){
  if (on){
     motor_esquerda(85, 0);
     motor_direita(55, 0);
  }
  else return;
}

void vira_esq_tras( uint8_t on){
  if (on){
     motor_esquerda(55, 0);
     motor_direita(85, 0);
  }
  else return;
}

void vira_forte_dir( uint8_t on){
  if (on){
     motor_esquerda(50, 1);
     motor_direita(50, 0);
  }
  else return;
}

void vira_forte_esq( uint8_t on){
  if (on){
     motor_esquerda(50, 0);
     motor_direita(50, 1);
  }
  else return;
}

void para( uint8_t on){
  if (on){
     motor_esquerda(0, 1);
     motor_direita(0, 1);
  }
  else return;
}


void tc2_init (void){ //pwm
  cli();
  TCCR2A=0; //limpar o que la estava
  TCCR2B=0; //limpar o que la estava
  
  TCCR2A |= (1 << COM2A1)  | (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); //pwm nao invertido (tanto OC2A e OC2B); Modo 3
  
  TIMSK2 = (1<<TOIE2); // Enable Ovf intrpt

  OCR2A=0;  // pmw para motor A com duty cycle de 0% (iniciar a off por segurança)
  OCR2B=0;  // pmw para motor B com duty cycle de 0% (iniciar a off por segurança)

  TCCR2B |= (1 << CS22) | (1 << CS21); //pwm Modo 3; F oc2a = 244MHz e F clk = 16MHz e TOP=0xFF (pq estamos em modo3) ->> temos um prescaler(N) de 256 (Foc2a=Fclk/(N*(1+TOP)))
  
  sei();
  // depois para aumentar a velocidade temos de aumentar o ocr2a e/ou ocr2b
  // vai de 0 a 255... mas dutycycle vai de 0 a 100 (OCR2A= (dutycycle/100.0)*255.0 )     
}


void imanOn(uint8_t on){
  if (on==1)
    PORTD |= (1 << 5);  //eletroiman on
  else
    PORTD &= ~(1 << 5);  //eletroiman off

  return;
}


///////////////////////////////////////////////////
//PID
 uint8_t ERRO, ERRO_anterior=0, lado;  //ERRO PID / lado=1 vira direita; lado=0 vira esquerda
 uint8_t ki=0 , kp=50 , kd=40;  
 uint8_t I , P , D, PID;
 uint8_t vel_esq, vel_dir;  
 uint8_t vel_base=40;  
 uint8_t angulo=0;  
 
void calcular_PID(){

  ERRO=angulo;

  
  if (ERRO==0)  I=0;

  P=ERRO;
  I=I+ERRO;
  if (I>100)  I=100;
  else if (I<-100)  I=-100;
  D=ERRO- ERRO_anterior;
  PID=(kp* (P*0.1) )+ (ki* (I*0.1) )+ (kd* (D*0.1) );
  //PID=(kp* (P) )+ (ki* (I) )+ (kd* (D) );
  ERRO_anterior=ERRO; 

}



void move_forward(){
  
    calcular_PID();
    vel_esq= vel_base + PID;
    vel_dir= vel_base - PID;

    if(vel_esq>0)
      motor_esquerda(vel_esq, 1);
    else  
      motor_esquerda(-vel_esq, 0);

    if(vel_dir>0)
      motor_direita(vel_dir, 1);
    else  
      motor_direita(-vel_dir, 0);

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rotate(){
    calcular_PID();
    vel_esq= PID;
    vel_dir= -PID;

    if(vel_esq>0)
      motor_esquerda(vel_esq, 1);
    else  
      motor_esquerda(-vel_esq, 0);

    if(vel_dir>0)
      motor_direita(vel_dir, 1);
    else  
      motor_direita(-vel_dir, 0);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int main(void)
{


  //inicializaçao das saidas pwm
  DDRB |= (1 << 3); //definir pino 11 como output (pwm)
  DDRD |= (1 << 3); //definir pino 3 como output (pwm)

  //inicializaçao das saidas de controlode direçao de motores
  DDRB |= (1 << 2); 
  DDRB |= (1 << 4);
  DDRD |= (1 << 2); 
  DDRD |= (1 << 4); 
  
  ////inicializaçao das saidas de eletroiman
  DDRD |= (1 << 5);
   
  if ((EEPROM_read(1<<1) > 101) || (EEPROM_read(1<<1) < 100)) {  //se a eeprom nao tiver o estado da maquina 10 (que vai de 100 a 101), usar o estado 100
    EEPROM_write((1<<1), 100);
  }
 

  
  
  tc1_init();
  tc2_init();

  cli();
  T1=8;
  sei();

  printf_init();        // Init the serial port to have the ability to printf
  printf("Serial I/O Demo\n");   // into a terminal  
  
  

  while (1) {

    switch (state_FSM1) {
  
      case 0: {
        if (T1<=0) {                 
  		  	next_state_FSM1 = 1;           
          
          cli();
          T1=8;
          sei();
        }
  		} break;
  
      case 1: {
        if (T1<=0) {                  
  		  	next_state_FSM1 = 2;           
          
          cli();
          T1=8;
          sei();
        }
  		} break;

      case 2: {
        if (T1<=0) {                  
  		  	next_state_FSM1 = 0;           
          
          cli();
          T1=8;
          sei();
        }
  		} break;
  
      
  
      default:
        next_state_FSM1 = 0;          
                
      break;
    }
        if (next_state_FSM1 != state_FSM1) {      // If needed, change state_FSM5
      state_FSM1 = next_state_FSM1;
    } 

    
    
    //ATIVAÇAO DE MOTORES e outras saidas
    frente        (  state_FSM1==1   );
    vira_dir      (  state_FSM1==2   );
    vira_forte_dir(  0    );
    vira_esq_tras (  0    );
    tras          (  0    );
    vira_dir_tras (  0    );
    vira_forte_esq(  0    );
    vira_esq      (  0    );
    para          (  state_FSM1==0    );

    imanOn( state_FSM1==0 );

 
    //Debug
    printf(" state_FSM1 = %u \n", state_FSM1 );


 
  /*
    if (Tshow == 0) {      // It is time to show things...
      
      cli();
      Tshow = 5;
      sei(); 

      printf(" \t\t\t\t\t\t 1-%u \t 2-%u \t 3-%u \t 4-%u \t 5-%u \t \n", sensor_IR_1, sensor_IR_2, sensor_IR_3, sensor_IR_4, sensor_IR_5 );
      printf(" \t\t\t\t\t\t 1-%u \t 2-%u \t 3-%u \t 4-%u \t 5-%u \t \n", max_esq(), esq(), meio(), dir(), max_dir() );
      
      printf(" \n\n\t STOP-%u \t \n", stop_frente() );
      

      printf(" \t\t esq-%u \t dir-%u  \n", vel_esq, vel_dir );
      printf(" \t\t pid-%u \t erro-%u  \n", PID, ERRO);

      printf("state_FSM0: %u \n", state_FSM0);
      printf("state_FSM1: %u \n", state_FSM1);
      printf("state_FSM2: %u \n\n", state_FSM2 );      

    }
  */
    
  }
}

