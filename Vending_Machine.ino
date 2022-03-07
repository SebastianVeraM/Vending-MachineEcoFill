/*
 * Interruption_LCD.cpp
 *
 * Created: 12/10/2021 04:22:12 p. m.
 * Author : Sebastian Vera Morales
 * 
 * Conexiones del proyecto
 * 
 * LCD
 * 
 * MÓDULO I2C PIN   ARDUINO NANO PIN
 *      GND               GND
 *      VCC               5V
 *      SDA               A4
 *      SCL               A5
 * 
 * MONEDERO MULTIMONEDA
 * 
 *  PIN MONEDERO    ARDUINO NANO PIN
 *      DC12V           
 *      COIN              D2
 *      GND
 *      
 *      
 *     BOTONES     ARDUINO NANO PIN
 *      BOTÓN 1          A0
 *      BOTÓN 2          A1
 *
 * CAUDALÍMETRO
 *
 *  PIN CAUDALÍMETRO  ARDUINO NANO PIN
 *        PULSO            D0/PD0
 *
 */  

//                                               LIBRERÍAS
//--------------------------------------------------------------------------------------------------------
#include <avr/io.h> 
#include <avr/interrupt.h>//SE AGREGA LA LIBRERÍA DE LAS INTERRUPCIONES
#include <util/delay.h> 
#include <EEPROM.h>
#include <LiquidCrystal.h>
//                                                VARIABLES
//--------------------------------------------------------------------------------------------------------
const int rs = 8, en = 9, d4 = 10, d5 = 11, d6 = 12, d7 = 13, measureInterval = 100;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
volatile bool flag1 = false, flag2 = false, flag3 = false;
const short int UnPeso = 5, DosPesos = 7, CincoPesos = 11, DiezPesos = 13;
const short int E1 = A0, E2 = A1, S1 = 3, S2 = 4, Q = 0;
short int PulsosAcum = 0, CreditoAcum = 0, MaxTimePulse = 190;
volatile int pulso = 0, pulseConter;
const float factorK = 6.6;
float volume = 0, frequency = 0, flow_Lmin = 0;
volatile unsigned long MillisUltPulso = 0;
unsigned long Contador = 0, t0 = 0;
String F,Qf,t,V;
//                                               DECLARACIÓN DE FUNCIONES
//--------------------------------------------------------------------------------------------------------
void Saldo();
void Llenado();
void Finalizado();
void Pantalla1();
void Pantalla2();
void Sinsaldo();
void Pocosaldo();
void Evalvula(const short int B, float L);
void Monedero();
void Mensaje();
float GetFrequency();
void SumVolume(float dV);
//                                                  FUNCIONES
//--------------------------------------------------------------------------------------------------------
void(* Reset) (void) = 0;

void Saldo()
{
  String cadena = (String) CreditoAcum;
  lcd.setCursor(0,0);
  lcd.print("  Saldo disponible:");
  lcd.setCursor(0,1);
  lcd.print("        $");
  lcd.setCursor(9,1);
  lcd.print(cadena);
  lcd.setCursor(1,2);
  lcd.print("Inserte las monedas");
  lcd.setCursor(3,3);
  lcd.print("una por una");
}

void Llenado()
{
  //CreditoAcum -= 10;
  lcd.setCursor(2,0);//
  lcd.print("Espere a que se");
  lcd.setCursor(3,1);
  lcd.print("llene su envase");
  lcd.setCursor(4,3);
  lcd.print("NO DA CAMBIO");
}

void Finalizado()
{
  lcd.setCursor(2,0);//
  lcd.print("Recoja su envase");
  lcd.setCursor(3,1);
  lcd.print("de la charola");
  lcd.setCursor(4,3);
  lcd.print("NO DA CAMBIO");
}

void Pantalla1()
{
  lcd.setCursor(2,0);
  lcd.print("Maquina EcoFill");
  lcd.setCursor(3,1);
  lcd.print("Kelco Quimicos");
  lcd.setCursor(1,2);
  lcd.print("Por favor inserte");
  lcd.setCursor(4,3);
  lcd.print("las monedas ");
}

void Pantalla2()
{
  lcd.setCursor(0,0);
  lcd.print("Venta minima 1 litro");
  lcd.setCursor(4,1);
  lcd.print("por producto");
  lcd.setCursor(0,2);
  lcd.print("Introduzca su envase");
  lcd.setCursor(4,3);
  lcd.print("NO DA CAMBIO");
}

void Sinsaldo()
{
  lcd.setCursor(3,0);
  lcd.print("Noy hay ningun");
  lcd.setCursor(2,1);
  lcd.print("saldo disponible");
  lcd.setCursor(1,2);
  lcd.print("Por favor inserte");
  lcd.setCursor(4,3);
  lcd.print("las monedas");
}

void Pocosaldo()
{
  lcd.setCursor(2,0);
  lcd.print("Saldo disponible");
  lcd.setCursor(4,1);
  lcd.print("insuficiente");
  lcd.setCursor(2,2);
  lcd.print("Por favor inserte");
  lcd.setCursor(4,3);
  lcd.print("mas monedas");
}

void Evalvula(const short int B,float L)
{ 
  flag3 = false;
  lcd.clear();
  digitalWrite(B,HIGH);
  do
  {
    frequency = GetFrequency();
    flow_Lmin = frequency/factorK;
    SumVolume(flow_Lmin);
    //F = (String) flow_Lmin;
    float Vl = (volume/L)*100; 
    V = (String) Vl;//Porcentaje de llenado
    if (volume >= L)
    {
      digitalWrite(B,LOW);
      lcd.setCursor(5,2);
      lcd.print("Llenado al:");
      lcd.setCursor(7,3);
      lcd.print("100%");
      //_delay_ms(1000);
      volume = 0;
      frequency = 0;
      flow_Lmin = 0;
      t0 = 0;
      flag3 = true;
    }
    lcd.setCursor(0,0);
    lcd.print("No retire su envase");
    lcd.setCursor(1,1);
    lcd.print("mientras se llena");
    lcd.setCursor(5,2);
    lcd.print("Llenado al:");
    lcd.setCursor(7,3);
    lcd.print(V);
    lcd.setCursor(11,3);
    lcd.print("%");
  }while(flag3 == false);
}

void Monedero()
{
   unsigned long lastTime = millis() - MillisUltPulso;
  if((pulso > 0) && (lastTime >= MaxTimePulse))
  {
    PulsosAcum = pulso;
    pulso = 0;
  }
  
  switch (PulsosAcum)
  {
    case UnPeso:
    PulsosAcum = 0;
    CreditoAcum += 1;
    //EEPROM.put(0,CreditoAcum);
    lcd.clear();
    break;

    case DosPesos:
    PulsosAcum = 0;
    CreditoAcum += 2;
    //EEPROM.put(0,CreditoAcum);
    lcd.clear();
    break;
    
    case CincoPesos:
    PulsosAcum = 0;
    CreditoAcum += 5;
    //EEPROM.put(0,CreditoAcum);
    lcd.clear();
    break;

    case DiezPesos:
    PulsosAcum = 0;
    CreditoAcum += 10;
    //EEPROM.put(0,CreditoAcum);
    lcd.clear();
    break;
  }
}
void Mensaje()
{
  if ((flag1 == true || flag2 == true) && (CreditoAcum > 0 && CreditoAcum < 10))
  {
    flag1 = false;
    flag2 = false;
    lcd.clear();
    Pocosaldo();
    _delay_ms(2000);
    lcd.clear();
  }
}

float GetFrequency()
{
  pulseConter = 0;
  sei();
  //delay(measureInterval);
  Contador = millis() + measureInterval;
  do{
      //__asm__("nop\n\t"); 
    }while(Contador >= millis());
  Contador = 0;
  cli();
  return (float)pulseConter * 1000 / measureInterval;
}

void SumVolume(float dV)
{
  volume += dV / 60 * (millis() - t0) / 1000.0;
  t0 = millis();
}

//                                                                  INTERRUPCIONES
//--------------------------------------------------------------------------------------------------------------------------------------------
ISR (INT0_vect)//INTERRUPCIÓN EXTERNA 0 PARA EL CONTEO DE PULSOS DE PARTE DEL MONEDERO MULTIMONEDA
{
  pulso++;   // Cada vez que insertamos una moneda valida, incrementamos el contador de monedas y encendemos la variable de control,
  MillisUltPulso = millis();
}

//INTERRUPCIÓN POR CAMBIO DE PIN
ISR (PCINT1_vect)//SE REALIZA LA ACCIÓN CUANDO UNA INTERRUPCIÓN CORRESPONDIENTE AL VECTOR PCINT1 SE ACTIVE ENTONCES.....
{//PARA PODER DIFERENCIAR ENTRE LOS PINES PARA QUE HAGAN UNA ACCIÓN ES ESPECÍFICO SE NECESITA PREGUNTAR POR EL PIN DE DONDE VIENE LA SEÑAL
  if ((digitalRead(E1) == HIGH))//SÍ LA INTERRUPCIÓN PROVIENE DEL PIN A0 ENTONCES HACE LA ACCIÓN DENTRO DE LA CONDICIÓN
  {
    flag1 = true;
    //Mensaje();
    //lcd.clear();
  }
  else if ((digitalRead(E2) == HIGH))//SÍ LA INTERRUPCIÓN PROVIENE DEL PIN A1 ENTONCES HACE LA ACCIÓN DENTRO DE LA CONDICIÓN
  {
    flag2 = true;
    //Mensaje();
    //lcd.clear();
  }
} 

ISR (PCINT2_vect)//SE REALIZA LA ACCIÓN CUANDO UNA INTERRUPCIÓN CORRESPONDIENTE AL VECTOR PCINT2 SE ACTIVE ENTONCES.....
{
  if ((digitalRead(Q) == HIGH))//SÍ LA INTERRUPCIÓN PROVIENE DEL PIN D0 ENTONCES HACE LA ACCIÓN DENTRO DE LA CONDICIÓN
    pulseConter++;
} 

//                                                                  CONFIGURACIÓN
//--------------------------------------------------------------------------------------------------------------------------------------------
void setup() 
{
  DDRC &=~ (1<<DDC0)|(1<<DDC1);//SE CONFIGURAN LAS ENTRADAS DE LOS BOTONES EN A0/PC0 Y A1/PC1
  DDRD |= (1<<DDD3)|(1<<DDD4);//SE CONFIGURAN LAS SALIDAS DE LAS VÁLVULAS EN D3/PD3 Y D4/PD4
  DDRD &=~ (1<<DDD2)|(1<<DDD0);//SE CONFIRGURA EL PIN PD2 y PD0 COMO ENTRADA DE LA INTERRUPCIÓN
  DDRB = 0xFF;//SE CONFIGURA COMO SALIDA EL PUERTO B PARA LA LCD EN MODO DE 4BITS
  cli();//SE DESACTIVAN LAS INTERRUPCIONES
  EIMSK |= (1<<INT0);//SE CONFIGURA LA INTERRUPCIÓN EXTERNA 0
  EICRA |= (1<<ISC01)|(1<<ISC00);//CUANDO SE DETECTE UN FLANCO DE SUBIDA PARA INT0
  PCIFR = 0x00;//SE LIMPIAN LAS BANDERAS
  PCICR |= (1<<PCIE1);//SE ACTIVA EL BIT PCIE1 DEL REGISTRO DE INTERRUPCIONES PARA ELEGIR EL CONJUNTO DE INTERRUPCIONES QUE SE VA A USAR, 
                      //EN ESTE CASO SE CAMBIO AL PCIE1 YA QUE ES DONDE SE ENCUENTRAN LAS INTERRUPCIONES PCINT8 Y PCINT9
  PCMSK1 |= (1<<PCINT8)|(1<<PCINT9);//SE ACTIVAN LAS INTERRUPCIONES PCINT8 Y PCINT9 DEL REGISTRO PCMSK1
  PCICR |= (1<<PCIE2);//SE ACTIVA EL BIT PCIE2 DEL REGISTRO DE INTERRUPCIONES PARA ELEGIR EL CONJUNTO DE INTERRUPCIONES QUE SE VA A USAR, 
                      //EN ESTE CASO SE CAMBIO AL PCIE2 YA QUE ES DONDE SE ENCUENTRA LA INTERRUPCION PCINT16
  PCMSK2 |= (1<<PCINT16);//SE ACTIVA LA INTERRUPCION PCINT16 DEL REGISTRO PCMSK2
  sei();//SE ACTIVAN NUEVAMENTE LAS INSTRUCCIONES  
  lcd.begin(20,4);
  lcd.clear();
  //EEPROM.get(0,CreditoAcum);
  //CreditoAcum = 0;
}
//                                                                      BUCLE
//---------------------------------------------------------------------------------------------------------------------------------------------
void loop() 
{
  Monedero();
  if (CreditoAcum > 0)
  {
    Saldo();
    Mensaje();
    if ((flag1 == true) && CreditoAcum >= 10)//SÍ LA INTERRUPCIÓN PROVIENE DEL PIN A0 ENTONCES HACE LA ACCIÓN DENTRO DE LA CONDICIÓN
    {
      flag1 = false;
      flag2 = false;
      lcd.clear();
      Llenado();
      Evalvula(S1,0.77);//25 segundos
      lcd.clear();
      Finalizado();
      _delay_ms(2000);
      lcd.clear();
      CreditoAcum -= 10;
      //if (CreditoAcum == 0)
      //  Reset();
    }
    else if ((flag2 == true) && CreditoAcum >= 10)//SÍ LA INTERRUPCIÓN PROVIENE DEL PIN A0 ENTONCES HACE LA ACCIÓN DENTRO DE LA CONDICIÓN
    {
      flag1 = false;
      flag2 = false;
      lcd.clear();
      Llenado();
      Evalvula(S2,0.93);//9 segundos 0.95 Lt
      lcd.clear();
      Finalizado();
      _delay_ms(2000);
      lcd.clear();
      CreditoAcum -= 10;
      //if (CreditoAcum == 0)
      //  Reset();
    }
  }
  
  else if ((flag1 == true || flag2 == true) CreditoAcum == 0)
  {
    flag1 = false;
    flag2 = false;
    lcd.clear();
    Sinsaldo();
    _delay_ms(2000);
    lcd.clear();
  }
     
  else
  {
    Pantalla1();
    _delay_ms(2000);
    lcd.clear();
    Pantalla2();
    _delay_ms(2000);
    lcd.clear();
  }
}


  /*
  //Contador = millis() + (L*1000);
  Contador = millis();
  flag3 = false;
  float T;
  //digitalWrite(B,HIGH);
  do
  {
    //digitalWrite(B,HIGH);
    float frequency = GetFrequency();
    float flow_Lmin = frequency / factorK; 
    T = (L/flow_Lmin)*60000.0;
    unsigned long Tvalve = (unsigned long) (T + Contador);
    //if (millis() >= Contador + T)
    if (millis() >= Tvalve)
    {
      Contador = 0;
      T = 0;
      frequency = 0;
      flag3 = true;
      //digitalWrite(B,LOW);
    }
   Qf = (String) flow_Lmin;  
   F = (String) frequency;
   //T = (0.7/flow_Lmin)*60000;
   t = (String) T; 
   lcd.setCursor(0,0);
   lcd.print("Caudal:");
   lcd.setCursor(8,0);
   lcd.print(Qf);
   lcd.setCursor(0,1);
   lcd.print("Tiempo:");
   lcd.setCursor(8,1);
   lcd.print(t);
  }while(flag3 == false);
  digitalWrite(B,LOW);
  */
  
  
  /*
   * Delay con millis()
   * 
  Contador = millis() + Retardo;
  do 
  {
    digitalWrite(B,HIGH);
  } while(Contador >= millis());
  digitalWrite(B,LOW);
  Contador = 0;*/
