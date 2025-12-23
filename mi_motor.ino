#include <avr/io.h>
#include <avr/interrupt.h>
#include <PID_v1.h>


#define sbi(port,bit) (port) |= (1 << (bit))
#define cbi(port,bit) (port) &= ~(1 << (bit))

//defines de mis pines para la pantalla LCD: 

#define LCD_RS  A3   

#define LCD_E   13
#define LCD_D0  12
#define LCD_D1  11
#define LCD_D2  7
#define LCD_D3  6
#define LCD_D4  5
#define LCD_D5  4
#define LCD_D6  A4
#define LCD_D7  A5
#define LCD_BL_P  A1
#define LCD_BL_N  A2

//variable para saber casd cuanto hacer update en la pantalla
uint32_t lcd_timer = 0;


//defines para adc: 
volatile uint16_t adc_teclado = 0;
volatile bool adc_nuevo = false;

volatile uint8_t viejo_A = 0;
volatile uint8_t viejo_B = 0;

volatile long pulsos_totales = 0;
volatile long pulsos_ventana = 0;
volatile long pulsos_last_window = 0;
// la velocidad que se quiere llegar
double Setpoint = 0;

//la velocidad que entra
double Input = 0;    
//la velocidad real
double Output = 0;     

// PID inicial "seguro"
double KP = 1.0;
double KI = 0.3;
double KD = 0.0;

//indicador del flag de 40 ms
volatile bool flag_40ms = false;
// de forma directa de forma inversa en refrigeradoras
PID myPID(&Input, &Output, &Setpoint, KP, KI, KD, DIRECT);



float aceleracion_rampa = 5;
bool rampa_en_proceso = false;

double velocidad_final = 0; 

//funcion de mis milis para no usar el dea rduino:
volatile uint32_t reloj_ms = 0;

     
 
//motor estado:

bool motor_activo = false;

//funcion de delay

void delay_ms(uint32_t ms)
{
    uint32_t start = millis_mio();
    while ((millis_mio() - start) < ms)
    {
        // espera activa
    }
}


void timer3_init_millis()
{
    cli();

    TCCR3A = 0;
    TCCR3B = 0;

    // Modo CTC
    TCCR3B |= (1 << WGM32);

    // 16 MHz / 64 = 250 kHz → 1 ms = 250 ticks
    TCCR3B |= (1 << CS31) | (1 << CS30); // prescaler 64
    OCR3A = 249;

    // Interrupción Compare Match A
    TIMSK3 |= (1 << OCIE3A);

    sei();
}

uint32_t millis_mio()
{
    uint32_t m;
    cli();
    m = reloj_ms;
    sei();
    return m;
}

ISR(TIMER3_COMPA_vect)
{
    reloj_ms++;

    static uint8_t cnt40 = 0;
    if (++cnt40 >= 40)
    {
        cnt40 = 0;

        pulsos_last_window = pulsos_ventana;
        pulsos_ventana = 0;

        flag_40ms = true;
    }
    //se empieza siempre la conversion y al acabarse se detecta y manda a la rutina ISR
    if (!(ADCSRA & (1 << ADSC))) 
    {
     ADCSRA |= (1 << ADSC);  
    }
}

ISR(ADC_vect)
{
    adc_teclado = ADC;
    adc_nuevo = true;
}


void pwm1_init()
{
  cli();
//reseteo registro por si acaso
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1C = 0;
//pongo el modo fast
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << WGM12);
//la salida habilitada por el pin 10
  TCCR1A |= (1 << COM1B1);
//pongo el tope
  ICR1 = 799;
//el prescaler que no lo necesito
  TCCR1B |= (1 << CS10);
 

  sei();
}


//inicializo las interupciones para los contadores de pulsos: 

void encoders_pulsos_innit()
{
    cli();

    // Encoder en D3 (INT0 = PD0) y D2 (INT1 = PD1)
    cbi(DDRD, PD0);
    cbi(DDRD, PD1);

    cbi(PORTD, PD0);
    cbi(PORTD, PD1);

    // Interrupción en cualquier flanco
    EICRA |= (1 << ISC00) | (1 << ISC10);
    EICRA &= ~((1 << ISC01) | (1 << ISC11));

    // Habilitar INT0 e INT1
    EIMSK |= (1 << INT0) | (1 << INT1);

    sei();
}



//estableciendo el parametro

void setDuty(float valor_p) 
{
    OCR1B = (uint16_t)(valor_p * ICR1);
}

void encoder_update()
{
    uint8_t nuevo_A = (PIND >> PD0) & 1;
    uint8_t nuevo_B = (PIND >> PD1) & 1;

    if (nuevo_A == viejo_A && nuevo_B == viejo_B)
        return;

    int dir = 0;

    if (nuevo_A != viejo_A) {
        dir = (nuevo_A == nuevo_B) ? +1 : -1;
    } else {
        dir = (nuevo_A != nuevo_B) ? +1 : -1;
    }

    pulsos_totales += dir;
    pulsos_ventana += dir;

    viejo_A = nuevo_A;
    viejo_B = nuevo_B;
}



ISR(INT0_vect) 
{ 
    encoder_update(); 
}

ISR(INT1_vect) 
{ 
    encoder_update(); 
 }


//configurando nuestroo ADC 
void setup_ADC()
{
    cli();

    // usamos ek ADC con 5V nos vale no necesitamos tanta precision
    ADMUX = (1 << REFS0) | (7 & 0x1F);

    // usamos en analog 0 asi que lo ponemos explicitamente a 0
    ADCSRB &= ~(1 << MUX5);

    //el prescaler(128 para que salga bien con nuestra velocidad de reloj) + activaos ADC 
    ADCSRA = (1 << ADEN) | (1 << ADIE) |
             (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    sei();
}




void adc_descartar_primera()
{
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC)); 
    (void)ADC;                    
}







float medir_rpm()
{
    const float PULSOS_POR_VUELTA = 1496.0;  // pulsos segun el calculo

    // usando la formula para el intervalo de 40 ms
    float revs_per_40ms = pulsos_last_window / PULSOS_POR_VUELTA;
    float revs_per_sec = revs_per_40ms / 0.04;

    // Calculamos el rpm y lo imprimimos
    float rpm = revs_per_sec * 60.0;



    return rpm;
}

void setup_PID()
{
   myPID.SetOutputLimits(0, 799);  
  myPID.SetSampleTime(40);        
  myPID.SetMode(AUTOMATIC);  
}


void cambiar_velocidad_obj(float rpm_obj)
{
    velocidad_final = rpm_obj;
     
    rampa_en_proceso = true;
}

void setup_PINS()
{
     
  // Direction pins (Motor A)
  sbi(DDRB, DDB4);   // pin 8 OUTPUT
  sbi(DDRB, DDB5);   // pin 9 OUTPUT

  sbi(PORTB, PB4);   // pin 8 HIGH = IN1
  cbi(PORTB, PB5);   // pin 9 LOW  = IN2

  // PWM output pins
  sbi(DDRB, DDB6);   // PB6 = pin 10 (OC1B)
  sbi(PORTB, PB6);   // pin 8 HIGH = IN1

  //pins para el mando 
  //activando como entrada el analog 0  
    cbi(DDRF, 7);
    cbi(PORTF, 7);

    
}


//funciones del teclado

int funcion_valor(int valor)
{
    if (valor < 72)
    {
        
             return 1;
    }
    else if (valor < 216)
    {
            return 2;
    }
    else if (valor < 377) 
    {
            return 3;
    } 
    else if (valor < 567) 
    {
        return 4;
    }
    else if (valor < 856) 
    {
        return 5;
    }
    else if (valor > 1000)
    {

         return 0;
    }
    else return 0; 
}






int leer_teclado()
{
    static int tecla_estable = 0;
    static int tecla_anterior = 0;
    static uint32_t tiempo_cambio = 0;
    static uint32_t tiempo_repeticion = 0;
    const uint16_t debounce_ms = 30;
    const uint16_t retardo_repeat = 1000;
    const uint16_t intervalo_repeat = 70; 

    uint32_t ahora = millis_mio();

    int valor = adc_teclado;
    int tecla_actual = funcion_valor(valor);

    
    

    
    if (tecla_actual != tecla_estable)
    {
        tecla_estable = tecla_actual;
        tiempo_cambio = ahora;
    }

    // calculando el debounce con tiempo
    if (ahora - tiempo_cambio < debounce_ms)
        return 0;

    
    if (tecla_estable != tecla_anterior)
    {
        tecla_anterior = tecla_estable;

        if (tecla_estable != 0)
        {
            
            tiempo_repeticion = ahora + retardo_repeat;
            return tecla_estable;
        }
        else
        {
            
            return 0;
        }
    }

    if (tecla_estable != 0)
    {
        if (ahora >= tiempo_repeticion)
        {
            
            tiempo_repeticion = ahora + intervalo_repeat;
            return tecla_estable;
        }
    }

    return 0;
}



//fin funciones del teclado




void actualizar_rpm()
{
    Input = medir_rpm();
}

//siempre subo y bajo 
void actualizar_rampa()
{
    if (!rampa_en_proceso) return;
    //transformar acceleracion a rpm como medimos cada 40ms
    float acc_rpm = aceleracion_rampa * 0.04;

    if (Setpoint < velocidad_final)
    {
        Setpoint += acc_rpm;
        if (Setpoint > velocidad_final)
            Setpoint = velocidad_final;
    }
    else if (Setpoint > velocidad_final)
    {
        Setpoint -= acc_rpm;
        if (Setpoint < velocidad_final)
            Setpoint = velocidad_final;
    }

    if (Setpoint == velocidad_final)
        rampa_en_proceso = false;
}


void control_velocidad_PID()
{
    myPID.Compute();
    OCR1B = (uint16_t)Output;
}


//enlazar velocidad con teclas:
void gestionar_tecla(int tecla)
{
    float nueva;
    switch(tecla)
    {
        case 1:
            
          
            if (aceleracion_rampa > 1) aceleracion_rampa--;
            break;

        case 2:
           
            if (aceleracion_rampa < 1000) aceleracion_rampa++;
            break;

        case 3:
            
            nueva = Setpoint - 1;
            cambiar_velocidad_obj(nueva);
       
            break;

        case 4:
            
            nueva = Setpoint + 1;
            cambiar_velocidad_obj(nueva);
         
            break;

        case 5:
            // apagar 
            motor_activo = !motor_activo;
            if (motor_activo)
            {
                cambiar_velocidad_obj(velocidad_final);
            }
            else
            {
                cambiar_velocidad_obj(0);
            }
            break;
    }
}

//cosas para la pantalla LED
void setup_automatico() {

 // LCD control pins
sbi(DDRF, PF4);   // LCD_RS  (A3)
sbi(DDRC, PC7);   // LCD_E   (13)

// LCD data pins
sbi(DDRD, PD6);   // LCD_D0  (12)
sbi(DDRB, PB7);   // LCD_D1  (11)
sbi(DDRE, PE6);   // LCD_D2  (7)
sbi(DDRD, PD7);   // LCD_D3  (6)
sbi(DDRC, PC6);   // LCD_D4  (5)
sbi(DDRD, PD4);   // LCD_D5  (4)
sbi(DDRF, PF1);   // LCD_D6  (A4)
sbi(DDRF, PF0);   // LCD_D7  (A5)

// LCD Backlight
sbi(DDRF, PF6);   // LCD_BL_P (A1)
sbi(DDRF, PF5);   // LCD_BL_N (A2)


  sbi(DDRF, PF6);   
sbi(DDRF, PF5);   

// Backlight ON: P = HIGH, N = LOW
sbi(PORTF, PF6);  // LCD_BL_P HIGH
cbi(PORTF, PF5);  // LCD_BL_N LOW

  delay_ms(15);

  lcd_comando_send(0x30);
  delay_ms(5);
  lcd_comando_send(0x30);
  delayMicroseconds(110);
  lcd_comando_send(0x30);
  delayMicroseconds(50);

  lcd_comando_send(0x38); // 8-bit, 2 líneas
  lcd_comando_send(0x0C); // display ON
  delayMicroseconds(50);

  lcd_comando_send(0x01); // clear
  delayMicroseconds(50);

  lcd_comando_send(0x06); // entry mode
  delay_ms(3);
}



void lcd_comando_send(byte comando) {

  // RS = 0  → comando
cbi(PORTF, PF4);   // LCD_RS LOW

// E = 0
cbi(PORTC, PC7);   // LCD_E LOW

// Data bus
((comando & (1 << 0)) ? sbi(PORTD, PD6) : cbi(PORTD, PD6));  // D0
((comando & (1 << 1)) ? sbi(PORTB, PB7) : cbi(PORTB, PB7));  // D1
((comando & (1 << 2)) ? sbi(PORTE, PE6) : cbi(PORTE, PE6));  // D2
((comando & (1 << 3)) ? sbi(PORTD, PD7) : cbi(PORTD, PD7));  // D3
((comando & (1 << 4)) ? sbi(PORTC, PC6) : cbi(PORTC, PC6));  // D4
((comando & (1 << 5)) ? sbi(PORTD, PD4) : cbi(PORTD, PD4));  // D5
((comando & (1 << 6)) ? sbi(PORTF, PF1) : cbi(PORTF, PF1));  // D6
((comando & (1 << 7)) ? sbi(PORTF, PF0) : cbi(PORTF, PF0));  // D7

  sbi(PORTC, PC7);  
  delayMicroseconds(2);
  cbi(PORTC, PC7); 

  delayMicroseconds(40);
}


void lcd_write(byte dato) {

  // RS = 1 → dato
    sbi(PORTF, PF4);     // LCD_RS HIGH

    // E = 0
    cbi(PORTC, PC7);     // LCD_E LOW

    // Data bus
    ((dato & (1 << 0)) ? sbi(PORTD, PD6) : cbi(PORTD, PD6));  // D0
    ((dato & (1 << 1)) ? sbi(PORTB, PB7) : cbi(PORTB, PB7));  // D1
    ((dato & (1 << 2)) ? sbi(PORTE, PE6) : cbi(PORTE, PE6));  // D2
    ((dato & (1 << 3)) ? sbi(PORTD, PD7) : cbi(PORTD, PD7));  // D3
    ((dato & (1 << 4)) ? sbi(PORTC, PC6) : cbi(PORTC, PC6));  // D4
    ((dato & (1 << 5)) ? sbi(PORTD, PD4) : cbi(PORTD, PD4));  // D5
    ((dato & (1 << 6)) ? sbi(PORTF, PF1) : cbi(PORTF, PF1));  // D6
    ((dato & (1 << 7)) ? sbi(PORTF, PF0) : cbi(PORTF, PF0));  // D7



  sbi(PORTC, PC7); 
  delayMicroseconds(2);
    cbi(PORTC, PC7);  

  delayMicroseconds(40);
}


void lcd_setcursor(byte row, byte column) {

  static const uint8_t rowBase[4] = {0x00, 0x40, 0x14, 0x54};
  byte address = rowBase[row] + column;
  byte comando = 0x80 | address;

  lcd_comando_send(comando);
}


void lcd_print(const char* s) 
{
  for (int i = 0; s[i] != '\0'; i++) 
  {
    lcd_write(s[i]);
  }
}

void lcd_print(int value)
{
    char buf[12];
    itoa(value, buf, 10);
    lcd_print(buf);
}

void lcd_clear() {
  lcd_comando_send(0x01);
}

void screen_shift_derecha() {
  lcd_comando_send(0x1C);
}

void screen_shift_izquierda() {
  lcd_comando_send(0x18);
}
void lcd_actualizar_estado()
{
    char buf[8];

    lcd_setcursor(0, 0);
    lcd_print("RPM:");
    itoa((int)Input, buf, 10);
    lcd_print(buf);
    lcd_print("     ");   // relleno fijo

    lcd_setcursor(1, 0);
    lcd_print("SET:");
    itoa((int)velocidad_final, buf, 10);
    lcd_print(buf);
    lcd_print("     ");

    lcd_setcursor(2, 0);
    lcd_print("ACC:");
    itoa((int)aceleracion_rampa, buf, 10);
    lcd_print(buf);
    lcd_print("     ");

    lcd_setcursor(3, 0);
    lcd_print("Motor:");
    lcd_print(motor_activo ? " ON " : " OFF");
}

















void setup() 
{

  
   
  setup_PINS();
   
  pwm1_init();
 
    timer3_init_millis(); 
  encoders_pulsos_innit();

    setup_ADC();               
    adc_descartar_primera();  
    setup_automatico();

  setup_PID();
  cambiar_velocidad_obj(120);
  

}


void loop()
{
     int tecla = leer_teclado();
    if (tecla != 0)
        gestionar_tecla(tecla);
   
    if (flag_40ms)
    {
        flag_40ms = false;

        static uint32_t t=0;

        t = millis_mio();

        actualizar_rpm();      

        actualizar_rampa();
        myPID.Compute();

        
        OCR1B = (uint16_t)Output;

        lcd_actualizar_estado();
    }
}

