#include "MK64F12.h"

#define PIN_OUT_CFG (PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK)

/* ---------- Program modes ---------- */
#define MODE_RESET   0      /* show 0000, LED chase 1-2-3-4  */
#define MODE_IDLE    1      /* wait for S2                   */
#define MODE_RUN     2      /* measuring                     */
#define MODE_FREEZE  3      /* value frozen                  */

/* ---------- Calibration ---------- */
#define TEMP_OFFSET_C  5.0f /* compensate self-heating of MCU chip */

/* ---------- 7-Segment (segment: active LOW, digit: active HIGH) ---------- */
const unsigned char SEG_MAP[10] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90};
const unsigned char DIG_MAP[4]  = {0x01, 0x02, 0x04, 0x08};

#define SEG_BLANK  0xFF
#define SEG_C      0xC6
#define SEG_E      0x86
#define SEG_r      0xAF

/* ---------- LED on shield: D1=PTD1, D2=PTD3, D3=PTD2, D4=PTD0 (active LOW) ---------- */
const int LED_BIT[4] = {1, 3, 2, 0};

/* ---------- Global variables ---------- */
unsigned char disp[4] = {0xC0, 0xC0, 0xC0, 0xC0};   /* segment code of each digit  */
int temp10 = 0;                                     /* temperature x10 (273=27.3C) */

/* Interrupt Handler (Lab 6)  */
volatile int press1 = 0;
volatile int press2 = 0;
volatile int press3 = 0;


void PORTB_IRQHandler(void); /* Interrupt Handler(Lab 6) */
void short_delay(void);
void led_show(int mask);
void buzzer_on(void);
void buzzer_off(void);
void shiftOut(int data);
void display_7seg(int digit_pos, int seg);
void show_zero(void);
void show_error(void);
void show_temp(int t);
void ADC0_init(void);
int read_ADC0(void);
float convertToTemp(int adc_val);
void Init_Shield(void);
void delayMs(int n) ;
void UART0_init(void);
/* =====================================================
 * 1. SysTick (Lab 4) : Interrupt 1 ms
 * ===================================================== */
void delayMs(int n) { 
    int i; 
    SysTick->LOAD = 20480 - 1; 
    /* Enable the timer and choose sysclk as the clock source */ 
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;      
     
    for (i = 0; i < n; i++) 
        /* wait until the COUNT flag is set */ 
        while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0) {  
        } 
     
    SysTick->CTRL = 0;      /* Stop the timer (Enable = 0) */ 
}
/* =====================================================
 * 2. PORTB Interrupt ( Lab 6) :  SW1, SW2, SW3
 * ===================================================== */
void PORTB_IRQHandler(void) {
    /*  Interrupt  A1 (PTB3)  */
    if (PORTB->ISFR & (1 << 3)) {
        press1 = 1;
        PORTB->ISFR = (1 << 3); 
    }
    /*  Interrupt  A2 (PTB10)  */
    if (PORTB->ISFR & (1 << 10)) {
        press2 = 1;
        PORTB->ISFR = (1 << 10);
    }
    /*  Interrupt  A3 (PTB11)  */
    if (PORTB->ISFR & (1 << 11)) {
        press3 = 1;
        PORTB->ISFR = (1 << 11);
    }
}


/* =====================================================
 * Delay & Hardware Controls
 * ===================================================== */
void short_delay(void) {
    volatile int d;
    for (d = 0; d < 50; d++) __NOP();
}

void led_show(int mask) {
    int i;
    for (i = 0; i < 4; i++) {
        if (mask & (1 << i)) PTD->PCOR = (1 << LED_BIT[i]);   /* 0 = on  */
        else                 PTD->PSOR = (1 << LED_BIT[i]);   /* 1 = off */
    }
}

void buzzer_on(void) {
    PTA->PDDR |= (1 << 1);       
    PTA->PCOR = (1 << 1);        
}

void buzzer_off(void) {
    PTA->PDDR &= ~(1 << 1);      
}

/* =====================================================
 * 7-Segment (Shift Register)
 * ===================================================== */
void shiftOut(int data) {
    int i;
    for (i = 0; i < 8; i++) {
        if (data & 0x80) PTC->PSOR = (1 << 12);     
        else             PTC->PCOR = (1 << 12);
        data = data << 1;
        short_delay();

        PTC->PSOR = (1 << 3);                       
        short_delay();
        PTC->PCOR = (1 << 3);
        short_delay();
    }
}

void display_7seg(int digit_pos, int seg) {
    PTB->PCOR = (1 << 23);                          
    short_delay();
    shiftOut(0xFF);
    shiftOut(0x00);
    PTB->PSOR = (1 << 23);                          
    short_delay();

    PTB->PCOR = (1 << 23);
    short_delay();
    shiftOut(seg);
    shiftOut(DIG_MAP[digit_pos]);
    PTB->PSOR = (1 << 23);
    short_delay();
}

void show_zero(void) {
    disp[0] = SEG_MAP[0];
    disp[1] = SEG_MAP[0];
    disp[2] = SEG_MAP[0];
    disp[3] = SEG_MAP[0];
}

void show_error(void) {         
    disp[0] = SEG_E;
    disp[1] = SEG_r;
    disp[2] = SEG_r;
    disp[3] = SEG_BLANK;
}

void show_temp(int t) {
    int tens, units, tenth;

    if (t < 0)   t = 0;          
    if (t > 999) t = 999;

    tens  = t / 100;
    units = (t / 10) % 10;
    tenth = t % 10;

    if (tens == 0) disp[0] = SEG_BLANK;
    else           disp[0] = SEG_MAP[tens];

    disp[1] = SEG_MAP[units] & 0x7F;      
    disp[2] = SEG_MAP[tenth];
    disp[3] = SEG_C;
}


/* =====================================================
 * 3. ADC0 
 * ===================================================== */
void ADC0_init(void) {
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;         /* Step 4: clock to ADC0 */
    ADC0->SC2 &= ~ADC_SC2_ADTRG_MASK;          /* Step 5: software trigger */
    
    /* Step 6: clock div by 4, long sample time, single ended 12 bit, bus clock */
    ADC0->CFG1 = ADC_CFG1_ADIV(2) | ADC_CFG1_ADLSMP_MASK | ADC_CFG1_MODE(1) | ADC_CFG1_ADICLK(0);
}

int read_ADC0(void) {
    ADC0->SC1[0] = 0x1A; /* Step 7: start conversion on channel 26 (Internal Temp) */
    while(!(ADC0->SC1[0] & ADC_SC1_COCO_MASK)) { 
        /* Step 8: wait for conversion complete */
    }
    return ADC0->R[0];   /* Step 9: read conversion result */
}

float convertToTemp(int adc_val) {
    float v_temp = ((float)adc_val / 4096.0f) * 3.3f;
    float temp_c = 25.0f - ((v_temp - 0.716f) / 0.00162f);
    return temp_c - TEMP_OFFSET_C;
}


/* =====================================================
 * Init Shield
 * ===================================================== */
void Init_Shield(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK;

    /* 1. Buzzer (PTA1) */
    PORTA->PCR[1] = PORT_PCR_MUX(1);
    PTA->PDDR &= ~(1 << 1);

    /* 2. Switch A1(PTB3), A2(PTB10), A3(PTB11) */
		__disable_irq();     /* PORT_PCR_IRQC(0xA)  Falling-Edge Interrupt */
    PORTB->PCR[3]  = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0xA);
    PORTB->PCR[10] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0xA);
    PORTB->PCR[11] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0xA);
    PTB->PDDR &= ~((1 << 3) | (1 << 10) | (1 << 11));
    
    NVIC_EnableIRQ(PORTB_IRQn); /*  Interrupt port B */
		__enable_irq();
    /* 3. LED on shield */
    PORTD->PCR[0] = PIN_OUT_CFG; PORTD->PCR[1] = PIN_OUT_CFG;
    PORTD->PCR[2] = PIN_OUT_CFG; PORTD->PCR[3] = PIN_OUT_CFG;
    PTD->PDDR |= (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
    PTD->PSOR = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);

    /* 4. 7-Segment */
    PORTB->PCR[23] = PIN_OUT_CFG; PTB->PDDR |= (1 << 23); PTB->PSOR = (1 << 23);
    PORTC->PCR[3]  = PIN_OUT_CFG; PTC->PDDR |= (1 << 3);  PTC->PCOR = (1 << 3);
    PORTC->PCR[12] = PIN_OUT_CFG; PTC->PDDR |= (1 << 12); PTC->PCOR = (1 << 12);
}
void UART0_init(void) {
    SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;    /*  Clock  UART0 */
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;    /*  Clock  PORTB */
    
    /* PTB16  PTB17  UART0 (ALT3) */
    PORTB->PCR[16] = PORT_PCR_MUX(3);      /* UART0_RX */
    PORTB->PCR[17] = PORT_PCR_MUX(3);      /* UART0_TX */
    
    /*  Transmitter and Receiver  */
    UART0->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
    
    /* 
     * ????? Baud rate = 9600 
     * System Clock = 20.48 MHz ( SysTick 20480 = 1ms)
     * SBR = 20480000 / (16 * 9600) = 133 
     */
    UART0->BDH = 0;
    UART0->BDL = 133;
    UART0->C4 = 0x0F;                      /* Oversampling ratio = 16 */
    UART0->C1 = 0;
    
    /* Transmitter */
    UART0->C2 |= UART_C2_TE_MASK;
}
int transmit_count = 0;     
int transmit_timer = 0;      

/* =====================================================
 * Main
 * ===================================================== */
int main(void) {
    int mode;
    int pos = 0;                         
    int led_step = 0;
    int adc_val;
    float temp_c;

    int led_timer = 0, conv_timer = 0, buzzer_timer = 0;     

    Init_Shield();
    ADC0_init();
    UART0_init();

    mode = MODE_RESET;
    show_zero();
    led_step = 0;
    led_show(0x01);
    led_timer = 250;

    while (1) {
        display_7seg(pos, disp[pos]);
        pos++;
        if (pos >= 4) pos = 0;

        if (led_timer > 0)  led_timer--;
        if (conv_timer > 0) conv_timer--;
        if (buzzer_timer > 0) {
            buzzer_timer--;
            if (buzzer_timer == 0) buzzer_off();     
        }

        if (press1) {
            press1 = 0;
            buzzer_on();
            buzzer_timer = 250;
            mode = MODE_RESET;
            show_zero();
            led_step = 0;
            led_show(0x01);              
            led_timer = 50;
        }

        if (press2) {
            press2 = 0;
            if (mode == MODE_IDLE) {
                mode = MODE_RUN;
                conv_timer = 0;          
                PTD->PCOR = (1 << LED_BIT[0]);         
                short_delay();
                PTD->PCOR = (1 << LED_BIT[1]);
            }
        }

       if (press3) {
            press3 = 0;
            if (mode == MODE_RUN) {
                
                transmit_count = 100;    
                transmit_timer = 0;
                
                PTD->PSOR = (1 << LED_BIT[0]);         
                short_delay();
                PTD->PSOR = (1 << LED_BIT[1]);
                PTD->PCOR = (1 << LED_BIT[2]);         
                short_delay();
                PTD->PCOR = (1 << LED_BIT[3]);
                buzzer_on();
                buzzer_timer = 50;
            }
        } 

        if (transmit_count > 0) {
            if (transmit_timer > 0) {
                transmit_timer--; 
            } 
            else {
                int t_tens = temp10 / 100;
                int t_units = (temp10 / 10) % 10;
                int t_tenth = temp10 % 10;
                
                while(!(UART0->S1 & UART_S1_TDRE_MASK)); UART0->D = '0' + t_tens;
                while(!(UART0->S1 & UART_S1_TDRE_MASK)); UART0->D = '0' + t_units;
                while(!(UART0->S1 & UART_S1_TDRE_MASK)); UART0->D = '.';
                while(!(UART0->S1 & UART_S1_TDRE_MASK)); UART0->D = '0' + t_tenth;
                while(!(UART0->S1 & UART_S1_TDRE_MASK)); UART0->D = '\r';
                while(!(UART0->S1 & UART_S1_TDRE_MASK)); UART0->D = '\n';
                
                transmit_count--;     
                if (transmit_count > 0) {
                    transmit_timer = 100; 
                } else {
          
                    PTD->PCOR = (1 << LED_BIT[0]);         
                    PTD->PCOR = (1 << LED_BIT[1]);
                    PTD->PSOR = (1 << LED_BIT[2]);         
                    PTD->PSOR = (1 << LED_BIT[3]);
                }
            }
        }

       
        if (mode == MODE_RESET) {
            if (led_timer == 0) {        
                led_step++;
                if (led_step < 4) {
                    led_show(1 << led_step);
                    led_timer = 250;
                } else {
                    led_show(0x00);      
                    mode = MODE_IDLE;
                }
            }
        }

        if (mode == MODE_RUN) {
            if (conv_timer == 0) {
                adc_val = read_ADC0();           
                temp_c  = convertToTemp(adc_val);
                temp10  = (int)(temp_c * 10.0f );
                show_temp(temp10);
                conv_timer = 300;                
            }
        }

        
        delayMs(1);
    }
}