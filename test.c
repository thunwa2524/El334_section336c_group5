#include "MK64F12.h"

#define PIN_OUT_CFG (PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK)

#define MODE_RESET   0
#define MODE_IDLE    1
#define MODE_RUN     2
#define MODE_FREEZE  3

const unsigned char SEG_MAP[10] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90};
const unsigned char DIG_MAP[4]  = {0x01, 0x02, 0x04, 0x08};

#define SEG_BLANK  0xFF
#define SEG_C      0xC6
#define SEG_E      0x86
#define SEG_r      0xAF

const int LED_BIT[4] = {1, 3, 2, 0};

unsigned char disp[4] = {0xC0, 0xC0, 0xC0, 0xC0};
int temp10 = 0;

volatile int press1 = 0;
volatile int press2 = 0;
volatile int press3 = 0;

void PORTB_IRQHandler(void); 
void short_delay(void);
void led_show(int mask);
void buzzer_on(void);
void buzzer_off(void);
void shiftOut(int data);
void display_7seg(int digit_pos, int seg);
void show_zero(void);
void show_error(void);
void show_temp(int t);
void delayMs(int n);
void delay_us(int us);
void Init_Shield(void);
void UART0_init(void);

void DHT11_init(void);
int read_DHT11(void);

void delayMs(int n) { 
    int i; 
    SysTick->LOAD = 20480 - 1; 
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;      
     
    for (i = 0; i < n; i++) 
        while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0) {  
        } 
     
    SysTick->CTRL = 0; 
}

void delay_us(int us) {
    volatile int i;
    for (i = 0; i < (us * 3); i++) {
        __NOP();
    }
}

void PORTB_IRQHandler(void) {
    if (PORTB->ISFR & (1 << 3)) {
        press1 = 1;
        PORTB->ISFR = (1 << 3); 
    }
    if (PORTB->ISFR & (1 << 10)) {
        press2 = 1;
        PORTB->ISFR = (1 << 10);
    }
    if (PORTB->ISFR & (1 << 11)) {
        press3 = 1;
        PORTB->ISFR = (1 << 11);
    }
}

void short_delay(void) {
    volatile int d;
    for (d = 0; d < 50; d++) __NOP();
}

void led_show(int mask) {
    int i;
    for (i = 0; i < 4; i++) {
        if (mask & (1 << i)) PTD->PCOR = (1 << LED_BIT[i]);
        else                 PTD->PSOR = (1 << LED_BIT[i]);
    }
}

void buzzer_on(void) {
    PTA->PDDR |= (1 << 1);       
    PTA->PCOR = (1 << 1);        
}

void buzzer_off(void) {
    PTA->PDDR &= ~(1 << 1);      
}

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

void DHT11_init(void) {
    PORTC->PCR[10] = PORT_PCR_MUX(1);
    PTC->PDDR |= (1 << 10);
    PTC->PSOR = (1 << 10);
}

int read_DHT11(void) {
    int byte_idx, bit_idx;
    unsigned char data[5] = {0, 0, 0, 0, 0};
    int time_out;

    PTC->PDDR |= (1 << 10);
    
    PTC->PCOR = (1 << 10);
    delayMs(18);
    
    PTC->PSOR = (1 << 10);
    delay_us(30);

    PTC->PDDR &= ~(1 << 10);
    
    time_out = 0; 
    while ((PTC->PDIR & (1 << 10)) && (time_out < 100)) { delay_us(1); time_out++; } 
    
    time_out = 0; 
    while (!(PTC->PDIR & (1 << 10)) && (time_out < 100)) { delay_us(1); time_out++; } 
    
    time_out = 0; 
    while ((PTC->PDIR & (1 << 10)) && (time_out < 100)) { delay_us(1); time_out++; } 

    for (byte_idx = 0; byte_idx < 5; byte_idx++) {
        for (bit_idx = 0; bit_idx < 8; bit_idx++) {
            
            time_out = 0;
            while (!(PTC->PDIR & (1 << 10)) && (time_out < 100)) { delay_us(1); time_out++; }
            
            time_out = 0;
            while ((PTC->PDIR & (1 << 10)) && (time_out < 100)) { delay_us(1); time_out++; }
            
            data[byte_idx] = data[byte_idx] << 1;
            
            if (time_out > 40) { 
                data[byte_idx] = data[byte_idx] | 1;
            }
        }
    }

    if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
        return (data[2] * 10) + data[3]; 
    }
    
    return -1; 
}

void Init_Shield(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK;

    PORTA->PCR[1] = PORT_PCR_MUX(1);
    PTA->PDDR &= ~(1 << 1);

    __disable_irq();     
    PORTB->PCR[3]  = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0xA);
    PORTB->PCR[10] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0xA);
    PORTB->PCR[11] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0xA);
    PTB->PDDR &= ~((1 << 3) | (1 << 10) | (1 << 11));
    
    NVIC_EnableIRQ(PORTB_IRQn); 
    __enable_irq();

    PORTD->PCR[0] = PIN_OUT_CFG; PORTD->PCR[1] = PIN_OUT_CFG;
    PORTD->PCR[2] = PIN_OUT_CFG; PORTD->PCR[3] = PIN_OUT_CFG;
    PTD->PDDR |= (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
    PTD->PSOR = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);

    PORTB->PCR[23] = PIN_OUT_CFG; PTB->PDDR |= (1 << 23); PTB->PSOR = (1 << 23);
    PORTC->PCR[3]  = PIN_OUT_CFG; PTC->PDDR |= (1 << 3);  PTC->PCOR = (1 << 3);
    PORTC->PCR[12] = PIN_OUT_CFG; PTC->PDDR |= (1 << 12); PTC->PCOR = (1 << 12);
}

void UART0_init(void) {
    SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;    
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;    
    
    PORTB->PCR[16] = PORT_PCR_MUX(3);      
    PORTB->PCR[17] = PORT_PCR_MUX(3);      
    
    UART0->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
    
    UART0->BDH = 0;
    UART0->BDL = 133;
    UART0->C4 = 0x0F;                      
    UART0->C1 = 0;
    
    UART0->C2 |= UART_C2_TE_MASK;
}

int transmit_count = 0;     
int transmit_timer = 0;     

int main(void) {
    int mode;
    int pos = 0;                         
    int led_step = 0;
    int new_temp;

    int led_timer = 0, conv_timer = 0, buzzer_timer = 0;     

    Init_Shield();
    DHT11_init();
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
                new_temp = read_DHT11();
                if (new_temp != -1) {
                    temp10 = new_temp;
                }
                show_temp(temp10);
                conv_timer = 300;
            }
        }

        delayMs(1);
    }
}
