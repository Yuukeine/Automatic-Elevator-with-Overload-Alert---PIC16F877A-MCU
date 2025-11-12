#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

// CONFIGURATION 
#pragma config FOSC = HS       
#pragma config WDTE = OFF     
#pragma config PWRTE = OFF    
#pragma config BOREN = OFF  
#pragma config LVP  = OFF     
#pragma config CPD  = OFF       
#pragma config WRT  = OFF       
#pragma config CP   = OFF       
#pragma config DEBUG = OFF

#define _XTAL_FREQ 8000000UL   // 8 MHz

// 7-SEG
const uint8_t seven_seg_digits[10] = {
    0xC0,  
    0xF9,  
    0xA4,  
    0xB0,  
    0x99,  
    0x92,  
    0x82,  
    0xF8,  
    0x80,  
    0x90   
};

// HX711 
// PORTE: RE0 = DOUT (input), RE1 = SCK (output)
#define HX711_DOUT   PORTEbits.RE0
#define HX711_SCK    PORTEbits.RE1

static long HX711_OFFSET_RAW = 0L;
static float HX711_SCALE     = 0.0f;
#define WEIGHT_THRESHOLD_KG  5.0f   // 5kg
static long THRESHOLD_RAW    = 0L;

// VISITOR FSM STATES & GLOBALS 
typedef enum {
    STATE_IDLE = 0,
    STATE_S1_TRIGGERED,
    STATE_S2_TRIGGERED,
    STATE_WAIT_RELEASE
} state_t;

volatile uint8_t visitor_count = 0;
volatile state_t fsm_state = STATE_IDLE;

// MOVING-AVERAGE FILTER SETUP
#define MA_SIZE 8
static long ma_buffer[MA_SIZE];
static uint8_t ma_index   = 0;
static bool ma_filled     = false;

// FUNCTION PROTOTYPES
void init_pins(void);
void init_hx711(void);
void display_digit(uint8_t d);
bool read_IR1(void);
bool read_IR2(void);
bool read_IR3(void);
bool read_button(void);
void process_fsm(void);
void servo_open_once(void);
void servo_close_once(void);
long hx711_read_raw(void);
long hx711_read_raw_ma(void);
void calibrate_hx711(void);
bool load_overload(void);

// MAIN
void main(void) {
    init_pins();
    init_hx711();
    calibrate_hx711();
    display_digit(visitor_count);

    bool ir3_prev = false;
    bool btn_prev = false;
    bool waiting_close = false;

    while (1) {
        bool weight_overloaded = load_overload();
        bool load_or_count_overload = (visitor_count > 5) || weight_overloaded;

        // Edge-detect IR3 or button
        bool ir3_now = read_IR3();
        bool btn_now = read_button();

        if (!load_or_count_overload) {
            if ((ir3_now && !ir3_prev) || (btn_now && !btn_prev)) {
                servo_open_once();
                if (visitor_count > 5) {
                    waiting_close = true;
                } else {
                    servo_close_once();
                }
            }
        }
        ir3_prev = ir3_now;
        btn_prev = btn_now;

        process_fsm();
        display_digit(visitor_count);

        // Overload LED - RD3
        PORTDbits.RD3 = load_or_count_overload ? 1 : 0;

        // Buzzer - RD2
        if (load_or_count_overload) {
            PORTDbits.RD2 = 1; __delay_us(250);
            PORTDbits.RD2 = 0; __delay_us(250);
        } else {
            PORTDbits.RD2 = 0;
        }

        __delay_ms(10);
    }
}

// INIT PINS
void init_pins(void) {
    ADCON1 = 0x07;   
    CMCON  = 0x07;  

    // IR modules - RC2 (IR1), RC1 (IR2), RC0 (IR3)
    TRISCbits.TRISC2 = 1;
    TRISCbits.TRISC1 = 1;
    TRISCbits.TRISC0 = 1;

    // 7-segment - RB0-RB6
    TRISB = 0x80;    
    PORTB = 0xFF;    

    // Push-button - RD4 (active LOW)
    TRISDbits.TRISD4 = 1;

    // LED - RD3
    TRISDbits.TRISD3 = 0;
    PORTDbits.RD3 = 0;

    // Buzzer - RD2
    TRISDbits.TRISD2 = 0;
    PORTDbits.RD2 = 0;

    // HX711 ? RE0 (DOUT in), RE1 (SCK out)
    TRISEbits.TRISE0 = 1;
    TRISEbits.TRISE1 = 0;
    PORTEbits.RE1   = 0;

    // Servo ?- RE2 (PWM out)
    TRISEbits.TRISE2 = 0;
    PORTEbits.RE2   = 0;
}

void init_hx711(void) {
    PORTEbits.RE1 = 0;
}

// DISPLAY DIGIT
void display_digit(uint8_t d) {
    if (d > 9) d = 0;
    PORTB = seven_seg_digits[d];
}

// READ INPUTS
bool read_IR1(void) { return (PORTCbits.RC2 == 0); }
bool read_IR2(void) { return (PORTCbits.RC1 == 0); }
bool read_IR3(void) { return (PORTCbits.RC0 == 0); }
bool read_button(void) { return (PORTDbits.RD4 == 0); }

// FSM
void process_fsm(void) {
    static uint16_t timeout_counter = 0;
    const uint16_t TIMEOUT_LIMIT = 500;
    bool ir1 = read_IR1();
    bool ir2 = read_IR2();

    switch (fsm_state) {
        case STATE_IDLE:
            if (ir1 && !ir2) { timeout_counter = 0; fsm_state = STATE_S1_TRIGGERED; }
            else if (ir2 && !ir1) { timeout_counter = 0; fsm_state = STATE_S2_TRIGGERED; }
            break;

        case STATE_S1_TRIGGERED:
            if (ir2) {
                visitor_count = (visitor_count < 9) ? visitor_count + 1 : 0;
                fsm_state = STATE_WAIT_RELEASE;
            } else if (++timeout_counter >= TIMEOUT_LIMIT) {
                fsm_state = STATE_IDLE;
            }
            break;

        case STATE_S2_TRIGGERED:
            if (ir1) {
                visitor_count = (visitor_count > 0) ? visitor_count - 1 : 9;
                fsm_state = STATE_WAIT_RELEASE;
            } else if (++timeout_counter >= TIMEOUT_LIMIT) {
                fsm_state = STATE_IDLE;
            }
            break;

        case STATE_WAIT_RELEASE:
            if (!ir1 && !ir2) {
                fsm_state = STATE_IDLE;
            }
            break;

        default:
            fsm_state = STATE_IDLE;
            break;
    }
}

// SERVO CONTROL
void servo_open_once(void) {
    for (uint8_t i = 0; i < 50; i++) {
        PORTEbits.RE2 = 1; __delay_us(2000);
        PORTEbits.RE2 = 0; __delay_ms(18);
    }
}

void servo_close_once(void) {
    for (uint8_t i = 0; i < 50; i++) {
        PORTEbits.RE2 = 1; __delay_us(1000);
        PORTEbits.RE2 = 0; __delay_ms(19);
    }
}

// HX711 RAW READ
long hx711_read_raw(void) {
    unsigned long count = 0;
    while (HX711_DOUT) { }
    for (uint8_t i = 0; i < 24; i++) {
        HX711_SCK = 1; __delay_us(1);
        count <<= 1;
        HX711_SCK = 0; __delay_us(1);
        if (HX711_DOUT) count |= 1;
    }
    // extra clock-pulse
    HX711_SCK = 1; __delay_us(1);
    HX711_SCK = 0; __delay_us(1);
    if (count & 0x800000UL) count |= 0xFF000000UL;
    return (long)count;
}

// MOVING-AVERAGE
long hx711_read_raw_ma(void) {
    long raw = hx711_read_raw();
    ma_buffer[ma_index] = raw;
    ma_index = (ma_index + 1) % MA_SIZE;
    if (ma_index == 0) ma_filled = true;
    unsigned long sum = 0;
    uint8_t cnt = ma_filled ? MA_SIZE : ma_index;
    for (uint8_t i = 0; i < cnt; i++) sum += (unsigned long)ma_buffer[i];
    return (long)(sum / cnt);
}

// CALIBRATION
void calibrate_hx711(void) {
    const uint8_t N_SAMPLES = 32;
    long sum0 = 0, sum1 = 0;
    for (uint8_t i = 0; i < N_SAMPLES; i++) {
        sum0 += hx711_read_raw(); __delay_ms(100);
    }
    HX711_OFFSET_RAW = sum0 / N_SAMPLES;
    for (uint8_t i = 0; i < N_SAMPLES; i++) {
        sum1 += hx711_read_raw(); __delay_ms(100);
    }
    long raw1 = sum1 / N_SAMPLES;
    HX711_SCALE = (float)(raw1 - HX711_OFFSET_RAW);
    THRESHOLD_RAW = HX711_OFFSET_RAW + (long)(WEIGHT_THRESHOLD_KG * HX711_SCALE);
    for (uint8_t i = 0; i < MA_SIZE; i++) {
        ma_buffer[i] = HX711_OFFSET_RAW;
    }
    ma_filled = true;
    ma_index = 0;
}

// OVERLOAD CHECK 
bool load_overload(void) {
    long filtered = hx711_read_raw_ma();
    return (filtered > THRESHOLD_RAW);
}
