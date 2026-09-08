/*
 * Reloj HH:MM:SS con HDSP-2112 y ATmega8 + SCROLL VERTICAL de digitos
 *
 * Conexiones (Pines.csv):
 *   Puerto C: PC0-PC4 = A0-A4, PC5 = FL
 *   Puerto B: PB0-PB3 = D0-D3, PB4 = CE, PB6 = RESET
 *   Puerto D: PD2 = WR, PD3 = RD, PD4-PD7 = D4-D7
 *   UART (PD0=RX, PD1=TX) a 9600 baud.
 *
 * Timer1 CTC: 8 MHz / 256 = 31250 Hz, OCR1A = 31249 -> 1 interrupcion/s.
 *
 * El scroll vertical no es posible con la fuente ASCII fija en ROM,
 * por eso los glifos intermedios se construyen en la UDC RAM
 * (caracteres definidos por el usuario, 16 disponibles).
 * Cada transicion usa el slot UDC 0 con 6 cuadros intermedios
 * (k = 1..6) mostrados cada 100 ms; el cuadro k muestra las filas
 * 7-k superiores del digito viejo desplazadas + las k filas del
 * nuevo entrando por abajo. El cuadro final se escribe como ASCII.
 */

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/* ---------------------- Mapeo de pines ---------------------- */
#define HDSP_FL   PC5
#define HDSP_CE   PB4
#define HDSP_WR   PD2
#define HDSP_RD   PD3
#define HDSP_RST  PB6

/* Direcciones de memoria del HDSP-2112 */
#define HDSP_ADDR_CTRL      0x10   /* Control Word: FL=1 A4=1 A3=0 */
#define HDSP_ADDR_UDCADDR   0x00   /* UDC Address Register: FL=1 A4=0 A3=0 */
#define HDSP_ADDR_UDCRAM    0x08   /* UDC RAM: FL=1 A4=0 A3=1 + fila(A2..A0) */
#define HDSP_ADDR_CHAR0     0x18   /* Character RAM: FL=1 A4=1 A3=1 + digito */

#define HDSP_CTRL_BRIGHT_100  0x00
#define HDSP_CTRL_BRIGHT_13  0x06
#define HDSP_UDC_SLOT         0    /* slot UDC usado para la animacion */

/* ---------------------- Reloj ---------------------- */
volatile uint8_t horas    = 12;    /* arranca en 12:00:00 */
volatile uint8_t minutos  = 0;
volatile uint8_t segundos = 0;
volatile uint8_t tick_1s  = 0;

/* ---------------------- Fuente 5x7 de digitos ---------------------- */
/* Columnas (1 byte por columna, bit0 = fila superior), estilo clasico */
static const uint8_t font_col[10][5] = {
    {0x3E, 0x51, 0x49, 0x45, 0x3E},   /* 0 */
    {0x00, 0x42, 0x7F, 0x40, 0x00},   /* 1 */
    {0x42, 0x61, 0x51, 0x49, 0x46},   /* 2 */
    {0x21, 0x41, 0x45, 0x4B, 0x31},   /* 3 */
    {0x18, 0x14, 0x12, 0x7F, 0x10},   /* 4 */
    {0x27, 0x45, 0x45, 0x45, 0x39},   /* 5 */
    {0x3C, 0x4A, 0x49, 0x49, 0x30},   /* 6 */
    {0x01, 0x71, 0x09, 0x05, 0x03},   /* 7 */
    {0x36, 0x49, 0x49, 0x49, 0x36},   /* 8 */
    {0x06, 0x49, 0x49, 0x29, 0x1E},   /* 9 */
};

/* Misma fuente transpuesta a FILAS para la UDC RAM:
 * fila[r] con D4 = columna izquierda ... D0 = columna derecha */
static uint8_t font_row[10][7];

static void font_build_rows(void)
{
    for (uint8_t d = 0; d < 10; d++) {
        for (uint8_t r = 0; r < 7; r++) {
            uint8_t row = 0;
            for (uint8_t c = 0; c < 5; c++) {
                if (font_col[d][c] & (1 << r)) {
                    row |= (1 << (4 - c));     /* D4 = col izquierda */
                }
            }
            font_row[d][r] = row;
        }
    }
}

/* ---------------------- Funciones del display ---------------------- */
static void hdsp_set_address(uint8_t addr)
{
    PORTC = (1 << HDSP_FL) | (addr & 0x1F);
}

static void hdsp_set_data(uint8_t data)
{
    PORTB = (PORTB & 0xF0) | (data & 0x0F);   /* D0-D3 */
    PORTD = (PORTD & 0x0F) | (data & 0xF0);   /* D4-D7 */
}

static void hdsp_write(uint8_t addr, uint8_t data)
{
    hdsp_set_address(addr);
    hdsp_set_data(data);
    _delay_us(1);

    PORTB &= ~(1 << HDSP_CE);
    PORTD &= ~(1 << HDSP_WR);
    _delay_us(1);
    PORTD |=  (1 << HDSP_WR);
    PORTB |=  (1 << HDSP_CE);
    _delay_us(1);
}

static void hdsp_reset(void)
{
    PORTB &= ~(1 << HDSP_RST);
    _delay_ms(10);
    PORTB |=  (1 << HDSP_RST);
    _delay_ms(2);
}

/* Define un caracter 5x7 en un slot de la UDC RAM */
static void udc_define(uint8_t slot, const uint8_t rows[7])
{
    hdsp_write(HDSP_ADDR_UDCADDR, slot);            /* direccion del slot */
    for (uint8_t r = 0; r < 7; r++) {
        hdsp_write(HDSP_ADDR_UDCRAM | r, rows[r]);  /* fila r en D0-D4 */
    }
}

/* Escribe un caracter (ASCII o UDC) en el digito 'pos' (0 = izquierdo) */
static void hdsp_putc(uint8_t pos, uint8_t ch)
{
    hdsp_write(HDSP_ADDR_CHAR0 + pos, ch);
}

/* ---------------------- Scroll vertical de un digito ---------------------- */
/* 6 cuadros intermedios (100 ms cada uno): el nuevo digito entra desde
 * abajo desplazando al viejo hacia arriba. El cuadro final es ASCII. */
static void scroll_digit(uint8_t pos, char oldc, char newc)
{
    const uint8_t *fo = font_row[oldc - '0'];
    const uint8_t *fn = font_row[newc - '0'];

    for (uint8_t k = 1; k <= 6; k++) {
        uint8_t g[7];
        for (uint8_t i = 0; i < 7; i++) {
            g[i] = (i < 7 - k) ? fo[i + k]      /* parte alta: digito viejo */
                               : fn[i - (7 - k)]; /* parte baja: digito nuevo */
        }
        udc_define(HDSP_UDC_SLOT, g);
        hdsp_putc(pos, 0x80 | HDSP_UDC_SLOT);   /* D7=1 -> usa UDC */
        _delay_ms(100);
    }
    hdsp_putc(pos, newc);                        /* cuadro final: ASCII ROM */
}

/* ---------------------- Timer1: 1 interrupcion por segundo ---------------------- */
void timer1_init_1s(void)
{
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS12);         /* CTC, prescaler 256 */
    OCR1A  = 31249;                              /* 1 s exacto a 8 MHz */
    TCNT1  = 0;
    TIMSK |= (1 << OCIE1A);
}

ISR(TIMER1_COMPA_vect)
{
    segundos++;
    if (segundos >= 60) {
        segundos = 0;
        minutos++;
        if (minutos >= 60) {
            minutos = 0;
            horas++;
            if (horas >= 24) horas = 0;
        }
    }
    tick_1s = 1;
}

/* ---------------------- Setup ---------------------- */
void setup()
{
    DDRC  = 0x3F;
    DDRB |= 0x5F;
    DDRD |= 0xFC;

    PORTB |= (1 << HDSP_CE);
    PORTD |= (1 << HDSP_WR) | (1 << HDSP_RD);

    Serial.begin(9600);
    Serial.println(F("=== Reloj HDSP-2112 con scroll vertical ==="));

    font_build_rows();

    hdsp_reset();
    hdsp_write(HDSP_ADDR_CTRL, HDSP_CTRL_BRIGHT_13);

    /* Hora inicial estatica: 12:00:00 con ':' en las posiciones 2 y 5 */
    const char *ini = "12:00:00";
    for (uint8_t i = 0; i < 8; i++) hdsp_putc(i, ini[i]);

    Serial.println(F("Reloj iniciado en 12:00:00"));

    timer1_init_1s();
    sei();
}

/* ---------------------- Loop ---------------------- */
void loop()
{
    static char viejo[9] = "12:00:00";

    if (!tick_1s) return;
    tick_1s = 0;

    /* Nuevo estado del reloj como cadena "HH:MM:SS" */
    char nuevo[9];
    snprintf(nuevo, sizeof(nuevo), "%02u:%02u:%02u",
             horas, minutos, segundos);

    /* Recorre los 8 digitos; los que cambiaron hacen scroll.
     * Se hace de derecha a izquierda (segundos primero), como los
     * relojes de estacion: cascada suave cuando ruedan los minutos. */
    for (int8_t pos = 7; pos >= 0; pos--) {
        if (nuevo[pos] == viejo[pos]) continue;
        if (nuevo[pos] == ':' || viejo[pos] == ':') {
            hdsp_putc(pos, nuevo[pos]);          /* los ':' no hacen scroll */
        } else {
            scroll_digit(pos, viejo[pos], nuevo[pos]);
        }
    }

    memcpy(viejo, nuevo, 9);

    /* Depuracion por UART */
    Serial.println(nuevo);
}