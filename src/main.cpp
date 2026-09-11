/*
 * Reloj HH:MM:SS con HDSP-2112 y ATmega8 + SCROLL VERTICAL de digitos
 * Sincronización GPS por puerto serial (GPRMC) - GMT -05 (Ecuador)
 *
 * Conexiones (Pines.csv):
 *   Puerto C: PC0-PC4 = A0-A4, PC5 = FL
 *   Puerto B: PB0-PB3 = D0-D3, PB4 = CE, PB6 = RESET
 *   Puerto D: PD2 = WR, PD3 = RD, PD4-PD7 = D4-D7
 *   UART (PD0=RX, PD1=TX) a 9600 baud (Entrada GPS NMEA)
 *
 * Timer1 CTC: 8 MHz / 256 = 31250 Hz, OCR1A = 31249 -> 1 interrupción/s.
 */

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include <EEPROM.h>

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
#define HDSP_CTRL_BRIGHT_13   0x06
#define HDSP_CTRL_BRIGHT_HIGH HDSP_CTRL_BRIGHT_100  /* alto: 100% */
#define HDSP_CTRL_BRIGHT_MED  0x03                  /* medio (~40%, entre 0x00 y 0x06) */
#define HDSP_CTRL_BRIGHT_LOW  HDSP_CTRL_BRIGHT_13   /* bajo: 13% */
#define HDSP_UDC_SLOT         0    /* slot UDC usado para la animacion */

/* ---------------------- Brillo por pulsante + EEPROM ---------------------- */
#define BRIGHT_BTN_PIN  PB5
#define BRIGHT_EEPROM_ADDR 0
/* ---------------------- Depuracion serial ----------------------
 * El UART (PD0/PD1, 9600) se comparte entre GPS y monitor USB: no se pueden
 * tener GPS-TX y adaptador USB-TX a la vez sobre PD0 (contienda de drivers).
 * Para ver estos mensajes, desconectar el TX del GPS (o poner jumper).
 * Ademas el TX del ATmega llega al RX del GPS: el volcado por segundo puede
 * confundir al modulo; poner GPS_DEBUG_RMC=0 si el GPS se comporta raro. */
#ifndef GPS_DEBUG_RMC
#define GPS_DEBUG_RMC 0   /* 1 = imprime [GPS RMC] en cada trama; 0 = solo [GPS SYNC] */
#endif

static void hdsp_write(uint8_t addr, uint8_t data); /* definida abajo */
static void read_gps_serial(void); /* definida abajo; se bombea tambien durante el scroll */
static const uint8_t bright_levels[3] = {
    HDSP_CTRL_BRIGHT_HIGH, HDSP_CTRL_BRIGHT_MED, HDSP_CTRL_BRIGHT_LOW
};
static const char *const bright_names[3] = { "ALTO", "MEDIO", "BAJO" };
static uint8_t bright_idx = 0;

static void bright_apply(uint8_t idx)
{
    bright_idx = idx % 3;
    hdsp_write(HDSP_ADDR_CTRL, bright_levels[bright_idx]);
}

static void bright_next(void)
{
    bright_apply((uint8_t)(bright_idx + 1));
    EEPROM.update(BRIGHT_EEPROM_ADDR, bright_idx);
    Serial.print(F("[BRIGHT] "));
    Serial.println(bright_names[bright_idx]);
}

/* Llamar en cada iteracion de loop(): detecta flanco alto->bajo con antirebote no bloqueante */
static void bright_poll_button(void)
{
    static uint8_t stable = HIGH;      /* ultimo estado estable */
    static uint8_t last_raw = HIGH;    /* ultima lectura cruda */
    static unsigned long last_change_ms = 0;
    uint8_t raw = (PINB & (1 << BRIGHT_BTN_PIN)) ? HIGH : LOW;

    if (raw != last_raw) {
        last_raw = raw;
        last_change_ms = millis();
        return;
    }
    if ((unsigned long)(millis() - last_change_ms) < 50) return; /* antirebote 50 ms */
    if (raw != stable) {
        stable = raw;
        if (stable == LOW) bright_next();  /* pulso de alto a bajo */
    }
}

/* ---------------------- Configuracion GPS ---------------------- */
uint8_t Nigual = 5;                  /* Intervalo en minutos para igualar la hora */
volatile bool gps_locked = false;    /* Estado de enganche GPS (true = 'A', false = 'V') */
bool gps_synced = false;             /* Indica si ya se realizo al menos una sincronizacion */
static unsigned long last_gps_sync_ms = 0; /* millis() de la ultima igualacion aceptada */

/* ---------------------- Reloj ---------------------- */
volatile uint8_t horas    = 12;      /* arranca en 12:00:00 */
volatile uint8_t minutos  = 0;
volatile uint8_t segundos = 0;
volatile uint8_t tick_1s  = 0;

/* ---------------------- Fuente 5x7 de digitos ---------------------- */
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

static uint8_t font_row[10][7];

static void font_build_rows(void)
{
    for (uint8_t d = 0; d < 10; d++) {
        for (uint8_t r = 0; r < 7; r++) {
            uint8_t row = 0;
            for (uint8_t c = 0; c < 5; c++) {
                if (font_col[d][c] & (1 << r)) {
                    row |= (1 << (4 - c));
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
    PORTB = (PORTB & 0xF0) | (data & 0x0F);
    PORTD = (PORTD & 0x0F) | (data & 0xF0);
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

static void udc_define(uint8_t slot, const uint8_t rows[7])
{
    hdsp_write(HDSP_ADDR_UDCADDR, slot);
    for (uint8_t r = 0; r < 7; r++) {
        hdsp_write(HDSP_ADDR_UDCRAM | r, rows[r]);
    }
}

static void hdsp_putc(uint8_t pos, uint8_t ch)
{
    hdsp_write(HDSP_ADDR_CHAR0 + pos, ch);
}

/* ---------------------- Scroll vertical de un digito ---------------------- */
static void scroll_digit(uint8_t pos, char oldc, char newc)
{
    if (oldc < '0' || oldc > '9' || newc < '0' || newc > '9') {
        hdsp_putc(pos, newc);
        return;
    }

    const uint8_t *fo = font_row[oldc - '0'];
    const uint8_t *fn = font_row[newc - '0'];

    for (uint8_t k = 1; k <= 6; k++) {
        uint8_t g[7];
        for (uint8_t i = 0; i < 7; i++) {
            g[i] = (i < 7 - k) ? fo[i + k]
                               : fn[i - (7 - k)];
        }
        udc_define(HDSP_UDC_SLOT, g);
        hdsp_putc(pos, 0x80 | HDSP_UDC_SLOT);
        /* Demora fraccionada: bombea la UART para no perder tramas GPS.
         * Antes un _delay_ms(100) bloqueante desbordaba el buffer RX de 64 B
         * (~67 ms a 9600 baud) y las tramas llegaban corruptas/incompletas. */
        for (uint8_t w = 0; w < 10; w++) {
            _delay_ms(10);
            read_gps_serial();
        }
    }
    hdsp_putc(pos, newc);
}

/* ---------------------- Lectura y Procesamiento GPS ---------------------- */
static char gps_buf[85];
static uint8_t gps_idx = 0;

static void parse_gprmc(char *sentence)
{
    /* Verificar que sea una trama GPRMC o GNRMC */
    if (strncmp(sentence, "$GPRMC", 6) != 0 && strncmp(sentence, "$GNRMC", 6) != 0) {
        return;
    }

    char *field;
    uint8_t field_idx = 0;
    char *time_str = NULL;
    char status = 'V';

    field = strtok(sentence, ",");
    while (field != NULL) {
        field_idx++;
        if (field_idx == 2) {
            time_str = field;   /* Campo 1: Hora UTC (hhmmss.ss) */
        } else if (field_idx == 3) {
            status = field[0];  /* Campo 2: Estado ('A' = Valido, 'V' = Invalido) */
            break;
        }
        field = strtok(NULL, ",");
    }

    /* Depuracion: mostrar el time_str crudo en cada trama RMC recibida */
#if GPS_DEBUG_RMC
    Serial.print(F("[GPS RMC] time="));
    if (time_str) Serial.println(time_str);
    else Serial.println(F("-"));
#endif

    /* Estado del fix: solo controla el parpadeo de ':' en loop()
     * ('A' = fijos, 'V' = titilan). La igualacion de hora se hace siempre,
     * incluso con 'V', porque el GPS mantiene hora UTC valida sin fix. */
    gps_locked = (status == 'A');

    /* Igualar cada Nigual minutos (por tiempo transcurrido) o en la primera trama,
     * con cualquier estado ('A' o 'V'). */
    unsigned long now = millis();
    bool interval_elapsed =
        (unsigned long)(now - last_gps_sync_ms) >= (unsigned long)Nigual * 60UL * 1000UL;
    if (!gps_synced || interval_elapsed) {
        if (time_str && strlen(time_str) >= 6) {
            uint8_t utc_h = (time_str[0] - '0') * 10 + (time_str[1] - '0');
            uint8_t utc_m = (time_str[2] - '0') * 10 + (time_str[3] - '0');
            uint8_t utc_s = (time_str[4] - '0') * 10 + (time_str[5] - '0');

            /* Ajuste de Huso Horario GMT -05 (Ecuador) */
            int8_t local_h = (int8_t)utc_h - 5;
            if (local_h < 0) {
                local_h += 24;
            }

            cli();
            horas    = (uint8_t)local_h;
            minutos  = utc_m;
            segundos = utc_s;
            sei();

            gps_synced = true;
            last_gps_sync_ms = now;

            /* Eco por serial para visualizar la hora leida del GPS en cada igualacion */
            Serial.print(F("[GPS SYNC] UTC "));
            if (utc_h < 10) Serial.print('0');
            Serial.print(utc_h);
            Serial.print(':');
            if (utc_m < 10) Serial.print('0');
            Serial.print(utc_m);
            Serial.print(':');
            if (utc_s < 10) Serial.print('0');
            Serial.print(utc_s);
            Serial.print(F(" -> Local "));
            if ((uint8_t)local_h < 10) Serial.print('0');
            Serial.print((uint8_t)local_h);
            Serial.print(':');
            if (utc_m < 10) Serial.print('0');
            Serial.print(utc_m);
            Serial.print(':');
            if (utc_s < 10) Serial.print('0');
            Serial.println(utc_s);
        }
    }
}

static void read_gps_serial(void)
{
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (gps_idx > 0) {
                gps_buf[gps_idx] = '\0';
                parse_gprmc(gps_buf);
                gps_idx = 0;
            }
        } else if (gps_idx < sizeof(gps_buf) - 1) {
            gps_buf[gps_idx++] = c;
        }
    }
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
    DDRB |= 0x5F;   /* PB5 queda como entrada (pulsante brillo) */
    DDRD |= 0xFC;

    PORTB |= (1 << HDSP_CE) | (1 << BRIGHT_BTN_PIN); /* CE idle + pull-up pulsante */
    PORTD |= (1 << HDSP_WR) | (1 << HDSP_RD);

    Serial.begin(9600);
    Serial.println(F("RELOJ HDSP2112"));

    font_build_rows();

    hdsp_reset();
    uint8_t saved = EEPROM.read(BRIGHT_EEPROM_ADDR);
    if (saved > 2) saved = 0;   /* EEPROM virgen (0xFF) u otro valor -> ALTO */
    bright_apply(saved);

    /* Hora inicial estatica: 12:00:00 */
    const char *ini = "12:00:00";
    for (uint8_t i = 0; i < 8; i++) hdsp_putc(i, ini[i]);

    timer1_init_1s();
    sei();
}

/* ---------------------- Loop ---------------------- */
void loop()
{
    /* Procesa tramas GPS que lleguen por el puerto Serial */
    read_gps_serial();

    bright_poll_button();   /* pulsante PB5: flanco alto->bajo cambia brillo (no bloqueante) */

    if (!tick_1s) return;
    tick_1s = 0;

    /* Construir cadena de tiempo "HH:MM:SS" */
    char nuevo[9];
    snprintf(nuevo, sizeof(nuevo), "%02u:%02u:%02u", horas, minutos, segundos);

    /* Titileo de los separadores ':' (posiciones 2 y 5 en índice 0) si no hay enganche GPS */
    if (!gps_locked) {
        if (segundos % 2 != 0) {
            nuevo[2] = ' ';
            nuevo[5] = ' ';
        }
    }

    static char viejo[9] = "12:00:00";

    /* Renderizar cambios en el display HDSP-2112 */
    for (int8_t pos = 7; pos >= 0; pos--) {
        if (nuevo[pos] == viejo[pos]) continue;

        if (nuevo[pos] == ':' || viejo[pos] == ':' || nuevo[pos] == ' ' || viejo[pos] == ' ') {
            hdsp_putc(pos, nuevo[pos]);          /* Los dos puntos o espacios no realizan scroll */
        } else {
            scroll_digit(pos, viejo[pos], nuevo[pos]);
        }
    }

    memcpy(viejo, nuevo, 9);
}