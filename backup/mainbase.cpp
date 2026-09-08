/*
 * Prueba de display HDSP-2112 con ATmega8 (PlatformIO / VSCode)
 *
 * Conexiones (ver Pines.csv):
 *   Puerto C: PC0-PC4 = A0-A4, PC5 = FL
 *   Puerto B: PB0-PB3 = D0-D3, PB4 = CE, PB6 = RESET
 *   Puerto D: PD2 = WR, PD3 = RD, PD4-PD7 = D4-D7
 *   UART (PD0=RX, PD1=TX) a 9600 baud para monitoreo.
 *
 * Funcionamiento: muestra cada caracter ASCII imprimible (0x20-0x7E)
 * en los 8 digitos, cambiando cada 2 segundos, e informa por UART.
 */

#include <Arduino.h>
#include <util/delay.h>

/* ---------------------- Mapeo de pines (registros AVR) ---------------------- */
#define HDSP_FL   PC5
#define HDSP_CE   PB4
#define HDSP_WR   PD2
#define HDSP_RD   PD3
#define HDSP_RST  PB6

/* Direcciones de las secciones de memoria del HDSP-2112 (segun hoja de datos):
 *   Control Word Register : FL=1, A4=1, A3=0            -> 0x10
 *   Character RAM         : FL=1, A4=1, A3=1 + A2..A0   -> 0x18 + digito
 */
#define HDSP_ADDR_CTRL   0x10
#define HDSP_ADDR_CHAR0  0x18   /* digito 0 = izquierdo, 7 = derecho */

/* Control Word: bits 0-2 = brillo (000 = 100%), resto en 0 = operacion normal */
#define HDSP_CTRL_BRIGHT_100  0x00
#define HDSP_CTRL_BRIGHT_53  0x02
#define HDSP_CTRL_BRIGHT_13  0x06

/* ----------------------------- Funciones ----------------------------------- */

/* Coloca A0-A4 y FL en el puerto C (FL siempre alto salvo acceso a Flash RAM) */
static void hdsp_set_address(uint8_t addr)
{
    PORTC = (1 << HDSP_FL) | (addr & 0x1F);
}

/* Coloca el dato en el bus D0-D7 (partido entre PB0-3 y PD4-7) */
static void hdsp_set_data(uint8_t data)
{
    PORTB = (PORTB & 0xF0) | (data & 0x0F);       /* D0-D3 */
    PORTD = (PORTD & 0x0F) | (data & 0xF0);       /* D4-D7 */
}

/* Ciclo de escritura: CE bajo + WR bajo (ver diagrama de tiempos, fig. 3) */
static void hdsp_write(uint8_t addr, uint8_t data)
{
    hdsp_set_address(addr);
    hdsp_set_data(data);
    _delay_us(1);                       /* tACS: setup de direccion */

    PORTB &= ~(1 << HDSP_CE);           /* CE = 0 */
    PORTD &= ~(1 << HDSP_WR);           /* WR = 0 */
    _delay_us(1);                       /* tW: write activo (min 100 ns) */
    PORTD |=  (1 << HDSP_WR);           /* WR = 1 */
    PORTB |=  (1 << HDSP_CE);           /* CE = 1 */
    _delay_us(1);                       /* tCER: recuperacion */
}

/* Reset del display: RST bajo, luego alto y esperar 3 ciclos de reloj (>110 us) */
static void hdsp_reset(void)
{
    PORTB &= ~(1 << HDSP_RST);          /* RST = 0 */
    _delay_ms(10);
    PORTB |=  (1 << HDSP_RST);          /* RST = 1 */
    _delay_ms(2);                       /* margen de seguridad (min 110 us) */
}

/* Escribe el mismo caracter en los 8 digitos */
static void hdsp_show_char_all(uint8_t ch)
{
    for (uint8_t i = 0; i < 8; i++) {
        hdsp_write(HDSP_ADDR_CHAR0 + i, ch);
    }
}

/* --------------------------------- Setup ----------------------------------- */
void setup()
{
    /* Configurar pines como salida */
    DDRC  = 0x3F;                        /* PC0-PC5 (A0-A4, FL) */
    DDRB |= 0x5F;                        /* PB0-PB4 (D0-D3, CE), PB6 (RST) */
    DDRD |= 0xFC;                        /* PD2-PD7 (WR, RD, D4-D7) */

    /* Estado inactivo de las lineas de control */
    PORTB |= (1 << HDSP_CE);             /* CE = 1 */
    PORTD |= (1 << HDSP_WR) | (1 << HDSP_RD);  /* WR = 1, RD = 1 */

    /* UART a 9600 baud para monitoreo */
    Serial.begin(9600);
    Serial.println(F("=== Prueba HDSP-2112 / ATmega8 ==="));

    /* Secuencia de inicializacion del display */
    hdsp_reset();
    hdsp_write(HDSP_ADDR_CTRL, HDSP_CTRL_BRIGHT_13);   /* brillo 100% */

    Serial.println(F("Display inicializado. Iniciando barrido ASCII..."));
}

/* ---------------------------------- Loop ----------------------------------- */
void loop()
{
    static uint8_t ch = 0x20;            /* primer ASCII imprimible: espacio */

    hdsp_show_char_all(ch);

    Serial.print(F("Mostrando: '"));
    if (ch == 0x20) {
        Serial.print(F("<ESPACIO>"));
    } else {
        Serial.write(ch);
    }
    Serial.print(F("'  (0x"));
    Serial.print(ch, HEX);
    Serial.println(F(")"));

    ch++;
    if (ch > 0x7E) {                     /* ultimo ASCII imprimible: '~' */
        ch = 0x20;
        Serial.println(F("--- Ciclo completo, reiniciando ---"));
    }

    delay(2000);                         /* cambio cada 2 segundos */
}
