# AGENTS.md

Firmware: reloj HH:MM:SS en HDSP-2112 + ATmega8, sincronizado por GPS NMEA (GPRMC/GNRMC).

## Comandos
- Compilar: `pio run -e ATmega8`
- Monitor serial (GPS entra a 9600): `pio device monitor -b 9600`
- Fuses/clock: RC interno 8 MHz (`board_build.f_cpu`, lfuse `0xE4` en `platformio.ini`). No cambiar sin recalcular `OCR1A=31249` en `timer1_init_1s()`.
- Subida: `upload_protocol = usbasp` con `upload_flags = -B 4`.

## Arquitectura (`src/main.cpp`, único fuente)
- Display por bit-bang directo a puertos (PC0-5, PB0-4/6, PD2-7); tiempos con `_delay_us/_delay_ms`, no usar `delay()` dentro de `scroll_digit()`.
- Base de tiempo: Timer1 CTC 1 Hz (`ISR(TIMER1_COMPA_vect)`) incrementa `horas/minutos/segundos` (`volatile`); acceso desde `loop()`/`parse_gprmc()` con `cli()/sei()`.
- GPS: `loop()` → `read_gps_serial()` → `parse_gprmc()`. Solo tramas `$GPRMC/$GNRMC`. Hora UTC `hhmmss` → local GMT-5 con cualquier estado (`A` o `V`); el estado solo controla `:` (`A` = fijos, `V` = titilan vía `gps_locked`).
- Igualación: primera trama válida + luego cada `Nigual` (min, default 5) por tiempo transcurrido con `millis()` (`last_gps_sync_ms`). No usar condición sobre el reloj local (`minutos % N == 0 && segundos == 0`): nunca coincide con la llegada asíncrona a 1 Hz.
- Debug GPS por serial a 9600 en cada igualación: `[GPS SYNC] UTC hh:mm:ss -> Local hh:mm:ss`. El mismo UART (PD0/PD1) se comparte entre GPS y monitor/USB.
- UART compartida: GPS-TX y USB-TX no pueden ir juntos a PD0 (contienda). Para ver logs, desconectar TX del GPS. El TX del ATmega llega al RX del GPS: si el módulo se comporta raro, poner `GPS_DEBUG_RMC=0` (solo `[GPS SYNC]` cada 5 min).
- Buffer RX ampliado a 256 B vía `-DSERIAL_RX_BUFFER_SIZE=256` en `platformio.ini` (una RMC ~70 B no cabe en los 64 B por defecto + resto de sentencias NMEA). `scroll_digit()` fracciona su demora en 10×10 ms bombeando `read_gps_serial()`; no volver a `_delay_ms(100)` bloqueante (desborda el buffer en ~67 ms a 9600 baud).
- Brillo: pulsante en PB5 (entrada con pull-up, activo a GND) → flanco alto-bajo rota ALTO→MEDIO→BAJO (`bright_levels` = `0x00/0x03/0x06` en Control Word). Polling no bloqueante con antirebote 50 ms vía `millis()` en `bright_poll_button()`, llamado en `loop()` antes del `return` por `tick_1s`. Índice persistido en EEPROM addr 0 (`EEPROM.update`, validado `>2 → 0`); `hdsp_set_data()` preserva PB5 (`PORTB & 0xF0`), no quitar la máscara.

## Convenciones
- `platformio.ini` es la única config de build; no hay tests (`test/` vacío), ni lint/format.
- Fuente de dígitos 5x7 en `font_col`, convertida a filas en `font_build_rows()`; scroll vertical vía slot UDC 0 (`0x80 | slot`).
