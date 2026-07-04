# Perfusor ESP32 (TTGO T-Display)

Firmware de control para perfusor con ESP32, pantalla ST7789, entradas tactiles capacitivas y motor paso a paso.

## Estado actual

- Compila correctamente con ESP-IDF 6.0.1.
- Motor con generacion de pulsos continua por GPTimer (sin comportamiento por bloques).
- Arquitectura dual-core para movimiento automatico: la secuencia de movimiento se ejecuta en worker task fijada a Core 1.
- Modo manual continuo (jog): al mantener pulsado ADEL/ATRAS se mueve de forma continua mientras dure la pulsacion.
- En modo marcha, el porcentaje se redibuja solo cuando cambia el numero mostrado.
- Cancelacion por pulsacion larga de MARCHA durante ciclo automatico.

## Hardware y pines

### LCD ST7789 (SPI)

- MOSI: GPIO19
- CLK: GPIO18
- CS: GPIO5
- DC: GPIO16
- RST: GPIO23
- BL: GPIO4

### Driver de motor paso a paso

- STEP: GPIO25
- DIR: GPIO26
- EN: GPIO27

Estos GPIO son aptos para salida en ESP32 clasico.

### Tactil capacitivo (touch_sens API nueva)

- ADEL: GPIO2 (TOUCH_ADV_CHAN)
- MARCHA: GPIO15 (TOUCH_RUN_CHAN)
- ATRAS: GPIO13 (TOUCH_BACK_CHAN)

## Interfaz de uso

### Pantalla principal

- Muestra porcentaje de posicion y contador de ciclos.
- Feedback visual de estado de los 3 botones tactiles.

### Acciones en pantalla principal

- ADEL mantenido: movimiento manual continuo hacia adelante.
- ATRAS mantenido: movimiento manual continuo hacia atras.
- MARCHA pulsacion de 1 a <2 s: ejecuta un ciclo automatico.
- MARCHA pulsacion >=4 s: entra a configuracion.

### Durante ciclo automatico

- Avance con rampa de aceleracion/deceleracion.
- Pausa configurable.
- Retroceso con rampa.
- Pulsacion larga de MARCHA (~1 s) cancela la operacion actual.

### Menu de configuracion

- Velocidad avance (mm/s)
- Distancia avance (mm)
- Velocidad retroceso (mm/s)
- Distancia retroceso (mm)
- Pausa tras avance (s)
- Acerca de
- Guardar y salir (NVS)
- Cancelar cambios

## Parametros y limites

- STEPS_PER_MM: 800.0
- Velocidad minima: 0.10 mm/s
- Velocidad maxima: 20.00 mm/s
- Distancia maxima: 999.9 mm
- Aceleracion: 5.0 mm/s^2

## Estructura del proyecto

- main/perfusorESP32.c: firmware principal (UI, touch, control motor, NVS)
- main/CMakeLists.txt: dependencias del componente principal
- CMakeLists.txt: proyecto ESP-IDF (MINIMAL_BUILD activado)

## Build y flash

### Recomendado en VS Code

Usar los comandos del extension ESP-IDF (Build / Flash / Monitor).

### En terminal (si el entorno esta exportado)

```bash
cd /home/eugenio/MEGA/Proyectos/Perfusor/Micro_LCD/perf_esp32
. /home/eugenio/.espressif/v6.0.1/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

Si `idf.py` no aparece en tu shell habitual, usa VS Code ESP-IDF o abre una terminal con el entorno exportado.

## Dependencias ESP-IDF relevantes

Definidas en `main/CMakeLists.txt`:

- driver
- esp_driver_gpio
- esp_driver_gptimer
- esp_driver_touch_sens
- esp_lcd
- esp_timer
- nvs_flash

## Notas tecnicas

- El proyecto usa API tactil nueva (`driver/touch_sens.h`) para evitar deprecaciones de API legacy.
- Puede haber avisos de include en IntelliSense aunque la compilacion real de ESP-IDF sea correcta; valida siempre con build.

## Cambios 2026/07/04
    Modificado la pulsación de marcha, es difícil acertar entre 1 y 2 sg. Ahora es una pulsación entre 100 ms. y 2sg. Además alterno los pines de avanzar y retroceder, ya que en la placa física queda mejor.
    
