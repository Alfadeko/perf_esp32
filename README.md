# Perfusor ESP32 (TTGO T-Display)

Firmware de control para perfusor con ESP32, pantalla ST7789, entradas tactiles capacitivas y motor paso a paso.

## Estado actual

- Compila correctamente con ESP-IDF 6.0.1.
- Motor con generacion de pulsos continua por GPTimer y rampa de aceleracion/deceleracion.
- Control principal basado en posicion: el equipo arranca asumiendo posicion 0 y mantiene la posicion actual mientras siga encendido.
- El boton central ya no lanza un ciclo automatico: ahora actua como PARO instantaneo durante cualquier movimiento y sigue sirviendo para entrar y usar configuracion.
- La pantalla principal prioriza un contador grande de llegadas al final de recorrido (contando como llegada cuando se alcanza la zona de 5 mm previa al final).

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

- ADEL: GPIO13 (TOUCH_ADV_CHAN)
- PARO: GPIO15 (TOUCH_RUN_CHAN)
- ATRAS: GPIO2 (TOUCH_BACK_CHAN)

## Interfaz de uso

### Pantalla principal

- Muestra porcentaje de posicion y, como elemento principal, el contador de llegadas al final de recorrido.
- Feedback visual de estado de los 3 botones tactiles.

### Acciones en pantalla principal

- ADEL pulsacion >100 ms: avanza hasta el final del recorrido configurado a la velocidad de avance.
- ATRAS pulsacion >100 ms: retrocede hasta la posicion 0 a la velocidad de retroceso.
- PARO durante movimiento: detiene el avance o retroceso de forma inmediata, conservando la posicion alcanzada.
- PARO pulsacion >=4 s: entra a configuracion.

### Configuracion disponible

- Velocidad avance (mm/s)
- Recorrido total (mm)
- Velocidad retroceso (mm/s)
- Acerca de
- Guardar y salir (NVS)
- Cancelar cambios

## Reglas de posicion

- Al encender, el firmware considera que la posicion actual es 0.
- La posicion se actualiza segun los pasos realmente ejecutados, de modo que un PARO a mitad de recorrido conserva la referencia interna.
- Si el recorrido total configurado vale 0 mm, ADEL no produce movimiento.

## Parametros y limites

- STEPS_PER_MM: 800.0
- Margen para considerar "final alcanzado": 5.0 mm antes del final configurado
- Velocidad minima: 0.10 mm/s
- Velocidad maxima: 20.00 mm/s
- Distancia maxima: 999.9 mm
- Aceleracion: 5.0 mm/s^2

## Valores por defecto

- Recorrido total: 112.0 mm
- Velocidad avance: 0.60 mm/s
- Velocidad retroceso: 10.00 mm/s

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
- La ejecucion del movimiento sigue usando worker task en Core 1, pero ya no existe una secuencia de ciclo automatico; ahora cada accion ordena un unico desplazamiento posicional.

## Cambios 2026/07/06

- Eliminado el modo automatico.
- ADEL pasa a ejecutar el recorrido restante hasta el final configurado.
- ATRAS pasa a volver siempre al cero logico.
- MARCHA pasa a ser PARO instantaneo.
- La pantalla principal vuelve a priorizar el contador de llegadas al final de recorrido.
- El conteo de llegadas usa un umbral de 5 mm antes del final configurado.
- Valores por defecto actualizados: 112 mm, 0.60 mm/s (avance) y 10.00 mm/s (retroceso).
    
