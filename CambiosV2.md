# Cambios para la versión 2, a petición de usuarios.

## Desaparición del modo automático

Hay que quitar el modo automático, no es práctico. En su lugar se procederá de la siguiente forma:

    - En el encendido, consideramos que estamos en la posición 0.
    - Los pulsadores ahora serán:
        - Avanzar. Una pulsación de más de 100 msg. hará el recorrido completo configurado, a la velocidad condigurada de avance.
        - Retroceder. Retrocederá hasta el cero.
        - Marcha ahora ya no es marcha, sino paro. Detendrá el movimiento instantáneamente. Seguirá sirviendo para configurar el sistema.

    - El sistema mantendrá la posición mientras esté encendido, de modo que si paramos a mitad de recorrido, sabremos dónde estamos.
    - Más de 4 segundos pulsando retroceso permiten volver hacia atrás aunque estemos en la posición cero, para poder retroceder cuando no estemos en origen.
    