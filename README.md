Propongo crear una carpeta `tests` para generar código básico de cada
componente del carrito por aparte. Dentro de `tests` se crea una carpeta
para cada componente, por ejemplo `tests/uart`.
Abrir un branch para cada nuevo test, opcionalmente con el formato `nombrecomponente-test`.
Hacer merge al `main` solo hasta que se haya completado el test.

Cuando se haya verificado que un componente funciona, abrir un branch para implementarlo
en `main.ino`. Hacer merge al `main` solo cuando se haya probado que ese componente funciona sin
afectar a ninguna otra parte del código.