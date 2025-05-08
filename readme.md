
# Máquina de Caricias

Código y detalles de hardware de la máquina de caricias.

Mas detalles del proyecto [aca](https://docs.google.com/document/d/17xnVaVKrxB9BN-NF7uzriGS5WBftuUeRk3l-3zqq1ZQ/edit?tab=t.0#heading=h.jm6503qgkb28) y [aca](https://labsensacional.com/proyectos/maquina-de-caricias/).


## Otras notas

- pi x diamtetro x vueltas = cantida de cable recorrido
- microsteps se habilitan mandando m0, m1 y m2 a 5V. (2,4,8, ... 32) del DRV. Una vuelta sin microsteps es 200 sin microsteps, con los microsteps gana mucho torque
- seteo potencimetro del dvr. Arrancar girando todo en sentido horario y probando girarlo de a poco hasta que ande, chequeando tambien que no caliente mucho

## Cuestiones de Montaje
para la sujecion de motores recomiendo utilizar [esta pieza 3d](https://www.thingiverse.com/thing:3539999) o similes
 * medidas de tornillos M3 x 6 mm

A su vez en el proyecto se encuentran poleas desarrolladas para el caso.
* pensadas para utilizar un tornillo y tuerca  de 5/32 x 3/4 (20mm)
  se realizo una prueba en [poleas para cadenas de cortinas](https://www.thingiverse.com/thing:3147) blackout pero fue descartado por el error acumulado; dejando el diseño del mismo para futuras investigaciones 
