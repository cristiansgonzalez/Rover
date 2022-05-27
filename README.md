# Rover

El Rover esta compuesto por una armadura metalica, dos motores, dos cadenas, un Rasberry pi 2, un modulo de potencia Roboteq. Se mostrada graficamente como esta constituido el Rover.

Entradas y salidas del sistema maestro (Raspberry pi).
<img src="Imagenes/rasberry.png">

Entradas y salidas del sistema exclavo (Roboteq).
<img src="Imagenes/robteq.png">

La Raspberry Actúa como tarjeta master, es el cerebro de todo, se hacen todos los cálculos de control, recepción del control RF y acción sobre los motores, el Roboteq es un dispositivo esclavo, solo se le hacen peticiones de información, recolecta la información y posteriormente se envía por USB a la rapsberry Pi la cual está esperando la información.

<table class="default">
  <tr>
    <td><img src="Imagenes/1.png" width="400" height="341"></td>
    <td><img src="Imagenes/2.png" width="400" height="341"></td>
  </tr>
  
  <tr>
    <td><img src="Imagenes/3.png" width="400" height="341"></td>
    <td><img src="Imagenes/4.png" width="400" height="341"></td>
  </tr>
  
  <tr>
    <td><img src="Imagenes/5.png" width="400" height="341"></td>
    <td><img src="Imagenes/6.png" width="400" height="341"></td>
  </tr>
</table>

# Ejecutar el codigo en C++

Se descarga la carpeta `C++` y se ejecuta primero el makefile y despues si se ejecuta el codigo principal

```
$ make Makefile
$ cc Principal.c
```

# Ejecutar el codigo en python 

Se descarga la carpeta `Rover_python` y se ejecuta 
```
$ python codigo.py
```

## Autor

- Cristian González (<cristian-saul-66@hotmail.com>)
