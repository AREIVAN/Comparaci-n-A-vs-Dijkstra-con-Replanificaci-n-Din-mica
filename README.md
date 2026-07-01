# Comparación A* vs Dijkstra con Replanificación Dinámica

Este proyecto implementa una demostración en **MATLAB** para comparar el comportamiento de los algoritmos **A\*** y **Dijkstra** en un entorno tipo almacén con obstáculos estáticos y un obstáculo dinámico que aparece durante la ejecución.

La simulación muestra cómo ambos algoritmos calculan una ruta inicial, cómo el robot comienza a seguirla y cómo se realiza una nueva planificación cuando aparece un obstáculo sobre la trayectoria. Al final, el programa compara métricas como costo de ruta, nodos explorados, tiempo de ejecución y si existió replanificación.

## Objetivo del proyecto

El objetivo principal es demostrar de forma visual y cuantitativa la diferencia entre **A\*** y **Dijkstra** cuando se aplican a un problema de planificación de rutas en una cuadrícula.

La demo permite observar que:

- Ambos algoritmos pueden encontrar una ruta válida desde un punto inicial hasta una meta.
- El entorno puede cambiar durante la ejecución mediante la aparición de obstáculos dinámicos.
- El robot puede replanificar desde su posición actual cuando la ruta original queda bloqueada.
- A\* normalmente explora menos nodos gracias al uso de una heurística.
- Dijkstra suele explorar más nodos porque evalúa principalmente el costo acumulado sin una heurística hacia la meta.

## Características principales

- Simulación en mapa de cuadrícula 2D.
- Obstáculos estáticos tipo almacén.
- Punto de inicio y punto objetivo.
- Comparación lado a lado entre A\* y Dijkstra.
- Movimiento del robot sobre la ruta calculada.
- Aparición de obstáculo dinámico durante la ejecución.
- Replanificación automática desde la posición actual.
- Visualización de nodos explorados.
- Cálculo de costo inicial, costo replanificado y costo total.
- Registro del número de nodos explorados.
- Medición del tiempo de ejecución.
- Exportación automática de la simulación en formato `.mp4`.

## Vista general de la simulación

El programa genera una figura con dos ventanas:

| Lado izquierdo | Lado derecho |
|---|---|
| A\* con replanificación dinámica | Dijkstra con replanificación dinámica |

Durante la ejecución se observa:

1. El mapa con obstáculos.
2. El punto de inicio.
3. La meta.
4. Los nodos explorados por cada algoritmo.
5. La ruta inicial encontrada.
6. El movimiento del robot.
7. La aparición del obstáculo dinámico.
8. La nueva ruta calculada.
9. Las métricas finales de cada algoritmo.

## Algoritmos implementados

### A\*

A\* utiliza una función de costo:

```text
f(n) = g(n) + h(n)
```

Donde:

- `g(n)` es el costo acumulado desde el nodo inicial hasta el nodo actual.
- `h(n)` es la heurística estimada desde el nodo actual hasta la meta.
- En este proyecto se utiliza distancia euclidiana como heurística.

Esto permite que A\* dirija la búsqueda hacia la meta y normalmente explore menos nodos.

### Dijkstra

Dijkstra utiliza únicamente el costo acumulado:

```text
f(n) = g(n)
```

No utiliza una heurística hacia la meta, por lo que explora de manera más uniforme el espacio disponible. Esto puede hacerlo más costoso computacionalmente en mapas grandes, aunque sigue garantizando rutas de costo mínimo cuando los costos son positivos.

## Estructura del proyecto

```text
.
├── comparacionreplanificacion.m
├── README.md
└── astar_vs_dijkstra_replanificacion.mp4
```

> El archivo `.mp4` se genera automáticamente al ejecutar el script en MATLAB.

## Requisitos

Para ejecutar este proyecto necesitas:

- MATLAB instalado.
- Soporte para `VideoWriter`.
- Sistema compatible con exportación MPEG-4.

El script fue diseñado para ejecutarse directamente sin archivos adicionales.

## Cómo ejecutar el proyecto

1. Clona este repositorio o descarga el archivo principal.

```bash
git clone <URL_DEL_REPOSITORIO>
```

2. Abre MATLAB en la carpeta del proyecto.

3. Ejecuta el archivo:

```matlab
comparacionreplanificacion
```

4. Al finalizar, se generará automáticamente el video:

```text
astar_vs_dijkstra_replanificacion.mp4
```

## Parámetros principales

Dentro del archivo `comparacionreplanificacion.m` se pueden modificar los siguientes valores:

```matlab
rows = 20;
cols = 30;
```

Definen el tamaño del mapa.

```matlab
startNode = [18, 3];
goalNode  = [3, 27];
```

Definen la posición inicial y la meta.

```matlab
video = VideoWriter('astar_vs_dijkstra_replanificacion.mp4', 'MPEG-4');
video.FrameRate = 30;
video.Quality = 95;
```

Configuran la exportación del video.

## Representación del mapa

El mapa se representa como una matriz:

```text
0 = espacio libre
1 = obstáculo
```

Los obstáculos estáticos se definen manualmente para simular una distribución tipo almacén con pasillos y paredes.

Ejemplo:

```matlab
mapBase(4:16, 8) = 1;
mapBase(4:16, 14) = 1;
mapBase(4:16, 20) = 1;
```

## Movimientos permitidos

El robot puede moverse en 8 direcciones:

- Arriba
- Abajo
- Izquierda
- Derecha
- Diagonal superior izquierda
- Diagonal superior derecha
- Diagonal inferior izquierda
- Diagonal inferior derecha

Los costos utilizados son:

```text
Movimiento horizontal o vertical = 1
Movimiento diagonal = sqrt(2)
```

## Métricas calculadas

Al finalizar la simulación, el programa muestra en consola una tabla comparativa con:

| Métrica | Descripción |
|---|---|
| Costo inicial | Costo de la primera ruta calculada |
| Costo replanificado | Costo de la nueva ruta después del obstáculo dinámico |
| Costo total | Costo recorrido antes de replanificar más el costo de la nueva ruta |
| Nodos explorados | Cantidad total de nodos evaluados |
| Tiempo total | Tiempo de cálculo del algoritmo |
| Replan | Indica si se realizó replanificación |

## Resultado esperado

Al ejecutar el programa, MATLAB mostrará una animación donde ambos algoritmos planean y replanean una ruta. También se imprimirá en consola una comparación similar a esta:

```text
COMPARACIÓN A* VS DIJKSTRA CON REPLANIFICACIÓN DINÁMICA

Algoritmo       Costo inicial   Costo repl.     Costo total     Explorados      Tiempo total    Replan
A*              ...             ...             ...             ...             ...             true
Dijkstra        ...             ...             ...             ...             ...             true
```

Además, se guardará un video con la simulación completa:

```text
astar_vs_dijkstra_replanificacion.mp4
```

## Interpretación de resultados

En general, se espera que **A\*** explore menos nodos que **Dijkstra**, ya que utiliza una heurística para orientar la búsqueda hacia la meta.

Dijkstra, al no usar heurística, puede explorar una mayor cantidad de nodos antes de llegar al objetivo. Sin embargo, ambos algoritmos pueden encontrar rutas válidas y realizar replanificación cuando aparece un obstáculo dinámico.

## Posibles mejoras futuras

Algunas mejoras que se pueden agregar al proyecto son:

- Crear obstáculos dinámicos aleatorios.
- Permitir múltiples obstáculos durante la trayectoria.
- Agregar comparación con otros algoritmos como RRT, RRT\*, D\* Lite o Theta\*.
- Implementar mapas cargados desde imágenes.
- Simular sensores de detección de obstáculos.
- Agregar un robot móvil con cinemática diferencial.
- Exportar métricas a archivo `.csv`.
- Crear una interfaz gráfica en MATLAB App Designer.
- Ejecutar la simulación sobre mapas más grandes.
- Comparar consumo computacional en diferentes tamaños de mapa.

## Aplicaciones

Este tipo de simulación puede aplicarse en:

- Robots móviles autónomos.
- Vehículos guiados automáticamente.
- Robots de almacén.
- Micromouse.
- Planeación de rutas en celdas.
- Sistemas de navegación con obstáculos dinámicos.
- Demostraciones académicas de inteligencia artificial y robótica móvil.

## Autor

Proyecto desarrollado como demostración académica de planificación de rutas y replanificación dinámica en MATLAB.

**Autor:** Areivan  
**Área:** Robótica móvil, planificación de rutas e inteligencia artificial aplicada.
