# walker_centroid_support

Estima el centro de gravedad (centroide de apoyo) del usuario sobre el
andador, siguiendo el método de J. Ballesteros (IROS 2018), combinando la
carga soportada en cada pierna y en cada maneta con la posición de esos
cuatro puntos de apoyo.

Este paquete no mide nada por sí mismo: consume las salidas ya calibradas
de [`walker_loads`](../walker_loads) (carga en kg por pierna y por maneta)
y las posiciones/TFs que publican `walker_step_detector` y
`walker_description`. Es, esencialmente, una media ponderada por peso de
cuatro posiciones.

## Cómo funciona

En cada punto de apoyo hay una carga (kg) y una posición (m, en el frame de
los pies, normalmente `base_link`):

| Punto de apoyo | Carga                                   | Posición                                         |
|-----------------|------------------------------------------|---------------------------------------------------|
| Pie derecho     | `right_leg_load` (de `/right_loads`)      | `position` del propio mensaje `StepStamped`       |
| Pie izquierdo   | `left_leg_load` (de `/left_loads`)        | `position` del propio mensaje `StepStamped`       |
| Maneta derecha  | `right_handle_weight` (de `/right_hand_loads`) | TF `right_handle` → frame de los pies (ver abajo) |
| Maneta izquierda| `left_handle_weight` (de `/left_hand_loads`)   | TF `left_handle` → frame de los pies (ver abajo)  |

El centroide publicado es la media ponderada de esas 4 posiciones:

```
centroide = (right_leg_load  * pos(pie_derecho)   +
             left_leg_load   * pos(pie_izquierdo) +
             right_hand_load * pos(maneta_derecha) +
             left_hand_load  * pos(maneta_izquierda)) / carga_total
```

donde `carga_total` es la suma de las cuatro cargas medidas (no el peso
declarado del usuario, ver [Decisiones de diseño](#decisiones-de-diseño)).

### Posición de las manetas

Las manetas no publican su propia posición, solo su carga. La posición se
obtiene con una consulta TF (`tf2`) entre el frame de la maneta
(`header.frame_id` del mensaje de `/left_hand_loads` / `/right_hand_loads`,
p. ej. `left_handle_id`) y el frame en el que vienen los pies (el
`header.frame_id` del propio mensaje de step, normalmente `base_link`).
Esa consulta se repite en **cada ciclo de publicación**, sin cachear el
resultado, para reflejar siempre la altura de maneta actual (ver
[Decisiones de diseño](#decisiones-de-diseño)).

### Ciclo de publicación

El nodo no tiene timer ni lee el reloj. Por defecto publica **cada vez que cambia cualquiera de las cuatro entradas**: cada
mensaje de `/left_hand_loads`, `/right_hand_loads`, `/left_loads` o `/right_loads` produce un centroide, calculado con lo último
que se sabe de cada una, sin esperar a las demás. Lo que lleva el `header.stamp` es el **tiempo de dato que han alcanzado las
piernas**: el más antiguo de los dos stamps más recientes de las cargas de pierna (`walker_loads.stamp_sequencer.Watermark`;
los stamps de las manetas no cuentan: en los bags llegan a 2.4 Hz y retrasarían la salida). Efectos, medidos en 4 bags
(MF_test07, CA_test03, LA_test04, MT_test13, rate 4) frente al modo ordenado:

- **Un cambio de entrada se ve al instante.** Con el orden por stamp, una carga de pierna tardaba en aparecer 0.12 s de media
  (p90 0.42 s) y la de una maneta 0.5–0.6 s; ahora salen en el mismo callback. La edad de lo que sale (reloj menos stamp,
  mediana) baja solo de 0.93 a 0.85 s, porque las cargas de pierna ya llegan a este nodo con ~0.75 s (`partial_loads` espera a
  las manetas): el centroide no puede ser más reciente que la posición de pie que recibe.
- **Salen 2.5–4.6 veces más mensajes** (24–26 Hz con las manetas a 2.4 Hz, 87 Hz en CA, cuya maneta izquierda publica a 50 Hz) y
  el 76 % comparte stamp con otro (antes el 36 %): los que provoca una maneta, o una carga recalculada sobre la misma posición de paso.
- **El resultado depende del orden en que llegan los mensajes**: el mismo bag no da exactamente los mismos centroides dos veces
  (entre tres ejecuciones a rate 4, el RMS sin sesgo frente al mocap varía ±0.1 cm); con otro `--rate` puede cambiar más.
- **Concuerda algo peor con el mocap**: error medio 14.1 → 14.0 cm, RMS sin sesgo 8.2 → 9.3 cm, correlación en x 0.30 → 0.26 y
  en y 0.47 → 0.43. El estado de una salida mezcla tiempos (piernas al stamp, manetas hasta 0.4 s más nuevas) y las cargas
  recalculadas llevan el stamp de su posición, no el de su cálculo.

Con `stamp_ordered:=true` vuelve el comportamiento anterior, **para los estudios que necesiten el resultado reproducible**: las
4 entradas se ordenan por stamp
(`walker_loads.stamp_sequencer.StampSequencer`: cada mensaje se procesa cuando ninguna entrada puede ya entregar
algo más antiguo) y, sobre una rejilla fija de tiempo de dato (`period`, 50 ms por defecto), se publica el
centroide del estado de cada tick si ha llegado algo nuevo desde el tick anterior (`new_data_available`, marcado
por cualquiera de las 4 entradas). Así la salida depende solo de los mensajes: **el mismo bag da los mismos
centroides, bit a bit, con cualquier carga de la máquina y a cualquier `--rate`** (comprobado a rates 1, 4 y 8), y
sale con un retardo de hasta ~un periodo de la entrada más lenta (las manetas, ~0.5 s); si una entrada se calla,
las demás esperan como mucho `max_wait_s`. En los 4 bags da byte a byte el mismo CSV que antes del modo por defecto nuevo.
Se activa con el argumento del launcher (`ros2 launch walker_centroid_support centroid_support.launch.py stamp_ordered:=true`; llega también a los launchers que lo incluyen, p. ej. los `compare_*` de mocap4w) o con `--ros-args -p stamp_ordered:=true`.

En ambos modos, antes de publicar nada:

1. Espera a haber recibido al menos un mensaje de cada una de las 4 fuentes
   de carga (`first_data_ready`).
2. Resuelve la TF de ambas manetas; si alguna falla (aún no hay TF, o el
   frame no existe), se salta ese ciclo y se reintenta en el siguiente.
3. Si la carga total medida es ≤ 0 (todavía no hay carga real, p.ej. nada
   más arrancar), también se salta el ciclo.

Cuando las tres condiciones se cumplen, publica un `geometry_msgs/PointStamped` en el frame de los pies. Su stamp es el del
mensaje de step derecho en el modo ordenado, y el de las piernas descrito arriba en el modo por defecto.

## Topics

### Suscritos

| Topic (parámetro)                              | Tipo                        | Publicado por |
|-------------------------------------------------|------------------------------|----------------|
| `/left_hand_loads` (`left_hand_loads_topic_name`)  | `walker_msgs/msg/ForceStamped` | `walker_loads` (`partial_loads`) |
| `/right_hand_loads` (`right_hand_loads_topic_name`)| `walker_msgs/msg/ForceStamped` | `walker_loads` (`partial_loads`) |
| `/left_loads` (`left_loads_topic_name`)          | `walker_msgs/msg/StepStamped`  | `walker_loads` (`partial_loads`) |
| `/right_loads` (`right_loads_topic_name`)        | `walker_msgs/msg/StepStamped`  | `walker_loads` (`partial_loads`) |

`ForceStamped.force` en `/left_hand_loads` y `/right_hand_loads` ya viene
calibrado en kg (no es la lectura cruda del sensor); `StepStamped.load` en
`/left_loads`/`/right_loads` ya viene repartido entre piernas según la
lógica de `partial_loads` (Kalman de diferencia de velocidad + reparto
suave). Este nodo no vuelve a calibrar ni a repartir nada, solo pondera y
combina.

Además usa `tf2` (vía `tf2_ros.Buffer`/`TransformListener`) para resolver
la posición de las manetas; no hay un parámetro de topic para esto, es la
infraestructura TF estándar (necesita que `walker_description`'s
`handle_tf_publisher` esté publicando la TF `<frame de los pies> ->
{left,right}_handle_id`).

### Publicados

| Topic (parámetro)                         | Tipo                          |
|---------------------------------------------|--------------------------------|
| `/support_centroid` (`centroid_topic_name`) | `geometry_msgs/msg/PointStamped` |

## Parámetros

| Parámetro                     | Por defecto        | Descripción |
|--------------------------------|---------------------|--------------|
| `left_hand_loads_topic_name`   | `/left_hand_loads`  | Carga (kg) de la maneta izquierda, ya calibrada |
| `right_hand_loads_topic_name`  | `/right_hand_loads` | Carga (kg) de la maneta derecha, ya calibrada |
| `left_loads_topic_name`        | `/left_loads`       | Posición + carga (kg) del pie izquierdo |
| `right_loads_topic_name`       | `/right_loads`      | Posición + carga (kg) del pie derecho |
| `centroid_topic_name`          | `/support_centroid` | Topic de salida del centroide |
| `stamp_ordered`                | `false`             | `false`: publica con cada mensaje de entrada. `true`: entradas en orden de stamp y salida sobre una rejilla de tiempo de dato, reproducible a cualquier rate |
| `period`                       | `0.05` (20 Hz)      | Periodo de la rejilla de publicación, en tiempo de dato (s); solo con `stamp_ordered` |
| `max_wait_s`                   | `5.0`               | Tiempo de dato que se espera a una entrada callada antes de darla por perdida; debe ser mayor que el mayor desfase entre entradas. Sin `stamp_ordered` solo limita cuánto puede quedarse atrás el stamp de la salida si una pierna se calla |

Este nodo **no** tiene parámetro de `user_desc`: no necesita el peso
declarado del usuario, ver más abajo.

## Cómo reproducir su uso

Este paquete no funciona solo; necesita el resto del pipeline de carga y
la TF de las manetas ya en marcha:

1. **`walker_loads`** (`partial_loads`, C++): necesita `/left_handle`,
   `/right_handle` (fuerza cruda, `ForceStamped`, de `walker_arduino`),
   `/detected_step_left`, `/detected_step_right` (de
   `walker_step_detector`) y `/user_desc` (`walker_msgs/msg/UserDesc`, el
   peso declarado del usuario). Publica `/left_loads`, `/right_loads`
   (carga por pierna) y `/left_hand_loads`, `/right_hand_loads` (carga por
   maneta, calibrada).
2. **`walker_description`** (`handle_tf_publisher`): publica la TF
   `<frame de los pies> -> {left,right}_handle_id`, imprescindible para que
   este nodo sepa dónde están las manetas.
3. **`walker_centroid_support`** (este paquete): lanzar
   [`centroid_support.launch.py`](launch/centroid_support.launch.py) una
   vez lo anterior esté publicando.

```bash
ros2 launch walker_loads partial_loads.launch.py
ros2 run walker_description handle_tf_publisher.py
ros2 launch walker_centroid_support centroid_support.launch.py
```

## Decisiones de diseño

- **Normaliza por la carga total medida, no por el peso declarado
  (`/user_desc`).** `right_leg_load + left_leg_load` en `partial_loads` se
  calcula como `weight - hand_loads`, recortado a `[0, weight]` para
  absorber ruido/desajustes de calibración de las manetas. Cuando ese
  recorte no actúa, dividir por la carga medida o por el peso declarado da
  exactamente el mismo resultado; cuando sí actúa, dividir por lo
  realmente medido mantiene el centroide como un baricentro autoconsistente
  (los pesos siempre suman 1) en vez de desviarlo silenciosamente. Como
  consecuencia, este nodo **no necesita `/user_desc` en absoluto**.
- **La TF de las manetas se resuelve en cada ciclo, sin caché.** Las
  manetas están fijas al andador, pero su altura (`handle_height`) puede
  cambiarse en caliente desde `handle_tf_publisher`. Cachear la TF la
  primera vez (como hacía una versión anterior de este nodo) dejaría el
  centroide calculado con una altura de maneta obsoleta si se cambia
  después de arrancar.
- **El despacho izquierda/derecha es por topic, no por `frame_id`.** Cada
  lado tiene su propia suscripción y su propio callback
  (`left_hand_load_callback`/`right_hand_load_callback`,
  `left_load_callback`/`right_load_callback`), en vez de mirar si
  `'left'`/`'right'` aparece como substring en `header.frame_id` del
  mensaje (patrón usado en otras partes de WalKit, más frágil ante un
  renombrado de frames).

## Estructura del paquete

```
walker_centroid_support/
├── launch/
│   └── centroid_support.launch.py   # lanza el nodo con los topics/periodo por defecto
├── walker_centroid_support/
│   └── centroid_support.py          # nodo CentroidSupport
└── test/                            # copyright / flake8 / pep257 (ament lint)
```

## Tests

```bash
colcon test --packages-select walker_centroid_support
```

Corre los linters estándar de ROS 2 (`ament_copyright`, `ament_flake8`,
`ament_pep257`); no hay tests funcionales del cálculo del centroide.
