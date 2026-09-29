# walker_mocap_eval

Herramientas offline para recalcular, a partir de los rosbags etiquetados
con mocap (`soma13kp`/`datasets/labeled_bags`), los mismos parametros que
publican los nodos de `walker_loads` en marcha real -- y, en el futuro,
cualquier otro estudio que cruce mocap con datos del andador (postura,
centroide, estabilidad...).

No es un paquete ROS: como `walker_step_detector/scripts/*.py`, son
scripts de analisis que se ejecutan directamente con
`python3 scripts/xxx.py` tras sourcear `/opt/ros/jazzy/setup.bash` (hace
falta `rosbag2_py`/`rclpy` para deserializar los bags), y no forman parte
del build de colcon.

## Por que un paquete nuevo (y no meterlo en walker_loads o walker_step_detector)

- `walker_loads` es el paquete del nodo ROS en produccion
  (`gait_monitor_speed`, `partial_loads`); no es el sitio para tooling de
  analisis offline.
- `walker_step_detector/scripts/mocap_bag_io.py` ya hace algo parecido,
  pero esta acoplado a las necesidades del detector de pasos (frame
  "laser", solo tobillos/rodillas). Los estudios del plan (paso 3: postura,
  centroide, angulos de codo...) necesitan el resto de keypoints
  etiquetados (hombros, caderas, muñecas...), asi que se generalizo aqui en
  vez de forzar ese acoplamiento.

La calibracion mocap->base_footprint (`mocap_to_base_footprint.yaml`) SI se
reutiliza tal cual desde `walker_step_detector/config/` (fuente unica),
solo se referencia por ruta relativa por defecto.

## Estructura

```
mocap_eval/            <- libreria importable (sin dependencia de ROS salvo bag_io.py)
  bag_io.py             lectura de bags etiquetados, marcadores/rigid_bodies, proyeccion mocap->frame andador
  contact_events.py     eventos/intervalos de apoyo por pie a partir de cinematica del tobillo (mocap no mide carga)
  gait_metrics.py       puerto puro-Python de walker_loads/gait_monitor_speed.py (GaitMonitorSp)
  posture_metrics.py    inclinacion de tronco, ancho de paso, doble apoyo, margen de estabilidad lateral (punto 3)

scripts/
  compute_gait_from_mocap.py       CLI: bag(s) etiquetado(s) -> Step/Stride time/length, NoS, d, CAD, WV (punto 1 del plan)
  selftest_gait_metrics.py         auto-test con datos sinteticos (sin bags), valida el puerto del algoritmo
  mocap_odom_bridge.py             nodo ROS: republica rigid_bodies[0] (mocap) como /odom (ver punto 2 mas abajo)
  compare_gait_mocap_vs_walker.py  punto 2: lanza el pipeline real (replay_offline.launch.py) contra cada bag,
                                    graba lo que publica gait_monitor_speed y lo compara con compute_gait_from_mocap.py
  compute_posture_from_mocap.py    CLI: bag(s) etiquetado(s) -> parametros posturales/estabilidad (punto 3)
  selftest_posture_metrics.py      auto-test con datos sinteticos (sin bags), valida la geometria de posture_metrics.py
```

## Uso

```bash
source /opt/ros/jazzy/setup.bash
cd WalKit/walker_mocap_eval

# validar la logica portada (no requiere bags ni $DATASETS)
python3 scripts/selftest_gait_metrics.py

# un bag
python3 scripts/compute_gait_from_mocap.py --bag $DATASETS/labeled_bags/CA_test01

# todos los bags etiquetados
python3 scripts/compute_gait_from_mocap.py --bags-dir $DATASETS/labeled_bags \
    --out /tmp/mocap_gait_results.json
```

## Diseño: como se recalcula cada parametro

| Parametro (GaitStamped/WalkStats) | Como se obtiene aqui | Fuente en el andador real |
|---|---|---|
| Step/Stride time y length por pierna | `gait_metrics.py`, puerto **literal** de `GaitMonitorSp.update_stats()`/`timer_callback()`: mismas formulas, mismo criterio de arranque (>2 eventos por pierna), mismas medias acumuladas (`nanmean` desde el inicio) | `walker_loads/gait_monitor_speed.py` |
| Evento de "apoyo" por pierna | `contact_events.detect_stance_events()`: minimo de velocidad horizontal del marcador de tobillo (`l_ankle`/`r_ankle`) por episodio bajo un umbral (0.25 m/s por defecto) | `StepStamped.load` pasando a >0 (celula de carga) |
| Posicion del evento | Se proyecta desde el frame "map" del mocap al frame del andador ("laser") con `rigid_bodies[0].pose` + la calibracion ICP existente + el offset fijo laser<-base_footprint (igual que `walker_step_detector/scripts/mocap_bag_io.py`) | `StepStamped.position`, ya en frame "laser"/"base_footprint" (deteccion laser) |
| Distancia recorrida (`d`) | Arclength del propio `rigid_bodies[0]` (andador) en el mocap, acumulada entre el primer y el ultimo evento de paso | **Ver discrepancia abajo** |
| `avg_step_speed_{left,right,all}` | Media de (longitud de paso / tiempo de paso), no publicada por el nodo real pero si mencionada en el docstring/spec original de `GaitMonitorSp` | No publicada actualmente por ningun topic |

## Bug corregido en el nodo real (encontrado antes del punto 2)

`walker_loads/gait_monitor_speed.py::GaitMonitorSp.getTravelledDist()`
**no usaba la distancia real acumulada**: `self.travelled` se calculaba
correctamente en `odom_callback()` (suma de desplazamientos consecutivos de
`/odom`), pero `timer_callback()` llamaba a `getTravelledDist()`, que
ignoraba `self.travelled` y devolvia una formula con una constante de
prueba hardcodeada (`test_time = 14.48`, pista de `8.0` m, resultado capado
a 8m). Es decir, `d`, `CAD` y `WV` del nodo real no reflejaban la distancia
ni la velocidad reales del test. **Corregido**: `timer_callback()` ahora usa
`self.travelled` directamente; `getTravelledDist()` se elimino.

## Punto 2: pipeline real vs mocap, resultados sobre los 47 bags

`compare_gait_mocap_vs_walker.py` lanza `walker_loads/launch/
replay_offline.launch.py` (box_filter -> km_detect_steps -> partial_loads ->
mocap_odom_bridge -> gait_monitor_speed) contra cada uno de los 47 bags de
`datasets/labeled_bags`, graba el ULTIMO mensaje de `/left_gait_stats`,
`/right_gait_stats` y `/global_gait_stats` (GaitStamped/WalkStats son
acumulados desde el arranque del nodo, asi que el ultimo mensaje ya refleja
todo el historial) y lo compara campo a campo con `compute_gait_from_mocap.py`.

**`/odom` real no es viable en estos bags**: `/left_wheel`/`/right_wheel`
(walker_diff_odom) estan rotos -- comprobado en CA_test01, `/right_wheel`
no tiene ni un mensaje y `/left_wheel` esta congelado (3475->3476 ticks en
~90s). `mocap_odom_bridge.py` republica `rigid_bodies[0]` (mocap) como
`/odom` en su lugar -- valido porque `self.travelled` solo acumula deltas
de posicion consecutivos, invariante a que frame se use.

Resultados **tras el fix de timestamp** (43/47 bags con datos suficientes;
ver tambien el docstring de `replay_offline.launch.py` para el detalle del
bug):

| Campo | MAE | MAPE | Lectura |
|---|---|---|---|
| `d` (distancia) | 0.17 m (mediana 2.3 cm) | 1.1% | **Validado**: coincide con mocap |
| `wv` (velocidad) | 0.044 m/s (mediana 0.034) | 15.7% | Bueno, hereda la calidad de `d` |
| `cad`/`NoS`/`SpT`/`SdT` | ver abajo | ver abajo | **Bimodal**: 13 bags muy bien, 30 todavia mal -- no es un error uniforme |

**Causa raiz encontrada y corregida** (no era un bug de `gait_monitor_speed`):
se descarto primero la hipotesis de zona de deteccion fija en tests de
"movimiento en circulos" (la mayoria de los 47) -- los tests "ida y vuelta"
(LA_test\*, sin ese patron) mostraban el mismo subconteo severo, y
`fixed_frame_active_area_x`/`_y` (que este launcher pasaba a
`km_detect_steps`) resultaron ser parametros **muertos** (el .cpp nunca los
lee, ya quitados del launcher).

La causa real estaba en `walker_step_detector/src/km_detect_steps.cpp`:
con `kalman_enabled: true` (necesario aqui -- con `false`,
`StepStamped.tracked` nunca se pone a `true` y `partial_loads` no publica
nada en absoluto), `laserCallback()` sellaba cada `StepStamped` predicho con
`this->now()` (reloj de pared) en vez del timestamp del propio escaner. En
vivo eso no importa (reloj de pared ~= tiempo del sensor), pero reproduciendo
un bag a `--rate 4.0` (el valor por defecto de este launcher) el reloj de
pared avanza 4 veces mas rapido que la linea de tiempo real del bag -- y
`Tr`/`SpT`/`SdT` (calculados como diferencias de esos timestamps) salian
truncados a ~1/rate de su valor real. Confirmado reproduciendo a
`rate:=1.0` con el bug todavia presente (Tr paso de ~12% a ~73% de la
duracion real del bag, sin tocar nada mas) antes de tocar el C++.
**Corregido**: se sella con `rclcpp::Time(scan->header.stamp)` en su lugar
(recompilado con `colcon build --packages-select walker_step_detector`).
Tras el fix, a `rate=4.0`, `Tr` en CA_test01/02/03 paso de ~10s a ~50-60s
(de ~85-90s reales), y `CA_test03` (antes sin datos) ya produce resultados.

Tras el fix, sobre los 47 bags, el resultado es **bimodal**, no un error
uniforme:

- **13 bags funcionan bien** (`MF_test17-20`, `MT_test11-20` salvo
  `MT_test13`): NoS del walker entre 70% y 153% del de mocap, `Tr` cubre
  todo el bag, errores de SpT/SdT por debajo de ~2s.
- **30 bags siguen mal** (todos los `CA_test*` y `LA_test*` restantes, y
  `MF_test01-15`): NoS del walker solo 8-23% del de mocap, igual de mal que
  antes del fix.
- **4 bags sin datos**: `CA_test03`, `LA_test5`, `MF_test02`, `MF_test16`.

Se comprobo si la diferencia esta en la calidad del propio `/scan` (huecos,
timestamps duplicados o no monotonos) entre un bag "bueno" y uno "malo":
**no lo esta** -- `CA_test01` (malo) y `MF_test17` (bueno) tienen el mismo
comportamiento de timestamp en `/scan` (mediana de intervalo ~0.12-0.18s,
sin duplicados, sin retrocesos), y `MF_test16` (misma sesion y calidad de
escaneo que `MF_test17`) sigue sin producir ningun dato. Es decir, lo que
diferencia un bag bueno de uno malo pasa DESPUES del `/scan` -- dentro del
clustering/tracking de `km_detect_steps` o de la asignacion de carga de
`partial_loads` -- y no se ha perseguido mas en esta sesion: hay una tarea
marcada para `walker_step_detector` sobre esto (el usuario ya la inicio
como sesion aparte mientras esta investigacion seguia en marcha aqui --
revisar ambas antes de seguir, y usar esta lista de bags buenos/malos como
punto de partida).

### Segunda causa encontrada (walker_step_detector): `ms_period` mal nombrado

La tarea marcada arriba ("hay una tarea marcada para walker_step_detector...
revisar ambas antes de seguir") ya tiene respuesta: la diferencia entre los
13 bags "buenos" y los 30 "malos" no estaba en `km_detect_steps` ni en
`/scan`, sino en `partial_loads.launch.py`. Ese launcher pasaba
`{'period': 0.05}` al nodo `partial_loads`, pero `partial_loads.cpp` nunca
declara un parametro `period` -- solo `ms_period` (int, ms, default 500).
El valor de 0.05 (pensado como 50ms/20Hz) se ignoraba en silencio y el nodo
corria siempre a su default de 2Hz.

Por que eso explica justo el patron **bimodal**: `partial_loads` decide
"que pierna carga" en un `create_wall_timer` -- reloj de PARED, no afectado
por `--rate` del bag. A 500ms de periodo real y `rate=4.0`, cada muestra de
esa decision cae cada ~2s de tiempo de BAG. Un bag con zancada lenta (SpT
mayor que esos ~2s) todavia captura suficientes cruces load>0 para verse
razonable; un bag con zancada mas rapida que el intervalo de muestreo
alias-a por completo la señal -- exactamente los 30 bags "malos" (y los 4
sin ningun dato, caso extremo de lo mismo).

**Fix**: `partial_loads.launch.py` ahora pasa `{'ms_period': 50}` (nombre
correcto). Sin recompilar (es un parametro de lanzamiento, no C++).
Revalidado con `compare_gait_mocap_vs_walker.py` sobre los 47 bags
completos (`ROS_DOMAIN_ID` propio, ver `ROS_DOMAIN_ID_EVAL` en el script):

| Campo | Antes (tras fix de timestamp, arriba) | Despues (+ fix de `ms_period`) |
|---|---|---|
| Bags con datos suficientes | 43/47 (4 sin ningun `GaitStamped`) | **47/47** |
| `NoS` | bimodal (13 bien, 30 al 8-23% del real) | MAE=23.5 pasos, medianaAE=18, MAPE=25.1% (uniforme, sin bags a cero) |
| `d` | MAPE 1.1% | MAPE 1.3% (sin cambio, esperado) |
| `wv` | MAPE 15.7% | MAPE 13.2% |
| `cad` | — | MAE=22.5, MAPE=29.4% |
| `SpT`/`SdT`/`SpL`/`SdL` | heredaban el error bimodal de NoS | siguen con MAPE 30-230% -- heredan el ~25% de error de `NoS` que queda |

`NoS` pasa de infracontar severamente (o fallar del todo) a un error
uniforme del ~25%, sin bags catastroficos. Ese ~25% residual es,
presumiblemente, ruido de `km_detect_steps` (fuerza siempre exactamente 2
clusters por k-means sobre `/scan_feet`, sin filtro de confianza como el
Random Forest de `detect_steps` -- ver `getCentroids()`): con la señal ya
muestreada a un ritmo razonable, ese ruido dispara algun cruce `load>0` de
mas o de menos. Detalle y numeros del spot-check inicial (2 bags) en el
comentario de `partial_loads.launch.py`. No se ha perseguido mas aqui --
encaja con el trabajo ya activo en `walker_step_detector` sobre la calidad
de deteccion de `km_detect_steps` frente a `detect_steps` (RF).

## Punto 3: parametros posturales/de estabilidad desde mocap

`posture_metrics.py` (+ CLI `compute_posture_from_mocap.py`) implementa los
4 candidatos "baratos" de la tabla del plan (los que no requieren datos del
andador para calcularse, solo mocap) sobre la misma base que el punto 1
(`bag_io.py`, `contact_events.py`):

| Parametro | Como se calcula | Candidato a correlar con (andador) |
|---|---|---|
| Inclinacion de tronco (total, sagital, frontal) | Angulo entre el vector cadera_media->hombro_medio y la vertical, en cada frame donde esten los 4 marcadores (`l/r_shoulder`, `l/r_hip`) | Reparto de carga manetas/pies (`partial_loads`), centroide de soporte (`walker_centroid_support`) |
| Ancho de paso | Distancia lateral (proyectada al eje del andador via `rigid_bodies[0]` yaw) entre cada evento de apoyo y el del pie contrario mas cercano en tiempo -- reutiliza `contact_events.detect_stance_events()` del punto 1 | No publicado hoy; derivable de las mismas posiciones que ya usa el detector de pasos |
| Tiempo de doble apoyo (%) | Solape entre `contact_events.detect_stance_intervals()` de ambos pies, sobre el total | `is_left_load`/`is_right_load` de `gait_monitor_speed.py` ya existen internamente, no se publican |
| Margen de estabilidad lateral | Punto medio de caderas (proxy de CoM) proyectado sobre el eje lateral del andador, normalizado entre los DOS TOBILLOS reales (0=borde, 1=centro) -- misma formula que `walker_stability.py::y_sta`, pero con BoS real en vez de estimado por tablas antropometricas | **Validacion directa** de `StabilityStamped.sta` |

`indexed_nearest()` en `bag_io.py`: la primera version de `trunk_lean_series()`
usaba `nearest()` (escaneo lineal) contra 3 tracks por cada muestra de otro
(miles a ~120Hz) -- O(n^2), minutos por bag. Sustituido por una version
indexada con `bisect` (una vez por track, O(log n) por consulta): segundos
por bag. Cualquier funcion futura que consulte un track repetidamente
deberia usar `indexed_nearest()`, no `nearest()`.

Resultados sobre los 47 bags: `/tmp/mocap_posture_full.json`
(`compute_posture_from_mocap.py --bags-dir ... --out ...`). Rangos
observados: inclinacion de tronco 4.8-41.8 (mean~16), ancho de paso
0.07-0.42 m (5 bags sin dato: el emparejado con `--max-dt-ms` por defecto
no encontro apoyo contrario cercano), doble apoyo 6-67%, margen lateral
0.26-0.85 (0.5=centrado).

**Primera correlacion real, ya visible sin cruzar nada mas**: los bags
marcados en `test_config.txt` como "postura inclinada, manetas muy bajas"
(`handle_height=0`, en vez de 4) tienen mas inclinacion de tronco que el
resto de bags del MISMO sujeto -- CA: 15.2 grados de media (test07/08) vs
11.3 (resto); MT: 13.4 (test19/20) vs 9.0 (resto). Señal real pero con
solapamiento (algun bag "normal" como CA_test03 sale igual de inclinado),
no una separacion limpia -- consistente con que la maneta baja fuerza mas
inclinacion pero no es el unico factor. Es exactamente el tipo de
correlacion mocap<->parametro-del-andador (aqui, `handle_height`) que
busca el punto 3 del plan.

Cruzar el resto de estos parametros con datos PUBLICADOS por el andador
(inclinacion de tronco vs asimetria real de `/left_handle`-`/right_handle`,
o el margen de estabilidad de aqui contra `StabilityStamped.sta`) es el
paso siguiente logico -- requiere decidir que bags tienen datos de
manetas/centroide utilizables (ver limitaciones del punto 2 arriba: la
extraccion en vivo depende del mismo pipeline con el problema de
`km_detect_steps` ya descrito) y no se ha hecho todavia.
