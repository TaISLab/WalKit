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
  cog_estimation.py     centro de gravedad por modelo segmentario (De Leva 1996) + sexo/peso del usuario (punto 3)
  extra_metrics.py      codo, muñeca, reparto de peso en manetas, yaw/curvatura, oscilacion vertical del CoM, indice de
                        inestabilidad, Symmetry Index (puntos 3-elbow/4/5/8/9/11 del plan + punto 2 de la ampliacion)
  imu_metrics.py        RMS, Harmonic Ratio, regularidad por autocorrelacion, y aceleracion AP/ML del andador por mocap
                        (punto 1 de la ampliacion)
  correlation_utils.py  correlacion de Pearson por sujeto (CA/LA/MF/MT) + TOTAL (puntos 2 y 5 de la ampliacion)

scripts/   (orden de ejecucion y ejemplos de comandos: ver scripts/README.md)
  compute_gait_from_mocap.py       CLI: bag(s) etiquetado(s) -> Step/Stride time/length, NoS, d, CAD, WV (punto 1 del plan)
  selftest_gait_metrics.py         auto-test con datos sinteticos (sin bags), valida el puerto del algoritmo
  mocap_odom_bridge.py             nodo ROS: republica rigid_bodies[0] (mocap) como /odom (ver punto 2 mas abajo)
  compare_gait_mocap_vs_walker.py  punto 2: lanza el pipeline real (replay_offline.launch.py) contra cada bag,
                                    graba lo que publica gait_monitor_speed y lo compara con compute_gait_from_mocap.py
  compute_posture_from_mocap.py    CLI: bag(s) etiquetado(s) -> parametros posturales/estabilidad (punto 3)
  selftest_posture_metrics.py      auto-test con datos sinteticos (sin bags), valida la geometria de posture_metrics.py
  compute_cog_from_mocap.py        CLI: bag(s) etiquetado(s) -> centro de gravedad (modelo De Leva + sexo/peso, punto 3)
  selftest_cog_estimation.py       auto-test con datos sinteticos (sin bags), valida el modelo segmentario
  compare_cog_mocap_vs_walker.py   punto 3 (comparacion): CoM de mocap vs /support_centroid real (walker_centroid_support)
  eval_odom_against_mocap.py       evalua una odometria grabada (/odom del EKF laser+IMU, walker_bringup) contra el mocap:
                                    ATE tras alinear con Umeyama 2D + deriva en % del recorrido (antes en walker_step_detector/scripts)
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

## Recorte a la ventana `/tag` (punto 1 y 2, por defecto)

Cada bag de `datasets/labeled_bags` trae exactamente 2 mensajes en el topic
`/tag` (`builtin_interfaces/msg/Time`) -- los mismos marcadores de
inicio/fin de test (pulsacion de boton) que usa `mocap_analysis/scripts/
merge_rosbags.py` para sincronizar y recortar bags al fusionarlos. Fuera de
esa ventana suele haber tiempo de calibracion/colocacion del usuario, no
marcha real: en `CA_test01`, por ejemplo, la ventana `/tag` dura 52.9 s de
los 89.3 s totales del bag.

Por defecto (`compute_gait_from_mocap.py` y `compare_gait_mocap_vs_walker.py`,
`--ignore-tags` para desactivarlo en ambos):
  - **Lado mocap**: `/labeled_markers` y `/rigid_bodies` se recortan a
    `bag_io.read_tag_window()` antes de detectar eventos de apoyo.
  - **Lado andador** (punto 2): `replay_offline.launch.py` arranca el
    replay en el primer `/tag` (nuevo argumento `start_offset`, segundos de
    bag desde su propio inicio, `ros2 bag play --start-offset`) y
    `compare_gait_mocap_vs_walker.py` solo espera lo que dura la ventana
    entre los dos `/tag` (en vez de la duracion completa del bag), para que
    ambos lados vean la misma porcion "activa" del test.

Impacto en `CA_test01` (mocap-only, bag completo -> ventana `/tag`): cadencia
59.2 -> 85.0 pasos/min, velocidad 0.329 -> 0.542 m/s, NoS 87 -> 75 (menos
pasos porque la ventana es mas corta, pero mas *densos* -- el tiempo muerto
fuera de la ventana no aportaba pasos reales, solo alargaba `Tr` y diluia
la cadencia). Es una correccion sistematica: todo el reparto de tiempo real
de marcha estaba subestimando la velocidad/cadencia real del usuario al
promediar sobre tiempo que no era marcha.

## Diseño: como se recalcula cada parametro

| Parametro (GaitStamped/WalkStats) | Como se obtiene aqui | Fuente en el andador real |
|---|---|---|
| Step/Stride time y length por pierna | `gait_metrics.py`, puerto **literal** de `GaitMonitorSp.update_stats()`/`timer_callback()`: mismas formulas, mismo criterio de arranque (>2 eventos por pierna), mismas medias acumuladas (`nanmean` desde el inicio) | `walker_loads/gait_monitor_speed.py` |
| Evento de "apoyo" por pierna | `contact_events.detect_stance_events()`: minimo de velocidad horizontal del marcador de tobillo (`l_ankle`/`r_ankle`) por episodio bajo un umbral (0.25 m/s por defecto) | `StepStamped.load` pasando a >0 (celula de carga) |
| Posicion del evento | Se proyecta desde el frame "map" del mocap al frame del andador ("laser") con `rigid_bodies[0].pose` + la calibracion ICP existente + el offset fijo laser<-base_footprint (igual que `walker_step_detector/scripts/mocap_bag_io.py`) | `StepStamped.position`, ya en frame "laser"/"base_footprint" (deteccion laser) |
| Distancia recorrida (`d`) | Arclength del propio `rigid_bodies[0]` (andador) en el mocap, acumulada entre el primer y el ultimo evento de paso | **Ver discrepancia abajo** |
| `avg_step_speed_{left,right,all}` | Media de (longitud de paso / tiempo de paso), no publicada por el nodo real pero si mencionada en el docstring/spec original de `GaitMonitorSp` | No publicada actualmente por ningun topic |

**Resultados sobre los 47 bags, solo mocap, con recorte `/tag`**
(`/tmp/mocap_gait_tagged_full.json`): sin fallos en ningun bag. Cadencia
media 100.4 pasos/min (std 29.4), velocidad media 0.403 m/s (std 0.119) --
sensiblemente mas altas y mas representativas de la marcha real que las
que salian promediando sobre el bag completo (esas incluian el tiempo de
calibracion/colocacion, ver seccion anterior). `SpL`/`SdL` medios
0.14-0.31 m segun pierna/campo, consistentes entre bags del mismo sujeto.

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

**Reejecutado sobre los 47 bags completos** para este informe (mismo
`compare_gait_mocap_vs_walker.py`, `ROS_DOMAIN_ID` propio): **47/47 bags con
datos**, ningun fallo. `NoS` MAE=31.4 pasos, medianaAE=31, MAPE=34.6% --
algo peor que el 25.1% del spot-check anterior, esperable dado que el
muestreo de `partial_loads` sigue en reloj de pared (`create_wall_timer`,
no en tiempo de bag) y por tanto tiene algo de varianza de una corrida a
otra. `d` (MAPE 1.3%) y `wv` (MAPE 13.4%) se mantienen validados.

Encontrado en esta corrida: `CA_test09` tiene valores de `SpL`/`SdL` del
andador disparatados (`left_sdl`=87m, `right_spl`=39m, `right_sdl`=76m de
"longitud de zancada") que inflan artificialmente el MAPE agregado de esos
4 campos (200-2800%) muy por encima de lo que reflejan el resto de bags
(medianaAE de esos mismos campos: 0.15-0.21 m, mucho mas razonable -- usar
la mediana, no la media, al leer esa fila de la tabla).

**Causa real de `CA_test09` y estado final, tras coordinarse con el hilo
`walker_step_detector` (mejoras)**: mi hipotesis inicial (clustering sin
filtro de confianza en `km_detect_steps`) no era la causa -- esa sesion
diagnostico el verdadero origen: `TrackLeg::t` se inicializaba a 0 como
centinela de "nunca predicho", pero `predict_step()` lo usaba tal cual en
el primer `u.dt()=(ti-t)*1e-9`, dando un dt de ~1.79e9s (segundos desde el
epoch Unix) en vez de un intervalo real. Ese dt gigante dispara la
covarianza del EKF desde el primer ciclo (via el jacobiano de
`SystemModelLeg.hpp`), y la siguiente actualizacion salta a cientos de
metros -- afecta por igual a las tres variantes de detector (`km`, `rf`,
`seg`, mismo `TrackLeg` compartido); `rf`/`seg` no lo mostraban en
`CA_test09` por pura suerte de timing, no porque no les afecte. Fix (su
sesion, `track_leg.cpp::predict_step()`): si `t==0`, fijar `t=ti` antes de
calcular `dt` -- la primera prediccion usa `dt=0` en vez de "tiempo desde
el epoch".

De paso, esa sesion recomendo cambiar de `km_detect_steps` a `detect_steps`
(Random Forest) en `replay_offline.launch.py` -- validado por ellos en
95.7-95.9% de match_rate sobre estos mismos 47 bags (vs ~81% de `km`).
Aplicado, y de camino encontramos (yo) que `detect_steps.cpp` y
`detect_steps_s.cpp` tenian el MISMO bug de timestamp de reloj de pared que
ya habia corregido en `km_detect_steps.cpp` (`this->now()` en vez de
`scan->header.stamp`/`seg_array->header.stamp`) -- corregido en los tres.

**Resultado final, los tres fixes juntos (timestamp x3 variantes + `t==0`
del EKF + cambio a RF), sobre los 47 bags completos**:

| Campo | Solo timestamp+`ms_period` (arriba) | + `t==0` fix + RF (final) |
|---|---|---|
| Bags con datos | 47/47 | 47/47 |
| `NoS` | MAE=31.4, MAPE=34.6% | **MAE=24.6, medianaAE=20, MAPE=24.0%** |
| `d` | MAPE 1.3% | MAPE 1.3% (sin cambio, esperado) |
| `wv` | MAPE 13.4% | MAPE 15.4% |
| `cad` | MAPE 40.1% | MAPE 27.7% |
| `SpT`/`SdT` | MAPE 28-137% | MAPE 26-58% |
| `SpL`/`SdL` | MAPE 200-2800% (outlier `CA_test09`) | **MAPE 52-98%, sin outliers** |

`CA_test09` ya no tiene valores disparatados (antes 87/39/76 m de "longitud
de zancada", ahora 0.07-0.13 m como el resto de bags). El error residual de
`SpL`/`SdL`/`SpT`/`SdT` (25-100% MAPE) es ahora ruido "normal" de deteccion,
no un unico frame corrupto contaminando toda la media -- consistente con
que `gait_monitor_speed.py` sigue promediando con `nanmean` simple sin
rechazar outliers puntuales (no corregido, y ya no hace tanta falta con el
EKF corregido, pero sigue siendo una fragilidad de diseño a tener en cuenta
si aparece otro bug de esta familia).

### Rehecho con recorte a la ventana `/tag` (puntos 1 y 2)

Con el recorte a `/tag` activado por defecto (ver seccion arriba) en ambos
lados de la comparacion, sobre los 47 bags completos:

| Campo | Bag completo (estado anterior) | Ventana `/tag` (final) |
|---|---|---|
| `NoS` | MAE=24.6, MAPE=24.0% | MAE=22.9, medianaAE=16, MAPE=25.6% (similar) |
| `d` | MAPE 1.3% | MAPE 3.5% (peor, ver nota abajo) |
| `wv` | MAPE 15.4% | **MAPE 5.9%** (mejor) |
| `cad` | MAPE 27.7% | **MAPE 23.4%** (mejor) |
| `SpT`/`SdT` | MAPE 26-58% | MAPE 24-76% (similar, `right_spt` peor en algun bag puntual) |
| `SpL`/`SdL` | MAPE 52-98% | MAPE 49-87% (similar) |

`wv`/`cad` mejoran claramente (igual que ya se vio en mocap-only: promediar
sobre la ventana activa da una cadencia/velocidad mas representativa que
diluirla con tiempo de calibracion). `d` empeora un poco -- hipotesis mas
probable: con una ventana mas corta, el tiempo de arranque del pipeline real
(`partial_loads` necesita ver las 4 fuentes frescas antes de publicar nada,
`first_data_ready_`) es una fraccion mayor del total, así que se pierde
proporcionalmente mas distancia real al principio de la ventana que con el
bag completo. No investigado mas a fondo. El resto de campos se mantiene en
el mismo orden de magnitud que sin recortar -- el recorte a `/tag` no
"arregla" nada adicional sobre el punto 2 en si (esa mejora ya vino de los
fixes del hilo walker_step_detector), pero da una medida mas representativa
de la marcha real del usuario en ambos metodos, que es lo que se pidio.

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

Resultados con recorte `/tag` (por defecto ahora, `--ignore-tags` para el
bag completo): `/tmp/mocap_posture_tagged_full.json`, sin fallos en los 47
bags. El recorte cambia mucho una metrica en concreto: **doble apoyo**, que
con el bag completo salia 6-67% (dominado por el tiempo de pie quieto antes
y despues del test, que es doble apoyo permanente pero no marcha), baja a
un rango mucho mas realista de **3-56%** una vez restringido a la ventana
activa -- p.ej. `CA_test01` 44.8% -> 11.6%, `CA_test02` 56.8% -> 9.9%. El
resto de parametros cambia menos: inclinacion de tronco 3.9-35.7 (similar
rango, algo mas alta en general al quitar los tramos de pie quieto/recto),
ancho de paso 0.08-0.42 m, margen lateral 0.26-0.85.

**Correlacion "maneta baja -> mas inclinacion" mas clara con el recorte**:
CA (postura inclinada, `handle_height=0`, test07/08) 24.2 grados de media
vs 11.7 en el resto de tests del mismo sujeto (antes del recorte: 15.2 vs
11.3 -- la separacion se hace mucho mas nitida al quitar el ruido de los
tramos sin marcha real); MT (test19/20) 14.6 vs 9.8 (antes 13.4 vs 9.0).
Es exactamente el tipo de correlacion mocap<->parametro-del-andador (aqui,
`handle_height`) que busca el punto 3 del plan, y ahora con menos
solapamiento que antes de recortar a `/tag`.

Cruzar el resto de estos parametros con datos PUBLICADOS por el andador
(inclinacion de tronco vs asimetria real de `/left_handle`-`/right_handle`,
o el margen de estabilidad de aqui contra `StabilityStamped.sta`) es el
paso siguiente logico -- requiere decidir que bags tienen datos de
manetas/centroide utilizables (ver limitaciones del punto 2 arriba: la
extraccion en vivo depende del mismo pipeline con el problema de
`km_detect_steps` ya descrito) y no se ha hecho todavia.

## Punto 3: centro de gravedad (De Leva) vs `support_centroid` del andador

`cog_estimation.py` estima el CoM de cuerpo completo por frame combinando
la postura del mocap con el sexo y el peso del usuario (de
`test_config.txt`), usando el modelo antropometrico segmentario de
**De Leva (1996)** ("Adjustments to Zatsiorsky-Seluyanov's segment inertia
parameters", J Biomech 29(9):1223-30 -- tabla verificada contra el paper
original, no de memoria): cada segmento (tronco, brazo, antebrazo, muslo,
pantorrilla) se calcula interpolando entre sus dos articulaciones segun la
fraccion de posicion de De Leva; los 3 segmentos sin marcador propio en el
esquema de 13 keypoints (cabeza, manos, pies -- sin vertice/dedos/punta de
pie) se aproximan con su masa concentrada en la articulacion proximal
disponible (hombro, muñeca, tobillo). Esos 3 segmentos suman ~8.5% de la
masa corporal, y el sesgo que introduce esa aproximacion es casi todo
vertical (con el tronco razonablemente recto, cabeza/manos/pies caen casi
encima de su articulacion proximal en el plano horizontal) -- para
comparar la posicion horizontal contra `support_centroid` (el uso
previsto), el efecto es minimo. La ALTURA del usuario no se usa para
derivar la posicion (los segmentos ya tienen longitud real, medida por el
mocap en cada frame) -- solo como contraste (`com_height_frac_of_stature`).

**Resultados mocap-only sobre los 47 bags, con recorte `/tag`**
(`compute_cog_from_mocap.py`, `/tmp/mocap_cog_tagged_full.json`): sin
fallos en ningun bag. Fraccion CoM/estatura entre 0.552 y 0.608 (media
~0.59) -- practicamente identica al calculo sobre el bag completo (0.552-
0.613): a diferencia del doble apoyo (arriba), la altura del CoM no se ve
afectada por el tiempo de pie quieto (la postura en reposo es similar en
altura a la postura de marcha). Consistente con el valor tipico de
literatura para un adulto de pie (~0.55) -- algo mas alto, razonable dado
que el usuario sujeta las manetas del andador con los brazos levantados en
vez de colgando a los lados. Patron por sujeto: MT (67 kg) sistematicamente
mas bajo (0.552-0.578) que CA/LA/MF (0.571-0.608) -- para interpretar esto
como algo mas que variacion individual de proporciones corporales hace
falta cruzarlo con la altura real de manetas usada en cada test
(`handle_height` de `test_config.txt`), no hecho todavia.

**Comparacion contra el andador**: `compare_cog_mocap_vs_walker.py` anade
`walker_centroid_support` (topics por defecto, sin remapear: ya coinciden
con lo que publica `partial_loads`) a `replay_offline.launch.py`, graba
`/support_centroid` durante el replay real, y lo compara (media sobre toda
la grabacion, en frame "laser", proyectando el CoM de mocap con la misma
calibracion ICP que el punto 1/2 -- comparar medias en vez de instante a
instante porque `support_centroid` hereda el ruido de deteccion de piernas
que le queda a `km_detect_steps`).

**Resultado sobre los 47 bags: error medio ~15 cm entre el CoM de mocap
(De Leva) y el centroide de carga real del andador** -- mediana 0.155 m,
media 0.145 m excluyendo el bag con outlier (ver abajo), rango 3.6-29 cm en
el resto. Es una correlacion solida para una primera comparacion: dos
metodos completamente independientes (postura+antropometria vs celdas de
carga) situan el punto de equilibrio del usuario en el mismo sitio con
~15 cm de margen, sobre un andador cuya base mide del orden de 1 m.

`x` de ambos metodos es consistentemente negativo (-0.4 a -0.85 m mocap,
-0.5 a -0.79 m andador) y del mismo orden en todos los bags -- el usuario
carga tipicamente por detras del origen del frame "laser" en ambos metodos,
como cabria esperar. `y` de ambos ronda 0 en la mayoria de bags (usuario
centrado lateralmente), con las mismas excepciones logicas (MF_test17/18,
CA_test06: asimetria real, mayor en el metodo de mocap que en el del
andador -- consistente con que `support_centroid` reparte peso segun carga
MEDIDA en los 4 puntos de contacto, mientras que el CoM de mocap refleja la
postura completa, que puede desviarse mas).

**`CA_test09` sale como outlier severo** (`walker_y`=35.7 m, `dist_err_m`
=35.8 m): la MISMA deteccion espuria de `km_detect_steps` que ya contamino
`SpL`/`SdL` de `gait_monitor_speed` en este bag (punto 2 arriba) contamina
tambien `support_centroid` -- confirma que es un problema sistemico de
"sin filtro de confianza + promediado no robusto" que afecta a CUALQUIER
nodo que promedie sobre toda la sesion (`gait_monitor_speed` Y
`centroid_support`, ambos), no algo especifico de uno de los dos.

### Rehecho con el fix del EKF (`t==0`) + recorte a `/tag`

Repetido tras los fixes del punto 2 (RF + `t==0` del EKF) y con el recorte
a la ventana `/tag` activado por defecto en ambos lados: **error medio
0.107 m, mediana 0.106 m, maximo 0.199 m -- sin ningun outlier en los 47
bags** (antes: mediana 0.155 m con `CA_test09` disparado a 35.8 m). Mejora
en las tres cosas a la vez: el fix del EKF quita el outlier de raiz (igual
que en el punto 2), y el recorte a `/tag` quita el tiempo de pie quieto
(que desplaza el centroide real hacia una postura de reposo distinta de la
postura de marcha que describe el CoM de mocap). El patron de `x`/`y` por
sujeto/bag se mantiene igual que antes (ver parrafo anterior).

## Informe completo y flujo de ejecucion

`scripts/README.md` documenta el orden de ejecucion de todos los scripts
(metricas mocap -> `export_bag_timeseries.py` -> `derive_comparison_metrics.py`
-> `build_master_report_data.py` -> `generate_latex_report.py`). El informe
LaTeX resultante (11 puntos + ampliacion, tablas por bag con agregados POR
SUJETO CA/LA/MF/MT mas una fila TOTAL, formulas, bibliografia) se genera
con `generate_latex_report.py`; no se versiona el .tex/.pdf, se regenera.

Convencion de agregacion: cada bag es una repeticion de test de uno de 4
sujetos (prefijo `CA`/`LA`/`MF`/`MT`; 10/7/20/10 bags). Los agregados se
dan por sujeto, no mezclando los 47 bags, porque difieren en sexo, peso y
altura.

## Ampliacion: parametros nuevos con los sensores actuales del andador

Candidatos de la literatura que usan sensores que el andador YA lleva y
cuya precision se puede corroborar con mocap. Se comprobo contra los
topics realmente grabados en los bags (no solo contra el codigo): cada bag
trae `/bwt901cl/imu` + `/imu_filtered` (IMU WitMotion, en produccion solo
alimenta la velocidad angular del EKF) y los ticks crudos
`/left_wheel`/`/right_wheel`.

| # | Parametro | Sensor real | Referencia mocap | Modulo / script |
|---|---|---|---|---|
| 1 | RMS, Harmonic Ratio (Moe-Nilssen 1998), regularidad de paso/zancada Ad1/Ad2 (Moe-Nilssen & Helbostad 2004) | IMU del chasis (`/bwt901cl/imu_filtered`) | 2a derivada de `rigid_bodies[0]` | `imu_metrics.py`, `compute_imu_stability_metrics.py` |
| 2 | Symmetry Index (Robinson 1987) de fuerza en manetas vs asimetria cinematica de paso | `/left_handle`, `/right_handle` calibrados | SpT/SdT/SpL/SdL izq-dcha de `compute_gait_from_mocap.py` | `extra_metrics.symmetry_index`, `compute_handle_symmetry_vs_gait.py` |
| 5 | Correlacion inclinacion sagital de tronco vs margen de estabilidad lateral | (ninguno nuevo) | dos parametros ya calculados | `compute_lean_vs_margin_correlation.py` |
| 3 | Validacion de `walker_stability.py` (`/user_stability`) vs margen mocap | -- | -- | **bloqueado**, ver abajo |

(El candidato 4, auditar los encoders de rueda crudos, se descarto por
decision del usuario.)

**Se aprendio por el camino (bugs propios, ya corregidos):**
- Derivar mocap dos veces sobre sus timestamps reales NO funciona: el dt
  de `rigid_bodies` es irregular (mediana ~8.3 ms, pero con valores desde
  ~0.06 ms hasta ~80 ms), y dividir por ese dt casi nulo daba RMS de ~60
  m/s2 frente a ~0.5 del IMU. Solucion en `imu_metrics.walker_accel_ap_ml`:
  remuestrear a rejilla uniforme (60 Hz) -> media movil -> 2a derivada
  centrada de 3 puntos con dt constante.
- El Symmetry Index por muestra es inestable: la calibracion de maneta
  extrapola y da kg ligeramente negativos cerca de carga nula, con lo que
  el SI sale negativo o enorme. Para el escalar por bag se calcula el SI
  sobre la fuerza MEDIA de cada lado; para la serie por muestra
  (`handle_force_symmetry_series`) se recorta a kg>=0 y el SI es NaN si
  L+R < 0.5 kg.
- `autocorr_regularity` usa normalizacion sesgada (N en el denominador,
  N-lag en el numerador): para una señal perfectamente periodica el maximo
  teorico es (N-lag)/N < 1, no exactamente 1.

**Resultados (47 bags; IMU: 46/47, `MT_test11` sin muestras de IMU en la
ventana `/tag`):**
- IMU vs mocap: RMS_AP del mismo orden en ambas fuentes (tipicamente
  0.2-0.6 m/s2); HR y Ad1/Ad2 de la misma escala pero con mucha dispersion
  bag a bag. Excepcion conocida: `MF_test06` (RMS_AP mocap 3.2 vs IMU 0.4),
  un salto de tracking mocap que sobrevive al suavizado. El IMU esta en el
  chasis del andador, no en el tronco del usuario: los valores absolutos no
  son comparables con normas de la literatura (que usan tronco). El eje V
  del IMU incluye la gravedad (~9.8 m/s2); solo se compara AP/ML (el
  chasis apenas se mueve en z, no hay V fiable en mocap).
- Simetria por maneta: SI de maneta ~190% en el sujeto CA (carga muy
  asimetrica), 1-60% en el resto. Correlacion con la asimetria cinematica
  debil (|r| < 0.62 por sujeto, |r| < 0.16 en TOTAL): son manifestaciones
  parcialmente independientes de la asimetria; resultado honesto, no un
  fallo del metodo.
- Inclinacion vs margen lateral: r = -0.475 (CA), -0.298 (LA), -0.240
  (MF), +0.002 (MT), TOTAL -0.033. Ojo: la literatura (andadores
  instrumentados) habla del margen ANTERO-POSTERIOR, no del lateral que
  calcula `posture_metrics`; es una prueba indirecta de la hipotesis.

### Punto 3 bloqueado: `walker_stability.py`

Dos bloqueos, verificados contra el codigo y contra el contenido de los bags:
1. **Bug de aridad**: `centroid_callback()` llamaba a `get_bos_x()` con 5
   argumentos y el metodo acepta 4 (`TypeError` en cada mensaje de
   centroide). Corregido quitando el argumento sobrante `self.handle_x`
   (`handle_z` es la altura de maneta que espera `handlebar_height_m`).
2. **Frames TF ausentes**: el nodo hace `lookup_transform(base_footprint,
   left_handle_id / right_handle_id)`, y `/tf_static` de los bags solo trae
   `base_link`, `laser`, `wheel_left/right`, `witmotion`, `camera_*`. Con
   estos datos el nodo no puede calcular nada ni arreglado el bug 1. No se
   invento ningun TF para forzarlo: habria metido supuestos no verificables
   sobre la geometria de las manetas. Necesita que el robot real emita esos
   frames al grabar. Trabajo posterior de `walker_stability` va en un hilo
   aparte.

### CSV de las series nuevas

`python3 scripts/export_bag_timeseries.py --bags-dir $DATASETS --out-dir DIR --ampliacion-only`
(sin replay, solo lectura de bag; tambien se generan en la ejecucion normal).
Mismo eje `t_s` (segundos desde el primer `/tag`) que el resto de CSV:

| CSV | Columnas | Notas |
|---|---|---|
| `{bag}_imu_real_accel.csv` | `t_s, ap_m_s2, ml_m_s2, v_m_s2` | crudo, sin remuestrear; asume x=AP, y=ML, z=V (montaje no verificado fisicamente); incluye gravedad |
| `{bag}_mocap_walker_accel.csv` | `t_s, ap_m_s2, ml_m_s2` | rejilla uniforme 60 Hz |
| `{bag}_mocap_handle_symmetry.csv` | `t_s, left_kg, right_kg, si_pct` | kg>=0; `si_pct`=NaN si L+R<0.5 kg |

### Autotests nuevos

`scripts/selftest_imu_metrics.py` (RMS, HR, autocorrelacion, aceleracion
AP/ML sintetica) y `scripts/selftest_correlation_utils.py`.
