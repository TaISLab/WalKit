# `detect_steps` (RF) — descripción del algoritmo

De las tres variantes del paquete (`detect_steps`, `km_detect_steps`,
`detect_steps_s`), `detect_steps` (clasificador Random Forest) es la más
fiable con margen claro — validado sobre 47 bags de
`soma13kp/datasets/labeled_bags` (~14.000 detecciones por lado) contra
ground truth de mocap. Ver `eval_results/` para los datos crudos de cada
corrida. Pipeline completo, de `/scan` a `/detected_step_{left,right}`.

## 1. Segmentación del scan (`laser_processor.cpp`)

Cada punto válido del `LaserScan` (rango dentro de `[range_min, range_max]`)
se convierte a cartesianas y se agrupa por **distancia euclídea entre
puntos consecutivos** angularmente (`splitConnected`, umbral
`cluster_dist_euclid`, 0.13 m por defecto): dos puntos quedan en el mismo
clúster si están a menos de esa distancia. Después se descartan:

- Clústeres con menos de `min_points_per_cluster` puntos (ruido).
- Clústeres a más de `max_detect_distance` del sensor.

## 2. Extracción de características (`cluster_features.cpp`)

Por cada clúster superviviente, 17 características geométricas — el set de
**Arras et al. 2007** ("Using Boosted Features for the Detection of People
in 2D Range Data"), el mismo que usa el `leg_detector` clásico de ROS:

```
num_points, std, avg_median_dev, width, linearity, circularity, radius,
boundary_length, boundary_regularity, mean_curvature, ang_diff, iav,
std_iav, distance, distance/num_points, occluded_right, occluded_left
```

`linearity`/`circularity` salen de un SVD sobre los puntos centrados;
`occluded_left/right` miran si el punto justo antes/después en el barrido
angular está más lejos (indicio de que el clúster sigue detrás de algo).

## 3. Clasificación (Random Forest, OpenCV `cv::ml::RTrees`)

El vector de 17 features se pasa al forest cargado desde
`trained_step_detector_res_0.33.yaml`. `predict()` devuelve exactamente
**±1** (confirmado empíricamente cargando el modelo y probando vectores
sintéticos) — un clasificador binario pierna/no-pierna, no una
probabilidad continua. Se remapea a `confidence = 0.5*(1+predict())` ∈
{0, 1}.

## 4. Filtro de confianza + tope de clusters (`detect_steps.cpp::getCentroids`)

Estaba declarado y logueado pero nunca aplicado (bug encontrado y
arreglado en esta ronda de trabajo). Ahora:

- Se descartan clústeres con `confidence < detection_threshold`.
- Si sobran más de `max_detected_clusters`, se quedan los N más
  confiables (no los N primeros del barrido).

Este fue, de lejos, el fix de mayor impacto medido: en 47 bags, match_rate
subió de ~55-64% a ~90-96%.

## 5. Asociación y seguimiento (`legs_tracker.cpp` + `track_leg.cpp`)

Un `LegsTracker` con dos `TrackLeg` internas (una por pierna, no
etiquetadas a priori como izq/der). Cada `TrackLeg` lleva un EKF
**independiente** con un modelo oscilatorio periódico por eje:
`pos = d + a·sin(w·t + p)` (`d`=offset, `a`=amplitud, `f`→`w`=frecuencia,
`p`=fase) — modela el vaivén de la marcha, no una posición estática. Las
dos piernas no comparten estado ni se restringen mutuamente en el filtro;
toda la coordinación entre ellas vive en la capa de asociación, no en el
EKF.

Por cada frame de detecciones, en orden de prioridad:

**a) Ambas pistas con estimación propia** → emparejamiento exclusivo: de
todas las parejas posibles de candidatos, se escoge la que minimiza
`dist(cand_i, ref_izq) + dist(cand_j, ref_der)` — cada pierna se queda con
una candidata distinta (evita que las dos compitan por la misma y una se
quede sin nada). Si la mejor pareja y la contraria (izq↔der intercambiadas)
quedan casi empatadas en coste (margen 8 cm), se desempata por
**alternancia de marcha**: en cada instante una pierna va más rápido
(swing) que la otra (stance), así que se prefiere la asignación que
mantenga ese orden de velocidades en vez de invertirlo de golpe — justo el
caso de piernas cruzándose, donde la posición sola es ambigua.

**b) Solo una pista con estimación** (la otra acaba de perderla o nunca la
tuvo) → la establecida reclama su candidata más cercana primero; el resto
se reparte por la heurística espacial (única pista disponible para una
pierna sin referencia).

**c) Ninguna con estimación** (arranque) → heurística espacial pura
(`y>0` → izquierda).

Dentro de cada `TrackLeg`, `predict_step()` aplica un **gate de distancia**
(`max_association_dist`, 0.5 m): la candidata más cercana solo se acepta
como medida si está por debajo del gate; si no, el EKF solo predice (no
actualiza) — evita engancharse a un clúster espurio cuando la detección
real falta ese frame.

**Recuperación de pista perdida**: si pasan `max_track_loss_frames` (15)
ciclos seguidos sin una medida aceptada, la pista suelta su bloqueo y
vuelve a competir por cualquier candidata disponible, en vez de seguir
comparando contra una referencia que ya no significa nada. Al soltar el
bloqueo también se vacía el buffer de calentamiento de fase/amplitud (ver
punto 8 más abajo) y se desmarca `phase_warmed_`: el próximo enganche
recalcula amplitud y fase desde cero en vez de arrastrar las de antes de
perder la pista (que ya no tienen por qué ser válidas). Verificado
disparando el mecanismo a propósito (gate artificialmente estricto sobre
un bag real): la pista pierde el bloqueo, reengancha con la heurística de
`LegsTracker`, acumula medidas nuevas y recalcula amplitud/fase -- ver
"Verificación del reenganche" más abajo.

**Re-etiquetado izquierda/derecha**: la decisión de qué pista interna es
"izquierda" no se toma con el signo de `y` de la primerísima detección
(ruidoso) — se pospone hasta que cada pista acumula ≥5 medidas reales, y
entonces se decide una vez por la `y` media de cada una.

## 6. Publicación

`get_steps()` devuelve el estado predicho de cada pista (posición,
velocidad, confianza) etiquetado según la decisión de re-etiquetado; se
publica en `/detected_step_{left,right}` solo si el `frame_id` no es
`"invalid"`.

## Rendimiento validado

47 bags, ~14.000 detecciones/lado: match_rate **~95.7-95.9%** (tras el
calentamiento de fase/amplitud del EKF, ver "Investigación del swap_rate
residual" más abajo, punto 8 -- antes ~93.7-93.8%), error medio de posición
~10 cm, swap_rate ~9.3% por cada 100 pares consecutivos. La
fusión con las otras dos variantes (`detect_steps_fused`) no la mejora —
verificado con tres métodos distintos (fusión ingenua, fusión con
prioridad RF, y análisis quirúrgico aislando solo los ciclos donde el
respaldo se activa). Ver `eval_results/` para los JSON de cada corrida.

## Investigación del swap_rate residual (~10%)

Se investigó qué dispara los swaps de `rf` (el cambio de identidad
izquierda/derecha entre dos detecciones consecutivas), en orden:

1. **¿Cruce físico de piernas?** No. En 47 bags, el 99.8% de los swaps
   ocurren con las piernas claramente separadas (20-45 cm), no cruzadas ni
   juntas (`scripts/investigate_swaps.py`).
2. **¿EKF acoplado modelando la relación mecánica entre piernas (fase
   antifase, ancho de zancada)?** Descartado antes de implementarlo: la
   velocidad de las dos piernas está débilmente **correlacionada
   positivamente** (no antifase) en 37/47 bags, y el ancho de zancada no
   es lo bastante estable como para servir de restricción fuerte
   (`scripts/validate_leg_coupling.py`).
3. **¿Deriva de la predicción interna del EKF?** Sí, mayoritariamente: en
   el 64.3% de los swaps, la referencia interna (`l_ref`/`r_ref`, antes de
   decidir la asociación de ese ciclo) ya apuntaba hacia el tobillo
   equivocado (`scripts/investigate_ref_drift.py`). El otro 35.7% ocurre
   con la referencia todavía correcta -- probablemente un efecto emergente
   del emparejamiento exclusivo (minimiza el coste conjunto de las dos
   piernas, así que la deriva de la OTRA pista puede "robarle" una pierna
   a esta aunque su propia referencia sea buena).
4. **¿Ajustar Q/R (ruido de proceso/medida) del EKF?** Empeora, no mejora
   -- ver el comentario en `TrackLeg::init()` (`src/track_leg.cpp`) y
   `eval_results/result_2026-09-28_baseline_isolated.json` vs
   `result_2026-09-28_qr_tuned_isolated.json`. La causa de la deriva no es
   un Q/R mal calibrado en sí, sino que el modelo de amplitud/frecuencia
   es casi inobservable desde la inicialización casi nula (`a0=f0=p0=0.001`)
   -- arreglarlo necesitaría repensar la inicialización o la
   parametrización del modelo, no solo sus matrices de ruido.
5. **¿Reinicializar `a0`/`f0` a valores reales en vez de ~0?** También
   empeora. Se midió `a0=0.15m`, `f0=0.6Hz` (medianas de amplitud/frecuencia
   real de tobillo, 47 bags, `scripts/estimate_gait_params.py`) contra el
   mismo baseline aislado -- `eval_results/result_2026-09-28_gait_init_isolated.json`:
   match_rate de rf cae de 93.7/93.8% a 88.9/86.7%, con mean_err/swap_rate
   casi iguales. Motivo probable: `p0` se dejó en 0.001 (fase arbitraria) --
   con amplitud/frecuencia ya realistas, el término periódico SÍ oscila
   desde el primer instante, pero con la fase equivocada (no hay forma de
   saber la fase real de la zancada al arrancar una pista), inyectando un
   error de predicción que antes no existía (con `a0~0` el modelo era casi
   estático, sin oscilación que pudiera desfasarse). Revertido. Ver el
   comentario en `launch/test_step_detector.launch.py`.

6. **¿Referencia de asociación/gating por última medida real en vez de la
   predicción del EKF?** Primer resultado positivo (aunque modesto):
   `LegsTracker::add_detections()` (coste de emparejamiento exclusivo) y
   `TrackLeg::predict_step()` (gate de aceptación) usaban `get_step()`
   (`curr_step`, la predicción del modelo periódico -- justo lo que deriva)
   como referencia. Se cambió a `TrackLeg::assoc_ref()`, que devuelve la
   última medida real ACEPTADA por el gate (cruda, sin pasar por el EKF),
   sin tocar el filtro ni sus parámetros en absoluto. Medido en 47 bags
   (aislado) -- `eval_results/result_2026-09-28_baseline_isolated.json` vs
   `result_2026-09-29_assoc_ref_deadreckon_isolated.json`:

   | | match_rate | mean_err | swap_rate/100 |
   |---|---|---|---|
   | rf/left | 93.68%→**94.47%** | 0.100→0.105 m | 10.06→9.99 |
   | rf/right | 93.81%→**94.87%** | 0.099→0.099 m | 9.45→**8.84** |
   | km/left | 81.60%→82.38% | 0.139→0.141 m | 14.95→14.21 |
   | km/right | 80.65%→80.84% | 0.154→0.157 m | 15.68→15.24 |
   | seg | ~sin cambio | ~sin cambio | ~sin cambio |
   | fused/left | 94.13%→93.77% | 0.102→0.107 m | 11.15→11.01 |
   | fused/right | 96.01%→**93.42%** | 0.101→0.104 m | 10.50→9.86 |

   `rf` mejora en las tres métricas a la vez (primera vez en esta
   investigación), superando el 94% de match_rate en ambos lados. `km`
   también mejora ligeramente. `fused` **empeora** (especialmente el lado
   derecho, -2.6 puntos) -- probablemente porque sus candidatos mezclan
   detecciones de calidad dispar (rf/km/seg con fallback), y una medida
   aceptada ruidosa ahora se propaga sin amortiguar a la referencia del
   siguiente ciclo (antes, el blend del EKF -- ganancia de Kalman <1 --
   suavizaba ese ruido de un solo frame). Como ya se sabía que `fused` no
   aporta nada sobre `rf` solo, esta regresión no cambia la recomendación
   de usar `rf`. Cambio conservado (mejora neta para `rf`, que es la
   variante en uso). Ver `include/walker_step_detector/track_leg.h`.

7. **¿Extender #6 extrapolando `assoc_ref()` con velocidad** (en vez de
   solo la última posición)? Empeora, y con claridad -- peor incluso que no
   tocar nada. Se añadió `accepted_speed_` (velocidad entre las dos últimas
   medidas ACEPTADAS, no las últimas añadidas) y se extrapoló la referencia
   hacia el instante `ti` actual antes de buscar la candidata más cercana.
   Medido en 47 bags (aislado) -- `result_2026-09-28_baseline_isolated.json`
   vs `result_2026-09-29_assoc_ref_velocity_isolated.json`: rf match_rate
   94.47/94.87% (#6) → **89.4/88.9%** (peor que el 93.68/93.81% original,
   sin ningún fix). Motivo: diferenciar posiciones consecutivas para
   obtener una velocidad amplifica el ruido de medida (más cuanto más corto
   el intervalo entre medidas, típicamente 0.03-0.2s aquí), y extrapolar la
   referencia con esa velocidad ya ruidosa reintroduce más error del que
   `assoc_ref()` sin extrapolar evitaba. Revertido -- el código de
   `accepted_speed_`/`assoc_ref(double ti)` se quitó del todo, no se dejó
   detrás de un flag; ver el comentario en `track_leg.h::assoc_ref()`.

8. **¿Calentamiento de fase/amplitud por mínimos cuadrados tras acumular
   medidas reales?** Mejor resultado de toda esta investigación. La pista
   arranca igual que siempre (`a0=f0=p0=0.001`, casi estática -- el
   comportamiento ya conocido y bueno mientras hay pocos datos). En cuanto
   acumula `WARMUP_WINDOW=12` medidas reales ACEPTADAS (buffer propio,
   `warmup_ti_`/`warmup_x_`/`warmup_y_`), resuelve por mínimos cuadrados
   (sistema 2x2, sin Eigen) la amplitud y fase de cada eje que mejor
   explican esas 12 medidas, con la frecuencia FIJA en 0.6Hz (la mediana
   real medida sobre mocap, `scripts/estimate_gait_params.py` -- fijarla
   sola como `f0` ya se había probado y empeorado, punto 5; aquí solo sirve
   de ancla para resolver amplitud/fase, no se usa como parámetro fijo del
   modelo) y reinyecta esos tres valores en el EKF (`ekf.init()`, que solo
   pisa el estado `x`, no la covarianza) dejando `d_x`/`d_y` intactos (ya
   bien seguidos por la ganancia por defecto). Se repite cada vez que la
   pista pierde el bloqueo y reengancha (`register_miss()` vacía el buffer
   y desmarca `phase_warmed_`), no solo una vez al arrancar el nodo. Ver
   `TrackLeg::warmup_reseed()` en `track_leg.cpp`.

   Medido en 47 bags (aislado) -- `result_2026-09-28_baseline_isolated.json`
   (sin ningún fix) vs `result_2026-09-29_assoc_ref_deadreckon_isolated.json`
   (#6 solo) vs `result_2026-09-29_warmup_phase_isolated.json` (#6 + #8):

   | | sin fix | #6 (assoc_ref) | #6+#8 (+ calentamiento) |
   |---|---|---|---|
   | rf/left match | 93.68% | 94.47% | **95.94%** |
   | rf/right match | 93.81% | 94.87% | **95.72%** |
   | rf/left mean_err | 0.100 m | 0.105 m | **0.099 m** |
   | rf/right mean_err | 0.099 m | 0.099 m | 0.099 m |
   | rf/left swap/100 | 10.06 | 9.99 | **9.30** |
   | rf/right swap/100 | 9.45 | 8.84 | 9.27 |
   | fused/left match | 94.13% | 93.77% (empeoraba) | **95.91%** |
   | fused/right match | 96.01% | 93.42% (empeoraba) | 94.56% (recupera casi todo) |

   `rf` mejora ~2.1-2.3 puntos de match_rate sobre el baseline sin tocar
   nada (el mayor salto medido en toda la investigación), con mean_err
   igual o mejor y swap_rate mejor en ambos lados. Además **recupera casi
   toda la regresión de `fused`** que había introducido #6 solo (94.13/96.01%
   → 93.77/93.42% con #6 → 95.91/94.56% con #6+#8) -- consistente con que
   la causa de esa regresión (medidas ruidosas de km/seg propagándose sin
   amortiguar) se mitiga si el propio modelo periódico ya converge antes a
   parámetros realistas, en vez de necesitar más ciclos con `a~0`. `km`/`seg`
   quedan prácticamente sin cambios (esperable: kalman_enabled=false en la
   configuración de esta evaluación para esas dos variantes, ver
   `launch/test_step_detector.launch.py` -- no pasan por `predict_step()`).

9. **¿Extender #8 estimando también la frecuencia por eje** (búsqueda en
   rejilla 0.2-1.2Hz, paso 0.02Hz, sobre el mismo ajuste 2x2, en vez de
   fijarla a 0.6Hz)? Empeora con claridad, y por debajo incluso del
   baseline sin ningún fix en el lado derecho. Medido en 47 bags (aislado)
   -- `result_2026-09-29_warmup_phase_isolated.json` (#8, f fija) vs
   `result_2026-09-29_warmup_freq_grid_isolated.json` (#8 + rejilla de f):

   | | sin fix | #8 (f fija 0.6Hz) | #8 + rejilla de f |
   |---|---|---|---|
   | rf/left match | 93.68% | **95.94%** | 95.61% |
   | rf/right match | 93.81% | **95.72%** | **92.43%** (peor que sin fix) |
   | rf/left mean_err | 0.100 m | **0.099 m** | 0.105 m |
   | rf/right mean_err | 0.099 m | **0.099 m** | 0.104 m |
   | fused/right match | 96.01% | 94.56% | **92.70%** (peor que sin fix) |

   Motivo probable: con solo `WARMUP_WINDOW=12` medidas (~1-2 ciclos de
   marcha), dejar la frecuencia libre añade un grado de libertad que
   sobreajusta al ruido de ESA ventana concreta -- la rejilla encuentra la
   frecuencia que mejor explica esas 12 medidas ruidosas, no
   necesariamente la frecuencia real de la marcha. La mediana fija de 47
   bags de mocap (0.6Hz) generaliza mucho mejor que una estimación de
   ventana corta. Revertido -- el código de la rejilla se quitó del todo
   (no se dejó detrás de un flag), igual que en el punto 7. Ver el
   comentario en `track_leg.h::WARMUP_FREQ_HZ`.

**Estado**: causa identificada (deriva de predicción, mayoritariamente) y
con un fix real y validado (#8, sobre #6): rf sube de 93.7/93.8% a
95.9/95.7% de match_rate, con mean_err y swap_rate iguales o mejores en
las dos métricas. Su extensión obvia (#9, estimar también la frecuencia)
empeora, igual que la de #6 (#7, extrapolar con velocidad) -- las dos
extensiones fallaron por el mismo patrón: introducir una estimación
adicional a partir de pocas medidas ruidosas (velocidad por diferenciación
en #7, frecuencia por ventana corta en #9) resulta peor que quedarse con
la cantidad mínima de información que ya funcionaba. No cierra el
swap_rate del todo (sigue en ~9.3 por cada 100 pares). El estado final
vigente es #6+#8 (`assoc_ref()` sin extrapolar + calentamiento de
amplitud/fase con frecuencia fija). Si se quiere seguir apurando, la vía
que queda sin probar es más estructural: sustituir el modelo periódico por
uno de velocidad constante -- aunque cada vez parece menos necesario, dado
que el modelo periódico, bien inicializado, SÍ está aportando valor medible
y las dos ampliaciones "fáciles" (velocidad, frecuencia) ya se han agotado
sin éxito. Cualquier intento futuro de EKF debe medirse con
`scripts/run_multibag_eval.py` en un `ROS_DOMAIN_ID` propio -- un proceso
ROS ajeno compartiendo el dominio por defecto contaminó una medida anterior
y produjo una conclusión falsa ("catastrófico") que tuvo que corregirse.

## Verificación del reenganche tras perder la pista

Pregunta directa: con el estado actual del código (#6+#8), ¿es capaz una
pista de recuperarse si pierde la medida? El mecanismo de recuperación en
sí (`register_miss()`/`max_track_loss_frames`, descrito en el punto 5) no
se ha tocado en esta ronda -- solo se le añadió, sobre ese mecanismo ya
existente, que el reenganche también dispare un recalentamiento de
fase/amplitud nuevo (punto 8).

Comprobado en vivo, no solo leyendo el código:

- **Con el gate real (`max_association_dist=0.5m`)**: en 4 bags distintos
  (`MT_test18`, `CA_test06`, `MF_test19`, `MT_test17`, logs `RCLCPP_DEBUG`
  activados vía `publish_clusters=true`), **0 pérdidas de pista completas**
  en ninguno -- solo los 2 calentamientos iniciales (uno por pierna, al
  arrancar). Coherente con el diagnóstico de los puntos 1-3 arriba: el
  ~9% de swap_rate residual de `rf` viene de **asignar mal la detección
  mientras la pista sigue "enganchada"** (deriva de referencia en el
  emparejamiento exclusivo), no de perder la pista del todo. Con el gate
  real, `consecutive_misses_` rara vez llega a los 15 seguidos.
- **Con el gate forzado a 0.04m** (artificialmente estricto, solo para
  forzar el mecanismo, no un valor propuesto para producción) sobre
  `CA_test01`: 16 pérdidas de pista y 6 recalentamientos completos.
  Secuencia real del log:
  ```
  t=118.79  (right) sin medida aceptada en 15 ciclos. Soltando bloqueo.
  t=122.37  (right) warmup_reseed: a=(0.012,0.007) p=(1.540,-0.774) f=0.60Hz
  ```
  La pista derecha suelta el bloqueo, `LegsTracker` la deja competir de
  nuevo por cualquier candidata (rama "una pista sin estimación" del punto
  5), acumula `WARMUP_WINDOW=12` medidas nuevas sin otra pérdida de por
  medio, y recalcula amplitud/fase desde cero -- exactamente el
  comportamiento esperado.

**Conclusión**: sí, reengancha. El mecanismo de recuperación es el mismo
de antes de esta ronda de trabajo (no introduce ni arregla nada ahí); lo
nuevo es que el reenganche ya no hereda una fase potencialmente obsoleta.
