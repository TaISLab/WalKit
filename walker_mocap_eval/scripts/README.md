# scripts/ — orden de ejecución

Todos estos scripts son Python plano (no forman parte del build de
colcon). Antes de ejecutar cualquiera, desde una terminal nueva:

```bash
source /opt/ros/jazzy/setup.bash
source /home/mfcarmona/workspace/walker_ws/install/setup.bash
cd WalKit/walker_mocap_eval
```

`$DATASETS` en los ejemplos = `/home/mfcarmona/workspace/walker_ws/src/soma13kp/datasets/labeled_bags`
(47 bags etiquetados, 4 sujetos: `CA`, `LA`, `MF`, `MT`).

## 0. Autotests (opcional, sin bags)

Validan la lógica portada con datos sintéticos, antes de tocar bags reales:

```bash
python3 scripts/selftest_gait_metrics.py
python3 scripts/selftest_posture_metrics.py
python3 scripts/selftest_cog_estimation.py
python3 scripts/selftest_imu_metrics.py
python3 scripts/selftest_correlation_utils.py
```

## 1. Cálculo de métricas desde mocap (4 scripts independientes)

Cada uno lee los bags etiquetados directamente (sin reproducir nada) y
escribe un JSON con un registro por bag. Se pueden correr en cualquier
orden / en paralelo.

```bash
python3 scripts/compute_gait_from_mocap.py \
    --bags-dir $DATASETS --out /tmp/report_gait_mocap.json
# Punto 1: SpT/SdT/SpL/SdL por pierna, NoS, Tr, d, CAD, WV

python3 scripts/compute_posture_from_mocap.py \
    --bags-dir $DATASETS --out /tmp/report_posture_mocap.json
# Inclinación de tronco, ancho de paso, doble apoyo, margen de estabilidad lateral

python3 scripts/compute_cog_from_mocap.py \
    --bags-dir $DATASETS --out /tmp/report_cog_mocap.json
# Punto 3: centro de gravedad (modelo De Leva 1996), requiere test_config.txt
# (peso/altura/sexo por bag; por defecto ~/bagsFolder_unified/test_config.txt)

python3 scripts/compute_extra_metrics_from_mocap.py \
    --bags-dir $DATASETS --out /tmp/report_extra_mocap.json
# Codo, muñeca, reparto de peso en manetas, yaw/curvatura, oscilación
# vertical del CoM, índice de inestabilidad (puntos 3-elbow/4/5/8/9/11)
```

Flags comunes: `--bag <dir>` para un único bag en vez de `--bags-dir`,
`--ignore-tags` para NO recortar a la ventana `/tag` (por defecto SÍ se
recorta — ver `readme.md` del paquete).

## 2. Exportar series temporales completas (el único paso que reproduce bags)

```bash
python3 scripts/export_bag_timeseries.py \
    --bags-dir $DATASETS --out-dir /tmp/report_timeseries
```

Para cada bag: calcula las series de mocap directamente (sin reproducir),
y hace **una única reproducción** del pipeline real completo
(`box_filter` → `detect_steps` → `partial_loads` → `mocap_odom_bridge` →
`gait_monitor_speed` + `walker_centroid_support`) grabando todo lo que
publica. Escribe un CSV por serie y por fuente
(`{bag}_mocap_*.csv`, `{bag}_walker_*.csv`), todas con el mismo eje
temporal `t_s` (segundos desde el inicio del bag) para poder graficar
mocap y andador juntos. Usa `ROS_DOMAIN_ID` propio internamente para no
interferir con otros procesos ROS en la máquina.

Tarda bastante sobre los 47 bags (hay que reproducir cada uno en tiempo
real/`--rate`); usarlo en background. `--mocap-only` salta la
reproducción si solo hacen falta las series de mocap. `--limit N` para
probar sobre los N primeros bags antes de lanzar los 47.

## 3. Derivar comparación mocap-vs-andador (punto 2 + CoG) desde los CSV

No reproduce nada: relee los CSV del paso 2 y el JSON del paso 1 para
calcular errores (MAE/MedAE/Bias/RMSE/MAPE — fórmulas en el informe).

```bash
python3 scripts/derive_comparison_metrics.py \
    --bags-dir $DATASETS \
    --timeseries-dir /tmp/report_timeseries \
    --gait-json /tmp/report_gait_mocap.json \
    --out-json /tmp/report_comparison.json \
    --out-csv-dir /tmp/report_timeseries
```

Solo se usan filas del andador con `t_s` <= fin de la ventana `/tag`:
`export_bag_timeseries.py` graba unos segundos más allá del segundo `/tag`
(espera `duración/rate + 3 s`) y ese tramo, con el usuario ya parado, no
corresponde a lo que mide mocap. Además de MAE/MedAE/Bias/RMSE/MAPE se
calcula MedAPE (mediana del error porcentual), robusta a los pocos bags con
errores enormes; el informe añade filas de mediana por sujeto y TOTAL.

Añade además `{bag}_cog_comparison.csv` a `--out-csv-dir` (mocap vs
`/support_centroid` muestra a muestra). Nota: solo el plano `x`/`y` del
CoG es comparable con el centroide de soporte real — `z` y la distancia
3D se calculan pero se marcan como no comparables (ver `§2.7` del informe:
son dos magnitudes físicas distintas, no el mismo punto en otro frame).

## 3b. Ampliación: 3 scripts adicionales (sin reproducir bags)

Nuevos parámetros candidatos de la literatura, con los sensores que el
andador ya lleva (ver `§10` del informe). Se pueden correr en paralelo
entre sí y en cualquier momento después del paso 1 (necesitan sus JSON).

```bash
python3 scripts/compute_imu_stability_metrics.py \
    --bags-dir $DATASETS --gait-json /tmp/report_gait_mocap.json \
    --out /tmp/report_imu_mocap.json
# Punto 1 ampliación: Harmonic Ratio / RMS / regularidad por autocorrelación,
# IMU real del andador (/bwt901cl/imu_filtered) vs equivalente por mocap

python3 scripts/compute_handle_symmetry_vs_gait.py \
    --extra-json /tmp/report_extra_mocap.json --gait-json /tmp/report_gait_mocap.json \
    --out /tmp/report_handle_symmetry.json
# Punto 2 ampliación: Symmetry Index de fuerza en manetas (sensor real) vs
# asimetría cinemática de paso (mocap) -- post-procesa JSON ya calculados,
# no lee bags

python3 scripts/compute_lean_vs_margin_correlation.py \
    --posture-json /tmp/report_posture_mocap.json \
    --out /tmp/report_lean_vs_margin.json
# Punto 5 ampliación: correlación inclinación de tronco vs margen de
# estabilidad lateral -- sin sensor nuevo, post-procesa JSON ya calculado
```

(El punto 3 de la ampliación, validación de `walker_stability.py`, quedó
bloqueado por un bug real del nodo y por frames TF ausentes en los bags —
no tiene script propio, ver `§10.4` del informe.)

## 4. Fusionar todo en un master JSON por bag

```bash
python3 scripts/build_master_report_data.py \
    --gait /tmp/report_gait_mocap.json \
    --posture /tmp/report_posture_mocap.json \
    --cog /tmp/report_cog_mocap.json \
    --extra /tmp/report_extra_mocap.json \
    --comparison /tmp/report_comparison.json \
    --imu /tmp/report_imu_mocap.json \
    --handle-symmetry /tmp/report_handle_symmetry.json \
    --lean-vs-margin /tmp/report_lean_vs_margin.json \
    --out /tmp/report_master.json \
    --out-csv /tmp/report_timeseries/master_per_bag_summary.csv
```

No recalcula nada: solo junta los JSON de los pasos 1, 3 y 3b en un único
registro por bag (y opcionalmente una tabla plana en CSV para abrir en
una hoja de cálculo). `--imu`/`--handle-symmetry`/`--lean-vs-margin` son
opcionales (se omiten si no se han corrido los scripts de la ampliación).

## 5. Generar el informe LaTeX

```bash
python3 scripts/generate_latex_report.py \
    --master /tmp/report_master.json --out /tmp/informe/informe.tex
cd /tmp/informe && pdflatex -interaction=nonstopmode informe.tex && pdflatex -interaction=nonstopmode informe.tex
```

(`pdflatex` se ejecuta dos veces: la primera resuelve el contenido, la
segunda arregla referencias cruzadas/índice). Las tablas desglosan las 47
repeticiones y agregan **por sujeto** (`CA`/`LA`/`MF`/`MT`, no una única
media de los 47 bags) más una fila TOTAL de referencia.

## Resumen del flujo

```
                 (paso 1, 4 scripts, sin reproducir bags)
labeled_bags/ --> compute_gait_from_mocap.py       --> report_gait_mocap.json      --\
             \--> compute_posture_from_mocap.py     --> report_posture_mocap.json   --\
             \--> compute_cog_from_mocap.py         --> report_cog_mocap.json       ---+--> build_master_report_data.py --> report_master.json --> generate_latex_report.py --> informe.tex --> informe.pdf
             \--> compute_extra_metrics_from_mocap.py -> report_extra_mocap.json    --/
             \
              \--> export_bag_timeseries.py (reproduce cada bag 1 vez)
                   --> report_timeseries/*.csv --> derive_comparison_metrics.py --> report_comparison.json --/

                 (paso 3b, ampliación -- post-procesan los JSON del paso 1, sin leer bags salvo el de IMU)
report_gait_mocap.json ------------------------> compute_imu_stability_metrics.py       --> report_imu_mocap.json       --\
report_extra_mocap.json + report_gait_mocap.json -> compute_handle_symmetry_vs_gait.py  --> report_handle_symmetry.json --+--> build_master_report_data.py
report_posture_mocap.json ---------------------> compute_lean_vs_margin_correlation.py  --> report_lean_vs_margin.json   --/
```

## Otros scripts (no forman parte del flujo anterior)

- `compare_gait_mocap_vs_walker.py`, `compare_cog_mocap_vs_walker.py`:
  versiones más antiguas que reproducen cada bag en vivo y comparan
  directamente (una reproducción por comparación, en vez de reutilizar
  los CSV de `export_bag_timeseries.py`). Se conservan como referencia /
  validación cruzada, pero el flujo recomendado es el de arriba.
- `mocap_odom_bridge.py`: nodo ROS auxiliar (no CLI de análisis) que
  republica el *rigid body* del andador en mocap como `/odom`, porque
  `/left_wheel`/`/right_wheel` reales están rotos en estos bags; lo lanza
  internamente `export_bag_timeseries.py`/`compare_*_vs_walker.py` como
  parte del pipeline de reproducción.
- `eval_odom_against_mocap.py`: evalúa una odometría grabada (por defecto
  `/odom` del EKF láser+IMU de `walker_bringup`) contra el mocap: ATE tras
  alinear con Umeyama 2D, error de rumbo y deriva en % del recorrido. Es
  la vía para validar la odometría REAL del andador, que el flujo de arriba
  NO valida: en la reproducción de `export_bag_timeseries.py` el `/odom`
  sale del propio mocap (`mocap_odom_bridge.py`), así que `d` y `WV` del
  informe solo validan la aritmética de `gait_monitor_speed`. No forma
  parte del flujo del informe; ver su docstring para qué grabar.
