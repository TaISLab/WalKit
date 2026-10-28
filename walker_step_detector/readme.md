step detector package

step detector relies on 

active area

leg detection

kalman filtering

Ver `docs/detect_steps_algorithm.md` para una descripción detallada del
algoritmo de `detect_steps` (la variante recomendada) y `mocap_step_detector/eval_results/`
para los datos de validación contra ground truth de mocap.

## Visualización

- Los pasos detectados (`/detected_step_left`, `/detected_step_right`) los dibuja `walker_rviz`: la capa `detected_steps` (apagada por
  defecto; `show_detected_steps:=true`) los pinta como esfera (izquierda) y cubo (derecha), y la capa `steps` pone cada pie en el paso
  detectado con su última carga. La vista `step_detector` (`ros2 launch walker_rviz walker_view.launch.py view:=step_detector
  sim_time:=true show_detected_steps:=true`) sustituye al antiguo `config/test_step_detector.rviz`. Probada en pantalla con un replay de
  `mocap_analysis/replay_feet.launch.py`: los pies caen sobre los arcos de `/scan_feet`, porque el paso detectado sale con el mismo retardo
  que el scan (0.13-0.14 s).
- Los marcadores que publican los propios detectores (`/clusters`, `/segments_viz`) no se han tocado: necesitan el estado interno del
  detector y las vistas solo los muestran.
- Los launchers de prueba (`test_step_detector.launch.py`, `test_step_detector_variants.launch.py`) usan la cadena de filtros de
  `walker_laser_filter` (`/scan_filtered`, `/scan_feet`). Más en el README de `walker_rviz`.
