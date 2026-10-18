# walker_replay

Única forma de reproducir los rosbags definitivos de Walkit. Quien necesite
datos grabados no diseña nada sobre la reproducción: añade este módulo a su
launcher y recibe exactamente lo grabado, con su tiempo.

## Qué hace

1. Reproduce `<bag_folder>/<bag_name>` con `ros2 bag play`, publicando `/clock`
   (200 Hz por defecto).
2. Publica, latcheado, lo que una grabación del andador no trae y el sistema
   espera, a partir de `<bag_folder>/test_config.txt` y del URDF:
   - `/user_desc` (`walker_msgs/UserDesc`; `tinetti_score` = 0, test_config no
     tiene esa columna; `gender` = `Masculino`/`Femenino` como en la GUI),
   - `/handle_height` (`std_msgs/Int32`),
   - `/robot_description` (`std_msgs/String`, `transient_local`): el URDF, de
     `walker_state_publisher` (los bags definitivos no lo traen),
   - `/tf_static`: el árbol del URDF (`walker_state_publisher`) y los frames
     `left_handle_id`/`right_handle_id` según el `handle_height` del test
     (`walker_description/config/params.yaml`). La altura de las manetas es
     constante durante un test, por eso son estáticos, no el `/tf` dinámico a
     0.5 s del sistema en vivo.
3. Propaga `use_sim_time:=true` (parámetro global y launch configuration
   `use_sim_time`) a todo lo que se lance **después** del módulo.
4. Solo reproduce bags definitivos (carpeta con `definitive_log.md`, ver
   `soma13kp/GENERAR_BAGS_DEFINITIVOS.md`); cualquier otro se rechaza con un
   mensaje que dice qué hacer.

Los nodos propios van en el namespace `/walker_replay`.

## Uso

```bash
ros2 launch walker_replay replay.launch.py bag_name:=MF_test07 rate:=1.0
ros2 launch walker_replay replay.launch.py bag_name:=CA_test01 topics:='/scan /left_handle /right_handle' \
    start_offset:=20.0 shutdown_on_end:=true
```

| argumento | por defecto | |
|---|---|---|
| `bag_folder` | `~/bagsFolder/definitive` | donde están los bags y `test_config.txt` |
| `bag_name` | `MF_test01` | |
| `rate` | `1.0` | velocidad de reproducción |
| `topics` | (todos) | topics a reproducir, separados por espacios |
| `exclude_topics` | | topics a no reproducir (siempre se excluyen `/rosout` y `/parameter_events`, salvo que se pidan en `topics`) |
| `start_offset` | `0.0` | segundos de bag desde los que empezar (no afectado por `rate`) |
| `duration` | (hasta el final) | segundos a reproducir desde `start_offset` |
| `delay` | `3.0` | espera antes de reproducir, para que los consumidores estén listos |
| `remap` | | `old:=new old2:=new2` |
| `loop` | `false` | |
| `clock` | `true` | publicar `/clock` |
| `clock_hz` | `200` | ver abajo |
| `sim_time` | `true` | propagar `use_sim_time:=true`; exige `clock` |
| `debug` | `false` | logs en debug e imprime el comando del reproductor |
| `shutdown_on_end` | `false` | apagar todo el launch al acabar el bag |
| `publish_context` | `true` | `/user_desc` y `/handle_height` |
| `publish_tf` | `true` | tf estático del andador y `/robot_description` (mismo `walker_state_publisher`) |

### Desde otro launcher

```python
from walker_replay.launch_utils import replay_actions

ld.add_action(...)                      # tus argumentos
for a in replay_actions(bag_name=LaunchConfiguration('bag_name'),
                        rate=LaunchConfiguration('rate'),
                        topics='/scan /left_handle /right_handle'):
    ld.add_action(a)                    # ANTES de tus nodos
ld.add_action(Node(...))                # recibe use_sim_time:=true
```

Los argumentos del launch del módulo no se filtran al launcher que lo llama
(`rate`, `bag_name`... son nombres muy comunes); `use_sim_time` sí, a propósito.
Los procesos lanzados con `ExecuteProcess` (`python3 script.py --ros-args`) no
reciben el parámetro global: pásales `-p use_sim_time:=true`.

## Leer un bag sin reproducirlo (`bagio`) y lanzar sesiones (`session`)

Reproducir un bag es de este paquete, y mirar dentro de uno también: las herramientas que analizan bags (comparaciones
contra el mocap) no llevan su propio lector.

```python
from walker_replay import bagio, session

bagio.topics(bag)                  # {topic: tipo}, de metadata.yaml (sin abrir el bag)
bagio.duration_s(bag); bagio.start_ns(bag); bagio.storage_id(bag); bagio.message_counts(bag)
bagio.find_bags(carpeta)           # carpetas con metadata.yaml
bagio.load_messages(bag, ['/scan', '/rigid_bodies'])    # {topic: [(t_ns, msg)]}; BagError si falta alguno
bagio.iter_messages(bag, topics)   # (topic, t_ns, msg) en orden de bag; iter_raw() sin deserializar
bagio.read_tag_window(bag)         # (t_inicio, t_fin) entre los dos /tag, o None
bagio.clip_series(serie, t0, t1)
with bagio.BagWriter(salida) as w: # escribir un bag nuevo
    w.add_topic('/odom', 'nav_msgs/msg/Odometry'); w.write('/odom', t_ns, msg)

proc = session.start('ros2 launch ...', domain_id=92, log='launch.log')   # en su propio grupo de procesos
session.stop(proc)                 # SIGINT, luego SIGTERM, luego SIGKILL, a todo el grupo
```

`bagio` no sabe qué significan los topics (eso es de quien los pide) y no necesita ROS para los metadatos; `rosbag2_py`
se importa al leer o escribir. `session` sustituye a las seis copias de `sh()`/`stop()` que había en los scripts de
evaluación; resetea la disposición de SIGINT (un script lanzado en segundo plano con `&` la hereda ignorada y
`ros2 bag record` no terminaría de escribir su bag).

## Decisiones

- **`clock_hz` 200**: `ros2 bag play` publica `/clock` a 40 Hz por defecto y los
  temporizadores en tiempo simulado (el del EKF, 30 Hz) disparan una vez por tick
  de reloj: a 40 Hz la salida del EKF caía a ~14-19 Hz reales y cambiaba con
  `rate`; a 200 Hz es ~30 Hz y no depende de `rate`.
- **Sin `--qos-profile-overrides-path`**: los bags definitivos no traen topics
  latcheados (`/tf`, `/tf_static` y `/robot_description` se eliminan al
  generarlos) ni perfiles QoS grabados. Los topics latcheados los publica este
  módulo con `transient_local`.
- **`sim_time` sin `clock` se rechaza**: sin `/clock` los nodos esperarían un
  reloj que nadie publica. `clock:=true sim_time:=false` es posible (solo para
  depurar): todo lo que los nodos estampen con `now()` será hora de pared, no de
  bag.
- **El tf sale del URDF, no de la grabación**: el `/tf_static` que traían los
  bags originales tenía el frame `witmotion` con roll 180° (un error, corregido en
  el URDF en WalKit 8f1e8a3: el IMU va con z arriba). Reproducir el URDF actual
  es, por tanto, correcto y distinto de aquella grabación.
