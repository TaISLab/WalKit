# walker_web_gui

Web GUI to monitor the Walker and configure a test session, plus a console
tool to publish the same configuration. Static HTML/JS (no build step) that
talks to ROS 2 through rosbridge.

## Web GUI

rosbridge must be running:

    ros2 launch rosbridge_server rosbridge_websocket_launch.xml

Serve the page and open `http://<robot-ip>:4243/`:

    cd gui && python3 -m http.server 4243

Two tabs:
- **Monitor** (read-only): handle/leg loads, wheel encoders, IMU and a top-down
  view. Values with no recent data turn amber (stale) and then `—` (dead), so a
  frozen topic is not mistaken for a live one.
- **Sesión**: user data (id, gender, age, height, weight, Tinetti, condition)
  and handle height. **Actualizar configuración** publishes everything and is
  red while the form differs from what was last published.

Published topics: `/user_desc` (`walker_msgs/UserDesc`) and `/handle_height`
(`std_msgs/Int32`), both latched. Subscribed topics: loads, wheels, IMU, laser,
odom, and the two above.

The GUI does not start/stop the rosbag recording.

### rosbridge address

Defaults to `ws://<hostname used to open the page>:9090`. To override it, open
the page with `?ros=ws://<host>:9090` or use the gear icon next to the
connection status. The choice is remembered per browser (`localStorage`).
The connection is retried automatically with backoff.

### Layout

- `gui/index.html`: page structure.
- `gui/css/style.css`: styling.
- `gui/js/ros-link.js`: rosbridge connection with auto-reconnect, and the
  stale/dead tracking of displayed values.
- `gui/js/plot.js`: top-down Plotly view.
- `gui/js/app.js`: wires topics to the DOM and handles the configuration form.

## Console tool

Publishes the same `/user_desc` and `/handle_height` (transient_local) without
the GUI:

    ros2 run walker_web_gui console_config.py user_id:="manusete" gender:="Masculino" \
        age:="46" height:="175" weight:="104" tinetti_score:="28" \
        description:="Test ida y vuelta" handle_height:="4"

- Omitted fields take the GUI defaults (`UCAmI`, `Masculino`, 42, 175, 92, 24,
  `No previous condition`, handle height 4).
- `gender` is `Masculino` or `Femenino`, `tinetti_score` is 0-28, `handle_height`
  is 0-4.
- The node keeps running so late subscribers still get the latched values;
  Ctrl+C to exit. Add `once:="true"` to publish and exit instead.

Publishing from the console does not update the GUI form fields, and pressing
**Actualizar configuración** in the GUI overwrites what the console published.

## Licenses

Package: Creative Commons (see the repository `LICENSE`).

Bundled third-party libraries in `gui/`:
- roslibjs (`static.robotwebtools.org/`): BSD.
- plotly.js v1.58.5 (`cdn.plot.ly/`): MIT.
