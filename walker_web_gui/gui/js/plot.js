// Top-down view of the walker: feet loads, handle loads, laser points and
// the current advance direction. Same geometry/idea as the original GUI,
// but wrapped so stale traces visibly fade out instead of sitting there
// looking live forever.

const STALE_MS = 3000;
const DEAD_OPACITY = 0.15;
const STALE_OPACITY = 0.45;

// base_link coordinates for the fixed markers (handles), converted to the
// plot's 90-degree-rotated frame (x_plot = -y, y_plot = x).
const ORIG = {
  handleLeft: { x: 0.2, y: 0.25 },
  handleRight: { x: 0.2, y: -0.25 },
};
const toPlot = (p) => ({ x: -p.y, y: p.x });
const handleLeft = toPlot(ORIG.handleLeft);
const handleRight = toPlot(ORIG.handleRight);

export class WalkerPlot {
  constructor(elId) {
    this.el = document.getElementById(elId);
    this._lastUpdate = { leftFoot: 0, rightFoot: 0, laser: 0, handles: 0, odom: 0 };

    this.data = [
      { // 0: left foot
        x: [0], y: [0.15], type: 'scatter', mode: 'markers', name: 'left',
        marker: { color: [0], size: 20, colorscale: 'Bluered', cmin: 0, cmax: 50, opacity: DEAD_OPACITY },
        showlegend: false,
      },
      { // 1: right foot
        x: [0], y: [-0.15], type: 'scatter', mode: 'markers', name: 'right',
        marker: {
          color: [0], size: 20, symbol: 'square', colorscale: 'Bluered', cmin: 0, cmax: 50,
          opacity: DEAD_OPACITY,
          colorbar: { title: 'Carga (kg)', titleside: 'top' },
        },
        showlegend: false,
      },
      { // 2: laser
        x: [], y: [], type: 'scatter', mode: 'markers', name: 'laser',
        marker: { color: 'rgb(255, 0, 255)', size: 2, opacity: DEAD_OPACITY },
        showlegend: false,
      },
      { // 3: advance direction arrow tip (used via annotation, kept as marker for legend-free simplicity)
        x: [0], y: [1.5 * handleRight.y], type: 'scatter', mode: 'markers', name: 'avance',
        marker: { color: 'rgb(0,0,0)', size: 10, symbol: 'star-triangle-up', opacity: DEAD_OPACITY },
        showlegend: false,
      },
      { // 4: handle loads
        x: [handleLeft.x, handleRight.x], y: [handleLeft.y, handleRight.y],
        type: 'scatter', mode: 'markers', name: 'handles',
        marker: {
          color: [0, 0], size: 20, symbol: 'diamond-tall', colorscale: 'Bluered', cmin: 0, cmax: 50,
          opacity: DEAD_OPACITY,
        },
        showlegend: false,
      },
    ];

    this.layout = {
      margin: { l: 40, r: 10, t: 10, b: 30 },
      xaxis: { autorange: false, range: [-1, 1], zeroline: false },
      yaxis: { autorange: false, range: [-1, 1], scaleanchor: 'x', scaleratio: 1, zeroline: false },
      annotations: [{
        text: '', arrowsize: 2, arrowwidth: 1, xref: 'x', yref: 'y', showarrow: true,
        arrowhead: 2, arrowcolor: 'rgb(0,180,0)', axref: 'x', ayref: 'y',
        x: 0, y: 0.3, ax: 0, ay: 0.1,
      }],
    };

    Plotly.newPlot(this.el, this.data, this.layout, { displayModeBar: false });
  }

  _touch(key) {
    this._lastUpdate[key] = Date.now();
  }

  setFoot(side, x, y, load) {
    const idx = side === 'left' ? 0 : 1;
    this.data[idx].x = [x];
    this.data[idx].y = [y];
    this.data[idx].marker.color = [load];
    this.data[idx].marker.opacity = 1;
    this._touch(side === 'left' ? 'leftFoot' : 'rightFoot');
    this._grow(x, y);
    this._redraw();
  }

  setLaser(xs, ys) {
    this.data[2].x = xs;
    this.data[2].y = ys;
    this.data[2].marker.opacity = 1;
    this._touch('laser');
    for (let i = 0; i < xs.length; i++) this._grow(xs[i], ys[i]);
    this._redraw();
  }

  setHandleLoads(left, right) {
    this.data[4].marker.color = [left, right];
    this.data[4].marker.opacity = 1;
    this._touch('handles');
    this._redraw();
  }

  setAdvance(x, y) {
    this.layout.annotations[0].x = x;
    this.layout.annotations[0].y = y;
    this._touch('odom');
    this._redraw();
  }

  _grow(x, y) {
    const xr = this.layout.xaxis.range;
    const yr = this.layout.yaxis.range;
    xr[0] = Math.min(xr[0], x);
    xr[1] = Math.max(xr[1], x);
    yr[0] = Math.min(yr[0], y);
    yr[1] = Math.max(yr[1], y);
  }

  _redraw() {
    Plotly.react(this.el, this.data, this.layout);
  }

  // Fade traces whose data hasn't updated in a while, so an operator can
  // tell "no laser data" apart from "robot standing still".
  tick(now) {
    let changed = false;
    const fade = (idx, key) => {
      const age = now - (this._lastUpdate[key] || 0);
      const target = !this._lastUpdate[key] ? DEAD_OPACITY : age > STALE_MS ? STALE_OPACITY : 1;
      if (this.data[idx].marker.opacity !== target) {
        this.data[idx].marker.opacity = target;
        changed = true;
      }
    };
    fade(0, 'leftFoot');
    fade(1, 'rightFoot');
    fade(2, 'laser');
    fade(4, 'handles');
    if (changed) this._redraw();
  }
}
