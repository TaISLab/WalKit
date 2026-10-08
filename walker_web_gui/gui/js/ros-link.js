// Thin wrapper around ROSLIB.Ros that keeps retrying the websocket
// connection (with backoff) instead of just dying silently, and reports
// its state so the UI can show something meaningful.
export class RosLink extends EventTarget {
  constructor(url) {
    super();
    this.url = url;
    this.ros = new ROSLIB.Ros({});
    this.state = 'idle'; // idle | connecting | connected | reconnecting | error
    this.nextRetryAt = null;
    this._baseRetryDelay = 1000;
    this._maxRetryDelay = 15000;
    this._retryDelay = this._baseRetryDelay;
    this._retryTimer = null;
    this._manuallyClosed = true;

    this.ros.on('connection', () => {
      this._retryDelay = this._baseRetryDelay;
      this.nextRetryAt = null;
      this._setState('connected');
    });

    this.ros.on('close', () => {
      if (this._manuallyClosed) {
        this._setState('idle');
        return;
      }
      this._setState('reconnecting');
      this._scheduleReconnect();
    });

    this.ros.on('error', (err) => {
      if (this._manuallyClosed) return;
      this._setState('error', err);
    });
  }

  connect(url) {
    if (url) this.url = url;
    this._manuallyClosed = false;
    clearTimeout(this._retryTimer);
    this._retryDelay = this._baseRetryDelay;
    this._setState('connecting');
    this.ros.connect(this.url);
  }

  reconnectNow() {
    clearTimeout(this._retryTimer);
    this._setState('connecting');
    this.ros.connect(this.url);
  }

  disconnect() {
    this._manuallyClosed = true;
    clearTimeout(this._retryTimer);
    this.ros.close();
  }

  get isConnected() {
    return this.state === 'connected';
  }

  _scheduleReconnect() {
    clearTimeout(this._retryTimer);
    const delay = this._retryDelay;
    this.nextRetryAt = Date.now() + delay;
    this._retryTimer = setTimeout(() => {
      if (this._manuallyClosed) return;
      this.ros.connect(this.url);
    }, delay);
    this._retryDelay = Math.min(this._retryDelay * 1.6, this._maxRetryDelay);
  }

  _setState(state, detail) {
    this.state = state;
    this.dispatchEvent(new CustomEvent('statechange', {
      detail: { state, detail, nextRetryAt: this.nextRetryAt }
    }));
  }
}

// A single displayed value that knows how "old" it is, so the UI can
// visibly mark it as stale/dead instead of showing a frozen number
// forever as if it were live.
export class StatField {
  constructor(el, { staleMs = 3000, deadMs = 10000, format = (v) => v, ageEl = null } = {}) {
    this.el = el;
    this.ageEl = ageEl;
    this.staleMs = staleMs;
    this.deadMs = deadMs;
    this.format = format;
    this.lastValue = null;
    this.lastTime = 0;
    this.el.classList.add('is-dead');
  }

  update(value) {
    this.lastValue = value;
    this.lastTime = Date.now();
    this.el.textContent = this.format(value);
    this.el.classList.remove('is-stale', 'is-dead');
    if (this.ageEl) this.ageEl.textContent = '';
  }

  tick(now) {
    if (!this.lastTime) return;
    const age = now - this.lastTime;
    if (age > this.deadMs) {
      this.el.classList.add('is-dead');
      this.el.classList.remove('is-stale');
      this.el.textContent = '—';
      if (this.ageEl) this.ageEl.textContent = 'sin datos';
    } else if (age > this.staleMs) {
      this.el.classList.add('is-stale');
      if (this.ageEl) this.ageEl.textContent = `hace ${Math.round(age / 1000)}s`;
    }
  }
}

export function relativeTime(ms) {
  if (ms < 1000) return 'ahora';
  return `hace ${Math.round(ms / 1000)}s`;
}
