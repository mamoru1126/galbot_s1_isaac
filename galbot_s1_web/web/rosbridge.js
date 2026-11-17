// Minimal rosbridge v2 client (no dependencies). Works with rosbridge_server on
// ROS 1 and ROS 2 - message types are given as "pkg/Type".
'use strict';

class RosBridge {
  constructor(url, onState) {
    this.url = url;
    this.onState = onState || (() => {});
    this.ws = null;
    this.connected = false;
    this.subs = new Map();      // topic -> {type, opts, callbacks:Set}
    this.adverts = new Map();   // topic -> type
    this.retryMs = 1000;
    this._connect();
  }

  _connect() {
    this.onState('connecting');
    let ws;
    try {
      ws = new WebSocket(this.url);
    } catch (e) {
      this._scheduleReconnect();
      return;
    }
    this.ws = ws;
    ws.onopen = () => {
      this.connected = true;
      this.retryMs = 1000;
      this.onState('connected');
      for (const [topic, type] of this.adverts) this._send({ op: 'advertise', topic, type });
      for (const [topic, s] of this.subs) this._sendSubscribe(topic, s);
    };
    ws.onclose = () => {
      const was = this.connected;
      this.connected = false;
      this.onState(was ? 'disconnected' : 'unreachable');
      this._scheduleReconnect();
    };
    ws.onerror = () => { /* onclose follows */ };
    ws.onmessage = (ev) => {
      let msg;
      try { msg = JSON.parse(ev.data); } catch (e) { return; }
      if (msg.op === 'publish') {
        const s = this.subs.get(msg.topic);
        if (s) for (const cb of s.callbacks) { try { cb(msg.msg); } catch (e) { console.error(e); } }
      } else if (msg.op === 'status' && msg.level === 'error') {
        console.warn('rosbridge:', msg.msg);
      }
    };
  }

  _scheduleReconnect() {
    setTimeout(() => this._connect(), this.retryMs);
    this.retryMs = Math.min(this.retryMs * 1.5, 5000);
  }

  _send(obj) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(obj));
      return true;
    }
    return false;
  }

  _sendSubscribe(topic, s) {
    this._send(Object.assign({ op: 'subscribe', topic, type: s.type }, s.opts));
  }

  subscribe(topic, type, callback, opts = {}) {
    let s = this.subs.get(topic);
    if (!s) {
      s = { type, opts, callbacks: new Set() };
      this.subs.set(topic, s);
      if (this.connected) this._sendSubscribe(topic, s);
    }
    s.callbacks.add(callback);
    return () => this.unsubscribe(topic, callback);
  }

  unsubscribe(topic, callback) {
    const s = this.subs.get(topic);
    if (!s) return;
    s.callbacks.delete(callback);
    if (s.callbacks.size === 0) {
      this.subs.delete(topic);
      this._send({ op: 'unsubscribe', topic });
    }
  }

  publish(topic, type, msg) {
    if (!this.adverts.has(topic)) {
      this.adverts.set(topic, type);
      this._send({ op: 'advertise', topic, type });
    }
    return this._send({ op: 'publish', topic, msg });
  }
}

window.RosBridge = RosBridge;
