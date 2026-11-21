/**
 * WebSocket and REST Client for AMR Fleet Dashboard.
 * Connects to /ws/fleet with automatic fallback to REST polling if needed.
 */

class FleetCommunicationClient {
  constructor(onStateUpdate, onConnectionChange) {
    this.onStateUpdate = onStateUpdate;
    this.onConnectionChange = onConnectionChange;

    this.socket = null;
    this.isConnected = false;
    this.reconnectTimer = null;
    this.pollInterval = null;

    this.host = window.location.host || '127.0.0.1:8000';
    this.wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    this.httpProtocol = window.location.protocol === 'https:' ? 'https:' : 'http:';

    this.connect();
  }

  connect() {
    const wsUrl = `${this.wsProtocol}//${this.host}/ws/fleet`;
    console.log(`Connecting to WebSocket: ${wsUrl}`);

    try {
      this.socket = new WebSocket(wsUrl);

      this.socket.onopen = () => {
        console.log('WebSocket connected to AMR fleet manager.');
        this.isConnected = true;
        if (this.pollInterval) {
          clearInterval(this.pollInterval);
          this.pollInterval = null;
        }
        if (this.onConnectionChange) this.onConnectionChange(true);
      };

      this.socket.onmessage = (event) => {
        try {
          const state = JSON.parse(event.data);
          if (this.onStateUpdate) {
            this.onStateUpdate(state);
          }
        } catch (e) {
          console.error('Error parsing state update:', e);
        }
      };

      this.socket.onclose = () => {
        console.warn('WebSocket closed. Retrying in 2.5s...');
        this.isConnected = false;
        if (this.onConnectionChange) this.onConnectionChange(false);
        this._startPollingFallback();
        this.reconnectTimer = setTimeout(() => this.connect(), 2500);
      };

      this.socket.onerror = (err) => {
        console.warn('WebSocket error, falling back to HTTP polling.');
        this.socket.close();
      };
    } catch (err) {
      console.warn('WebSocket init failed, using HTTP polling fallback.');
      this._startPollingFallback();
    }
  }

  _startPollingFallback() {
    if (this.pollInterval) return;
    this.pollInterval = setInterval(async () => {
      try {
        const res = await fetch(`${this.httpProtocol}//${this.host}/api/status`);
        if (res.ok) {
          const state = await res.json();
          if (this.onStateUpdate) this.onStateUpdate(state);
          if (this.onConnectionChange) this.onConnectionChange(true, 'HTTP Polling');
        }
      } catch (e) {
        if (this.onConnectionChange) this.onConnectionChange(false, 'Disconnected');
      }
    }, 300);
  }

  sendCommand(cmd, payload = {}) {
    if (this.isConnected && this.socket && this.socket.readyState === WebSocket.OPEN) {
      this.socket.send(JSON.stringify({ command: cmd, ...payload }));
    } else {
      // Fallback: send via REST API
      const endpoints = {
        'start': '/api/simulation/start',
        'pause': '/api/simulation/pause',
        'step': '/api/simulation/step',
        'reset': '/api/simulation/reset',
      };
      if (endpoints[cmd]) {
        fetch(`${this.httpProtocol}//${this.host}${endpoints[cmd]}`, { method: 'POST' });
      }
    }
  }

  start() { this.sendCommand('start'); }
  pause() { this.sendCommand('pause'); }
  step() { this.sendCommand('step'); }
  reset() { this.sendCommand('reset'); }
  
  setSpeed(speed) {
    if (this.isConnected) {
      this.sendCommand('set_speed', { speed });
    } else {
      fetch(`${this.httpProtocol}//${this.host}/api/simulation/speed`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ speed })
      });
    }
  }

  loadScenario(scenarioId) {
    if (this.isConnected) {
      this.sendCommand('scenario', { scenario_id: scenarioId });
    } else {
      fetch(`${this.httpProtocol}//${this.host}/api/scenarios/${scenarioId}`, { method: 'POST' });
    }
  }

  toggleObstacle(x, y) {
    if (this.isConnected) {
      this.sendCommand('toggle_obstacle', { x, y });
    } else {
      fetch(`${this.httpProtocol}//${this.host}/api/obstacles/toggle`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ x, y })
      });
    }
  }

  createTask(description, pickupX, pickupY, dropoffX, dropoffY, priority) {
    fetch(`${this.httpProtocol}//${this.host}/api/tasks`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        description,
        pickup_x: pickupX,
        pickup_y: pickupY,
        dropoff_x: dropoffX,
        dropoff_y: dropoffY,
        priority: parseInt(priority, 10)
      })
    });
  }
}
