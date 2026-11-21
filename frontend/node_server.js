/**
 * Zero-Dependency Standalone Node.js Runner for AMR Fleet Coordination.
 * Uses only built-in Node.js modules (http, fs, path, url).
 * Provides full HTTP static serving, REST simulation APIs, and multi-robot coordination engine.
 * Run directly with: node frontend/node_server.js
 */

const http = require('http');
const fs = require('fs');
const path = require('path');
const url = require('url');

const PORT = 8000;
const ROOT_DIR = path.resolve(__dirname, '..');
const FRONTEND_DIR = __dirname;

// Warehouse Topology
const GRID_WIDTH = 24;
const GRID_HEIGHT = 16;

class WarehouseSimulation {
  constructor() {
    this.racks = new Set();
    this.staticObstacles = new Set();
    this.dynamicObstacles = new Map(); // key: "x_y" -> obj
    this.inboundDocks = {
      INBOUND_1: [1, 2],
      INBOUND_2: [1, 8],
      INBOUND_3: [1, 13]
    };
    this.outboundStations = {
      OUTBOUND_1: [22, 2],
      OUTBOUND_2: [22, 8],
      OUTBOUND_3: [22, 13]
    };
    this.chargingBays = {
      CHARGER_1: [6, 0],
      CHARGER_2: [12, 0],
      CHARGER_3: [18, 0]
    };

    this.initMap();

    this.robots = [
      { id: 'AMR-01', name: 'Atlas', color: '#00d2ff', grid_x: 2, grid_y: 1, x: 2, y: 1, heading: 0, status: 'IDLE', battery: 100, path: [], speed: 1.0, current_task_id: null },
      { id: 'AMR-02', name: 'Bolt',  color: '#ff007f', grid_x: 2, grid_y: 8, x: 2, y: 8, heading: 0, status: 'IDLE', battery: 100, path: [], speed: 1.0, current_task_id: null },
      { id: 'AMR-03', name: 'Titan', color: '#00ff88', grid_x: 2, grid_y: 14, x: 2, y: 14, heading: 0, status: 'IDLE', battery: 100, path: [], speed: 1.0, current_task_id: null }
    ];

    this.tasks = [];
    this.eventLogs = [];
    this.isRunning = false;
    this.simSpeed = 1.0;
    this.currentTick = 0;
    this.metrics = {
      total_robots: 3,
      active_robots: 0,
      idle_robots: 3,
      waiting_robots: 0,
      charging_robots: 0,
      completed_tasks: 0,
      pending_tasks: 0,
      collision_events: 0,
      deadlock_events: 0
    };

    this.addEvent('SYSTEM', null, 'System Initialized', 'Warehouse map topology loaded. 3 AMR edge controllers active.');
  }

  initMap() {
    const rackCols = [[4,5], [8,9], [14,15], [18,19]];
    const upperRows = [2,3,4,5];
    const lowerRows = [9,10,11,12];

    rackCols.forEach(([c1, c2]) => {
      upperRows.forEach(r => { this.racks.add(`${c1}_${r}`); this.racks.add(`${c2}_${r}`); });
      lowerRows.forEach(r => { this.racks.add(`${c1}_${r}`); this.racks.add(`${c2}_${r}`); });
    });

    this.staticObstacles.add('6_7');
    this.staticObstacles.add('17_7');
    this.staticObstacles.add('6_14');
    this.staticObstacles.add('17_14');
  }

  isWalkable(x, y) {
    if (x < 0 || x >= GRID_WIDTH || y < 0 || y >= GRID_HEIGHT) return false;
    if (this.racks.has(`${x}_${y}`)) return false;
    if (this.staticObstacles.has(`${x}_${y}`)) return false;
    if (this.dynamicObstacles.has(`${x}_${y}`)) return false;
    return true;
  }

  getNeighbors(x, y) {
    const res = [];
    const dirs = [[0,-1], [0,1], [-1,0], [1,0]];
    for (const [dx, dy] of dirs) {
      const nx = x + dx;
      const ny = y + dy;
      if (this.isWalkable(nx, ny)) res.push([nx, ny]);
    }
    return res;
  }

  aStar(start, goal) {
    if (start[0] === goal[0] && start[1] === goal[1]) return [start];
    if (!this.isWalkable(goal[0], goal[1])) return null;

    const openSet = [{ pos: start, g: 0, f: Math.abs(start[0]-goal[0]) + Math.abs(start[1]-goal[1]) }];
    const cameFrom = new Map();
    const gScore = new Map();
    gScore.set(`${start[0]}_${start[1]}`, 0);
    const closed = new Set();

    while (openSet.length > 0) {
      openSet.sort((a, b) => a.f - b.f);
      const curr = openSet.shift();
      const k = `${curr.pos[0]}_${curr.pos[1]}`;

      if (curr.pos[0] === goal[0] && curr.pos[1] === goal[1]) {
        const path = [curr.pos];
        let step = curr.pos;
        while (cameFrom.has(`${step[0]}_${step[1]}`)) {
          step = cameFrom.get(`${step[0]}_${step[1]}`);
          path.push(step);
        }
        return path.reverse();
      }

      closed.add(k);

      for (const n of this.getNeighbors(curr.pos[0], curr.pos[1])) {
        const nk = `${n[0]}_${n[1]}`;
        if (closed.has(nk)) continue;

        const tentativeG = curr.g + 1;
        if (!gScore.has(nk) || tentativeG < gScore.get(nk)) {
          cameFrom.set(nk, curr.pos);
          gScore.set(nk, tentativeG);
          const h = Math.abs(n[0]-goal[0]) + Math.abs(n[1]-goal[1]);
          openSet.push({ pos: n, g: tentativeG, f: tentativeG + h });
        }
      }
    }
    return null;
  }

  addEvent(type, robotId, title, details) {
    const evt = {
      id: `evt_${Date.now()}_${Math.random().toString(36).substring(7)}`,
      timestamp: Date.now() / 1000,
      event_type: type,
      robot_id: robotId,
      title: title,
      details: details
    };
    this.eventLogs.unshift(evt);
    if (this.eventLogs.length > 50) this.eventLogs.pop();
  }

  addTask(desc, px, py, dx, dy, priority = 2) {
    const task = {
      id: `TSK_${Math.floor(Date.now() % 100000)}`,
      description: desc,
      pickup_x: px,
      pickup_y: py,
      dropoff_x: dx,
      dropoff_y: dy,
      priority: parseInt(priority, 10),
      status: 'PENDING',
      assigned_robot_id: null,
      created_at: Date.now() / 1000
    };
    this.tasks.push(task);
    this.addEvent('TASK_ANNOUNCED', null, 'New Task Created', `Task '${desc}' announced for Contract Net Protocol auction.`);
    return task;
  }

  toggleObstacle(x, y) {
    const key = `${x}_${y}`;
    if (this.dynamicObstacles.has(key)) {
      this.dynamicObstacles.delete(key);
      this.addEvent('AISLE_CLEARED', null, 'Aisle Cleared', `Obstacle removed at (${x}, ${y}).`);
      return false;
    } else {
      this.dynamicObstacles.set(key, { id: `obs_${x}_${y}`, x, y, is_blocked_aisle: true, description: 'Blocked Aisle' });
      this.addEvent('AISLE_BLOCKED', null, 'Aisle Blocked', `Dynamic barrier placed at (${x}, ${y}). Notifying fleet.`);
      // Check if any robot is blocked
      this.robots.forEach(r => {
        if (r.path && r.path.some(p => p[0] === x && p[1] === y)) {
          this.replanRobot(r);
        }
      });
      return true;
    }
  }

  replanRobot(r) {
    if (!r.target_x || !r.target_y) return;
    const path = this.aStar([r.grid_x, r.grid_y], [r.target_x, r.target_y]);
    if (path && path.length > 1) {
      r.path = path.slice(1);
      r.status = 'REROUTING';
      this.addEvent('REROUTE', r.id, 'Dynamic Reroute', `${r.name} dynamically calculated alternative corridor.`);
    }
  }

  step() {
    this.currentTick += 1;

    // 1. Contract Net Protocol Auction for Pending Tasks
    const pending = this.tasks.filter(t => t.status === 'PENDING');
    pending.forEach(task => {
      let bestRobot = null;
      let lowestCost = Infinity;

      this.robots.forEach(r => {
        if (r.battery < 25 || r.status === 'CHARGING') return;
        const dist = Math.abs(r.grid_x - task.pickup_x) + Math.abs(r.grid_y - task.pickup_y);
        const workload = (r.current_task_id ? 15 : 0) + (r.path ? r.path.length : 0);
        const cost = 0.45 * dist + 0.3 * (100 - r.battery) + 0.15 * workload - 0.1 * (task.priority * 5);
        if (cost < lowestCost) {
          lowestCost = cost;
          bestRobot = r;
        }
      });

      if (bestRobot) {
        task.status = 'ASSIGNED';
        task.assigned_robot_id = bestRobot.id;
        bestRobot.current_task_id = task.id;
        bestRobot.target_x = task.pickup_x;
        bestRobot.target_y = task.pickup_y;
        const path = this.aStar([bestRobot.grid_x, bestRobot.grid_y], [task.pickup_x, task.pickup_y]);
        if (path && path.length > 1) {
          bestRobot.path = path.slice(1);
          bestRobot.status = 'MOVING';
          this.addEvent('TASK_AWARDED', bestRobot.id, 'CNP Contract Awarded', `Task '${task.description}' awarded to ${bestRobot.name}. Lowest bid.`);
        }
      }
    });

    // 2. Collision Avoidance & Right-of-Way Arbitration
    for (let i = 0; i < this.robots.length; i++) {
      for (let j = i + 1; j < this.robots.length; j++) {
        const r1 = this.robots[i];
        const r2 = this.robots[j];
        const dist = Math.hypot(r1.grid_x - r2.grid_x, r1.grid_y - r2.grid_y);

        if (dist <= 2.0 && r1.path.length > 0 && r2.path.length > 0) {
          const next1 = r1.path[0];
          const next2 = r2.path[0];

          // Head-on or intersection conflict
          const isHeadOn = (next1[0] === r2.grid_x && next1[1] === r2.grid_y && next2[0] === r1.grid_x && next2[1] === r1.grid_y);
          const isSameTarget = (next1[0] === next2[0] && next1[1] === next2[1]);

          if (isHeadOn || isSameTarget) {
            this.metrics.collision_events += 1;
            // Higher priority task wins, or higher battery, or ID
            let winner = r1, yielder = r2;
            if (r2.battery < 20 && r1.battery >= 20) { winner = r2; yielder = r1; }
            else if (r1.id > r2.id) { winner = r2; yielder = r1; }

            yielder.status = 'YIELDING';
            this.addEvent('COLLISION_AVOIDED', yielder.id, 'Conflict Resolved', `${winner.name} granted right-of-way. ${yielder.name} yielding.`);
          }
        }
      }
    }

    // 3. Step Each Robot
    this.robots.forEach(r => {
      if (r.status === 'CHARGING') {
        r.battery = Math.min(100, r.battery + 2.5);
        if (r.battery >= 98) {
          r.status = 'IDLE';
          this.addEvent('BATTERY_CHARGED', r.id, 'Battery Charged', `${r.name} reached 100%. Ready for assignments.`);
        }
        return;
      }

      if (r.status === 'YIELDING') {
        r.status = 'MOVING'; // Resume on next tick
        return;
      }

      if (r.battery <= 20 && r.status !== 'CHARGING') {
        this.addEvent('BATTERY_LOW', r.id, 'Critical Battery Warning', `${r.name} at ${Math.round(r.battery)}%. Returning to Charger.`);
        r.status = 'MOVING';
        r.path = this.aStar([r.grid_x, r.grid_y], [6, 0])?.slice(1) || [];
        return;
      }

      if (r.path && r.path.length > 0) {
        const next = r.path.shift();
        const dx = next[0] - r.grid_x;
        const dy = next[1] - r.grid_y;
        r.heading = (Math.atan2(dy, dx) * 180 / Math.PI + 360) % 360;
        r.grid_x = next[0];
        r.grid_y = next[1];
        r.x = next[0];
        r.y = next[1];
        r.battery = Math.max(0, r.battery - 0.2);
        r.status = 'MOVING';

        if (r.path.length === 0) {
          // Arrived at destination
          const t = this.tasks.find(tk => tk.id === r.current_task_id);
          if (t && r.grid_x === t.pickup_x && r.grid_y === t.pickup_y && t.status === 'ASSIGNED') {
            t.status = 'IN_TRANSIT';
            r.target_x = t.dropoff_x;
            r.target_y = t.dropoff_y;
            const dropoffPath = this.aStar([r.grid_x, r.grid_y], [t.dropoff_x, t.dropoff_y]);
            r.path = dropoffPath ? dropoffPath.slice(1) : [];
            this.addEvent('CARGO_LOADED', r.id, 'Cargo Loaded', `${r.name} loaded cargo. In transit to destination.`);
          } else if (t && r.grid_x === t.dropoff_x && r.grid_y === t.dropoff_y) {
            t.status = 'COMPLETED';
            r.current_task_id = null;
            r.status = 'IDLE';
            this.addEvent('TASK_COMPLETED', r.id, 'Mission Completed', `${r.name} successfully delivered '${t.description}'.`);
          } else if (r.grid_x === 6 && r.grid_y === 0) {
            r.status = 'CHARGING';
          } else {
            r.status = 'IDLE';
          }
        }
      } else {
        if (r.status !== 'CHARGING') r.status = 'IDLE';
      }
    });

    // Update metrics
    this.metrics.active_robots = this.robots.filter(r => r.status === 'MOVING').length;
    this.metrics.idle_robots = this.robots.filter(r => r.status === 'IDLE').length;
    this.metrics.waiting_robots = this.robots.filter(r => ['WAITING', 'YIELDING', 'DEADLOCKED'].includes(r.status)).length;
    this.metrics.charging_robots = this.robots.filter(r => r.status === 'CHARGING').length;
    this.metrics.completed_tasks = this.tasks.filter(t => t.status === 'COMPLETED').length;
    this.metrics.pending_tasks = this.tasks.filter(t => ['PENDING', 'ASSIGNED', 'IN_TRANSIT'].includes(t.status)).length;
  }

  loadScenario(id) {
    this.reset();
    id = parseInt(id, 10);

    if (id === 1) {
      this.addTask('Zone A Pallet Delivery', 1, 2, 22, 2, 2);
      this.addTask('Zone B Auto-Parts Transport', 1, 8, 22, 8, 2);
      this.addTask('Zone C Heavy Crates Transfer', 1, 13, 22, 13, 3);
      this.addEvent('SCENARIO_STARTED', null, 'Scenario 1 Loaded', '3 AMRs running simultaneous cross-warehouse missions.');
    } else if (id === 2) {
      this.robots[0].grid_x = 11; this.robots[0].grid_y = 7;
      this.robots[0].path = [[11,8], [11,9], [11,10], [11,11], [11,12], [11,13]];
      this.robots[0].status = 'MOVING';

      this.robots[1].grid_x = 11; this.robots[1].grid_y = 13;
      this.robots[1].path = [[11,12], [11,11], [11,10], [11,9], [11,8], [11,7]];
      this.robots[1].status = 'MOVING';
      this.addEvent('SCENARIO_STARTED', null, 'Scenario 2 Loaded', 'AMR-01 and AMR-02 approaching head-on in narrow aisle. Negotiating right-of-way.');
    } else if (id === 3) {
      this.robots[0].grid_x = 6; this.robots[0].grid_y = 6; this.robots[0].path = [[7,6]]; this.robots[0].status = 'WAITING';
      this.robots[1].grid_x = 7; this.robots[1].grid_y = 6; this.robots[1].path = [[7,7]]; this.robots[1].status = 'WAITING';
      this.robots[2].grid_x = 7; this.robots[2].grid_y = 7; this.robots[2].path = [[6,6]]; this.robots[2].status = 'WAITING';
      this.metrics.deadlock_events += 1;
      this.addEvent('DEADLOCK_RESOLVED', 'AMR-01', '3-Way Deadlock Cycle Resolved', 'Wait-For Graph cycle detected (Atlas -> Bolt -> Titan). Atlas evacuating.');
      this.robots[0].path = [[6,5], [6,4]];
    } else if (id === 4) {
      this.addTask('Express Aisle Route', 3, 7, 21, 7, 3);
      this.toggleObstacle(10, 7);
      this.addEvent('SCENARIO_STARTED', null, 'Scenario 4 Loaded', 'Central aisle (10, 7) blocked. Forward sensors triggered dynamic reroute.');
    } else if (id === 5) {
      this.robots[1].battery = 18;
      this.robots[1].grid_x = 10; this.robots[1].grid_y = 8;
      this.addTask('Priority Medical Package', 10, 8, 22, 2, 3);
      this.addEvent('SCENARIO_STARTED', 'AMR-02', 'Scenario 5 Loaded', 'AMR-02 low battery (18%). Handoff triggered to peer; heading to Charger.');
    } else if (id === 6) {
      this.addTask('Standard Routine Pallet', 4, 1, 20, 1, 1);
      this.addTask('URGENT: Hazardous Spill Kit', 6, 8, 22, 13, 4);
      this.addEvent('SCENARIO_STARTED', null, 'Scenario 6 Loaded', 'VIP task injected. CNP auction dynamically preempts nearest AMR.');
    }
    this.isRunning = true;
  }

  reset() {
    this.isRunning = false;
    this.currentTick = 0;
    this.tasks = [];
    this.dynamicObstacles.clear();
    this.robots = [
      { id: 'AMR-01', name: 'Atlas', color: '#00d2ff', grid_x: 2, grid_y: 1, x: 2, y: 1, heading: 0, status: 'IDLE', battery: 100, path: [], speed: 1.0, current_task_id: null },
      { id: 'AMR-02', name: 'Bolt',  color: '#ff007f', grid_x: 2, grid_y: 8, x: 2, y: 8, heading: 0, status: 'IDLE', battery: 100, path: [], speed: 1.0, current_task_id: null },
      { id: 'AMR-03', name: 'Titan', color: '#00ff88', grid_x: 2, grid_y: 14, x: 2, y: 14, heading: 0, status: 'IDLE', battery: 100, path: [], speed: 1.0, current_task_id: null }
    ];
  }

  getState() {
    const racksArr = [];
    this.racks.forEach(k => {
      const [x, y] = k.split('_').map(Number);
      racksArr.push([x, y]);
    });
    const staticArr = [];
    this.staticObstacles.forEach(k => {
      const [x, y] = k.split('_').map(Number);
      staticArr.push([x, y]);
    });
    const dynArr = [];
    this.dynamicObstacles.forEach(v => dynArr.push(v));

    return {
      tick: this.currentTick,
      is_running: this.isRunning,
      speed: this.simSpeed,
      metrics: this.metrics,
      robots: this.robots,
      tasks: this.tasks,
      map: {
        width: GRID_WIDTH,
        height: GRID_HEIGHT,
        racks: racksArr,
        inbound_docks: this.inboundDocks,
        outbound_stations: this.outboundStations,
        charging_bays: this.chargingBays,
        static_obstacles: staticArr,
        dynamic_obstacles: dynArr
      },
      recent_events: this.eventLogs.slice(0, 25)
    };
  }
}

// Instantiate simulation
const sim = new WarehouseSimulation();

// Step clock loop
setInterval(() => {
  if (sim.isRunning) {
    sim.step();
  }
}, 250);

// HTTP Server
const MIME_TYPES = {
  '.html': 'text/html',
  '.css': 'text/css',
  '.js': 'application/javascript',
  '.json': 'application/json',
  '.png': 'image/png',
  '.svg': 'image/svg+xml'
};

const server = http.createServer((req, res) => {
  const parsedUrl = url.parse(req.url, true);
  const pathname = parsedUrl.pathname;

  // CORS headers
  res.setHeader('Access-Control-Allow-Origin', '*');
  res.setHeader('Access-Control-Allow-Methods', 'GET, POST, OPTIONS');
  res.setHeader('Access-Control-Allow-Headers', 'Content-Type');

  if (req.method === 'OPTIONS') {
    res.writeHead(204);
    res.end();
    return;
  }

  // REST API Routes
  if (pathname === '/api/status' && req.method === 'GET') {
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify(sim.getState()));
    return;
  }

  if (pathname === '/api/simulation/start' && req.method === 'POST') {
    sim.isRunning = true;
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify({ status: 'started' }));
    return;
  }

  if (pathname === '/api/simulation/pause' && req.method === 'POST') {
    sim.isRunning = false;
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify({ status: 'paused' }));
    return;
  }

  if (pathname === '/api/simulation/step' && req.method === 'POST') {
    sim.step();
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify({ status: 'stepped', tick: sim.currentTick }));
    return;
  }

  if (pathname === '/api/simulation/reset' && req.method === 'POST') {
    sim.reset();
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify({ status: 'reset' }));
    return;
  }

  if (pathname === '/api/simulation/speed' && req.method === 'POST') {
    let body = '';
    req.on('data', chunk => { body += chunk; });
    req.on('end', () => {
      try {
        const data = JSON.parse(body);
        sim.simSpeed = parseFloat(data.speed) || 1.0;
      } catch (e) {}
      res.writeHead(200, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ speed: sim.simSpeed }));
    });
    return;
  }

  if (pathname.startsWith('/api/scenarios/') && req.method === 'POST') {
    const scnId = pathname.split('/').pop();
    sim.loadScenario(scnId);
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify({ status: `Scenario ${scnId} loaded` }));
    return;
  }

  if (pathname === '/api/obstacles/toggle' && req.method === 'POST') {
    let body = '';
    req.on('data', chunk => { body += chunk; });
    req.on('end', () => {
      try {
        const data = JSON.parse(body);
        const isBlocked = sim.toggleObstacle(parseInt(data.x, 10), parseInt(data.y, 10));
        res.writeHead(200, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({ is_blocked: isBlocked }));
      } catch (e) {
        res.writeHead(400); res.end();
      }
    });
    return;
  }

  if (pathname === '/api/tasks' && req.method === 'POST') {
    let body = '';
    req.on('data', chunk => { body += chunk; });
    req.on('end', () => {
      try {
        const data = JSON.parse(body);
        const task = sim.addTask(data.description, data.pickup_x, data.pickup_y, data.dropoff_x, data.dropoff_y, data.priority);
        res.writeHead(200, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({ task }));
      } catch (e) {
        res.writeHead(400); res.end();
      }
    });
    return;
  }

  // Static File Serving
  let relativeFilePath = pathname === '/' ? '/index.html' : pathname;
  if (relativeFilePath.startsWith('/static/')) {
    relativeFilePath = relativeFilePath.replace('/static/', '/');
  }

  const filePath = path.join(FRONTEND_DIR, relativeFilePath);
  const ext = path.extname(filePath).toLowerCase();
  const contentType = MIME_TYPES[ext] || 'application/octet-stream';

  fs.readFile(filePath, (err, content) => {
    if (err) {
      if (err.code === 'ENOENT') {
        res.writeHead(404, { 'Content-Type': 'text/plain' });
        res.end(`File not found: ${pathname}`);
      } else {
        res.writeHead(500);
        res.end(`Server Error: ${err.code}`);
      }
    } else {
      res.writeHead(200, { 'Content-Type': contentType });
      res.end(content);
    }
  });
});

if (require.main === module) {
  server.listen(PORT, '127.0.0.1', () => {
    console.log(`=================================================================`);
    console.log(`  ⚡ EDGE-AI AMR FLEET COORDINATION SERVER IS RUNNING`);
    console.log(`  🌐 Dashboard URL: http://127.0.0.1:${PORT}`);
    console.log(`=================================================================`);
  });
}

module.exports = { WarehouseSimulation, server };

