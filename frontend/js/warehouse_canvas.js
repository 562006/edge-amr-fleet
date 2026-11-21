/**
 * HTML5 Canvas 2D Warehouse Renderer.
 * High-framerate rendering with continuous AMR position interpolation (lerp),
 * rack structures, charging pads, path ribbon traces, and interactive click handling.
 */

class WarehouseCanvasRenderer {
  constructor(canvasId) {
    this.canvas = document.getElementById(canvasId);
    this.ctx = this.canvas.getContext('2d');
    
    this.gridWidth = 24;
    this.gridHeight = 16;
    this.cellSize = 42;

    // Canvas size
    this.canvas.width = this.gridWidth * this.cellSize;
    this.canvas.height = this.gridHeight * this.cellSize;

    // Simulation state cache
    this.mapData = null;
    this.robots = [];
    this.interpolatedRobots = {}; // Smooth position cache

    // Interaction mode
    this.isBlockedAisleMode = false;
    this.onCellClick = null;

    this._bindEvents();
    this._startRenderLoop();
  }

  _bindEvents() {
    this.canvas.addEventListener('click', (e) => {
      const rect = this.canvas.getBoundingClientRect();
      const scaleX = this.canvas.width / rect.width;
      const scaleY = this.canvas.height / rect.height;
      const clickX = (e.clientX - rect.left) * scaleX;
      const clickY = (e.clientY - rect.top) * scaleY;

      const gridX = Math.floor(clickX / this.cellSize);
      const gridY = Math.floor(clickY / this.cellSize);

      if (gridX >= 0 && gridX < this.gridWidth && gridY >= 0 && gridY < this.gridHeight) {
        if (this.onCellClick) {
          this.onCellClick(gridX, gridY);
        }
      }
    });
  }

  updateState(state) {
    if (state.map) {
      this.mapData = state.map;
    }
    if (state.robots) {
      this.robots = state.robots;
      // Initialize or update interpolated positions
      this.robots.forEach(r => {
        if (!this.interpolatedRobots[r.id]) {
          this.interpolatedRobots[r.id] = {
            x: r.grid_x,
            y: r.grid_y,
            heading: r.heading || 0
          };
        }
      });
    }
  }

  _startRenderLoop() {
    const render = () => {
      this._interpolatePositions();
      this._draw();
      requestAnimationFrame(render);
    };
    requestAnimationFrame(render);
  }

  _interpolatePositions() {
    // Smooth lerp (linear interpolation) towards target grid position
    const lerpFactor = 0.22;
    this.robots.forEach(r => {
      const current = this.interpolatedRobots[r.id];
      if (current) {
        current.x += (r.grid_x - current.x) * lerpFactor;
        current.y += (r.grid_y - current.y) * lerpFactor;
        
        // Interpolate angle
        let targetAngle = r.heading || 0;
        let diff = targetAngle - current.heading;
        while (diff < -180) diff += 360;
        while (diff > 180) diff -= 360;
        current.heading += diff * lerpFactor;
      }
    });
  }

  _draw() {
    const ctx = this.ctx;
    const w = this.canvas.width;
    const h = this.canvas.height;
    const cs = this.cellSize;

    // 1. Clear background
    ctx.fillStyle = '#080c14';
    ctx.fillRect(0, 0, w, h);

    // 2. Draw subtle grid floor lines
    ctx.strokeStyle = '#152033';
    ctx.lineWidth = 1;
    for (let x = 0; x <= this.gridWidth; x++) {
      ctx.beginPath();
      ctx.moveTo(x * cs, 0);
      ctx.lineTo(x * cs, h);
      ctx.stroke();
    }
    for (let y = 0; y <= this.gridHeight; y++) {
      ctx.beginPath();
      ctx.moveTo(0, y * cs);
      ctx.lineTo(w, y * cs);
      ctx.stroke();
    }

    if (!this.mapData) return;

    // 3. Draw Charging Bays
    if (this.mapData.charging_bays) {
      Object.entries(this.mapData.charging_bays).forEach(([name, pos]) => {
        const px = pos[0] * cs;
        const py = pos[1] * cs;
        ctx.fillStyle = 'rgba(234, 179, 8, 0.15)';
        ctx.fillRect(px + 2, py + 2, cs - 4, cs - 4);
        ctx.strokeStyle = '#eab308';
        ctx.lineWidth = 2;
        ctx.strokeRect(px + 2, py + 2, cs - 4, cs - 4);

        // Icon
        ctx.fillStyle = '#eab308';
        ctx.font = 'bold 10px monospace';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText('⚡CHG', px + cs/2, py + cs/2);
      });
    }

    // 4. Draw Inbound Docks
    if (this.mapData.inbound_docks) {
      Object.entries(this.mapData.inbound_docks).forEach(([name, pos]) => {
        const px = pos[0] * cs;
        const py = pos[1] * cs;
        ctx.fillStyle = 'rgba(59, 130, 246, 0.15)';
        ctx.fillRect(px + 2, py + 2, cs - 4, cs - 4);
        ctx.strokeStyle = '#3b82f6';
        ctx.lineWidth = 2;
        ctx.strokeRect(px + 2, py + 2, cs - 4, cs - 4);

        ctx.fillStyle = '#60a5fa';
        ctx.font = 'bold 9px monospace';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText('IN-DOCK', px + cs/2, py + cs/2);
      });
    }

    // 5. Draw Outbound Stations
    if (this.mapData.outbound_stations) {
      Object.entries(this.mapData.outbound_stations).forEach(([name, pos]) => {
        const px = pos[0] * cs;
        const py = pos[1] * cs;
        ctx.fillStyle = 'rgba(168, 85, 247, 0.15)';
        ctx.fillRect(px + 2, py + 2, cs - 4, cs - 4);
        ctx.strokeStyle = '#a855f7';
        ctx.lineWidth = 2;
        ctx.strokeRect(px + 2, py + 2, cs - 4, cs - 4);

        ctx.fillStyle = '#c084fc';
        ctx.font = 'bold 9px monospace';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText('OUT-STN', px + cs/2, py + cs/2);
      });
    }

    // 6. Draw Storage Racks
    if (this.mapData.racks) {
      this.mapData.racks.forEach(([rx, ry]) => {
        const px = rx * cs;
        const py = ry * cs;
        ctx.fillStyle = '#1c283d';
        ctx.fillRect(px + 2, py + 2, cs - 4, cs - 4);
        ctx.strokeStyle = '#2d3f5e';
        ctx.lineWidth = 1.5;
        ctx.strokeRect(px + 2, py + 2, cs - 4, cs - 4);

        // Rack interior shelf pattern
        ctx.strokeStyle = '#3b4e72';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(px + 6, py + cs / 2);
        ctx.lineTo(px + cs - 6, py + cs / 2);
        ctx.stroke();

        // Small rack pallet dot
        ctx.fillStyle = '#475569';
        ctx.fillRect(px + cs/2 - 4, py + cs/2 - 4, 8, 8);
      });
    }

    // 7. Draw Static Obstacles (Pillars)
    if (this.mapData.static_obstacles) {
      this.mapData.static_obstacles.forEach(([ox, oy]) => {
        const px = ox * cs;
        const py = oy * cs;
        ctx.fillStyle = '#334155';
        ctx.fillRect(px + 4, py + 4, cs - 8, cs - 8);
        ctx.strokeStyle = '#64748b';
        ctx.lineWidth = 2;
        ctx.strokeRect(px + 4, py + 4, cs - 8, cs - 8);

        ctx.fillStyle = '#94a3b8';
        ctx.font = 'bold 11px sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText('✖', px + cs/2, py + cs/2);
      });
    }

    // 8. Draw Dynamic Blocked Aisles (Hazard Stripes)
    if (this.mapData.dynamic_obstacles) {
      this.mapData.dynamic_obstacles.forEach(obs => {
        const px = obs.x * cs;
        const py = obs.y * cs;
        
        // Hazard pattern
        ctx.fillStyle = 'rgba(239, 68, 68, 0.25)';
        ctx.fillRect(px + 2, py + 2, cs - 4, cs - 4);
        ctx.strokeStyle = '#ef4444';
        ctx.lineWidth = 2;
        ctx.strokeRect(px + 2, py + 2, cs - 4, cs - 4);

        // Diagonal stripes
        ctx.strokeStyle = '#f87171';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(px + 4, py + cs - 4);
        ctx.lineTo(px + cs - 4, py + 4);
        ctx.stroke();

        ctx.fillStyle = '#fff';
        ctx.font = 'bold 10px sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText('⛔', px + cs/2, py + cs/2);
      });
    }

    // 9. Draw Robot Planned Path Ribbons
    this.robots.forEach(r => {
      if (r.path && r.path.length > 0) {
        ctx.strokeStyle = r.color || '#3b82f6';
        ctx.lineWidth = 2.5;
        ctx.setLineDash([5, 4]);
        ctx.beginPath();

        const current = this.interpolatedRobots[r.id];
        const startX = (current ? current.x : r.grid_x) * cs + cs/2;
        const startY = (current ? current.y : r.grid_y) * cs + cs/2;
        ctx.moveTo(startX, startY);

        r.path.forEach(([wx, wy]) => {
          ctx.lineTo(wx * cs + cs/2, wy * cs + cs/2);
        });
        ctx.stroke();
        ctx.setLineDash([]); // Reset line dash
      }
    });

    // 10. Draw AMRs (Autonomous Mobile Robots)
    this.robots.forEach(r => {
      const interp = this.interpolatedRobots[r.id] || { x: r.grid_x, y: r.grid_y, heading: 0 };
      const cx = interp.x * cs + cs / 2;
      const cy = interp.y * cs + cs / 2;
      const radius = cs * 0.38;

      ctx.save();
      ctx.translate(cx, cy);

      // Status Aura / Glow
      if (r.status === 'DEADLOCKED') {
        ctx.fillStyle = 'rgba(239, 68, 68, 0.35)';
        ctx.beginPath();
        ctx.arc(0, 0, radius + 8, 0, Math.PI * 2);
        ctx.fill();
      } else if (r.status === 'YIELDING') {
        ctx.fillStyle = 'rgba(249, 115, 22, 0.3)';
        ctx.beginPath();
        ctx.arc(0, 0, radius + 6, 0, Math.PI * 2);
        ctx.fill();
      } else if (r.status === 'CHARGING') {
        ctx.fillStyle = 'rgba(16, 185, 129, 0.3)';
        ctx.beginPath();
        ctx.arc(0, 0, radius + 6, 0, Math.PI * 2);
        ctx.fill();
      }

      // Robot Chassis
      ctx.fillStyle = '#0f172a';
      ctx.beginPath();
      ctx.arc(0, 0, radius, 0, Math.PI * 2);
      ctx.fill();

      // Outer Ring with Team Color
      ctx.strokeStyle = r.color || '#00d2ff';
      ctx.lineWidth = 3.5;
      ctx.beginPath();
      ctx.arc(0, 0, radius, 0, Math.PI * 2);
      ctx.stroke();

      // Directional Heading Indicator
      ctx.rotate((interp.heading * Math.PI) / 180);
      ctx.fillStyle = r.color || '#00d2ff';
      ctx.beginPath();
      ctx.moveTo(radius - 2, 0);
      ctx.lineTo(radius - 9, -5);
      ctx.lineTo(radius - 9, 5);
      ctx.closePath();
      ctx.fill();
      ctx.rotate(-(interp.heading * Math.PI) / 180); // Restore orientation for text

      // AMR ID label
      ctx.fillStyle = '#ffffff';
      ctx.font = 'bold 9px monospace';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      const shortId = r.id.replace('AMR-0', 'R');
      ctx.fillText(shortId, 0, 0);

      ctx.restore();

      // Battery percentage tag above robot
      ctx.fillStyle = r.battery > 50 ? '#10b981' : (r.battery > 25 ? '#f59e0b' : '#ef4444');
      ctx.font = 'bold 8px monospace';
      ctx.textAlign = 'center';
      ctx.fillText(`${Math.round(r.battery)}%`, cx, cy - radius - 6);
    });
  }
}
