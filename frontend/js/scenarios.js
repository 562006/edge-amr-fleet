/**
 * Scenario Handlers, Task Dispatcher, and Simulation Controls.
 */

document.addEventListener('DOMContentLoaded', () => {
  const canvasRenderer = new WarehouseCanvasRenderer('warehouseCanvas');
  const dashboard = new TelemetryDashboard();

  let isBlockedAisleMode = false;

  // Initialize Communication Client
  const client = new FleetCommunicationClient(
    (state) => {
      // On state update from backend
      canvasRenderer.updateState(state);
      dashboard.updateMetrics(state.metrics, state.tick);
      dashboard.updateRobots(state.robots);
      dashboard.updateEvents(state.recent_events);

      // Update play/pause button state
      const playBtn = document.getElementById('btnPlayPause');
      if (playBtn) {
        if (state.is_running) {
          playBtn.innerHTML = '⏸ Pause';
          playBtn.className = 'btn btn-danger';
        } else {
          playBtn.innerHTML = '▶ Start';
          playBtn.className = 'btn btn-primary';
        }
      }
    },
    (connected, label = 'Online') => {
      const statusPill = document.getElementById('connectionStatus');
      if (statusPill) {
        if (connected) {
          statusPill.innerHTML = `<span class="pulse-dot"></span> ${label}`;
          statusPill.style.color = '#10b981';
        } else {
          statusPill.innerHTML = `<span class="pulse-dot" style="background:#ef4444;"></span> Offline`;
          statusPill.style.color = '#ef4444';
        }
      }
    }
  );

  // Canvas Cell Click Handler (for toggling blocked aisle)
  canvasRenderer.onCellClick = (gridX, gridY) => {
    client.toggleObstacle(gridX, gridY);
  };

  // Play / Pause Button
  const btnPlayPause = document.getElementById('btnPlayPause');
  btnPlayPause.addEventListener('click', () => {
    if (btnPlayPause.innerText.includes('Start')) {
      client.start();
    } else {
      client.pause();
    }
  });

  // Step Button
  document.getElementById('btnStep').addEventListener('click', () => {
    client.step();
  });

  // Reset Button
  document.getElementById('btnReset').addEventListener('click', () => {
    client.reset();
  });

  // Speed Buttons
  document.querySelectorAll('.btn-speed').forEach(btn => {
    btn.addEventListener('click', (e) => {
      document.querySelectorAll('.btn-speed').forEach(b => b.classList.remove('btn-primary'));
      e.target.classList.add('btn-primary');
      const speed = parseFloat(e.target.dataset.speed);
      client.setSpeed(speed);
    });
  });

  // Toggle Obstacle Mode Button
  const btnToggleObstacle = document.getElementById('btnToggleObstacle');
  btnToggleObstacle.addEventListener('click', () => {
    isBlockedAisleMode = !isBlockedAisleMode;
    if (isBlockedAisleMode) {
      btnToggleObstacle.classList.add('btn-toggle-active');
      btnToggleObstacle.innerText = '⛔ Click Map to Block (Active)';
    } else {
      btnToggleObstacle.classList.remove('btn-toggle-active');
      btnToggleObstacle.innerText = '⛔ Click Map to Block Aisle';
    }
  });

  // Scenario Buttons 1 to 6
  for (let i = 1; i <= 6; i++) {
    const btn = document.getElementById(`btnScenario${i}`);
    if (btn) {
      btn.addEventListener('click', () => {
        client.loadScenario(i);
      });
    }
  }

  // Task Creation Form
  const formCreateTask = document.getElementById('formCreateTask');
  if (formCreateTask) {
    formCreateTask.addEventListener('submit', (e) => {
      e.preventDefault();
      const desc = document.getElementById('taskDesc').value || 'Standard Cargo Delivery';
      const pickupVal = document.getElementById('selectPickup').value.split(',');
      const dropoffVal = document.getElementById('selectDropoff').value.split(',');
      const priority = document.getElementById('selectPriority').value;

      const px = parseInt(pickupVal[0], 10);
      const py = parseInt(pickupVal[1], 10);
      const dx = parseInt(dropoffVal[0], 10);
      const dy = parseInt(dropoffVal[1], 10);

      client.createTask(desc, px, py, dx, dy, priority);
      formCreateTask.reset();
    });
  }
});
