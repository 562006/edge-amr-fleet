/**
 * Telemetry and Dashboard State Binder.
 * Updates metric cards, AMR status cards, event logs, and status badges.
 */

class TelemetryDashboard {
  constructor() {
    this.amrCardsContainer = document.getElementById('amrCardsContainer');
    this.eventsFeed = document.getElementById('eventsFeed');
    this.lastEventIds = new Set();
  }

  updateMetrics(metrics, tick) {
    if (!metrics) return;

    document.getElementById('metricActive').innerText = metrics.active_robots ?? 0;
    document.getElementById('metricIdle').innerText = metrics.idle_robots ?? 0;
    document.getElementById('metricWaiting').innerText = metrics.waiting_robots ?? 0;
    document.getElementById('metricCharging').innerText = metrics.charging_robots ?? 0;
    document.getElementById('metricCompleted').innerText = metrics.completed_tasks ?? 0;
    document.getElementById('metricPending').innerText = metrics.pending_tasks ?? 0;
    document.getElementById('metricCollisions').innerText = metrics.collision_events ?? 0;
    document.getElementById('metricDeadlocks').innerText = metrics.deadlock_events ?? 0;

    if (tick !== undefined) {
      document.getElementById('tickCounter').innerText = `TICK: ${tick}`;
    }
  }

  updateRobots(robots) {
    if (!robots || !this.amrCardsContainer) return;

    this.amrCardsContainer.innerHTML = '';

    robots.forEach(r => {
      const card = document.createElement('div');
      card.className = 'amr-card';
      card.style.borderLeftColor = r.color || '#3b82f6';

      const battLevel = Math.round(r.battery);
      let battColor = '#10b981';
      if (battLevel <= 25) battColor = '#ef4444';
      else if (battLevel <= 50) battColor = '#f59e0b';

      const taskDesc = r.current_task_id ? r.current_task_id : 'None (Idle)';
      const remainingSteps = r.path ? r.path.length : 0;

      card.innerHTML = `
        <div class="amr-card-header">
          <span class="amr-name" style="color: ${r.color || '#fff'}">
            🤖 ${r.name} (${r.id})
          </span>
          <span class="amr-badge badge-${r.status}">
            ${r.status}
          </span>
        </div>
        <div class="amr-info-grid">
          <div>Pos: <strong>(${r.grid_x}, ${r.grid_y})</strong></div>
          <div>Path: <strong>${remainingSteps} steps</strong></div>
          <div>Task: <strong>${taskDesc}</strong></div>
          <div>Speed: <strong>${(r.speed || 1.0).toFixed(1)}x</strong></div>
        </div>
        <div class="battery-container">
          <div style="display:flex; justify-content:space-between; font-size:0.72rem; color:var(--text-dim);">
            <span>Battery</span>
            <span style="color:${battColor}; font-weight:700;">${battLevel}%</span>
          </div>
          <div class="battery-bar-bg">
            <div class="battery-bar-fill" style="width: ${battLevel}%; background-color: ${battColor};"></div>
          </div>
        </div>
      `;
      this.amrCardsContainer.appendChild(card);
    });
  }

  updateEvents(recentEvents) {
    if (!recentEvents || !this.eventsFeed) return;

    this.eventsFeed.innerHTML = '';

    recentEvents.forEach(evt => {
      const timeStr = new Date(evt.timestamp * 1000).toLocaleTimeString();
      const div = document.createElement('div');
      div.className = `event-entry evt-${evt.event_type}`;

      div.innerHTML = `
        <div class="event-top">
          <span>${evt.title}</span>
          <span class="event-time">${timeStr}</span>
        </div>
        <div class="event-desc">${evt.details}</div>
      `;
      this.eventsFeed.appendChild(div);
    });
  }
}
