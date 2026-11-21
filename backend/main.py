"""
FastAPI Server Entry Point for Edge-AI AMR Fleet Coordination System.
Provides REST APIs, WebSocket real-time broadcast, and serves the frontend dashboard.
"""

import asyncio
from typing import List, Dict, Any
from pathlib import Path
from contextlib import asynccontextmanager

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from config import TICK_INTERVAL_SEC, SERVER_HOST, SERVER_PORT
from backend.warehouse.fleet_manager import FleetManager
from backend import database
from backend.models import TaskPriority

# Global Fleet Coordinator instance
fleet_manager = FleetManager()

# Connected WebSocket clients set
active_connections: List[WebSocket] = []

async def broadcast_state():
    """Pushes current simulation state to all active WebSocket clients."""
    if not active_connections:
        return
    state = fleet_manager.get_simulation_state()
    disconnected = []
    for ws in active_connections:
        try:
            await ws.send_json(state)
        except Exception:
            disconnected.append(ws)
    for ws in disconnected:
        if ws in active_connections:
            active_connections.remove(ws)

async def simulation_loop():
    """Background simulation clock running at designated tick interval."""
    while True:
        try:
            if fleet_manager.is_running:
                fleet_manager.step()
                await broadcast_state()
            interval = max(0.05, TICK_INTERVAL_SEC / fleet_manager.sim_speed)
            await asyncio.sleep(interval)
        except asyncio.CancelledError:
            break
        except Exception as e:
            print(f"Simulation tick error: {e}")
            await asyncio.sleep(0.5)

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup: Initialize SQLite DB and launch simulation loop
    database.init_db()
    sim_task = asyncio.create_task(simulation_loop())
    yield
    # Shutdown
    sim_task.cancel()

app = FastAPI(
    title="Edge-AI AMR Fleet Coordination",
    description="Distributed Multi-Agent Fleet Coordination for Smart Warehouses",
    version="1.0.0",
    lifespan=lifespan
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Request Models
class CreateTaskRequest(BaseModel):
    description: str
    pickup_x: int
    pickup_y: int
    dropoff_x: int
    dropoff_y: int
    priority: int = 2 # 1=LOW, 2=NORMAL, 3=HIGH, 4=URGENT

class ToggleObstacleRequest(BaseModel):
    x: int
    y: int

class SetSpeedRequest(BaseModel):
    speed: float

# REST API Endpoints

@app.get("/api/status")
def get_status():
    return fleet_manager.get_simulation_state()

@app.post("/api/simulation/start")
async def start_sim():
    fleet_manager.is_running = True
    await broadcast_state()
    return {"status": "started", "is_running": True}

@app.post("/api/simulation/pause")
async def pause_sim():
    fleet_manager.is_running = False
    await broadcast_state()
    return {"status": "paused", "is_running": False}

@app.post("/api/simulation/step")
async def step_sim():
    fleet_manager.step()
    await broadcast_state()
    return {"status": "stepped", "tick": fleet_manager.current_tick}

@app.post("/api/simulation/reset")
async def reset_sim():
    fleet_manager.reset_simulation()
    await broadcast_state()
    return {"status": "reset"}

@app.post("/api/simulation/speed")
async def set_speed(req: SetSpeedRequest):
    fleet_manager.sim_speed = max(0.2, min(5.0, req.speed))
    await broadcast_state()
    return {"speed": fleet_manager.sim_speed}

@app.post("/api/scenarios/{scenario_id}")
async def run_scenario(scenario_id: int):
    if scenario_id < 1 or scenario_id > 6:
        raise HTTPException(status_code=400, detail="Scenario ID must be between 1 and 6")
    fleet_manager.load_scenario(scenario_id)
    fleet_manager.is_running = True
    await broadcast_state()
    return {"status": f"Scenario {scenario_id} loaded and started"}

@app.post("/api/tasks")
async def create_task(req: CreateTaskRequest):
    priority_enum = TaskPriority(req.priority)
    task = fleet_manager.add_task(
        description=req.description,
        pickup_x=req.pickup_x,
        pickup_y=req.pickup_y,
        dropoff_x=req.dropoff_x,
        dropoff_y=req.dropoff_y,
        priority=priority_enum
    )
    await broadcast_state()
    return {"task": task.to_dict()}

@app.post("/api/obstacles/toggle")
async def toggle_obstacle(req: ToggleObstacleRequest):
    is_blocked = fleet_manager.trigger_blocked_aisle(req.x, req.y)
    await broadcast_state()
    return {"x": req.x, "y": req.y, "is_blocked": is_blocked}

@app.get("/api/analytics")
def get_analytics():
    return database.get_analytics_summary()

@app.get("/api/events")
def get_events():
    return database.get_recent_events(limit=40)

# WebSocket Endpoint
@app.websocket("/ws/fleet")
async def websocket_fleet_endpoint(websocket: WebSocket):
    await websocket.accept()
    active_connections.append(websocket)
    try:
        # Send initial full state immediately upon connect
        await websocket.send_json(fleet_manager.get_simulation_state())
        while True:
            data = await websocket.receive_json()
            cmd = data.get("command")
            if cmd == "start":
                fleet_manager.is_running = True
            elif cmd == "pause":
                fleet_manager.is_running = False
            elif cmd == "step":
                fleet_manager.step()
            elif cmd == "reset":
                fleet_manager.reset_simulation()
            elif cmd == "scenario":
                sc_id = data.get("scenario_id", 1)
                fleet_manager.load_scenario(int(sc_id))
                fleet_manager.is_running = True
            elif cmd == "toggle_obstacle":
                x = data.get("x")
                y = data.get("y")
                if x is not None and y is not None:
                    fleet_manager.trigger_blocked_aisle(int(x), int(y))
            elif cmd == "set_speed":
                fleet_manager.sim_speed = float(data.get("speed", 1.0))

            await broadcast_state()
    except WebSocketDisconnect:
        if websocket in active_connections:
            active_connections.remove(websocket)
    except Exception:
        if websocket in active_connections:
            active_connections.remove(websocket)

# Mount Frontend Static Assets
frontend_dir = Path(__file__).resolve().parent.parent / "frontend"
if frontend_dir.exists():
    app.mount("/static", StaticFiles(directory=str(frontend_dir)), name="static")

@app.get("/")
def serve_dashboard():
    index_file = frontend_dir / "index.html"
    if index_file.exists():
        return FileResponse(str(index_file))
    return {"message": "Frontend not found at " + str(index_file)}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("backend.main:app", host=SERVER_HOST, port=SERVER_PORT, reload=True)
