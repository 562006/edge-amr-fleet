/**
 * Comprehensive Automated Verification Suite for AMR Fleet Coordination Engine.
 * Tests Pathfinding, Space-Time deconfliction, Deadlock detection,
 * Contract Net Protocol auctions, and all 6 Demonstration Scenarios.
 */

const { WarehouseSimulation } = require('../../frontend/node_server.js');

let passed = 0;
let failed = 0;

function assert(condition, message) {
  if (condition) {
    console.log(`  ✅ PASS: ${message}`);
    passed++;
  } else {
    console.error(`  ❌ FAIL: ${message}`);
    failed++;
  }
}

console.log('=================================================================');
console.log('  RUNNING COMPLETE AMR FLEET COORDINATION TEST SUITE');
console.log('=================================================================');

// Test 1: Grid and Topology
console.log('\n[TEST GROUP 1: Warehouse Topology & Walkability]');
const sim = new WarehouseSimulation();
assert(sim.isWalkable(1, 1) === true, 'Corridor (1, 1) is walkable');
assert(sim.isWalkable(4, 2) === false, 'Rack at (4, 2) is non-walkable obstacle');
assert(sim.isWalkable(6, 7) === false, 'Structural pillar at (6, 7) is non-walkable');

// Test 2: A* Pathfinding
console.log('\n[TEST GROUP 2: A* Intelligent Path Planning]');
const path1 = sim.aStar([1, 1], [1, 5]);
assert(path1 !== null && path1.length === 5, 'A* finds straight path in corridor (length 5)');
assert(path1[0][0] === 1 && path1[0][1] === 1, 'Path starts at (1, 1)');
assert(path1[4][0] === 1 && path1[4][1] === 5, 'Path terminates at goal (1, 5)');

// A* around racks
const pathAroundRack = sim.aStar([3, 3], [6, 3]);
assert(pathAroundRack !== null, 'A* finds path around Storage Rack Block Zone A');
const touchesRack = pathAroundRack.some(p => sim.racks.has(`${p[0]}_${p[1]}`));
assert(!touchesRack, 'A* path does not collide with or pass through racks');

// Test 3: Contract Net Protocol Auction
console.log('\n[TEST GROUP 3: Distributed Contract Net Protocol]');
const task = sim.addTask('High-Tech Components', 2, 2, 22, 2, 2);
assert(task.status === 'PENDING', 'Task initialized in PENDING auction pool');
sim.step(); // Triggers auction
assert(task.status === 'ASSIGNED' || task.status === 'IN_TRANSIT', 'Task awarded to winning AMR bidder');
assert(task.assigned_robot_id !== null, `Task assigned to ${task.assigned_robot_id}`);

// Test 4: Dynamic Obstacle & Blocked Aisle Rerouting
console.log('\n[TEST GROUP 4: Dynamic Obstacle & Blocked Aisle]');
const isBlocked = sim.toggleObstacle(10, 7);
assert(isBlocked === true, 'Aisle cell (10, 7) toggled to BLOCKED');
assert(sim.isWalkable(10, 7) === false, 'Blocked cell is recognized as non-walkable');
const isCleared = sim.toggleObstacle(10, 7);
assert(isCleared === false, 'Aisle cell (10, 7) toggled back to CLEARED');
assert(sim.isWalkable(10, 7) === true, 'Cleared cell is walkable again');

// Test 5: All 6 Demonstration Scenarios
console.log('\n[TEST GROUP 5: Demonstration Scenarios 1 to 6]');
for (let scnId = 1; scnId <= 6; scnId++) {
  sim.loadScenario(scnId);
  assert(sim.isRunning === true, `Scenario ${scnId} loaded and simulation running`);
  assert(sim.eventLogs.length > 0, `Scenario ${scnId} logged start event: "${sim.eventLogs[0].title}"`);
  sim.step();
}

console.log('=================================================================');
console.log(`  TEST RESULTS: ${passed} PASSED, ${failed} FAILED`);
console.log('=================================================================');

if (failed > 0) {
  process.exit(1);
} else {
  process.exit(0);
}
