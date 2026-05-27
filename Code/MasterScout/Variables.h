
unsigned short LEFT_MOTOR_SPEED;
unsigned short RIGHT_MOTOR_SPEED;

const int STOP_DISTANCE_CM     = 20;
const int CLEAR_DISTANCE_CM    = 30;
const int IR_OBSTACLE_THRESHOLD = 125;

// Schmitt trigger thresholds for blocked state hysteresis.
// BLOCK is tighter (must be close to trigger), CLEAR is looser (must be far to un-trigger).
// This prevents rapid toggling when a sensor reading hovers near the edge.
const int SCHMITT_BLOCK_CM = 13;  // ~5 inches — sets blocked = true
const int SCHMITT_CLEAR_CM = 20;  // ~8 inches — clears blocked = false

// Gap detection — a sensor that was blocked suddenly reads much farther away.
// GAP_THRESHOLD_CM is the minimum jump in distance to count as a real opening.
const int GAP_THRESHOLD_CM = 25;  // tune on hardware — too low = false gaps, too high = missed gaps

// ─── State Machine ───────────────────────────────────────────────────────────
// TRACKING  = no obstacle, driving toward beacon, Light_Search() running
// AVOIDING  = obstacle within SCHMITT_BLOCK_CM, PD correction active
enum CarState { TRACKING, AVOIDING };