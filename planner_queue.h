#ifndef PLANNER_QUEUE_H
#define PLANNER_QUEUE_H

// This struct is used when buffering the setup for each linear movement "nominal" values are as specified in
// the source g-code and may never actually be reached if acceleration management is active.
typedef struct {
  // Fields used by the bresenham algorithm for tracing the line
  long steps[NUM_AXIS];                     // Step count along each axis
  unsigned long step_event_count;           // The number of step events required to complete this block
  long accelerate_until;                    // The index of the step event on which to stop acceleration
  long decelerate_after;                    // The index of the step event on which to start decelerating
  long acceleration_rate;                   // The acceleration rate used for acceleration calculation
  unsigned char direction_bits;             // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)
  unsigned char active_extruder;            // Selects the active extruder
  #if ENABLED(ADVANCE)
    long advance_rate;
    volatile long initial_advance;
    volatile long final_advance;
    float advance;
  #endif

  // Fields used by the motion planner to manage acceleration
  // float speed_x, speed_y, speed_z, speed_e;          // Nominal mm/sec for each axis
  float nominal_speed;                               // The nominal speed for this block in mm/sec
  float entry_speed;                                 // Entry speed at previous-current junction in mm/sec
  float max_entry_speed;                             // Maximum allowable junction entry speed in mm/sec
  float millimeters;                                 // The total travel of this block in mm
  float acceleration;                                // acceleration mm/sec^2
  unsigned char recalculate_flag;                    // Planner flag to recalculate trapezoids on entry junction
  unsigned char nominal_length_flag;                 // Planner flag for nominal speed always reached

  // Settings for the trapezoid generator
  unsigned long nominal_rate;                        // The nominal step rate for this block in step_events/sec
  unsigned long initial_rate;                        // The jerk-adjusted step rate at start of block
  unsigned long final_rate;                          // The minimal rate at exit
  unsigned long acceleration_st;                     // acceleration steps/sec^2
  unsigned long fan_speed;
  #if ENABLED(BARICUDA)
    unsigned long valve_pressure;
    unsigned long e_to_p_pressure;
  #endif
  volatile char busy;
} block_t;

#define BLOCK_MOD(n) ((n)&(BLOCK_BUFFER_SIZE-1))

extern block_t block_buffer[BLOCK_BUFFER_SIZE];            // A ring buffer for motion instructions
extern volatile unsigned char block_buffer_head;           // Index of the next block to be pushed
extern volatile unsigned char block_buffer_tail;

// Returns true if the buffer has a queued block, false otherwise
FORCE_INLINE bool blocks_queued() { return (block_buffer_head != block_buffer_tail); }

// Get the number of buffered moves
FORCE_INLINE uint8_t movesplanned() { return BLOCK_MOD(block_buffer_head - block_buffer_tail + BLOCK_BUFFER_SIZE); }

// Called when the current block is no longer needed. Discards
// the block and makes the memory available for new blocks.
FORCE_INLINE void plan_discard_current_block() {
  if (blocks_queued())
    block_buffer_tail = BLOCK_MOD(block_buffer_tail + 1);
}

// Gets the current block. Returns NULL if buffer empty
FORCE_INLINE block_t* plan_get_current_block() {
  if (blocks_queued()) {
    block_t* block = &block_buffer[block_buffer_tail];
    block->busy = true;
    return block;
  }
  else
    return NULL;
}

#endif
