#ifndef CONFIG_H
#define CONFIG_H

// Particle Fileter
#define MOTION_MODEL_DEBUG false
#define NUM_PARTICLES 5000
#define POS_VAR 10
#define THETA_VAR 10

// Motion Model
#define ROT1_VAR 0.05
#define TRANS_VAR 0.05
#define ROT2_VAR 0.05
#define ALPHAS {0.1,0.1,0.1,0.1}

// Sensor Model
#define Z_MAX 5
#define Z_RAND 500
#define Z_SHORT 5
#define Z_HIT 100

#define Z_HIT_VAR 50.0
#define Z_LAMBDA_SHORT 0.01

#define RAY_CASTING_STEP_SIZE 2
#define RAY_SKIP_FACTOR 10

// Map
#define OBSTACLE_THRESHOLD 0.3
#define FREE_SPACE_THRESHOLD 0.1
// range should be ints
#define MAX_RANGE 8000
#define VISUALIZE_RAYS false

#endif
