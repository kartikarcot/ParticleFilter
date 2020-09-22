#ifndef CONFIG_H
#define CONFIG_H

// Particle Fileter
#define NUM_PARTICLES 2000

// Motion Model
#define ROT_VAR 0.05
#define TRANS_VAR 0.05

// Sensor Model
#define Z_MAX 0.001
#define Z_RAND 10.0
#define Z_SHORT 0.2
#define Z_HIT 2

#define Z_HIT_VAR 50.0
#define Z_LAMBDA_SHORT 0.001

#define RAY_CASTING_STEP_SIZE 2
#define RAY_SKIP_FACTOR 2

// Map
#define THRESHOLD 0.3
#define FREE_SPACE_THRESHOLD 0.1
// range should be ints
#define MAX_RANGE 8000
#define VISUALIZE_RAYS false

#endif
