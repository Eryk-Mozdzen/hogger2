#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#define TRAJECTORY_VALUES 12

#define TRAJECTORY_GET_X(trajectory)        ((trajectory)->values[0])
#define TRAJECTORY_GET_Y(trajectory)        ((trajectory)->values[1])
#define TRAJECTORY_GET_THETA(trajectory)    ((trajectory)->values[2])
#define TRAJECTORY_GET_D_X(trajectory)      ((trajectory)->values[3])
#define TRAJECTORY_GET_D_Y(trajectory)      ((trajectory)->values[4])
#define TRAJECTORY_GET_D_THETA(trajectory)  ((trajectory)->values[5])
#define TRAJECTORY_GET_D2_X(trajectory)     ((trajectory)->values[6])
#define TRAJECTORY_GET_D2_Y(trajectory)     ((trajectory)->values[7])
#define TRAJECTORY_GET_D2_THETA(trajectory) ((trajectory)->values[8])
#define TRAJECTORY_GET_D3_X(trajectory)     ((trajectory)->values[9])
#define TRAJECTORY_GET_D3_Y(trajectory)     ((trajectory)->values[10])
#define TRAJECTORY_GET_D3_THETA(trajectory) ((trajectory)->values[11])

typedef struct {
    float values[TRAJECTORY_VALUES];
} trajectory_t;

void trajectory_get(trajectory_t *trajectory, const float t);

#endif
