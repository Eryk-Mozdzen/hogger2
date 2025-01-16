#ifndef REFERENCE_H
#define REFERENCE_H

typedef enum {
    REFERENCE_TYPE_NONE,
    REFERENCE_TYPE_CONFIGURATION,
    REFERENCE_TYPE_TRAJECTORY,
} reference_type_t;

typedef struct {
    reference_type_t type;
    union {
        float configuration[6];
        float trajectory[3];
    };
} reference_t;

#endif
