#ifndef SERIALIZE_H
#define SERIALIZE_H

#include <stdbool.h>
#include <stdint.h>

#include "robot.h"
#include "reference.h"

uint32_t serialize_robot(const robot_t *robot, void *dest, const uint32_t dest_capacity);

bool deserialize_reference(const void *src, const uint32_t src_size, reference_t *reference);
bool deserialize_stop(const void *src, const uint32_t src_size);

#endif
