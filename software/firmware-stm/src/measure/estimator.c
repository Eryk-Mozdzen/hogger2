#include "generated/estimator.h"
#include "com/telemetry.h"

static void serialize(cmp_ctx_t *cmp, void *context) {
    (void)context;

    cmp_write_map(cmp, 4);
    cmp_write_str(cmp, "pos", 3);
    cmp_write_array(cmp, 2);
    cmp_write_float(cmp, ESTIMATOR_GET_POS_X);
    cmp_write_float(cmp, ESTIMATOR_GET_POS_Y);
    cmp_write_str(cmp, "theta", 5);
    cmp_write_float(cmp, ESTIMATOR_GET_POS_THETA);
    cmp_write_str(cmp, "vel", 3);
    cmp_write_array(cmp, 2);
    cmp_write_float(cmp, ESTIMATOR_GET_VEL_X);
    cmp_write_float(cmp, ESTIMATOR_GET_VEL_Y);
    cmp_write_str(cmp, "ang_vel", 7);
    cmp_write_float(cmp, ESTIMATOR_GET_VEL_THETA);
}

TELEMETRY_REGISTER("estimate", serialize, NULL)
