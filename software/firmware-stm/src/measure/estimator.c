#include "generated/estimator.h"
#include "com/stream.h"
#include "com/telemetry.h"

static void serialize(cmp_ctx_t *cmp, void *context) {
    (void)context;

    cmp_write_map(cmp, 4);
    cmp_write_str(cmp, "pos", 3);
    cmp_write_array(cmp, 2);
    cmp_write_float(cmp, ESTIMATOR_GET_POS_X());
    cmp_write_float(cmp, ESTIMATOR_GET_POS_Y());
    cmp_write_str(cmp, "theta", 5);
    cmp_write_float(cmp, ESTIMATOR_GET_POS_THETA());
    cmp_write_str(cmp, "vel", 3);
    cmp_write_array(cmp, 2);
    cmp_write_float(cmp, ESTIMATOR_GET_VEL_X());
    cmp_write_float(cmp, ESTIMATOR_GET_VEL_Y());
    cmp_write_str(cmp, "ang_vel", 7);
    cmp_write_float(cmp, ESTIMATOR_GET_VEL_THETA());
}

static void reset(mpack_t *mpack) {
    (void)mpack;

    theta0 = ESTIMATOR_GET_POS_THETA();

    ESTIMATOR_SET_POS_X(0);
    ESTIMATOR_SET_POS_Y(0);
    ESTIMATOR_SET_POS_THETA(0);
}

TELEMETRY_REGISTER("estimate", serialize, NULL)
STREAM_REGISTER("reset", reset)
