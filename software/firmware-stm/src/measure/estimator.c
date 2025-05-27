#include "generated/estimator.h"
#include "com/stream.h"
#include "com/telemetry.h"

static void serialize(cmp_ctx_t *cmp, void *context) {
    (void)context;

    cmp_write_map(cmp, 3);
    cmp_write_str(cmp, "pos", 3);
    cmp_write_array(cmp, 3);
    cmp_write_float(cmp, estimator_state_get_x());
    cmp_write_float(cmp, estimator_state_get_y());
    cmp_write_float(cmp, estimator_state_get_theta());
    cmp_write_str(cmp, "vel", 3);
    cmp_write_array(cmp, 3);
    cmp_write_float(cmp, estimator_state_get_dotx());
    cmp_write_float(cmp, estimator_state_get_doty());
    cmp_write_float(cmp, estimator_state_get_dottheta());
    cmp_write_str(cmp, "m0", 2);
    cmp_write_float(cmp, estimator_state_get_m0());
}

static void reset(mpack_t *mpack) {
    (void)mpack;

    float theta0 = estimator_param_get_theta0();
    theta0 += estimator_state_get_theta();
    estimator_param_set_theta0(theta0);

    estimator_state_set_x(0);
    estimator_state_set_y(0);
    estimator_state_set_theta(0);

    estimator_state_set_dotx(0);
    estimator_state_set_doty(0);
    estimator_state_set_dottheta(0);
}

TELEMETRY_REGISTER("estimate", serialize, NULL)
STREAM_REGISTER("reset", reset)
