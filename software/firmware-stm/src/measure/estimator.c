#include "generated/estimator.h"
#include "com/stream.h"
#include "com/telemetry.h"

static void serialize(cmp_ctx_t *cmp, void *context) {
    (void)context;

    cmp_write_map(cmp, 3);
    cmp_write_str(cmp, "pos", 3);
    cmp_write_array(cmp, 3);
    cmp_write_float(cmp, estimator_state_get_px());
    cmp_write_float(cmp, estimator_state_get_py());
    cmp_write_float(cmp, estimator_state_get_theta());
    cmp_write_str(cmp, "vel", 3);
    cmp_write_array(cmp, 3);
    cmp_write_float(cmp, estimator_state_get_vx());
    cmp_write_float(cmp, estimator_state_get_vy());
    cmp_write_float(cmp, estimator_state_get_vtheta());
    cmp_write_str(cmp, "m0", 2);
    cmp_write_float(cmp, estimator_state_get_m0());
}

static void reset(mpack_t *mpack) {
    (void)mpack;

    float theta0 = estimator_param_get_theta0();
    theta0 += estimator_state_get_theta();
    estimator_param_set_theta0(theta0);

    estimator_state_set_px(0);
    estimator_state_set_py(0);
    estimator_state_set_theta(0);

    estimator_state_set_vx(0);
    estimator_state_set_vy(0);
    estimator_state_set_vtheta(0);
}

TELEMETRY_REGISTER("estimate", serialize, NULL)
STREAM_REGISTER("reset", reset)
