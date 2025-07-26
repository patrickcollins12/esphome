from esphome import automation
import esphome.codegen as cg
import esphome.config_validation as cv

pid_shared_ns = cg.esphome_ns.namespace("pid_shared")
PIDAutotuneAction = pid_shared_ns.class_("PIDAutotuneAction", automation.Action)
PIDResetIntegralTermAction = pid_shared_ns.class_(
    "PIDResetIntegralTermAction", automation.Action
)
PIDSetControlParametersAction = pid_shared_ns.class_(
    "PIDSetControlParametersAction", automation.Action
)

CONF_KP = "kp"
CONF_KI = "ki"
CONF_KD = "kd"
CONF_STARTING_INTEGRAL_TERM = "starting_integral_term"
CONF_CONTROL_PARAMETERS = "control_parameters"
CONF_MIN_INTEGRAL = "min_integral"
CONF_MAX_INTEGRAL = "max_integral"
CONF_OUTPUT_AVERAGING_SAMPLES = "output_averaging_samples"
CONF_DERIVATIVE_AVERAGING_SAMPLES = "derivative_averaging_samples"
CONF_NOISEBAND = "noiseband"
CONF_POSITIVE_OUTPUT = "positive_output"
CONF_NEGATIVE_OUTPUT = "negative_output"
CONF_DEADBAND_PARAMETERS = "deadband_parameters"
CONF_THRESHOLD_HIGH = "threshold_high"
CONF_THRESHOLD_LOW = "threshold_low"
CONF_DEADBAND_OUTPUT_AVERAGING_SAMPLES = "deadband_output_averaging_samples"
CONF_KP_MULTIPLIER = "kp_multiplier"
CONF_KI_MULTIPLIER = "ki_multiplier"
CONF_KD_MULTIPLIER = "kd_multiplier"


def pid_control_schema():
    """Return the schema for the PID control parameters."""
    return cv.Schema(
        {
            cv.Required(CONF_KP): cv.float_,
            cv.Optional(CONF_KI, default=0.0): cv.float_,
            cv.Optional(CONF_KD, default=0.0): cv.float_,
            cv.Optional(CONF_STARTING_INTEGRAL_TERM, default=0.0): cv.float_,
            cv.Optional(CONF_MIN_INTEGRAL, default=-1): cv.float_,
            cv.Optional(CONF_MAX_INTEGRAL, default=1): cv.float_,
            cv.Optional(CONF_DERIVATIVE_AVERAGING_SAMPLES, default=1): cv.int_,
            cv.Optional(CONF_OUTPUT_AVERAGING_SAMPLES, default=1): cv.int_,
        }
    )


def pid_deadband_schema():
    """Return the schema for the PID deadband parameters."""
    return cv.Schema(
        {
            cv.Required(CONF_THRESHOLD_HIGH): cv.float_,
            cv.Required(CONF_THRESHOLD_LOW): cv.float_,
            cv.Optional(CONF_KP_MULTIPLIER, default=0.1): cv.float_,
            cv.Optional(CONF_KI_MULTIPLIER, default=0.0): cv.float_,
            cv.Optional(CONF_KD_MULTIPLIER, default=0.0): cv.float_,
            cv.Optional(CONF_DEADBAND_OUTPUT_AVERAGING_SAMPLES, default=1): cv.int_,
        }
    )


def add_pid_to_code(var, conf):
    """
    Set the PID parameters from the config.
    This is a helper function to avoid code duplication in the to_code functions
    of the pid and pid_control components.
    """
    params = conf[CONF_CONTROL_PARAMETERS]
    cg.add(var.set_kp(params[CONF_KP]))
    cg.add(var.set_ki(params[CONF_KI]))
    cg.add(var.set_kd(params[CONF_KD]))
    cg.add(var.set_starting_integral_term(params[CONF_STARTING_INTEGRAL_TERM]))
    cg.add(var.set_derivative_samples(params[CONF_DERIVATIVE_AVERAGING_SAMPLES]))
    cg.add(var.set_output_samples(params[CONF_OUTPUT_AVERAGING_SAMPLES]))
    cg.add(var.set_min_integral(params[CONF_MIN_INTEGRAL]))
    cg.add(var.set_max_integral(params[CONF_MAX_INTEGRAL]))

    if CONF_DEADBAND_PARAMETERS in conf:
        params = conf[CONF_DEADBAND_PARAMETERS]
        cg.add(var.set_threshold_low(params[CONF_THRESHOLD_LOW]))
        cg.add(var.set_threshold_high(params[CONF_THRESHOLD_HIGH]))
        cg.add(var.set_kp_multiplier(params[CONF_KP_MULTIPLIER]))
        cg.add(var.set_ki_multiplier(params[CONF_KI_MULTIPLIER]))
        cg.add(var.set_kd_multiplier(params[CONF_KD_MULTIPLIER]))
        cg.add(
            var.set_deadband_output_samples(
                params[CONF_DEADBAND_OUTPUT_AVERAGING_SAMPLES]
            )
        )
