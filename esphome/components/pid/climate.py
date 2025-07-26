from esphome import automation
import esphome.codegen as cg
from esphome.components import climate, output, sensor
import esphome.config_validation as cv
from esphome.const import CONF_HUMIDITY_SENSOR, CONF_ID, CONF_SENSOR

from ..pid_shared import config as pid_config

pid_ns = cg.esphome_ns.namespace("pid")
pid_shared_ns = cg.esphome_ns.namespace("pid_shared")
PIDClimate = pid_ns.class_("PIDClimate", climate.Climate, cg.Component)
PIDAutotuneAction = pid_shared_ns.class_("PIDAutotuneAction", automation.Action)
PIDResetIntegralTermAction = pid_shared_ns.class_(
    "PIDResetIntegralTermAction", automation.Action
)
PIDSetControlParametersAction = pid_shared_ns.class_(
    "PIDSetControlParametersAction", automation.Action
)

AUTO_LOAD = ["pid_shared"]

CONF_DEFAULT_TARGET_TEMPERATURE = "default_target_temperature"
CONF_COOL_OUTPUT = "cool_output"
CONF_HEAT_OUTPUT = "heat_output"

CONFIG_SCHEMA = cv.All(
    climate.CLIMATE_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(PIDClimate),
            cv.Required(CONF_SENSOR): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_HUMIDITY_SENSOR): cv.use_id(sensor.Sensor),
            cv.Required(CONF_DEFAULT_TARGET_TEMPERATURE): cv.temperature,
            cv.Optional(CONF_COOL_OUTPUT): cv.use_id(output.FloatOutput),
            cv.Optional(CONF_HEAT_OUTPUT): cv.use_id(output.FloatOutput),
            cv.Optional(
                pid_config.CONF_DEADBAND_PARAMETERS
            ): pid_config.pid_deadband_schema(),
            cv.Required(
                pid_config.CONF_CONTROL_PARAMETERS
            ): pid_config.pid_control_schema(),
        }
    ),
    cv.has_at_least_one_key(CONF_COOL_OUTPUT, CONF_HEAT_OUTPUT),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await climate.register_climate(var, config)

    sens = await cg.get_variable(config[CONF_SENSOR])
    cg.add(var.set_sensor(sens))

    if CONF_HUMIDITY_SENSOR in config:
        sens = await cg.get_variable(config[CONF_HUMIDITY_SENSOR])
        cg.add(var.set_humidity_sensor(sens))

    if CONF_COOL_OUTPUT in config:
        out = await cg.get_variable(config[CONF_COOL_OUTPUT])
        cg.add(var.set_cool_output(out))
    if CONF_HEAT_OUTPUT in config:
        out = await cg.get_variable(config[CONF_HEAT_OUTPUT])
        cg.add(var.set_heat_output(out))

    params = config[pid_config.CONF_CONTROL_PARAMETERS]
    cg.add(var.set_kp(params[pid_config.CONF_KP]))
    cg.add(var.set_ki(params[pid_config.CONF_KI]))
    cg.add(var.set_kd(params[pid_config.CONF_KD]))
    cg.add(
        var.set_starting_integral_term(params[pid_config.CONF_STARTING_INTEGRAL_TERM])
    )
    cg.add(
        var.set_derivative_samples(params[pid_config.CONF_DERIVATIVE_AVERAGING_SAMPLES])
    )
    cg.add(var.set_output_samples(params[pid_config.CONF_OUTPUT_AVERAGING_SAMPLES]))
    cg.add(var.set_min_integral(params[pid_config.CONF_MIN_INTEGRAL]))
    cg.add(var.set_max_integral(params[pid_config.CONF_MAX_INTEGRAL]))

    if pid_config.CONF_DEADBAND_PARAMETERS in config:
        params = config[pid_config.CONF_DEADBAND_PARAMETERS]
        cg.add(var.set_threshold_low(params[pid_config.CONF_THRESHOLD_LOW]))
        cg.add(var.set_threshold_high(params[pid_config.CONF_THRESHOLD_HIGH]))
        cg.add(var.set_kp_multiplier(params[pid_config.CONF_KP_MULTIPLIER]))
        cg.add(var.set_ki_multiplier(params[pid_config.CONF_KI_MULTIPLIER]))
        cg.add(var.set_kd_multiplier(params[pid_config.CONF_KD_MULTIPLIER]))
        cg.add(
            var.set_deadband_output_samples(
                params[pid_config.CONF_DEADBAND_OUTPUT_AVERAGING_SAMPLES]
            )
        )

    cg.add(var.set_default_target_temperature(config[CONF_DEFAULT_TARGET_TEMPERATURE]))


@automation.register_action(
    "climate.pid.reset_integral_term",
    PIDResetIntegralTermAction,
    automation.maybe_simple_id(
        {
            cv.Required(CONF_ID): cv.use_id(PIDClimate),
        }
    ),
)
async def pid_reset_integral_term(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action(
    "climate.pid.autotune",
    PIDAutotuneAction,
    automation.maybe_simple_id(
        {
            cv.Required(CONF_ID): cv.use_id(PIDClimate),
            cv.Optional(pid_config.CONF_NOISEBAND, default=0.25): cv.templatable(
                cv.float_
            ),
            cv.Optional(pid_config.CONF_POSITIVE_OUTPUT, default=1.0): cv.templatable(
                cv.possibly_negative_percentage
            ),
            cv.Optional(pid_config.CONF_NEGATIVE_OUTPUT, default=-1.0): cv.templatable(
                cv.possibly_negative_percentage
            ),
        }
    ),
)
async def pid_autotune(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)

    noiseband_ = await cg.templatable(config[pid_config.CONF_NOISEBAND], args, float)
    cg.add(var.set_noiseband(noiseband_))

    positive_output_ = await cg.templatable(
        config[pid_config.CONF_POSITIVE_OUTPUT], args, float
    )
    cg.add(var.set_positive_output(positive_output_))

    negative_output_ = await cg.templatable(
        config[pid_config.CONF_NEGATIVE_OUTPUT], args, float
    )
    cg.add(var.set_negative_output(negative_output_))

    return var


@automation.register_action(
    "climate.pid.set_control_parameters",
    PIDSetControlParametersAction,
    automation.maybe_simple_id(
        {
            cv.Required(CONF_ID): cv.use_id(PIDClimate),
            cv.Required(pid_config.CONF_KP): cv.templatable(cv.float_),
            cv.Optional(pid_config.CONF_KI, default=0.0): cv.templatable(cv.float_),
            cv.Optional(pid_config.CONF_KD, default=0.0): cv.templatable(cv.float_),
        }
    ),
)
async def set_control_parameters(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)

    kp_template_ = await cg.templatable(config[pid_config.CONF_KP], args, float)
    cg.add(var.set_kp(kp_template_))

    ki_template_ = await cg.templatable(config[pid_config.CONF_KI], args, float)
    cg.add(var.set_ki(ki_template_))

    kd_template_ = await cg.templatable(config[pid_config.CONF_KD], args, float)
    cg.add(var.set_kd(kd_template_))

    return var
