from esphome import automation
import esphome.codegen as cg
from esphome.components import output, sensor, switch
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_SENSOR

from ..pid_shared import config as pid_config

CODEOWNERS = ["@patrickcollins12"]
AUTO_LOAD = ["pid_shared"]

pidcontrol_ns = cg.esphome_ns.namespace("pid_control")
pid_shared_ns = cg.esphome_ns.namespace("pid_shared")

PIDControl = pidcontrol_ns.class_("PIDControl", cg.Component)
PIDAutotuneAction = pid_shared_ns.class_("PIDAutotuneAction", automation.Action)
PIDResetIntegralTermAction = pid_shared_ns.class_(
    "PIDResetIntegralTermAction", automation.Action
)
PIDSetControlParametersAction = pid_shared_ns.class_(
    "PIDSetControlParametersAction", automation.Action
)
PIDSetTargetValueAction = pidcontrol_ns.class_(
    "PIDSetTargetValueAction", automation.Action
)

CONF_ENABLE_SWITCH = "enable_switch"
CONF_INCREASE_OUTPUT = "increase_output"
CONF_DECREASE_OUTPUT = "decrease_output"
CONF_TARGET_VALUE = "target_value"
CONF_VALUE = "value"

PID_CONTROL_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(PIDControl),
        cv.Optional("name"): cv.string,
        cv.Required(CONF_SENSOR): cv.use_id(sensor.Sensor),
        cv.Required(CONF_TARGET_VALUE): cv.float_,
        cv.Optional(CONF_ENABLE_SWITCH): cv.use_id(switch.Switch),
        cv.Optional(CONF_INCREASE_OUTPUT): cv.use_id(output.FloatOutput),
        cv.Optional(CONF_DECREASE_OUTPUT): cv.use_id(output.FloatOutput),
        cv.Optional(
            pid_config.CONF_DEADBAND_PARAMETERS
        ): pid_config.pid_deadband_schema(),
        cv.Required(
            pid_config.CONF_CONTROL_PARAMETERS
        ): pid_config.pid_control_schema(),
    }
).extend(cv.COMPONENT_SCHEMA)

CONFIG_SCHEMA = cv.All(cv.ensure_list(PID_CONTROL_SCHEMA))


async def to_code(config):
    for conf in config:
        var = cg.new_Pvariable(conf[CONF_ID])
        await cg.register_component(var, conf)

        if "name" in conf:
            cg.add(var.set_name(conf["name"]))

        sens = await cg.get_variable(conf[CONF_SENSOR])
        cg.add(var.set_sensor(sens))

        if CONF_DECREASE_OUTPUT in conf:
            out = await cg.get_variable(conf[CONF_DECREASE_OUTPUT])
            cg.add(var.set_decrease_output(out))
        if CONF_INCREASE_OUTPUT in conf:
            out = await cg.get_variable(conf[CONF_INCREASE_OUTPUT])
            cg.add(var.set_increase_output(out))
        if CONF_ENABLE_SWITCH in conf:
            s = await cg.get_variable(conf[CONF_ENABLE_SWITCH])
            cg.add(var.set_enable_switch(s))

        pid_config.add_pid_to_code(var, conf)
        cg.add(var.set_target_value(conf[CONF_TARGET_VALUE]))


@automation.register_action(
    "pid_control.reset_integral_term",
    PIDResetIntegralTermAction,
    automation.maybe_simple_id(
        {
            cv.Required(CONF_ID): cv.use_id(PIDControl),
        }
    ),
)
async def pid_reset_integral_term(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action(
    "pid_control.autotune",
    PIDAutotuneAction,
    automation.maybe_simple_id(
        {
            cv.Required(CONF_ID): cv.use_id(PIDControl),
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
    "pid_control.set_control_parameters",
    PIDSetControlParametersAction,
    automation.maybe_simple_id(
        {
            cv.Required(CONF_ID): cv.use_id(PIDControl),
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


@automation.register_action(
    "pid_control.set_target_value",
    PIDSetTargetValueAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(PIDControl),
            cv.Required(CONF_VALUE): cv.templatable(cv.float_),
        }
    ),
)
def pid_set_target_value_to_code(config, action_id, template_arg, args):
    """Set up the Set Target Value action."""
    paren = yield cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    template_ = yield cg.templatable(config[CONF_VALUE], args, float)
    cg.add(var.set_target_value(template_))
    yield var
