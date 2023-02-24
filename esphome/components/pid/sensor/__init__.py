import esphome.codegen as cg
import esphome.config_validation as cv

# from esphome.components import climate, sensor
from esphome.components import sensor

from esphome.const import (
    STATE_CLASS_MEASUREMENT,
    UNIT_PERCENT,
    ICON_GAUGE,
    CONF_TYPE,
)

# AUTO_LOAD = ["pid_shared", "pid", "pid_control"]
AUTO_LOAD = ["pid_control"]

# PIDClimate
# pid_climate_ns = cg.esphome_ns.namespace("pid")
# PIDClimate = pid_climate_ns.class_("PIDClimate", climate.Climate, cg.Component)

# PIDControl
pid_control_ns = cg.esphome_ns.namespace("pid_control")
PIDControl = pid_control_ns.class_("PIDControl", cg.Component, cg.EntityBase)

# PIDBase
pid_shared_ns = cg.esphome_ns.namespace("pid_shared")
# PIDBase = pid_shared_ns.class_("PIDBase", cg.Component, cg.EntityBase)

# PIDSensor
PIDSensor = pid_shared_ns.class_("PIDSensor", sensor.Sensor, cg.Component)
PIDSensorType = pid_shared_ns.enum("PIDSensorType")

PID_CLIMATE_SENSOR_TYPES = {
    "RESULT": PIDSensorType.PID_SENSOR_TYPE_RESULT,
    "ERROR": PIDSensorType.PID_SENSOR_TYPE_ERROR,
    "PROPORTIONAL": PIDSensorType.PID_SENSOR_TYPE_PROPORTIONAL,
    "INTEGRAL": PIDSensorType.PID_SENSOR_TYPE_INTEGRAL,
    "DERIVATIVE": PIDSensorType.PID_SENSOR_TYPE_DERIVATIVE,
    "HEAT": PIDSensorType.PID_SENSOR_TYPE_HEAT,
    "COOL": PIDSensorType.PID_SENSOR_TYPE_COOL,
    "KP": PIDSensorType.PID_SENSOR_TYPE_KP,
    "KI": PIDSensorType.PID_SENSOR_TYPE_KI,
    "KD": PIDSensorType.PID_SENSOR_TYPE_KD,
}

CONF_ID = "id"
CONFIG_SCHEMA = (
    sensor.sensor_schema(
        PIDSensor,
        unit_of_measurement=UNIT_PERCENT,
        icon=ICON_GAUGE,
        accuracy_decimals=1,
        state_class=STATE_CLASS_MEASUREMENT,
    )
    .extend(
        {
            cv.GenerateID(CONF_ID): cv.use_id(PIDControl),
            cv.Required(CONF_TYPE): cv.enum(PID_CLIMATE_SENSOR_TYPES, upper=True),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_ID])
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)

    cg.add(var.set_parent(parent))
    cg.add(var.set_type(config[CONF_TYPE]))
