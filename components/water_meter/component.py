import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_PIN,
    CONF_UPDATE_INTERVAL,
    DEVICE_CLASS_WATER,
    DEVICE_CLASS_GAS, # Volume is often measured in m^3 or L like gas
    DEVICE_CLASS_DURATION,
    ICON_WATER,
    ICON_GAUGE,
    ICON_TIMER,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_LITERS_PER_MINUTE,
    UNIT_LITER,
    UNIT_SECOND,
)
from esphome import pins

# Namespace and class name
water_meter_ns = cg.esphome_ns.namespace("water_meter")
WaterMeterComponent = water_meter_ns.class_(
    "WaterMeterComponent", cg.PollingComponent
)

# Configuration keys specific to this component
# Match the const char* definitions in water_meter.h
CONF_SENSOR_PIN = "sensor_pin"
CONF_PULSE_FACTOR = "pulse_factor"
CONF_MAX_FLOW = "max_flow"
CONF_READ_FREQUENCY = "read_frequency"
CONF_SMOOTHING_ALPHA = "smoothing_alpha"
CONF_THRESHOLD = "threshold"
CONF_FLOW_SENSOR = "flow"
CONF_VOLUME_SENSOR = "volume"
CONF_DURATION_SENSOR = "duration"

# Define the configuration schema
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(WaterMeterComponent),
        cv.Required(CONF_SENSOR_PIN): pins.internal_gpio_input_pin_schema, # Use internal GPIO helper
        cv.Optional(CONF_PULSE_FACTOR, default=120000): cv.positive_int, # Pulses per m^3
        cv.Optional(CONF_MAX_FLOW, default=40.0): cv.positive_float,    # L/min
        cv.Optional(CONF_READ_FREQUENCY, default=10): cv.positive_int,  # ms
        cv.Optional(CONF_SMOOTHING_ALPHA, default=0.02): cv.positive_float,
        cv.Optional(CONF_THRESHOLD, default=0.5): cv.positive_float,

        # Sensors provided by this component
        cv.Optional(CONF_FLOW_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_LITERS_PER_MINUTE,
            icon=ICON_GAUGE,
            accuracy_decimals=2,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_VOLUME_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_LITER,
            icon=ICON_WATER,
            accuracy_decimals=3,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            device_class=DEVICE_CLASS_WATER, # Or DEVICE_CLASS_GAS if m³ preferred
        ),
        cv.Optional(CONF_DURATION_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_SECOND,
            icon=ICON_TIMER,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT, # Duration isn't strictly total_increasing
            device_class=DEVICE_CLASS_DURATION,
        ),

        cv.Optional(CONF_UPDATE_INTERVAL, default="30s"): cv.update_interval,
    }
).extend(cv.polling_component_schema("30s")) # Inherit polling interval


async def to_code(config):
    # Create the C++ component instance
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Set the sensor pin
    pin = await cg.gpio_pin_expression(config[CONF_SENSOR_PIN])
    cg.add(var.set_sensor_pin(pin))

    # Set other configuration parameters
    cg.add(var.set_pulse_factor(config[CONF_PULSE_FACTOR]))
    cg.add(var.set_max_flow(config[CONF_MAX_FLOW]))
    cg.add(var.set_read_frequency(config[CONF_READ_FREQUENCY]))
    cg.add(var.set_smoothing_alpha(config[CONF_SMOOTHING_ALPHA]))
    cg.add(var.set_threshold(config[CONF_THRESHOLD]))

    # Register associated sensors if configured
    if CONF_FLOW_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_FLOW_SENSOR])
        cg.add(var.set_flow_sensor(sens))

    if CONF_VOLUME_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_VOLUME_SENSOR])
        cg.add(var.set_volume_sensor(sens))

    if CONF_DURATION_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_DURATION_SENSOR])
        cg.add(var.set_duration_sensor(sens))