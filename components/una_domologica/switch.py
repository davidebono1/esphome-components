import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch, uart
from esphome.const import CONF_ID, CONF_NAME, CONF_SWITCHES, CONF_UART_ID

DEPENDENCIES = ['switch','uart']

una_domologica_ns = cg.esphome_ns.namespace("una_domologica")

# Define the UnaDomologica class
UnaDomologica = una_domologica_ns.class_("UnaDomologica", cg.Component)

# Define RelaySwitch class inheriting from switch.Switch (from the 'switch' component)
RelaySwitch = una_domologica_ns.class_("RelaySwitch", switch.Switch)

CONF_SWITCHES = "switches"
CONF_RELAY_NUMBER = "relay_number"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(UnaDomologica),
    cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
    cv.Required(CONF_SWITCHES): cv.ensure_list(
        cv.Schema({
            cv.GenerateID(): cv.declare_id(RelaySwitch),
            cv.Optional(CONF_NAME, ""): cv.string,
            cv.Required(CONF_RELAY_NUMBER): cv.int_range(min=1, max=4),
        })
    ),
}).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_UART_ID])
    cg.add(var.set_uart_parent(parent))

    for sw_conf in config[CONF_SWITCHES]:
        sw = cg.new_Pvariable(sw_conf[CONF_ID])
        cg.add(sw.set_parent(var))
        cg.add(sw.set_relay_number(sw_conf[CONF_RELAY_NUMBER]))
        cg.add(sw.set_name(sw_conf[CONF_NAME]))
    
        # Explicitly ensure 'disabled_by_default' exists
        if "disabled_by_default" not in sw_conf:
            sw_conf["disabled_by_default"] = False
    
        await switch.register_switch(sw, sw_conf)

