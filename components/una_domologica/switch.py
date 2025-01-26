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
        # Possibly set a name if you have a set_name(...)
        # cg.add(sw.set_name(sw_conf[CONF_NAME]))
        
        # -- do NOT register it as a component, remove the line below --
        # await cg.register_component(sw, sw_conf)
        
        # If you also don’t want the “App.register_switch()” behavior from standard ESPHome,  
        # do not call "await switch.register_switch(sw, sw_conf)" either.  
        # Instead, just keep your own registration code:
        cg.add(sw.set_parent(var))
        cg.add(sw.set_relay_number(sw_conf[CONF_RELAY_NUMBER]))
        cg.add(var.register_switch(sw, sw_conf[CONF_RELAY_NUMBER]))