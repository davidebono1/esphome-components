import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch, uart
from esphome.const import CONF_ID, CONF_SWITCHES, CONF_UART_ID

DEPENDENCIES = ['uart']

una_domologica_ns = cg.esphome_ns.namespace("una_domologica")

UnaDomologica = una_domologica_ns.class_("UnaDomologica", cg.Component)
RelaySwitch = una_domologica_ns.class_("RelaySwitch", switch.Switch)

CONF_SWITCHES = "switches"
CONF_RELAY_NUMBER = "relay_number"
CONF_DEVICE_ADDRESS = "device_address"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(UnaDomologica),
    cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
    cv.Optional(CONF_DEVICE_ADDRESS, default=0xAF2436): cv.hex_int_range(min=0, max=0xFFFFFF),
    cv.Required(CONF_SWITCHES): cv.ensure_list(
        switch.switch_schema(RelaySwitch).extend({
            cv.Required(CONF_RELAY_NUMBER): cv.int_range(min=1, max=4),
        })
    ),
}).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_UART_ID])
    cg.add(var.set_uart_parent(parent))
    cg.add(var.set_device_address(config[CONF_DEVICE_ADDRESS]))

    for sw_conf in config[CONF_SWITCHES]:
        sw = await switch.new_switch(sw_conf)
        cg.add(sw.set_parent(var))
        cg.add(sw.set_relay_number(sw_conf[CONF_RELAY_NUMBER]))
        cg.add(var.register_switch(sw, sw_conf[CONF_RELAY_NUMBER]))
