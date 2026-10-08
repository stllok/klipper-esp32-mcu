// Shared by build.rs (data dictionary JSON) and the crate (dispatch).
// Message id == index in MESSAGES. Ids 0/1 are hard-coded by the Klipper host.
// Keep this file free of `use` and crate paths: build.rs `include!`s it.

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// Response (MCU -> host).
    Resp,
    /// Command (host -> MCU), rejected while shut down.
    Cmd,
    /// Command allowed while shut down (Klipper HF_IN_SHUTDOWN).
    CmdAny,
}

pub const MESSAGES: &[(Kind, &str)] = &[
    (Kind::Resp, "identify_response offset=%u data=%.*s"),
    (Kind::CmdAny, "identify offset=%u count=%c"),
    (Kind::CmdAny, "get_uptime"),
    (Kind::Resp, "uptime high=%u clock=%u"),
    (Kind::CmdAny, "get_clock"),
    (Kind::Resp, "clock clock=%u"),
    (Kind::CmdAny, "get_config"),
    (Kind::Resp, "config is_config=%c crc=%u is_shutdown=%c move_count=%hu"),
    (Kind::Cmd, "finalize_config crc=%u"),
    (Kind::Cmd, "allocate_oids count=%c"),
    (Kind::CmdAny, "config_reset"),
    (Kind::CmdAny, "emergency_stop"),
    (Kind::CmdAny, "clear_shutdown"),
    (Kind::Resp, "shutdown clock=%u static_string_id=%hu"),
    (Kind::Resp, "is_shutdown static_string_id=%hu"),
    (Kind::Resp, "stats count=%u sum=%u sumsq=%u"),
    (Kind::Resp, "starting"),
    (Kind::Cmd, "config_digital_out oid=%c pin=%u value=%c default_value=%c max_duration=%u"),
    (Kind::Cmd, "set_digital_out_pwm_cycle oid=%c cycle_ticks=%u"),
    (Kind::Cmd, "queue_digital_out oid=%c clock=%u on_ticks=%u"),
    (Kind::Cmd, "update_digital_out oid=%c value=%c"),
    (Kind::Cmd, "set_digital_out pin=%u value=%c"),
    (Kind::Cmd, "config_analog_in oid=%c pin=%u"),
    (Kind::Cmd, "query_analog_in oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u bytes_per_report=%c min_value=%hu max_value=%hu range_check_count=%c"),
    (Kind::Resp, "analog_in_state oid=%c next_clock=%u values=%*s"),
    (Kind::Cmd, "config_stepper oid=%c step_pin=%c dir_pin=%c invert_step=%c step_pulse_ticks=%u"),
    (Kind::Cmd, "queue_step oid=%c interval=%u count=%hu add=%hi"),
    (Kind::Cmd, "set_next_step_dir oid=%c dir=%c"),
    (Kind::Cmd, "reset_step_clock oid=%c clock=%u"),
    (Kind::Cmd, "stepper_get_position oid=%c"),
    (Kind::Resp, "stepper_position oid=%c pos=%i"),
    (Kind::Cmd, "stepper_stop_on_trigger oid=%c trsync_oid=%c"),
    (Kind::Cmd, "config_endstop oid=%c pin=%c pull_up=%c"),
    (Kind::Cmd, "endstop_home oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u pin_value=%c trsync_oid=%c trigger_reason=%c"),
    (Kind::Cmd, "endstop_query_state oid=%c"),
    (Kind::Resp, "endstop_state oid=%c homing=%c next_clock=%u pin_value=%c"),
    (Kind::Cmd, "config_trsync oid=%c"),
    (Kind::Cmd, "trsync_start oid=%c report_clock=%u report_ticks=%u expire_reason=%c"),
    (Kind::Cmd, "trsync_set_timeout oid=%c clock=%u"),
    (Kind::Cmd, "trsync_trigger oid=%c reason=%c"),
    (Kind::Resp, "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u"),
    (Kind::Cmd, "config_buttons oid=%c button_count=%c"),
    (Kind::Cmd, "buttons_add oid=%c pos=%c pin=%u pull_up=%c"),
    (Kind::Cmd, "buttons_query oid=%c clock=%u rest_ticks=%u retransmit_count=%c invert=%c"),
    (Kind::Cmd, "buttons_ack oid=%c count=%c"),
    (Kind::Resp, "buttons_state oid=%c ack_count=%c state=%*s"),
    (Kind::Cmd, "config_i2c oid=%c"),
    (Kind::Cmd, "i2c_set_bus oid=%c i2c_bus=%u rate=%u address=%u"),
    (Kind::Cmd, "i2c_transfer oid=%c write=%*s read_len=%u"),
    (Kind::Resp, "i2c_response oid=%c i2c_bus_status=%c response=%*s"),
];

/// Shutdown reasons, exported as enumeration `static_string_id` starting at 2.
pub const STATIC_STRINGS: &[&str] = &[
    "Command request",
    "Shutdown cleared when not shutdown",
    "oids already allocated",
    "config_reset only available when shutdown",
    "Command parser error",
    "Invalid command",
    "Can't assign oid",
    "Invalid oid type",
    "Move queue overflow",
    "Can not set soft pwm cycle ticks while updates pending",
    "Scheduled digital out event will exceed max_duration",
    "update_digital_out not valid with active queue",
    "Missed scheduling of next digital out event",
    "Invalid analog_in bytes_per_report",
    "ADC out of range",
    "Invalid output pin",
    "Invalid analog pin",
    "Timer too close",
    "Stepper too far in past",
    "Invalid count parameter",
    "Can't reset time when stepper active",
    "Can't add signal that is already active",
    "Invalid input pin",
    "Max of 8 buttons",
    "Set button past maximum button count",
    "Invalid buttons retransmit count",
    "Unsupported i2c bus",
];
pub const STATIC_STRING_MIN: u16 = 2;

pub const CLOCK_FREQ: u32 = 40_000_000;
pub const SERIAL_BAUD: u32 = 250_000;
pub const STATS_SUMSQ_BASE: u32 = 256;
pub const RECEIVE_WINDOW: u32 = 192;
/// analog_in reports calibrated millivolts; 3300 mV = thermistor pull-up
/// supply, so value/ADC_MAX is the divider ratio the host expects.
pub const ADC_MAX: u32 = 3300;
/// First shift-register pin value (`sr0`), Marlin numbering 128+n.
pub const SR_PIN_BASE: u32 = 128;
pub const SR_PINS: u32 = 32;

/// Integer constants exported in `config`.
pub const INT_CONSTANTS: &[(&str, u32)] = &[
    ("CLOCK_FREQ", CLOCK_FREQ),
    ("SERIAL_BAUD", SERIAL_BAUD),
    ("STATS_SUMSQ_BASE", STATS_SUMSQ_BASE),
    ("RECEIVE_WINDOW", RECEIVE_WINDOW),
    ("ADC_MAX", ADC_MAX),
    // Steps toggle the pin on each edge when the host asks (invert_step=-1).
    ("STEPPER_STEP_BOTH_EDGE", 1),
];
/// String constants exported in `config`.
pub const STR_CONSTANTS: &[(&str, &str)] = &[
    ("MCU", "esp32"),
    ("RESERVE_PINS_serial", "gpio1,gpio3"),
    // OLED header: I2C0 on the Arduino-ESP32 default pins Marlin's Wire uses.
    ("BUS_PINS_i2c0", "gpio22,gpio21"),
];
/// Single-value enumerations: (enum, name, value).
pub const ENUM_VALUES: &[(&str, &str, u32)] = &[
    ("i2c_bus", "i2c0", 0),
    ("i2c_bus_status", "SUCCESS", 0),
    ("i2c_bus_status", "NACK", 1),
    ("i2c_bus_status", "START_NACK", 2),
    ("i2c_bus_status", "START_READ_NACK", 3),
    ("i2c_bus_status", "BUS_TIMEOUT", 4),
];
/// Enumeration ranges: (enum, first name, first value, count).
pub const ENUM_RANGES: &[(&str, &str, u32, u32)] = &[
    ("pin", "gpio0", 0, 40),
    ("pin", "sr0", SR_PIN_BASE, SR_PINS),
];
