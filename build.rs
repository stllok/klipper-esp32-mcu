// Generates the zlib-compressed Klipper data dictionary from src/dict_defs.rs.
#![allow(dead_code)]
include!("src/dict_defs.rs");

use std::fmt::Write;

// Debug escaping == JSON escaping for the ASCII names in dict_defs.rs.
fn q(s: &str) -> String {
    format!("{s:?}")
}

fn obj(items: impl Iterator<Item = String>) -> String {
    format!("{{{}}}", items.collect::<Vec<_>>().join(","))
}

fn main() {
    println!("cargo:rerun-if-changed=src/dict_defs.rs");
    let pick = |want: fn(Kind) -> bool| {
        obj(MESSAGES
            .iter()
            .enumerate()
            .filter(move |(_, (k, _))| want(*k))
            .map(|(i, (_, f))| format!("{}:{}", q(f), i)))
    };
    let commands = pick(|k| k != Kind::Resp);
    let responses = pick(|k| k == Kind::Resp);

    let config = obj(INT_CONSTANTS
        .iter()
        .map(|(k, v)| format!("{}:{}", q(k), v))
        .chain(STR_CONSTANTS.iter().map(|(k, v)| format!("{}:{}", q(k), q(v)))));

    let mut enums = Vec::new();
    enums.push(format!(
        "{}:{}",
        q("static_string_id"),
        obj(STATIC_STRINGS
            .iter()
            .enumerate()
            .map(|(i, s)| format!("{}:{}", q(s), i as u16 + STATIC_STRING_MIN)))
    ));
    let mut names: Vec<&str> = ENUM_RANGES.iter().map(|r| r.0).chain(ENUM_VALUES.iter().map(|v| v.0)).collect();
    names.dedup();
    for e in names {
        let ranges = ENUM_RANGES
            .iter()
            .filter(|r| r.0 == e)
            .map(|(_, name, start, count)| format!("{}:[{},{}]", q(name), start, count));
        let values = ENUM_VALUES.iter().filter(|v| v.0 == e).map(|(_, name, v)| format!("{}:{}", q(name), v));
        enums.push(format!("{}:{}", q(e), obj(ranges.chain(values))));
    }

    let version = format!("klipper-esp32-mcu-{}", std::env::var("CARGO_PKG_VERSION").unwrap());
    let mut json = String::new();
    write!(
        json,
        "{{\"commands\":{commands},\"responses\":{responses},\"output\":{{}},\
         \"config\":{config},\"enumerations\":{{{}}},\"version\":{},\
         \"build_versions\":{},\"app\":\"Klipper\",\"license\":\"GNU GPLv3\"}}",
        enums.join(","),
        q(&version),
        q("rust"),
    )
    .unwrap();

    let out = std::path::PathBuf::from(std::env::var("OUT_DIR").unwrap());
    let z = miniz_oxide::deflate::compress_to_vec_zlib(json.as_bytes(), 10);
    std::fs::write(out.join("dict.zlib"), z).unwrap();
}
