// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

use std::io::Cursor;

use specctra_core::{
    read::{ListTokenizer, ReadDsn},
    structure::{Circuit, Class, Parser, Via},
};

// Derived `ReadDsn` impls expect the list's opening `(name` token to have
// already been consumed by the caller, so inputs here are the list's contents
// plus the closing parenthesis.
fn parse<T: for<'a> ReadDsn<Cursor<&'a [u8]>>>(input: &str) -> T {
    let mut tokenizer = ListTokenizer::new(Cursor::new(input.as_bytes()));
    T::read_dsn(&mut tokenizer).unwrap()
}

#[test]
fn via_with_type_attribute() {
    let via: Via = parse(r#""Via[0-3]_600:300_um" 99491.6 -29998 (net +3V3)(type route))"#);

    // `#[anon]` string fields keep the raw token, quotes included.
    assert_eq!(via.name, "\"Via[0-3]_600:300_um\"");
    assert_eq!(via.net, "+3V3");
    assert_eq!(via.r#type.as_deref(), Some("route"));
}

#[test]
fn via_without_type_attribute() {
    let via: Via = parse(r#""V" 1 2 (net GND))"#);

    assert_eq!(via.net, "GND");
    assert_eq!(via.r#type, None);
}

#[test]
fn use_via_accepts_multiple_names() {
    let circuit: Circuit = parse("(use_via ViaA ViaB ViaC))");

    assert_eq!(circuit.use_via.names, ["ViaA", "ViaB", "ViaC"]);
}

#[test]
fn use_via_accepts_single_name() {
    let circuit: Circuit = parse("(use_via ViaA))");

    assert_eq!(circuit.use_via.names, ["ViaA"]);
}

#[test]
fn parser_accepts_quoted_value_without_string_quote_decl() {
    // EasyEDA emits quoted strings containing spaces but never declares
    // `(string_quote ...)`, so the lexer splits the value at the space.
    let parser: Parser = parse(r#"(host_cad "EasyEDA Pro")(host_version "2.2.47.7"))"#);

    assert_eq!(parser.host_cad.as_deref(), Some("EasyEDA Pro"));
    assert_eq!(parser.host_version.as_deref(), Some("2.2.47.7"));
}

#[test]
fn parser_accepts_unquoted_and_bare_values() {
    let parser: Parser = parse("(host_cad Topola)(host_version 1.0))");

    assert_eq!(parser.host_cad.as_deref(), Some("Topola"));
    assert_eq!(parser.host_version.as_deref(), Some("1.0"));
}

#[test]
fn class_allows_missing_name() {
    // Issue179 fixture: `(class (circuit …) (rule …))` — an anonymous
    // class with no name leaf.
    let class: Class = parse("(circuit (use_via via0))(rule (width 1)(clearance 0.4)))");

    assert_eq!(class.name, None);
    assert_eq!(class.circuit.use_via.names, ["via0"]);
}

#[test]
fn class_reads_name_and_nets() {
    let class: Class =
        parse("power GND VCC (circuit (use_via via0))(rule (width 1)(clearance 0.4)))");

    assert_eq!(class.name.as_deref(), Some("power"));
    assert_eq!(class.nets, ["GND", "VCC"]);
}
