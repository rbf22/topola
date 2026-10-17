// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

use std::collections::BTreeSet;

use topola::autorouter::selection::PinSelection;

mod common;

#[test]
fn select_pins_by_netname() {
    let autorouter = common::load_design("tests/unilayer/0603_breakout/0603_breakout.dsn");
    let board = autorouter.board();

    let named = BTreeSet::from(["Net-(J1-Pin_1)".to_string()]);
    assert!(!PinSelection::new_select_nets(board, &named)
        .selectors()
        .next()
        .is_none());

    let absent = BTreeSet::from(["NoSuchNet".to_string()]);
    assert!(PinSelection::new_select_nets(board, &absent)
        .selectors()
        .next()
        .is_none());
}

#[test]
fn exclude_pins_by_netname() {
    let autorouter = common::load_design("tests/unilayer/0603_breakout/0603_breakout.dsn");
    let board = autorouter.board();

    let all = PinSelection::new_select_all_except_nets(board, &BTreeSet::new());
    let without_named = PinSelection::new_select_all_except_nets(
        board,
        &BTreeSet::from(["Net-(J1-Pin_1)".to_string()]),
    );

    assert!(without_named.selectors().count() < all.selectors().count());
}
