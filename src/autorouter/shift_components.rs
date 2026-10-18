// SPDX-FileCopyrightText: 2025 Topola contributors
//
// SPDX-License-Identifier: MIT

//! Component shifting for congestion relief: translates every pad and
//! via of each named component by a requested displacement, re-solving
//! the bows of attached bands.
//!
//! A shift succeeds only when all the component's member dots can move
//! and the resulting geometry infringes nothing, in which case the
//! component is left in place. The displacements are typically derived
//! from channel-blocker analysis (`--suggest-shifts` in the CLI), but
//! any source may feed the command.

use serde::{Deserialize, Serialize};

use crate::{
    board::{
        edit::{BoardDataEdit, BoardEdit},
        AccessMesadata,
    },
    layout::LayoutEdit,
};

use super::{invoker::GetDebugOverlayData, Autorouter, AutorouterError};

/// A rigid displacement to apply to every pad and via of the component
/// named `refdes` (the part of a pin name before its last `-`).
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ComponentShift {
    pub refdes: String,
    /// `(dx, dy)` in board coordinate units.
    pub delta: (f64, f64),
}

#[derive(Debug)]
pub struct ShiftComponentsExecutionStepper {
    shifts: Vec<ComponentShift>,
    done: bool,
}

impl ShiftComponentsExecutionStepper {
    pub fn new(shifts: Vec<ComponentShift>) -> Result<Self, AutorouterError> {
        Ok(Self {
            shifts,
            done: false,
        })
    }

    pub fn doit(
        &mut self,
        autorouter: &mut Autorouter<impl AccessMesadata>,
    ) -> Result<Option<BoardEdit>, AutorouterError> {
        if self.done {
            return Ok(None);
        }
        self.done = true;

        let mut board_edit = BoardEdit::new_from_edits(BoardDataEdit::new(), LayoutEdit::new());
        let mut applied = 0usize;
        let mut skipped = vec![];

        for shift in &self.shifts {
            match autorouter
                .board
                .shift_component(&mut board_edit, &shift.refdes, shift.delta)
            {
                Some(moved) => {
                    applied += 1;
                    log::debug!(
                        "shifted {} ({} dots) by ({}, {})",
                        shift.refdes,
                        moved,
                        shift.delta.0,
                        shift.delta.1,
                    );
                }
                None => skipped.push(shift.refdes.clone()),
            }
        }

        log::debug!(
            "shift_components: {}/{} applied; skipped: {}",
            applied,
            self.shifts.len(),
            skipped.join(", ")
        );

        Ok(Some(board_edit))
    }
}

impl GetDebugOverlayData for ShiftComponentsExecutionStepper {}
