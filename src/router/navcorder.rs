// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

use thiserror::Error;

use crate::{
    drawing::{band::BandTermsegIndex, dot::FixedDotIndex, graph::GetMaybeNet, rules::AccessRules},
    geometry::GetLayer,
    layout::{Layout, LayoutEdit},
};

use super::{
    draw::{Draw, DrawException},
    navcord::Navcord,
    navmesh::{Navmesh, NavnodeIndex},
};

#[derive(Error, Debug, Clone)]
pub enum NavcorderException {
    #[error(transparent)]
    CannotDraw(#[from] DrawException),
    #[error("cannot wrap")]
    CannotWrap,
}

pub trait Navcorder {
    fn start(
        &mut self,
        recorder: LayoutEdit,
        source: FixedDotIndex,
        source_navnode: NavnodeIndex,
        width: f64,
    ) -> Navcord;

    fn finish(
        &mut self,
        _navmesh: &Navmesh,
        navcord: &mut Navcord,
        target: FixedDotIndex,
    ) -> Result<BandTermsegIndex, NavcorderException>;

    fn rework_path(
        &mut self,
        navmesh: &Navmesh,
        navcord: &mut Navcord,
        path: &[NavnodeIndex],
    ) -> Result<(), NavcorderException>;

    fn path(
        &mut self,
        navmesh: &Navmesh,
        navcord: &mut Navcord,
        path: &[NavnodeIndex],
    ) -> Result<(), NavcorderException>;

    fn undo_path(&mut self, navcord: &mut Navcord, step_count: usize);
}

impl<R: AccessRules> Navcorder for Layout<R> {
    fn start(
        &mut self,
        recorder: LayoutEdit,
        source: FixedDotIndex,
        source_navnode: NavnodeIndex,
        width: f64,
    ) -> Navcord {
        let source_primitive = self.drawing().primitive_ref(source);
        Navcord::new(
            recorder,
            source,
            source_navnode,
            width,
            source_primitive.maybe_net(),
            source_primitive.layer(),
        )
    }

    fn finish(
        &mut self,
        _navmesh: &Navmesh,
        navcord: &mut Navcord,
        target: FixedDotIndex,
    ) -> Result<BandTermsegIndex, NavcorderException> {
        Ok(self.finish_in_dot(&mut navcord.recorder, navcord.head, target, navcord.width)?)
    }

    fn rework_path(
        &mut self,
        navmesh: &Navmesh,
        navcord: &mut Navcord,
        path: &[NavnodeIndex],
    ) -> Result<(), NavcorderException> {
        debug_assert!(path[0] == navcord.path[0]);
        let __ret = {
            crate::metrics::incr(&crate::metrics::METRICS.rework_path_calls);
            let _timer = crate::metrics::Timer::start(&crate::metrics::METRICS.rework_path_nanos);

            let prefix_length = navcord
                .path
                .iter()
                .zip(path)
                .take_while(|(v1, v2)| v1 == v2)
                .count();

            let length = navcord.path.len();
            crate::metrics::add(
                &crate::metrics::METRICS.rework_steps_undone,
                length - prefix_length,
            );
            crate::metrics::add(
                &crate::metrics::METRICS.rework_steps_redrawn,
                path.len() - prefix_length,
            );
            self.undo_path(navcord, length - prefix_length);
            // XXX: If this fails now, there may be a race condition.
            self.path(navmesh, navcord, &path[prefix_length..])
        };
        debug_assert!(__ret.is_err() || navcord.path.len() == path.len());
        __ret
    }

    fn path(
        &mut self,
        navmesh: &Navmesh,
        navcord: &mut Navcord,
        path: &[NavnodeIndex],
    ) -> Result<(), NavcorderException> {
        let __old_len = navcord.path.len() + path.len();
        let __ret = (|| -> Result<(), NavcorderException> {
            for (i, vertex) in path.iter().enumerate() {
                if let Err(err) = navcord.step_to(self, navmesh, *vertex) {
                    self.undo_path(navcord, i);
                    return Err(err);
                }
            }

            Ok(())
        })();
        debug_assert!(__ret.is_err() || navcord.path.len() == __old_len);
        __ret
    }

    fn undo_path(&mut self, navcord: &mut Navcord, step_count: usize) {
        let __old_len = navcord.path.len() - step_count;
        for _ in 0..step_count {
            let _ = navcord.step_back(self);
        }
        debug_assert!(navcord.path.len() == __old_len);
    }
}
