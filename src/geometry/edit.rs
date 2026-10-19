// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

use std::collections::{btree_map::Entry, BTreeMap};

use crate::graph::{GenericIndex, GetIndex};

use super::{AccessBendWeight, AccessDotWeight, AccessSegWeight, GetLayer};

pub trait Edit: Sized + Default {
    fn reverse(&self) -> Self
    where
        Self: Clone,
    {
        let mut rev = self.clone();
        rev.reverse_inplace();
        rev
    }

    fn reverse_inplace(&mut self);

    fn merge(&mut self, edit: Self);

    fn merge_iter(iter: impl IntoIterator<Item = Self>) -> Self {
        let mut result_edit = Self::default();

        for curr_edit in iter {
            result_edit.merge(curr_edit);
        }

        result_edit
    }
}

pub trait ApplyGeometryEdit<
    DW: AccessDotWeight + GetLayer,
    SW: AccessSegWeight + GetLayer,
    BW: AccessBendWeight + GetLayer,
    CW: Clone,
    Cel: Copy,
    PI: GetIndex + TryInto<DI> + TryInto<SI> + TryInto<BI> + Eq + Ord + Copy,
    DI: GetIndex + Into<PI> + Eq + Ord + Copy,
    SI: GetIndex + Into<PI> + Eq + Ord + Copy,
    BI: GetIndex + Into<PI> + Eq + Ord + Copy,
>
{
    fn apply(&mut self, edit: &GeometryEdit<DW, SW, BW, CW, Cel, PI, DI, SI, BI>);
}

/// `(old, new)` payload of one recorded primitive change — `None` on
/// a side means the primitive did not exist there, so reversing the
/// edit inserts or removes it.
type EditValue<T> = (Option<T>, Option<T>);
type SegEditValue<DI, SW> = EditValue<((DI, DI), SW)>;
type BendEditValue<DI, BI, BW> = EditValue<((DI, DI, DI, Option<BI>), BW)>;
type CompoundEditValue<Cel, PI, CW> = EditValue<(Vec<(Cel, PI)>, CW)>;

#[derive(Clone, Debug)]
pub struct GeometryEdit<DW, SW, BW, CW, Cel, PI, DI, SI, BI> {
    pub(super) dots: BTreeMap<DI, EditValue<DW>>,
    pub(super) segs: BTreeMap<SI, SegEditValue<DI, SW>>,
    pub(super) bends: BTreeMap<BI, BendEditValue<DI, BI, BW>>,
    pub(super) compounds: BTreeMap<GenericIndex<CW>, CompoundEditValue<Cel, PI, CW>>,
}

impl<DW, SW, BW, CW, Cel, PI, DI, SI, BI> Default
    for GeometryEdit<DW, SW, BW, CW, Cel, PI, DI, SI, BI>
{
    fn default() -> Self {
        Self {
            dots: BTreeMap::new(),
            segs: BTreeMap::new(),
            bends: BTreeMap::new(),
            compounds: BTreeMap::new(),
        }
    }
}

impl<DW, SW, BW, CW, Cel, PI, DI, SI, BI> GeometryEdit<DW, SW, BW, CW, Cel, PI, DI, SI, BI> {
    #[inline(always)]
    pub fn new() -> Self {
        Self::default()
    }

    /// Dots this edit creates (i.e. removes when reversed).
    pub fn created_dots(&self) -> impl Iterator<Item = DI> + '_
    where
        DI: Copy,
    {
        self.dots
            .iter()
            .filter(|(_, (old, new))| old.is_none() && new.is_some())
            .map(|(index, _)| *index)
    }

    /// Segs this edit creates (i.e. removes when reversed).
    pub fn created_segs(&self) -> impl Iterator<Item = SI> + '_
    where
        SI: Copy,
    {
        self.segs
            .iter()
            .filter(|(_, (old, new))| old.is_none() && new.is_some())
            .map(|(index, _)| *index)
    }

    /// Bends this edit creates (i.e. removes when reversed).
    pub fn created_bends(&self) -> impl Iterator<Item = BI> + '_
    where
        BI: Copy,
    {
        self.bends
            .iter()
            .filter(|(_, (old, new))| old.is_none() && new.is_some())
            .map(|(index, _)| *index)
    }
}

impl<
        DW: AccessDotWeight + GetLayer,
        SW: AccessSegWeight + GetLayer,
        BW: AccessBendWeight + GetLayer,
        CW: Clone,
        Cel: Copy,
        PI: GetIndex + TryInto<DI> + TryInto<SI> + TryInto<BI> + Eq + Ord + Copy,
        DI: GetIndex + Into<PI> + Eq + Ord + Copy,
        SI: GetIndex + Into<PI> + Eq + Ord + Copy,
        BI: GetIndex + Into<PI> + Eq + Ord + Copy,
    > Edit for GeometryEdit<DW, SW, BW, CW, Cel, PI, DI, SI, BI>
{
    fn reverse_inplace(&mut self) {
        self.dots.reverse_inplace();
        self.segs.reverse_inplace();
        self.bends.reverse_inplace();
        self.compounds.reverse_inplace();
    }

    fn merge(&mut self, edit: Self) {
        Edit::merge(&mut self.dots, edit.dots);
        Edit::merge(&mut self.segs, edit.segs);
        Edit::merge(&mut self.bends, edit.bends);
        Edit::merge(&mut self.compounds, edit.compounds);
    }
}

impl<K: Eq + Ord, V> Edit for BTreeMap<K, (Option<V>, Option<V>)> {
    fn reverse_inplace(&mut self) {
        self.values_mut()
            .for_each(|x| core::mem::swap(&mut x.0, &mut x.1));
    }

    fn merge(&mut self, edit: Self) {
        for (index, (old, new)) in edit {
            match self.entry(index) {
                Entry::Vacant(vac) => {
                    vac.insert((old, new));
                }
                Entry::Occupied(mut occ) => match (occ.get(), new) {
                    ((None, _), None) => {
                        occ.remove();
                    }
                    (_, new) => {
                        occ.get_mut().1 = new;
                    }
                },
            }
        }
    }
}
