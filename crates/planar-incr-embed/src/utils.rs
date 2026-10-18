// SPDX-FileCopyrightText: 2025 Topola contributors
//
// SPDX-License-Identifier: MIT

use crate::RelaxedPath;
use alloc::vec::Vec;

fn handle_lifo<EP: Clone + Eq>(stack: &mut Vec<EP>, item: &EP) {
    if stack.last() == Some(item) {
        stack.pop();
    } else {
        stack.push(item.clone());
    }
}

pub fn handle_lifo_relaxed<EP: Clone + Eq, CT>(stack: &mut Vec<EP>, item: &RelaxedPath<EP, CT>) {
    match item {
        RelaxedPath::Normal(item) => handle_lifo(stack, item),
        RelaxedPath::Weak(_) => {}
    }
}

/// Rotates a finite iterator around such that it starts at `start`, and note the start index
pub fn rotate_iter<Item, Iter, F>(iter: Iter, is_start: F) -> (usize, impl Iterator<Item = Item>)
where
    Iter: Clone + Iterator<Item = Item>,
    F: Clone + Fn(&Item) -> bool,
{
    let not_is_start = move |i: &Item| !is_start(i);

    let mut it_first = iter.clone().peekable();
    let mut start_idx = 0;
    while it_first.peek().is_some_and(|item| not_is_start(item)) {
        it_first.next();
        start_idx += 1;
    }

    (start_idx, it_first.chain(iter.take_while(not_is_start)))
}

pub(crate) fn euclidean_distance<Scalar>(
    a: &spade::Point2<Scalar>,
    b: &spade::Point2<Scalar>,
) -> Scalar
where
    Scalar: num_traits::Float,
{
    let delta = spade::Point2 {
        x: a.x - b.x,
        y: a.y - b.y,
    };
    delta.y.hypot(delta.x)
}
