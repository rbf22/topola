// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

use std::f64::consts::TAU;

use enum_dispatch::enum_dispatch;
use geo::algorithm::line_measures::{Distance, Euclidean};
use geo::{point, polygon, Contains, Intersects, Line, Point, Polygon, Rotate};
use rstar::{RTreeObject, AABB};

use crate::{
    geometry::{
        shape::{AccessShape, MeasureLength},
        GetWidth,
    },
    math::{self, Circle},
};

#[enum_dispatch]
pub trait AccessPrimitiveShape: AccessShape + GetWidth {
    fn priority(&self) -> usize;
    fn inflate(&self, margin: f64) -> PrimitiveShape;
    fn intersects(&self, other: &PrimitiveShape) -> bool;

    fn envelope_3d(&self, margin: f64, layer: usize) -> AABB<[f64; 3]> {
        let envelope = self.bbox(margin);
        AABB::from_corners(
            [envelope.lower()[0], envelope.lower()[1], layer as f64],
            [envelope.upper()[0], envelope.upper()[1], layer as f64],
        )
    }

    fn full_height_envelope_3d(&self, margin: f64, layer_count: usize) -> AABB<[f64; 3]> {
        let envelope = self.bbox(margin);
        AABB::from_corners(
            [envelope.lower()[0], envelope.lower()[1], 0.0],
            [
                envelope.upper()[0],
                envelope.upper()[1],
                // XXX: Why isn't floating point infinity used here?
                (layer_count - 1) as f64,
            ],
        )
    }
}

#[enum_dispatch(AccessShape, AccessPrimitiveShape, GetWidth, MeasureLength)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PrimitiveShape {
    // Intentionally in different order to reorder `self.intersects(...)` properly.
    Dot(DotShape),
    Seg(SegShape),
    Bend(BendShape),
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DotShape {
    pub circle: Circle,
}

impl MeasureLength for DotShape {
    fn length(&self) -> f64 {
        0.0
    }
}

impl AccessShape for DotShape {
    fn center(&self) -> Point {
        self.circle.pos
    }

    fn contains_point(&self, p: Point) -> bool {
        Euclidean::distance(&p, &self.circle.pos) <= self.circle.r
    }

    fn bbox_without_margin(&self) -> AABB<[f64; 2]> {
        self.circle.bbox(0.0)
    }

    fn intersects_with_bbox(&self, bbox: &AABB<[f64; 2]>) -> bool {
        bbox.distance_2(&[self.circle.pos.x(), self.circle.pos.y()]) < self.circle.r * self.circle.r
    }
}

impl AccessPrimitiveShape for DotShape {
    fn priority(&self) -> usize {
        3
    }

    fn inflate(&self, margin: f64) -> PrimitiveShape {
        PrimitiveShape::Dot(DotShape {
            circle: Circle {
                pos: self.circle.pos,
                r: self.circle.r + margin,
            },
        })
    }

    fn intersects(&self, other: &PrimitiveShape) -> bool {
        if self.priority() < other.priority() {
            return other.intersects(&PrimitiveShape::from(*self));
        }

        match other {
            PrimitiveShape::Dot(other) => {
                Euclidean::distance(&self.circle.pos, &other.circle.pos)
                    < self.circle.r + other.circle.r
            }
            PrimitiveShape::Seg(other) => other.distance_to_point(self.circle.pos) < self.circle.r,
            PrimitiveShape::Bend(other) => {
                for point in math::intersect_circles(&self.circle, &other.inner_circle()) {
                    if other.between_ends(point) {
                        return true;
                    }
                }

                for point in math::intersect_circles(&self.circle, &other.outer_circle()) {
                    if other.between_ends(point) {
                        return true;
                    }
                }

                false
            }
        }
    }
}

impl GetWidth for DotShape {
    fn width(&self) -> f64 {
        self.circle.r * 2.0
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SegShape {
    pub from: Point,
    pub to: Point,
    pub width: f64,
}

impl SegShape {
    fn polygon(&self) -> Polygon {
        let [p1, p2, p3, p4] = self.corners();

        polygon![p1.0, p2.0, p3.0, p4.0]
    }

    /// The four corners of the segment's rectangular footprint, in the same
    /// order as the exterior ring of [`SegShape::polygon`].
    fn corners(&self) -> [Point; 4] {
        let tangent_vector = self.to - self.from;
        let tangent_vector_norm = Euclidean::distance(&tangent_vector, &point! {x: 0.0, y: 0.0});
        let unit_tangent_vector = tangent_vector / tangent_vector_norm;

        let normal = unit_tangent_vector.rotate_around_point(-90., point! {x: 0.0, y: 0.0});

        [
            self.from - normal * (self.width / 2.),
            self.from + normal * (self.width / 2.),
            self.to + normal * (self.width / 2.),
            self.to - normal * (self.width / 2.),
        ]
    }

    /// The four edges of the segment's rectangular footprint, equivalent to
    /// `self.polygon().exterior().lines()` but without allocating.
    fn edges(&self) -> [Line; 4] {
        let [p1, p2, p3, p4] = self.corners();

        [
            Line::new(p1, p2),
            Line::new(p2, p3),
            Line::new(p3, p4),
            Line::new(p4, p1),
        ]
    }

    /// Intersection test for the segments' rectangular footprints via the
    /// separating axis theorem. Exactly equivalent to
    /// `self.polygon().intersects(&other.polygon())` but without allocating.
    fn rect_intersects_rect(&self, other: &SegShape) -> bool {
        let axes = |seg: &SegShape| {
            let tangent_vector = seg.to - seg.from;
            let length = Euclidean::distance(&seg.to, &seg.from);
            let unit_tangent_vector = tangent_vector / length;
            let normal = unit_tangent_vector.rotate_around_point(-90., point! {x: 0.0, y: 0.0});
            (unit_tangent_vector, normal, length / 2.0)
        };

        // Preserve the old code path for degenerate segments, where the
        // polygon degenerates to NaN or a point.
        if Euclidean::distance(&self.to, &self.from) == 0.0
            || Euclidean::distance(&other.to, &other.from) == 0.0
        {
            return self.polygon().intersects(&other.polygon());
        }

        let center_a = (self.from + self.to) / 2.0;
        let center_b = (other.from + other.to) / 2.0;
        let delta = center_b - center_a;

        let (t_a, n_a, hl_a) = axes(self);
        let (t_b, n_b, hl_b) = axes(other);
        let (hw_a, hw_b) = (self.width / 2.0, other.width / 2.0);

        for axis in [t_a, n_a, t_b, n_b] {
            let separation = (delta.x() * axis.x() + delta.y() * axis.y()).abs();
            let extent_a = (t_a.x() * axis.x() + t_a.y() * axis.y()).abs() * hl_a
                + hw_a * (n_a.x() * axis.x() + n_a.y() * axis.y()).abs();
            let extent_b = (t_b.x() * axis.x() + t_b.y() * axis.y()).abs() * hl_b
                + hw_b * (n_b.x() * axis.x() + n_b.y() * axis.y()).abs();

            if separation > extent_a + extent_b {
                return false;
            }
        }

        true
    }

    /// Distance to the segment's rectangular footprint, computed in the
    /// segment's local frame where it is axis-aligned. This is exactly
    /// `Euclidean::distance(pos, self.polygon())` but without materialising
    /// the polygon.
    fn distance_to_point(&self, p: Point) -> f64 {
        let tangent = self.to - self.from;
        let length = Euclidean::distance(&self.to, &self.from);

        if length == 0.0 {
            return Euclidean::distance(&p, &self.from);
        }

        let rel = p - self.from;
        let along = (rel.x() * tangent.x() + rel.y() * tangent.y()) / length;
        let perp = (-rel.x() * tangent.y() + rel.y() * tangent.x()) / length;
        let d_along = along - along.clamp(0.0, length);
        let d_perp = perp - perp.clamp(-self.width / 2., self.width / 2.);
        (d_along * d_along + d_perp * d_perp).sqrt()
    }

    pub fn middle_line(&self) -> Line {
        Line {
            start: self.from.into(),
            end: self.to.into(),
        }
    }
}

impl From<SegShape> for Polygon {
    #[inline(always)]
    fn from(x: SegShape) -> Polygon {
        x.polygon()
    }
}

impl MeasureLength for SegShape {
    fn length(&self) -> f64 {
        Euclidean::distance(&self.to, &self.from)
    }
}

impl AccessShape for SegShape {
    fn center(&self) -> Point {
        (self.from + self.to) / 2.0
    }

    #[inline]
    fn contains_point(&self, p: Point) -> bool {
        self.polygon().contains(&p)
    }

    #[inline]
    fn bbox_without_margin(&self) -> AABB<[f64; 2]> {
        self.polygon().bbox_without_margin()
    }

    #[inline]
    fn intersects_with_bbox(&self, bbox: &AABB<[f64; 2]>) -> bool {
        self.polygon().intersects_with_bbox(bbox)
    }
}

impl AccessPrimitiveShape for SegShape {
    fn priority(&self) -> usize {
        2
    }

    fn inflate(&self, margin: f64) -> PrimitiveShape {
        PrimitiveShape::Seg(SegShape {
            from: self.from,
            to: self.to,
            width: self.width + 2.0 * margin,
        })
    }

    fn intersects(&self, other: &PrimitiveShape) -> bool {
        if self.priority() < other.priority() {
            return other.intersects(&PrimitiveShape::from(*self));
        }

        match other {
            PrimitiveShape::Dot(..) => unreachable!(),
            PrimitiveShape::Seg(other) => self.rect_intersects_rect(other),
            PrimitiveShape::Bend(other) => {
                for segment in self.edges() {
                    let inner_circle = other.inner_circle();
                    let outer_circle = other.outer_circle();

                    for point in math::intersect_circle_segment(&inner_circle, &segment) {
                        if other.between_ends(point) {
                            return true;
                        }
                    }

                    for point in math::intersect_circle_segment(&outer_circle, &segment) {
                        if other.between_ends(point) {
                            return true;
                        }
                    }
                }

                false
            }
        }
    }
}

impl GetWidth for SegShape {
    fn width(&self) -> f64 {
        self.width
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BendShape {
    pub from: Point,
    pub to: Point,
    pub inner_circle: Circle,
    pub width: f64,
}

impl BendShape {
    pub fn radius(&self) -> f64 {
        self.inner_circle.r + self.width / 2.0
    }

    pub fn inner_circle(&self) -> Circle {
        self.inner_circle
    }

    pub fn circle(&self) -> Circle {
        Circle {
            pos: self.inner_circle.pos,
            r: self.radius(),
        }
    }

    pub fn outer_circle(&self) -> Circle {
        Circle {
            pos: self.inner_circle.pos,
            r: self.inner_circle.r + self.width,
        }
    }

    pub fn between_ends(&self, point: Point) -> bool {
        math::between_vectors(
            point - self.inner_circle.pos,
            self.from - self.inner_circle.pos,
            self.to - self.inner_circle.pos,
        )
    }

    /// Determines if the provided point lies in the swept region between the rays cast by tangents
    /// to the arc endpoints of the bend's outer circle, starting at `from` and `to`.
    pub fn between_tangent_rays(&self, point: Point) -> bool {
        math::perp_dot_product(point - self.from, self.from_tangent_ray()) > 0.0
            && math::perp_dot_product(point - self.to, self.to_tangent_ray()) < 0.0
    }

    pub fn from_tangent_ray(&self) -> Point {
        // 90-degree CW rotation.
        let center = self.inner_circle.pos;
        let v_from = self.from - center;

        Point::new(v_from.y(), -v_from.x())
    }

    pub fn to_tangent_ray(&self) -> Point {
        // 90-degree CCW rotation.
        let center = self.inner_circle.pos;
        let v_to = self.to - center;

        Point::new(-v_to.y(), v_to.x())
    }

    pub fn start_angle(&self) -> f64 {
        let r = self.from - self.inner_circle.pos;
        math::vector_angle(r)
    }

    pub fn spanned_angle(&self) -> f64 {
        let r1 = self.from - self.inner_circle.pos;
        let r2 = self.to - self.inner_circle.pos;

        // bends always go counterclockwise from `from` to `to`
        // (this is the usual convention, no adjustment needed)
        let angle = math::angle_between(r1, r2);

        // atan2 returns values normalized into the range (-pi, pi]
        // so for angles below 0 we add 1 winding to get a nonnegative angle
        if angle < 0.0 {
            angle + TAU
        } else {
            angle
        }
    }

    /// Render this bend as a list of points on its circle.
    pub fn render_discretization(&self, point_count: usize) -> impl Iterator<Item = Point> + '_ {
        let circle = self.circle();
        let angle_from = self.start_angle();
        // we need to use one less than the whole point count
        // because we need to also emit the end-point
        let angle_step = self.spanned_angle() / ((point_count - 1) as f64);
        (0..point_count).map(move |i| circle.position_at_angle(angle_from + i as f64 * angle_step))
    }
}

impl MeasureLength for BendShape {
    fn length(&self) -> f64 {
        self.spanned_angle() * self.radius()
    }
}

impl AccessShape for BendShape {
    fn center(&self) -> Point {
        let sum = (self.from - self.inner_circle.pos) + (self.to - self.inner_circle.pos);
        self.inner_circle.pos
            + (sum / Euclidean::distance(&sum, &geo::point! { x: 0.0, y: 0.0 }))
                * self.inner_circle.r
    }

    fn contains_point(&self, p: Point) -> bool {
        let d = Euclidean::distance(&p, &self.inner_circle.pos);
        self.between_ends(p) && d >= self.inner_circle().r && d <= self.outer_circle().r
    }

    fn bbox_without_margin(&self) -> AABB<[f64; 2]> {
        self.inner_circle.bbox(self.width)
    }
}

impl AccessPrimitiveShape for BendShape {
    fn priority(&self) -> usize {
        1
    }

    fn inflate(&self, margin: f64) -> PrimitiveShape {
        PrimitiveShape::Bend(BendShape {
            from: self.from, // TODO: Is not inflated for now.
            to: self.to,     // TODO: Is not inflated for now.
            inner_circle: Circle {
                pos: self.inner_circle.pos,
                r: self.inner_circle.r - margin,
            },
            width: self.width + 2.0 * margin,
        })
    }

    fn intersects(&self, other: &PrimitiveShape) -> bool {
        if self.priority() < other.priority() {
            return other.intersects(&PrimitiveShape::from(*self));
        }

        match other {
            PrimitiveShape::Dot(..) | PrimitiveShape::Seg(..) => unreachable!(),
            PrimitiveShape::Bend(other) => {
                for point in math::intersect_circles(&self.inner_circle(), &other.inner_circle()) {
                    if self.between_ends(point) && other.between_ends(point) {
                        return true;
                    }
                }

                for point in math::intersect_circles(&self.inner_circle(), &other.outer_circle()) {
                    if self.between_ends(point) && other.between_ends(point) {
                        return true;
                    }
                }

                for point in math::intersect_circles(&self.outer_circle(), &other.inner_circle()) {
                    if self.between_ends(point) && other.between_ends(point) {
                        return true;
                    }
                }

                for point in math::intersect_circles(&self.outer_circle(), &other.outer_circle()) {
                    if self.between_ends(point) && other.between_ends(point) {
                        return true;
                    }
                }

                false
            }
        }
    }
}

impl GetWidth for BendShape {
    fn width(&self) -> f64 {
        self.width
    }
}

impl RTreeObject for PrimitiveShape {
    type Envelope = AABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        AccessShape::bbox(self, 0.0)
    }
}

#[cfg(test)]
mod tests {
    use geo::algorithm::line_measures::{Distance, Euclidean};
    use geo::point;

    use super::*;

    #[test]
    fn seg_distance_to_point_matches_polygon_distance() {
        let segs = [
            SegShape {
                from: point! {x: 0.0, y: 0.0},
                to: point! {x: 10.0, y: 0.0},
                width: 2.0,
            },
            SegShape {
                from: point! {x: -3.0, y: -4.0},
                to: point! {x: 5.0, y: 6.0},
                width: 1.3,
            },
            SegShape {
                from: point! {x: 7.0, y: 1.0},
                to: point! {x: -2.0, y: -5.0},
                width: 0.4,
            },
        ];

        for seg in segs {
            for x100 in (-800..=1200).step_by(7) {
                for y100 in (-800..=800).step_by(7) {
                    let p = point! {x: x100 as f64 / 100.0, y: y100 as f64 / 100.0};
                    let expected = Euclidean::distance(&p, &seg.polygon());
                    let actual = seg.distance_to_point(p);
                    assert!(
                        (actual - expected).abs() < 1e-9,
                        "seg {:?} p {:?}: got {}, want {}",
                        seg,
                        p,
                        actual,
                        expected,
                    );
                }
            }
        }
    }

    #[test]
    fn seg_rect_intersection_matches_polygon_intersection() {
        let seg = SegShape {
            from: point! {x: -3.0, y: -4.0},
            to: point! {x: 5.0, y: 6.0},
            width: 1.3,
        };

        // Sweep a second segment of varying orientation, width and position
        // across the first, comparing the SAT result against polygon
        // intersection at each position. The grid is dense enough to cross
        // the touching boundary many times.
        for angle_deg in (0..360).step_by(13) {
            let angle = (angle_deg as f64).to_radians();
            let dir = point! {x: angle.cos(), y: angle.sin()};
            let other = SegShape {
                from: dir * -4.5,
                to: dir * 4.5,
                width: 0.7,
            };

            for dx100 in (-1200..=1200).step_by(11) {
                for dy100 in (-1200..=1200).step_by(11) {
                    let offset = point! {x: dx100 as f64 / 100.0, y: dy100 as f64 / 100.0};
                    let moved = SegShape {
                        from: other.from + offset,
                        to: other.to + offset,
                        width: other.width,
                    };

                    let expected = seg.polygon().intersects(&moved.polygon());
                    let actual = seg.rect_intersects_rect(&moved);
                    assert_eq!(actual, expected, "seg {:?} other {:?}", seg, moved,);
                }
            }
        }
    }
}
