// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

//! Module for handling Specctra's mesadata - design rules, as well as layers
//! or net properties

use bimap::BiBTreeMap;
use std::collections::BTreeMap;

use crate::{
    rules::{AccessRules, Conditions},
    structure::Pcb,
};

/// Trait for managing the Specctra's mesadata
///
/// This trait implements generic function for accessing or modifying different
/// compounds of board parts like nets or layers
pub trait AccessMesadata: AccessRules + std::panic::RefUnwindSafe {
    /// Renames a layer based on its index.
    fn bename_layer(&mut self, layer: usize, layername: String);

    /// Retrieves the name of a layer from its index.
    fn layer_layername(&self, layer: usize) -> Option<&str>;

    /// Retrieves the index of a layer from its name.
    fn layername_layer(&self, layername: &str) -> Option<usize>;

    /// Renames a net based on its index.
    fn bename_net(&mut self, net: usize, netname: String);

    /// Retrieves the name of a net by its index.
    fn net_netname(&self, net: usize) -> Option<&str>;

    /// Retrieves the index of a net by its name.
    fn netname_net(&self, netname: &str) -> Option<usize>;

    /// Overrides every clearance to `clearance`. Relaxed channel
    /// probes use this to measure demand paths; run them on a cloned
    /// autorouter so the live board's rules are unaffected. The
    /// default is a no-op for mesadatas without clearance tracking.
    fn override_clearances(&mut self, _clearance: f64) {}

    /// Radius for vias the autorouter places (fanout, mid-route
    /// weaves). Designs declare via padstacks whose scale is arbitrary;
    /// the default preserves the historical constant for mesadatas
    /// that do not track them.
    fn default_via_radius(&self) -> f64 {
        100.0
    }

    /// Width for copper the autorouter generates (fanout stubs).
    /// Designs declare rule widths whose scale is arbitrary; the
    /// default preserves the historical constant for mesadatas that
    /// do not track them.
    fn default_trace_width(&self) -> f64 {
        100.0
    }
}

#[derive(Clone, Debug)]
/// [`SpecctraRule`] represents the basic routing constraints used by an auto-router, such as
/// the Topola auto-router, in a PCB design process. This struct defines two key design
/// rules: the width of the trace and the minimum clearance between electrical features.
pub struct SpecctraRule {
    /// Specifies the width of the trace (or conductor) in millimeters.
    /// This value ensures that the traces meet electrical
    /// and mechanical requirements, such as current-carrying capacity or signal integrity.
    pub width: f64,
    /// Defines the minimum clearance (spacing) between traces, pads,
    /// or other conductive features on the PCB. Adequate clearance is important for
    /// preventing electrical shorts or interference between signals, and is often
    /// dictated by manufacturing constraints or voltage considerations.
    pub clearance: f64,
}

impl SpecctraRule {
    fn from_dsn(rule: &super::structure::Rule) -> Self {
        Self {
            width: rule.width as f64,
            clearance: rule.clearances[0].value as f64, // picks the generic clearance only for now
        }
    }
}

#[derive(Clone, Debug)]
/// [`SpecctraMesadata`] holds the metadata required by the Specctra auto-router to
/// understand and enforce design rules across various net classes and layers in a PCB layout.
/// This struct encapsulates information about rules for individual nets, net classes,
/// layers, and their corresponding relationships.
pub struct SpecctraMesadata {
    /// The default routing rule applied globally if no specific net class rule is defined.
    structure_rule: SpecctraRule,

    // net class name -> rule
    /// A map from net class names to their specific `SpecctraRule` constraints.
    /// These rules are applied to all nets belonging to the respective net clas
    class_rules: BTreeMap<String, SpecctraRule>,

    // layername <-> layer for Layout
    /// A bidirectional map between layer indices and layer names, allowing translation
    /// between index-based layers in the layout and user-defined layer names.
    layer_layername: BiBTreeMap<usize, String>,

    // netname <-> net for Layout
    /// A bidirectional map between network indices and network names in the PCB layout,
    /// providing an easy way to reference nets by name or index.
    net_netname: BiBTreeMap<usize, String>,

    // net -> netclass
    /// A map that associates network indices with their respective net class names.
    /// This is used to apply net class-specific routing rules to each net.
    net_netclass: BTreeMap<usize, String>,

    // net -> clearance, resolved via net class (or structure rule) at load time
    /// Net ids are dense indices assigned by `from_pcb`, so a flat vector gives
    /// O(1) lookups on the collision-check hot path.
    net_clearance: Vec<f64>,

    /// Maximum clearance across all net class rules, computed once at load.
    largest_clearance: f64,

    /// Radius of the smallest via padstack the structure declares, for
    /// autorouter-placed vias.
    via_radius: f64,
}

impl SpecctraMesadata {
    /// Creates a [`SpecctraMesadata`] instance from a given `Pcb` reference.
    ///
    /// This function extracts the necessary metadata from the `Pcb` struct, such as
    /// layer-to-layer name mappings, net-to-net name mappings, and net class rules.
    pub fn from_pcb(pcb: &Pcb) -> Self {
        let layer_layername = BiBTreeMap::from_iter(
            pcb.structure
                .layers
                .iter()
                .enumerate()
                .map(|(index, layer)| (index, layer.name.clone())),
        );

        // assign IDs to all nets named in pcb.network
        let net_netname = {
            let mut tmp: Vec<_> = pcb
                .network
                .classes
                .iter()
                .flat_map(|class| &class.nets)
                .chain(pcb.network.nets.iter().map(|net| &net.name))
                .collect();
            // deduplicate net names
            tmp.sort_unstable();
            tmp.dedup();

            BiBTreeMap::from_iter(tmp.into_iter().cloned().enumerate())
        };

        let mut net_netclass = BTreeMap::new();
        let class_rules = BTreeMap::from_iter(
            pcb.network
                .classes
                .iter()
                .inspect(|class| {
                    for netname in &class.nets {
                        // this can't panic because we did iterate over
                        // `pcb.network.classes[].nets` above
                        let net = net_netname.get_by_right(netname).unwrap();
                        net_netclass.insert(*net, class.name.clone().unwrap_or_default());
                    }
                })
                .map(|class| {
                    (
                        class.name.clone().unwrap_or_default(),
                        SpecctraRule::from_dsn(&class.rule),
                    )
                }),
        );

        let mut structure_rule = super::structure::Rule {
            width: 0.0,
            clearances: Vec::new(),
        };
        // workaround for differing syntax
        // collapse multiple rule entries into a single one
        for rule in &pcb.structure.rules {
            if let Some(width) = rule.width {
                structure_rule.width = width
            }
            structure_rule
                .clearances
                .extend_from_slice(&rule.clearances);
        }

        let structure_rule = SpecctraRule::from_dsn(&structure_rule);

        let net_clearance = (0..net_netname.len())
            .map(|net| {
                net_netclass
                    .get(&net)
                    .and_then(|netclass| class_rules.get(netclass))
                    .unwrap_or(&structure_rule)
                    .clearance
            })
            .collect();

        let largest_clearance = class_rules
            .values()
            .map(|rule| rule.clearance)
            .reduce(f64::max)
            .unwrap_or(0.0);

        // The smallest declared via padstack: fanout placement has the
        // best chance of finding room with the smallest legal via.
        let via_radius = pcb
            .structure
            .via
            .names
            .iter()
            .filter_map(|name| pcb.library.find_padstack_by_name(name))
            .flat_map(|padstack| padstack.shapes.iter())
            .filter_map(|shape| match shape {
                super::structure::Shape::Circle(circle) => Some(circle.diameter / 2.0),
                _ => None,
            })
            .reduce(f64::min)
            .unwrap_or(100.0);

        Self {
            structure_rule,
            class_rules,
            layer_layername,
            net_netname,
            net_netclass,
            net_clearance,
            largest_clearance,
            via_radius,
        }
    }

    /// Overrides every clearance (structure, class, and per-net) to
    /// `clearance`. Probe passes use this to measure demand paths under
    /// relaxed rules; clone the mesadata beforehand to restore it.
    pub fn override_clearances(&mut self, clearance: f64) {
        self.structure_rule.clearance = clearance;
        for rule in self.class_rules.values_mut() {
            rule.clearance = clearance;
        }
        self.net_clearance.fill(clearance);
        self.largest_clearance = clearance;
    }

    pub fn rule(&self, net: usize) -> &SpecctraRule {
        self.net_netclass
            .get(&net)
            .and_then(|netclass| self.class_rules.get(netclass))
            .unwrap_or(&self.structure_rule)
    }

    /// The design-wide default rule, used for nets without a net
    /// class. Its `width` is the routed trace width the design
    /// declares — a sane default for the routed band width on boards
    /// whose units make a fixed default meaningless.
    pub fn structure_rule(&self) -> &SpecctraRule {
        &self.structure_rule
    }

    /// The design's declared via padstack radius — what
    /// autorouter-placed vias should use so fanout works on boards of
    /// any coordinate scale.
    pub fn via_radius(&self) -> f64 {
        self.via_radius
    }
}

impl AccessRules for SpecctraMesadata {
    fn clearance(&self, conditions1: &Conditions<'_>, conditions2: &Conditions<'_>) -> Option<f64> {
        if conditions1.maybe_layer != conditions2.maybe_layer {
            return None;
        }

        let net_clearance = |net| {
            self.net_clearance
                .get(net)
                .copied()
                .unwrap_or(self.structure_rule.clearance)
        };

        Some(f64::max(
            net_clearance(conditions1.net),
            net_clearance(conditions2.net),
        ))
    }

    fn largest_clearance(&self, _maybe_net: Option<usize>) -> f64 {
        self.largest_clearance
    }
}

impl AccessMesadata for SpecctraMesadata {
    fn bename_layer(&mut self, layer: usize, layername: String) {
        self.layer_layername.insert(layer, layername);
    }

    fn layer_layername(&self, layer: usize) -> Option<&str> {
        self.layer_layername.get_by_left(&layer).map(|s| s.as_str())
    }

    fn layername_layer(&self, layername: &str) -> Option<usize> {
        self.layer_layername.get_by_right(layername).copied()
    }

    fn bename_net(&mut self, net: usize, netname: String) {
        self.net_netname.insert(net, netname);
    }

    fn net_netname(&self, net: usize) -> Option<&str> {
        self.net_netname.get_by_left(&net).map(|s| s.as_str())
    }

    fn netname_net(&self, netname: &str) -> Option<usize> {
        self.net_netname.get_by_right(netname).copied()
    }

    fn override_clearances(&mut self, clearance: f64) {
        SpecctraMesadata::override_clearances(self, clearance)
    }

    fn default_via_radius(&self) -> f64 {
        self.via_radius()
    }

    fn default_trace_width(&self) -> f64 {
        self.structure_rule().width
    }
}

#[cfg(test)]
mod tests {
    use std::io::Cursor;

    use crate::{
        mesadata::{AccessMesadata, SpecctraMesadata},
        read::ListTokenizer,
        rules::{AccessRules, Conditions},
        structure::DsnFile,
    };

    fn mesadata(dsn: &str) -> SpecctraMesadata {
        let mut tokenizer = ListTokenizer::new(Cursor::new(dsn.as_bytes()));
        let file: DsnFile = tokenizer.read_value().unwrap();
        SpecctraMesadata::from_pcb(&file.pcb)
    }

    #[test]
    fn clearance_uses_net_class_rules() {
        let mesadata = mesadata(
            r#"(pcb test
                 (resolution um 10)
                 (structure
                   (layer F.Cu (type signal) (property (index 0)))
                   (boundary (path pcb 0 0 0 1000 0 1000 1000 0 1000 0 0))
                   (via "Via")
                   (rule (width 100) (clearance 100)))
                 (placement)
                 (library)
                 (network
                   (net A (pins P-1))
                   (net B (pins P-2))
                   (net C (pins P-3))
                   (class cls1 A B
                     (circuit (use_via Via))
                     (rule (width 200) (clearance 200)))
                   (class cls2 ""
                     (circuit (use_via Via))
                     (rule (width 300) (clearance 300))))
                 (wiring))"#,
        );

        let net = |netname| mesadata.netname_net(netname).unwrap();
        let cond = |netname| Conditions {
            net: net(netname),
            maybe_region: None,
            maybe_layer: Some(0),
        };

        // A and B share cls1's clearance; C is unclassed and falls back to the
        // structure rule.
        assert_eq!(mesadata.clearance(&cond("A"), &cond("B")), Some(200.0));
        assert_eq!(mesadata.clearance(&cond("A"), &cond("C")), Some(200.0));
        assert_eq!(mesadata.clearance(&cond("C"), &cond("C")), Some(100.0));

        // Different layers never have a clearance constraint between them.
        let other_layer = Conditions {
            net: net("A"),
            maybe_region: None,
            maybe_layer: Some(1),
        };
        assert_eq!(mesadata.clearance(&cond("A"), &other_layer), None);

        assert_eq!(mesadata.largest_clearance(Some(net("A"))), 300.0);
        assert_eq!(mesadata.largest_clearance(None), 300.0);

        // The cached per-net clearances must agree with the rule lookup they
        // replace.
        for net in 0..3 {
            let cached = Conditions {
                net,
                maybe_region: None,
                maybe_layer: Some(0),
            };
            assert_eq!(
                mesadata.clearance(&cached, &cached),
                Some(mesadata.rule(net).clearance),
            );
        }
    }
}
