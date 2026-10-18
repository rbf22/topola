// SPDX-FileCopyrightText: 2026 Topola contributors
//
// SPDX-License-Identifier: MIT

//! Minimal `RUST_LOG`-driven stderr logger.
//!
//! Supports the common `env_logger` filter syntax: a global level
//! (`RUST_LOG=debug`), bare module paths enabling all levels for that
//! target (`RUST_LOG=topola::router`), and `module=level` directives,
//! comma-separated. Regex postfilters are not supported.
//!
//! With `RUST_LOG` unset the default level is `warn`.

use log::{LevelFilter, Log, Metadata, Record};

struct CliLogger {
    global: LevelFilter,
    directives: Vec<(String, LevelFilter)>,
}

impl CliLogger {
    fn from_env() -> Self {
        let mut global = LevelFilter::Warn;
        let mut directives = vec![];

        if let Ok(spec) = std::env::var("RUST_LOG") {
            global = LevelFilter::Off;
            for directive in spec.split(',') {
                let directive = directive.trim();
                if directive.is_empty() {
                    continue;
                }
                match directive.split_once('=') {
                    Some((module, level)) => {
                        if let Ok(level) = level.parse::<LevelFilter>() {
                            directives.push((module.to_string(), level));
                        }
                    }
                    None => {
                        if let Ok(level) = directive.parse::<LevelFilter>() {
                            global = level;
                        } else {
                            directives.push((directive.to_string(), LevelFilter::Trace));
                        }
                    }
                }
            }
        }

        Self { global, directives }
    }
}

impl Log for CliLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        // Most specific (longest-prefix) directive wins, then global.
        let level = self
            .directives
            .iter()
            .filter(|(prefix, _)| metadata.target().starts_with(prefix.as_str()))
            .max_by_key(|(prefix, _)| prefix.len())
            .map(|(_, level)| *level)
            .unwrap_or(self.global);
        metadata.level() <= level
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            eprintln!(
                "[{}] {}: {}",
                record.level(),
                record.target(),
                record.args()
            );
        }
    }

    fn flush(&self) {}
}

pub fn init() {
    // Intentional one-time leak: `log::set_logger` requires a 'static ref
    // and `log`'s `alloc` feature (which would enable `set_boxed_logger`)
    // is disabled.
    let logger: &'static CliLogger = Box::leak(Box::new(CliLogger::from_env()));
    log::set_logger(logger).ok();
    log::set_max_level(LevelFilter::Trace);
}
