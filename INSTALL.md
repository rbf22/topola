<!--
SPDX-FileCopyrightText: 2024 Topola contributors

SPDX-License-Identifier: MIT
-->

# Installing Topola

## Building and installing Topola from source

### Prerequisites

Building Topola from source requires
[Git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git) and
[Cargo](https://doc.rust-lang.org/cargo/getting-started/installation.html)
to be installed on your system. Follow the instructions in the above links to
obtain these.

### Obtaining the source

From your command line, clone the
[repository](https://codeberg.org/topola/topola):

    git clone https://codeberg.org/topola/topola.git

### Preparing to build

Change your working directory to your clone of Topola's repository:

    cd topola

### Command-line application

Topola has a command-line application (CLI) written with the help of the
[`clap`](https://docs.rs/clap/latest/clap/) library.

#### Installation from source

(Topola can be also built and run without installation. If you do not want to install
new software on your system, skip now to the [*Debug build*](#debug-build)
subsection.)

Run the following command to build and install the Topola's command-line
application:

    cargo install --locked --path crates/topola-cli

You can then invoke the application from your terminal as `topola`. For example,

    topola --help

will display an explanation of the available command-line options and arguments.

#### Debug build

If you do not want to install new software on your system, or are interested
in developing or debugging Topola, you need to build a debug executable of the
Topola's command-line application inside your working directory by running

    cargo build -p topola-cli

Once built, you can invoke the debug executable by writing `cargo run -p
topola-cli --` in the same place and with same arguments as you would write an
installed `topola` command.

#### Autorouting example

As an example, running the following commands will autoroute a KiCad
project of a simple THT diode bridge rectifier:

    cd tests/unilayer/tht_diode_bridge_rectifier/
    topola tht_diode_bridge_rectifier.dsn

(Obviously, to use the debug executable, replace the second command with
`cargo run -p topola-cli -- tht_diode_bridge_rectifier.dsn`.)

By default, the output filename is the input filename with extension
changed to `ses`: `tht_diode_bridge_rectifier.ses`.

##### Viewing the results

You can view the results of the autorouting in KiCad if you have it
installed. First, open the layout in the KiCad PCB Editor:

    pcbnew tht_diode_bridge_rectifier.kicad_pcb

Then choose *File > Import > Specctra Session...* from the menu bar.
In the newly opened file dialog, choose the file named
*tht_diode_bridge_rectifier.ses*. This will load the autorouted traces.

### Automated tests

Topola has automated tests to make sure its basic functionalities work.
To execute these, run

    cargo test

### Internal invariants

When trying to locate the source of a bug, it may be helpful to know that
Topola's geometry mutation routines carry `debug_assert!` checks on their
node-count and bookkeeping invariants. They are active automatically in
any build with debug assertions enabled — including `cargo test` — and
compile away entirely in release builds, so no feature flags or nightly
toolchain are needed.
