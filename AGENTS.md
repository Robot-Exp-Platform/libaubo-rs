# libaubo: entry for developers and AI

Read [README](README.md) first: the driver adapts AUBO's ARCS protocols (JSON-RPC on TCP 30004, RTDE on TCP 30010) to the roplat_behavior contracts; its simulated controller and acceptance tests are `roplat_aubo_sim` in roplat_protocols; real-robot runs need an ARCS controller on the network.

- Library, contract and driver code follows the style rules in roplat-skills: [library-code-style.md](https://github.com/Robot-Exp-Platform/roplat-skills/blob/next/skills/roplat-development/references/library-code-style.md) (branch `next`; every rule carries a strength: must / should / may). New code follows them at once; existing code when it is touched.
