# libaubo-rs

## Independent source checkout

This internal development baseline pins `robot_behavior` to a specific GitHub
commit because its current 0.6 API has not been published on crates.io. The
manifest is complete and does not inherit dependencies from a parent drives
workspace. Use an authorized SSH key/agent and
`CARGO_NET_GIT_FETCH_WITH_CLI=true` when building from a standalone checkout;
no sibling `robot_behavior` or `roplat` directory is required. Native driver
builds do not enable the optional roplat framework adapter.
