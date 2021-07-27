autoware_demos {#autoware-demos-package-design}
====================

This is the design document for the `autoware_demos` package.


# Purpose / Use cases
<!-- Required -->
Autoware.Auto is designed to be flexible and cover many ODDs and use-cases. However, this
flexibility makes it difficult to build a node architecture for a single use case. The purpose
of this package is to provide demonstration launch and accompanying files to showcase feature sets
or complete architectures for specific use-cases.

The primary launch files for each ODD development cycle are also be stored here and named
according to the ODD.


# Design
<!-- Required -->
Each launch file within `autoware_demos` should:

- Be able to be launched independently;
- Showcase a single, self-contained pipeline or a complete architecture for a specific use-case;
- Use domain-specific launch files from `autoware_auto_launch` when feasible;
- Use use-case-agnostic configuration files under the `param/` folder when possible;
- Use use-case-specific configuration files under a subfolder of `param/` when necessary;


## Assumptions / Known limits
<!-- Required -->
N/A


## Inputs / Outputs / API
<!-- Required -->
Each launch file should contain parameters which expose at least the ability to change the
configuration YAML file path for each launched node.


## Error detection and handling
<!-- Required -->

# Security considerations
<!-- Required -->
<!-- Things to consider:
- Spoofing (How do you check for and handle fake input?)
- Tampering (How do you check for and handle tampered input?)
- Repudiation (How are you affected by the actions of external actors?).
- Information Disclosure (Can data leak?).
- Denial of Service (How do you handle spamming?).
- Elevation of Privilege (Do you need to change permission levels during execution?) -->


# Related issues
<!-- Required -->
- create `autoware_demos` package [#904](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/904)
