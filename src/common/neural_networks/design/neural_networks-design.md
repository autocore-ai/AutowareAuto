Neural networks design {#neural-networks-design}
=======================

# Purpose / Use cases
<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->

This package provides pre-compiled neural networks to packages using them for their inference.


# Design
<!-- Required -->
<!-- Things to consider:
    - How does it work? -->

The neural networks are compiled as part of the
[Model Zoo](https://github.com/autowarefoundation/modelzoo/) CI pipeline and saved to an S3 bucket.
They are downloaded and installed when this package is built when the `DOWNLOAD_ARTIFACTS` variable
is set.

The user can provide its own compiled networks to be made available through this package, or to
overwrite the pre-compiled ones by creating a `user` directory and using the same directory
structure.

The structure is as follow:

```
.
├── ${ARCH 1}
│   ├── ${MODEL 1}
│   │   ├── ${BACKEND 1}
│   │   │   ├── deploy_graph.json
│   │   │   ├── deploy_lib.so
│   │   │   ├── deploy_param.params
│   │   │   └── inference)_engine_tvm_config.hpp
│   │   └── ${BACKEND ...}
│   │       └── ...
│   └── ${MODEL ...}
│       └── ...
└── ${ARCH ...}
    └── ...
```

The pre-compiled networks are only downloaded the first time this package is built. To re-trigger
the download step, the user can remove the package's build directory.

## Assumptions / Known limits
<!-- Required -->

An internet connection is required at build time.


## Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

Inputs:
- `DOWNLOAD_ARTIFACTS` variable needs to be set to enable downloading the artifacts

Outputs:
- `neural_networks_NETWORKS_DIR` variable containing the path to the root directory of the networks
- `neural_networks_NAMES` variable containing the list of available networks

API: none


## Error detection and handling
<!-- Required -->

Building the package will not fail.
If no models are available for the target architecture, then a log message is displayed.


# Security considerations
<!-- Required -->
<!-- Things to consider:
- Spoofing (How do you check for and handle fake input?)
- Tampering (How do you check for and handle tampered input?)
- Repudiation (How are you affected by the actions of external actors?).
- Information Disclosure (Can data leak?).
- Denial of Service (How do you handle spamming?).
- Elevation of Privilege (Do you need to change permission levels during execution?) -->

The pre-compiled networks are downloaded from an S3 bucket and are under threat of spoofing,
tampering and denial of service.
Spoofing is mitigated by using an https connection.
Mitigations for tampering and denial of service are left to AWS.

The user-provided networks are installed as they are on the host system.
The user is in charge of securing the files they provide with regard to information disclosure.


# Related issues
<!-- Required -->

- #721: Machine learning inference framework in Autoware.auto
