# External dependencies {#external_depends}

This folder is reserved for all external libraries that are not available via the conan package manager. To add a library the go-to route would be to clone the repository as a submodule and include it via `add_subdirectory(...)`. If a library doesn't support cmake than it would be necessary to clone the repository into a sub-subfolder and build a custom cmake target.
