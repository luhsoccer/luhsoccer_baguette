# Baguette {#mainpage}

`Codename Frikandel. Let's win Eindhoven!`

The official luhsoccer software to make goals using many, many actions...

## Development Setup

Information on how to setup the development environment can be found in the [Development Guide](DEVELOPMENT.md).

## Important Links

All of these things are updated when a new change comes to the main branch:

- [Documentation](https://software.luhbots-hannover.de/)
- [Test coverage](https://software.luhbots-hannover.de/test_coverage.html)

## Installation

Baguette is distributed as a python package. To install it, run the following commands:

- (Only the first time) In order give `pip` access to the private gitlab repository, you need to setup the access key: `pip config set global.extra-index-url https://__token__:glpat-8mnx4F6fKs9XYPrL-LwL@gitlab.com/api/v4/projects/42697188/packages/pypi/simple --user`

- Install the package: `pip install luhsoccer-baguette`

- Update the package: `pip install luhsoccer-baguette --upgrade --pre`

- Install a development (pre-release) version: `pip install luhsoccer-baguette --upgrade --pre`

- Switch back to the latest stable version: `pip install --upgrade --force-reinstall luhsoccer-baguette`

- Uninstall the package: `pip uninstall luhsoccer-baguette`

<!-- @subpage modules_page -->
<!-- @subpage configs
@subpage include_module
@subpage external_depends
@subpage tests -->
