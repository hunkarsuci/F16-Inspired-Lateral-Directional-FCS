# F-16-Inspired Lateral-Directional Flight Control System (Simplified)

![CI](https://github.com/hunkarsuci/F16-Inspired-Lateral-Directional-FCS/actions/workflows/ci.yml/badge.svg)

A simplified, F-16-inspired lateral-directional flight control system simulation
with command shaping, state feedback, and second-order actuator dynamics.

## System Architecture

![System Architecture](architecture/f16_ld_fcs_architecture.png)

## Continuous Integration

This project uses **GitHub Actions** for continuous integration.

On every push and pull request, the CI pipeline:
- installs dependencies
- runs unit tests
- executes a short simulation
- uploads generated plots as build artifacts

This ensures the control architecture, actuator model, and simulation remain functional across Python versions.

## Running the simulation

```bash
python scripts/run_demo.py



