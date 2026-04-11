# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

Motion planning framework for Tesseract. Provides a command language for describing robot motions, pluggable motion planners, time parameterization, and a task composer for orchestrating complex planning pipelines.

## Architecture

### Packages

- **command_language** — Instruction types using type-erasure Poly wrappers (see `POLY_TYPE_DESIGN.md`). Key types: `CompositeInstruction`, `MoveInstruction`, `JointWaypoint`/`CartesianWaypoint`/`StateWaypoint`
- **motion_planners** — `MotionPlanner` interface with implementations: simple (interpolation), OMPL, Descartes, TrajOpt, TrajOpt-IFOPT
- **time_parameterization** — Path to trajectory conversion: ISP, TOTG, Ruckig, KDL
- **task_composer** — DAG-based task execution. Nodes pass data via `TaskComposerDataStorage`. All nodes loadable as plugins via YAML

### Key design patterns

- **Profile system** — All planners configured via named profiles stored in `ProfileDictionary`, keyed by planner name + instruction name. Enables YAML-driven configuration.
- **Request/Response** — `PlannerRequest` (environment + instructions + profiles) in, `PlannerResponse` (solution trajectory + status) out.
