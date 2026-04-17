# Frontend Development Guidelines

> Best practices for application-layer and user-facing firmware code in this project.

---

## Overview

This repository has no web UI.
Within Trellis, the `frontend` layer is repurposed for the application-facing part of the firmware:

- RTOS tasks
- command parsing and formatting
- user-visible serial/display behavior
- application state machines
- orchestration of low-level drivers

---

## Guidelines Index

| Guide | Description | Status |
|-------|-------------|--------|
| [Directory Structure](./directory-structure.md) | Module organization and file layout | Done |
| [Component Guidelines](./component-guidelines.md) | App-module and task patterns | Done |
| [Hook Guidelines](./hook-guidelines.md) | Callback and reusable RTOS helper patterns | Done |
| [State Management](./state-management.md) | Task ownership and cross-context state rules | Done |
| [Quality Guidelines](./quality-guidelines.md) | Application-layer standards | Done |
| [Type Safety](./type-safety.md) | C type-safety and validation patterns | Done |

---

## Pre-Development Checklist

Read these files before changing application-layer firmware:

1. `directory-structure.md`
2. `component-guidelines.md`
3. `state-management.md`
4. `quality-guidelines.md`
5. `type-safety.md`
6. `hook-guidelines.md` when introducing callbacks, notifications, or reusable RTOS helpers

---

**Language**: All documentation should be written in **English**.
