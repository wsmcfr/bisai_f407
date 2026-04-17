# Quality Guidelines

> Code quality standards for application-layer firmware development.

---

## Overview

Application-layer quality in this project means keeping user logic deterministic, observable, and separated from generated platform code.
Most future feature work will live in `User/App/` and depend on `User/Driver/`.

---

## Forbidden Patterns

Avoid these patterns:

- putting app logic in `Core/Src/freertos.c` or `main.c` when it belongs in `User/App`
- directly editing vendor or middleware sources for application features
- sending blocking UART output from ISR/callback context
- mixing raw hardware access into protocol or business logic modules
- using unexplained magic numbers for timing, calibration, or protocol framing

---

## Required Patterns

Required patterns for new app code:

- one clear module owner for each feature
- Chinese comments for functions, important variables, and tricky branches
- descriptive APIs and configuration structs
- explicit task/context ownership for shared data
- serial/display output defined in one formatting path instead of scattered `printf` calls

---

## Testing Requirements

There is no automated UI or integration test suite.
Minimum validation for app-layer changes:

1. build succeeds
2. firmware boots to the scheduler
3. the target task executes on hardware
4. serial/display behavior matches the expected user-visible format
5. error paths are exercised at least once when practical

---

## Code Review Checklist

Reviewers should check:

- does the change live in `User/App` or `User/Driver` instead of generated files?
- is there a clean boundary between driver code and app logic?
- is shared state owned and synchronized correctly?
- are comments sufficient to explain timing, units, and hardware assumptions?
- is user-visible output stable and easy to parse/debug?
