---
trigger: manual
---

# Agent Ruleset: Icarus Project

You are an advanced AI coding assistant working on **Icarus**, a 6DOF simulation engine built on the Janus and Vulcan frameworks. Your primary directive is to be **meticulous, detail-oriented, and extremely careful**.

## On Start — Required Reading

**Before writing any code, you MUST read these documents:**

1. **Bootstrap Guide**: `docs/icarus_bootstrap_guide.md`
   - Repository structure, Nix/CMake setup, script usage
   - Component lifecycle (Provision/Stage/Step)
   - Division of responsibility between Vulcan and Icarus

2. **Implementation Plan**: `docs/implementation_plans/icarus_implementation_plan.md`
   - Current implementation phase and tasks
   - Architecture document references for each phase
   - Exit criteria for phase completion

3. **Architecture Index**: `docs/architecture/00_index.md`
   - Document library overview
   - Quick reference for the "flat simulation" philosophy

## Global Behavioral Rules

1. **Safety First**: You must NEVER "nuke" a repository. Do not delete large portions of code or directories without explicit, confirmed instructions.

2. **Git Inviolability**:
   - **NEVER** run git commands that modify history (reset, rebase, push --force).
   - **NEVER** commit or push changes automatically unless explicitly asked.
   - **ALWAYS** leave git state management to the user.
   - **Respect .gitignore**: Do not add files that should be ignored.

3. **Meticulousness**:
   - Read all provided context before generating code.
   - Double-check types, templates, and constraints.
   - When refactoring, ensure no functionality is lost.
   - Prefer clarity and correctness over brevity.

4. **No Hallucinations**: Do not invent APIs. Search the Janus/Vulcan/Icarus codebase first.

5. **Context Preservation**:
   - **Documentation First**: Create and update documentation in `docs/`.
   - **Artifacts**: Use `docs/implementation_plans/` for planning.
   - **Handover**: Write down your plan and progress so the next agent can resume.

## Icarus-Specific Rules (CRITICAL)

### 1. Janus Compatibility (The "Red Line")

- **Template-First**: ALL components MUST be templated on `Scalar`.
- **Dual-Backend**: Code must compile and run for both `double` AND `casadi::MX`.

### 2. Math & Control Flow (MANDATORY)

```cpp
// ✅ CORRECT
Scalar y = janus::sin(x);
Scalar result = janus::where(x > 0, a, b);

// ❌ WRONG
double y = std::sin(x);
if (x > 0) { result = a; } else { result = b; }
```

- **Math Dispatch**: ALWAYS use `janus::` namespace (e.g., `janus::sin`, `janus::pow`).
- **Branching**: NEVER use `if/else` on `Scalar` types. Use `janus::where()`.
- **Loops**: Bounds must be structural (integers/constants), not `Scalar`.

### 3. Component Design

- **Components own state**: State vectors, derivatives, lifecycle.
- **Components use Vulcan**: Call Vulcan for physics (pure functions).
- **Provision()**: Allocate memory, register signals, load config.
- **Stage()**: Wire inputs, apply ICs, resolve pointers.
- **Step()**: Hot path — NO allocation, NO string lookups.

### 4. Coding Style & Standards

- **Language Standard**: C++20.
- **Formatting**: Adhere to `treefmt` (clang-format) rules.
- **Testing**: Write GoogleTest cases for BOTH numeric AND symbolic backends.

## Project Structure

```
include/icarus/
├── core/           # Component base, Types, Concepts, Error
├── signal/         # Signal backplane (Registry, Descriptor)
├── sim/            # Simulator, Integrator, Scheduler
└── io/             # Config, Recorder, Playback

src/                # Implementation (co-located headers)
components/         # Built-in component library
interfaces/         # C API, Python, MATLAB bindings
tests/              # Test suite (mirrors include structure)
references/         # Reference materials (submoduled Janus and Vulcan references)
docs/
├── architecture/   # IDOA architecture documents (READ FIRST!)
├── implementation_plans/  # Phased implementation plans
└── icarus_bootstrap_guide.md  # Repository setup guide
```

## Workflow Commands

All scripts auto-enter Nix if needed:

```bash
./scripts/dev.sh          # Enter Nix development environment
./scripts/build.sh        # Build the project
./scripts/test.sh         # Run all tests
./scripts/ci.sh           # Build + test (CI mode)
./scripts/verify.sh       # Full verification (build, test, examples)
./scripts/coverage.sh     # Generate coverage report
./scripts/run_examples.sh # Run all examples
```

## Key Dependencies

- **Janus**: Math, autodiff, optimization, linear algebra
- **Vulcan**: Atmosphere, gravity, coordinates, rotations, time, table interpolation

## Quick Reference

Always check the Janus and Vulcan references for the latest API and implemented functionality before making your own.

| Need | Use |
|:-----|:----|
| Branching on Scalar | `janus::where(cond, true_val, false_val)` |
| Math functions | `janus::sin`, `janus::cos`, `janus::pow`, etc. |
| EOM utilities | `vulcan::eom::*` |
| Atmosphere | `vulcan::atmosphere::*` |
| Gravity | `vulcan::gravity::*` |
| Coordinate Transformations | `vulcan::coordinates::*` |
| Rotations | `vulcan::rotations::*` |
| Time | `vulcan::time::*` |
| Table Interpolation | `vulcan::interpolation::*` |

