
  High-Impact Consolidation Opportunities

  | Issue                | Files                    | Problem                                                  | Lines Affected |
  |----------------------|--------------------------|----------------------------------------------------------|----------------|
  | Exception Zoo        | core/Error.hpp           | 15+ exception types that differ only by prefix string    | ~200 lines     |
  | Handle Duplication   | signal/VecHandle.hpp     | Vec3Handle, QuatHandle, Mat3Handle are copy-paste        | ~120 lines     |
  | Export Duplication   | io/DataDictionary.hpp    | ToYAML() and ToJSON() have identical logic               | ~80 lines      |
  | Template Boilerplate | core/ComponentConfig.hpp | 8 identical template specializations                     | ~150 lines     |
  | FullName Duplication | 4+ files                 | Same entity.empty() ? name : entity + "." + name pattern | ~20 lines      |

  Medium-Impact Issues

  | Issue          | Files                              | Problem                                     |
  |----------------|------------------------------------|---------------------------------------------|
  | Dual API       | signal/Registry.hpp                | Legacy + Modern APIs both maintained        |
  | Stale TODOs    | Simulator.hpp, SimulatorConfig.hpp | 6+ TODO comments for unimplemented features |
  | Logging Sprawl | LogService, MissionLogger, Console | 3 separate logging systems                  |
  | Phase Naming   | Types.hpp, MissionLogger.hpp       | Phase vs SimPhase don't align               |

  Small Files to Merge

- Playback.hpp (28 lines) + Recorder.hpp (33 lines) → Single recording module
- IntegratorTypes.hpp + Integrator.hpp → Combined integrator module
- InputHandle.hpp + VecHandle.hpp → All handles together

  ---
  Which would you like me to tackle first?

  1. Flatten Error.hpp - Reduce 15+ exceptions to 4-5 categories
  2. Consolidate VecHandle.hpp - Create generic ComponentHandle<N> template
  3. Clean up ComponentConfig.hpp - Unify template specializations with type traits
  4. Remove stale TODOs - Clean commented-out/unimplemented code
