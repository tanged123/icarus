# Future I/O Implementation Notes

## Vulcan Components to Reuse

When implementing Phase 6.2 (Recording/Playback), leverage these Vulcan components:

### `vulcan::io::TelemetrySchema`

- Chainable schema builder: `.add_double()`, `.add_vec3()`, `.add_quat()`
- 8-byte aligned offsets for wire format
- Dynamic/Static signal separation
- JSON serialization for schema exchange

### `vulcan::io::Frame`

- Timestamped data buffer with typed setters/getters
- Raw buffer access for HDF5/network serialization
- Vector convenience methods (set/get Vec3, Quat)

## Integration Approach

```cpp
// IcarusRecorder could pull signals from SignalRegistry into Frame
class Recorder {
    TelemetrySchema schema_;
    std::vector<Frame> frames_;
    
    void capture(const SignalRegistry<double>& registry, double time) {
        Frame frame(schema_);
        frame.set_time(time);
        // Copy from registry to frame using signal handles
    }
};
```

## See Also

- `references/vulcan/include/vulcan/io/Frame.hpp`
- `references/vulcan/include/vulcan/io/TelemetrySchema.hpp`
- `references/vulcan/include/vulcan/io/HDF5Writer.hpp`
