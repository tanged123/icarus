# Future I/O Implementation Notes

## Vulcan Components to Reuse

When implementing Phase 2 (Recording/Playback), leverage these Vulcan components:

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

The Recorder will need to handle multi-type schemas:

```cpp
class Recorder {
    TelemetrySchema schema_;
    std::vector<Frame> frames_;
    
    void capture(double time) {
        Frame frame(schema_);
        frame.set_time(time);
        
        // Use SignalHandle to read from component-owned storage
        for (const auto& sig : schema_.signals()) {
            if (sig.type == SignalType::Double) {
                auto handle = registry_.resolve<double>(sig.name);
                frame.set(sig.name, *handle);
            }
            // Similar for Int32, Int64...
        }
    }
};
```

## See Also

- `references/vulcan/include/vulcan/io/Frame.hpp`
- `references/vulcan/include/vulcan/io/TelemetrySchema.hpp`
