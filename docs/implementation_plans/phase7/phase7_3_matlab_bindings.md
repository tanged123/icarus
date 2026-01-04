# MATLAB Bindings Implementation

**Status:** Proposed
**Phase:** 7.3 (Optional)
**Related:** [16_external_bindings.md](../../architecture/16_external_bindings.md) | [phase7_1_c_api.md](phase7_1_c_api.md)

---

## Overview

Provide MATLAB bindings for Icarus to enable:
- GNC engineer workflow compatibility
- Simulink S-Function integration
- MATLAB-based analysis and visualization

### Binding Strategy

| Approach | Pros | Cons | Choice |
|----------|------|------|--------|
| **loadlibrary (C API)** | Simple, uses existing C API | Manual type conversion | ✓ Primary |
| MEX | Native speed, rich API | Complex build, platform-specific | Secondary |
| Python bridge | Reuses Python bindings | Requires Python runtime | Fallback |

---

## MATLAB API Design

### Class Wrapper

```matlab
classdef IcarusSim < handle
    %ICARUSSIM Icarus 6DOF Simulator MATLAB Interface
    %
    % Example:
    %   sim = IcarusSim('scenarios/x15_mission.yaml');
    %   sim.stage();
    %   while sim.time < sim.endTime
    %       sim.step();
    %       disp(sim.get('Vehicle.Nav.altitude'));
    %   end

    properties (SetAccess = private)
        handle  % Opaque C pointer
    end

    properties (Dependent)
        time      % Current simulation time (MET)
        dt        % Configured timestep
        endTime   % Configured end time
        state     % State vector as column vector
        signals   % List of signal names
    end

    methods
        function obj = IcarusSim(configPath)
            %ICARUSSIM Create simulator from YAML configuration
            obj.loadLibrary();
            obj.handle = calllib('icarus', 'icarus_create', configPath);
            if isempty(obj.handle) || obj.handle.isNull
                error('IcarusSim:CreateFailed', ...
                    'Failed to create simulator: %s', obj.getLastError());
            end
        end

        function delete(obj)
            %DELETE Destructor - free C resources
            if ~isempty(obj.handle) && ~obj.handle.isNull
                calllib('icarus', 'icarus_destroy', obj.handle);
            end
        end

        function stage(obj, configPath)
            %STAGE Stage the simulation
            if nargin < 2
                err = calllib('icarus', 'icarus_stage', obj.handle);
            else
                err = calllib('icarus', 'icarus_stage_with_config', ...
                    obj.handle, configPath);
            end
            obj.checkError(err, 'stage');
        end

        function step(obj, dt)
            %STEP Execute one simulation step
            if nargin < 2
                dt = obj.dt;
            end
            err = calllib('icarus', 'icarus_step', obj.handle, dt);
            obj.checkError(err, 'step');
        end

        function reset(obj)
            %RESET Reset simulation to initial state
            err = calllib('icarus', 'icarus_reset', obj.handle);
            obj.checkError(err, 'reset');
        end

        function value = get(obj, signalName)
            %GET Get signal value by name
            valuePtr = libpointer('doublePtr', 0);
            err = calllib('icarus', 'icarus_get_signal', ...
                obj.handle, signalName, valuePtr);
            obj.checkError(err, 'get');
            value = valuePtr.Value;
        end

        function set(obj, signalName, value)
            %SET Set signal value by name
            err = calllib('icarus', 'icarus_set_signal', ...
                obj.handle, signalName, value);
            obj.checkError(err, 'set');
        end

        % ===== Dependent Properties =====

        function t = get.time(obj)
            t = calllib('icarus', 'icarus_get_time', obj.handle);
        end

        function dt = get.dt(obj)
            dt = calllib('icarus', 'icarus_get_dt', obj.handle);
        end

        function t = get.endTime(obj)
            t = calllib('icarus', 'icarus_get_end_time', obj.handle);
        end

        function s = get.state(obj)
            n = calllib('icarus', 'icarus_get_state_size', obj.handle);
            if n == 0
                s = [];
                return;
            end
            buffer = zeros(n, 1);
            bufferPtr = libpointer('doublePtr', buffer);
            sizePtr = libpointer('uint64Ptr', 0);
            err = calllib('icarus', 'icarus_get_state_vector', ...
                obj.handle, bufferPtr, n, sizePtr);
            obj.checkError(err, 'get.state');
            s = bufferPtr.Value;
        end

        function set.state(obj, value)
            n = numel(value);
            err = calllib('icarus', 'icarus_set_state_vector', ...
                obj.handle, value, n);
            obj.checkError(err, 'set.state');
        end

        function names = get.signals(obj)
            n = calllib('icarus', 'icarus_get_signal_count', obj.handle);
            % TODO: Implement signal name retrieval
            names = {};
        end

        % ===== Utility Methods =====

        function [times, data] = run(obj, duration, signalNames)
            %RUN Run simulation for duration, recording signals
            %
            %   [times, data] = sim.run(10.0, {'Alt', 'Vel'})
            %
            if nargin < 3
                signalNames = {};
            end

            nSteps = ceil(duration / obj.dt);
            times = zeros(nSteps, 1);
            if isempty(signalNames)
                data = [];
            else
                data = zeros(nSteps, numel(signalNames));
            end

            for i = 1:nSteps
                times(i) = obj.time;
                for j = 1:numel(signalNames)
                    data(i, j) = obj.get(signalNames{j});
                end
                obj.step();
            end
        end

        function tbl = toTable(obj, signalNames)
            %TOTABLE Export current values as MATLAB table
            if nargin < 2
                signalNames = obj.signals;
            end
            values = zeros(1, numel(signalNames));
            for i = 1:numel(signalNames)
                values(i) = obj.get(signalNames{i});
            end
            tbl = array2table(values, 'VariableNames', signalNames);
        end
    end

    methods (Access = private)
        function loadLibrary(~)
            %LOADLIBRARY Load the Icarus C library
            if ~libisloaded('icarus')
                % Determine library path
                if ispc
                    libPath = 'icarus.dll';
                    headerPath = 'icarus.h';
                elseif ismac
                    libPath = 'libicarus.dylib';
                    headerPath = 'icarus.h';
                else
                    libPath = 'libicarus.so';
                    headerPath = 'icarus.h';
                end
                loadlibrary(libPath, headerPath);
            end
        end

        function checkError(obj, errCode, operation)
            %CHECKERROR Check C API error code and throw if non-zero
            if errCode ~= 0
                errMsg = obj.getLastError();
                errName = calllib('icarus', 'icarus_error_name', errCode);
                error('IcarusSim:%s:Failed', operation, ...
                    '%s: %s', errName, errMsg);
            end
        end

        function msg = getLastError(obj)
            %GETLASTERROR Get last error message from C API
            if isempty(obj.handle) || obj.handle.isNull
                msg = 'No handle';
            else
                msg = calllib('icarus', 'icarus_get_last_error', obj.handle);
            end
        end
    end
end
```

---

## Simulink S-Function

### Level-2 MATLAB S-Function

```matlab
function icarus_sfun(block)
%ICARUS_SFUN Simulink S-Function wrapper for Icarus
%
% Inputs:
%   u(1:n_inputs) - Control inputs mapped to Icarus signals
%
% Outputs:
%   y(1:n_outputs) - Sensor outputs mapped from Icarus signals
%
% Parameters:
%   p1: Config file path
%   p2: Cell array of input signal names
%   p3: Cell array of output signal names
%   p4: Timestep (0 = inherit from Simulink)

setup(block);

function setup(block)
    % Register parameters
    block.NumDialogPrms = 4;  % config, inputs, outputs, dt

    % Get signal counts from parameters
    inputSignals = block.DialogPrm(2).Data;
    outputSignals = block.DialogPrm(3).Data;

    % Configure ports
    block.NumInputPorts = numel(inputSignals);
    block.NumOutputPorts = numel(outputSignals);

    for i = 1:block.NumInputPorts
        block.InputPort(i).Dimensions = 1;
        block.InputPort(i).DirectFeedthrough = false;
    end

    for i = 1:block.NumOutputPorts
        block.OutputPort(i).Dimensions = 1;
    end

    % Sample time
    dt = block.DialogPrm(4).Data;
    if dt == 0
        block.SampleTimes = [-1, 0];  % Inherited
    else
        block.SampleTimes = [dt, 0];  % Discrete
    end

    % Register methods
    block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
    block.RegBlockMethod('InitializeConditions', @InitializeConditions);
    block.RegBlockMethod('Outputs', @Outputs);
    block.RegBlockMethod('Update', @Update);
    block.RegBlockMethod('Terminate', @Terminate);

function DoPostPropSetup(block)
    % Store work data
    block.NumDworks = 1;
    block.Dwork(1).Name = 'SimHandle';
    block.Dwork(1).Dimensions = 1;
    block.Dwork(1).DatatypeID = 0;  % double (stores pointer as double)

function InitializeConditions(block)
    % Create Icarus simulator
    configPath = block.DialogPrm(1).Data;
    sim = IcarusSim(configPath);
    sim.stage();

    % Store handle (as pointer value cast to double)
    block.Dwork(1).Data = double(sim.handle);

    % Store sim object in UserData for cleanup
    set_param(block.BlockHandle, 'UserData', sim);

function Outputs(block)
    % Get stored simulator
    sim = get_param(block.BlockHandle, 'UserData');
    outputSignals = block.DialogPrm(3).Data;

    % Read outputs from Icarus
    for i = 1:numel(outputSignals)
        block.OutputPort(i).Data = sim.get(outputSignals{i});
    end

function Update(block)
    % Get stored simulator
    sim = get_param(block.BlockHandle, 'UserData');
    inputSignals = block.DialogPrm(2).Data;

    % Write inputs to Icarus
    for i = 1:numel(inputSignals)
        sim.set(inputSignals{i}, block.InputPort(i).Data);
    end

    % Step Icarus
    dt = block.DialogPrm(4).Data;
    if dt == 0
        dt = block.SampleTime;  % Use Simulink sample time
    end
    sim.step(dt);

function Terminate(block)
    % Cleanup
    sim = get_param(block.BlockHandle, 'UserData');
    if ~isempty(sim)
        delete(sim);
    end
    set_param(block.BlockHandle, 'UserData', []);
```

---

## Tasks

### 7.4.1 MATLAB Class Wrapper

- [ ] Create `interfaces/matlab/+icarus/IcarusSim.m`
- [ ] Implement constructor with `loadlibrary`
- [ ] Implement lifecycle methods (`stage`, `step`, `reset`)
- [ ] Implement signal access (`get`, `set`)
- [ ] Implement state vector access
- [ ] Add utility methods (`run`, `toTable`)

### 7.4.2 Library Loading

- [ ] Create `icarus.h` compatible with MATLAB's loadlibrary
- [ ] Handle platform-specific library paths
- [ ] Add library search path configuration
- [ ] Test on Windows, macOS, Linux

### 7.4.3 Simulink Integration

- [ ] Create S-Function wrapper `icarus_sfun.m`
- [ ] Create Simulink library block `icarus_lib.slx`
- [ ] Add block mask for parameter configuration
- [ ] Test with simple control loop

### 7.4.4 MEX Alternative (Optional)

- [ ] Create MEX implementation for performance-critical use
- [ ] Benchmark loadlibrary vs MEX overhead
- [ ] Provide build scripts for MEX compilation

### 7.4.5 Documentation & Examples

- [ ] Create `examples/matlab/basic_sim.m`
- [ ] Create `examples/matlab/simulink_demo.slx`
- [ ] Write user guide for MATLAB integration

---

## Build & Installation

### Directory Structure

```
interfaces/matlab/
├── +icarus/
│   ├── IcarusSim.m        # Main class
│   ├── loadIcarusLib.m    # Library loader
│   └── private/
│       └── getLibPath.m   # Platform-specific path
├── simulink/
│   ├── icarus_sfun.m      # S-Function
│   └── icarus_lib.slx     # Library block
└── examples/
    ├── basic_sim.m
    └── simulink_demo.slx
```

### Installation Instructions

```matlab
% Add Icarus to MATLAB path
addpath('/path/to/icarus/interfaces/matlab');

% Set library path (if not in system PATH)
setenv('ICARUS_LIB_PATH', '/path/to/libicarus.so');

% Test
sim = icarus.IcarusSim('test_config.yaml');
sim.stage();
sim.step();
disp(sim.time);
```

---

## Verification Plan

### MATLAB Tests

```matlab
% tests/matlab/test_icarus.m

function tests = test_icarus
    tests = functiontests(localfunctions);
end

function testCreateSimulator(testCase)
    sim = icarus.IcarusSim('test_config.yaml');
    verifyNotEmpty(testCase, sim);
end

function testStageAndStep(testCase)
    sim = icarus.IcarusSim('test_config.yaml');
    sim.stage();
    t0 = sim.time;
    sim.step();
    verifyGreaterThan(testCase, sim.time, t0);
end

function testSignalAccess(testCase)
    sim = icarus.IcarusSim('test_config.yaml');
    sim.stage();

    alt = sim.get('Vehicle.Nav.altitude');
    verifyClass(testCase, alt, 'double');

    sim.set('Vehicle.Nav.altitude', 1000);
    verifyEqual(testCase, sim.get('Vehicle.Nav.altitude'), 1000);
end

function testStateVector(testCase)
    sim = icarus.IcarusSim('test_config.yaml');
    sim.stage();

    state = sim.state;
    verifyClass(testCase, state, 'double');
    verifySize(testCase, state, [NaN, 1]);  % Column vector

    state(1) = state(1) + 1;
    sim.state = state;
end

function testRun(testCase)
    sim = icarus.IcarusSim('test_config.yaml');
    sim.stage();

    [times, data] = sim.run(1.0, {'Vehicle.Nav.altitude'});
    verifyGreaterThan(testCase, numel(times), 0);
    verifySize(testCase, data, [numel(times), 1]);
end
```

### Simulink Tests

```matlab
% tests/matlab/test_simulink.m

function test_sfun_basic
    % Create simple Simulink model with Icarus block
    model = 'test_icarus_sfun';
    new_system(model);
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [model '/Icarus']);

    % Configure and run
    set_param([model '/Icarus'], 'MATLABFcn', 'icarus_sfun');
    % ... add source and sink blocks ...

    sim(model, 'StopTime', '10');

    % Verify outputs
    % ...

    close_system(model, 0);
end
```

---

## Example Usage

### Basic MATLAB Script

```matlab
% Load and run simulation
sim = icarus.IcarusSim('scenarios/falling_ball.yaml');
sim.stage();

% Collect data
[times, data] = sim.run(10.0, {'Ball.Nav.altitude', 'Ball.Nav.velocity'});

% Plot
figure;
subplot(2,1,1);
plot(times, data(:,1));
xlabel('Time (s)'); ylabel('Altitude (m)');
title('Falling Ball Simulation');

subplot(2,1,2);
plot(times, data(:,2));
xlabel('Time (s)'); ylabel('Velocity (m/s)');
```

### Simulink Integration

```matlab
% Setup Simulink model parameters
configPath = 'scenarios/x15_control.yaml';
inputSignals = {'Control.Throttle', 'Control.Elevator'};
outputSignals = {'Nav.Altitude', 'Nav.Velocity', 'Nav.Pitch'};

% Open and configure model
open_system('x15_control_demo');
set_param('x15_control_demo/Icarus', ...
    'configPath', configPath, ...
    'inputSignals', inputSignals, ...
    'outputSignals', outputSignals);

% Run simulation
sim('x15_control_demo');
```

---

## Dependencies

- MATLAB R2019b or later (for loadlibrary improvements)
- Simulink (optional, for S-Function)
- C compiler (for MEX, optional)

### Platform Requirements

| Platform | Library File | Notes |
|----------|-------------|-------|
| Windows | `icarus.dll` | Must be on PATH or in MATLAB current dir |
| macOS | `libicarus.dylib` | Set `DYLD_LIBRARY_PATH` or use full path |
| Linux | `libicarus.so` | Set `LD_LIBRARY_PATH` or use full path |

---

## Limitations

1. **loadlibrary Overhead:** ~10μs per call due to FFI overhead
2. **No Symbolic Support:** MATLAB bindings are numeric-only
3. **Single-threaded:** MATLAB's loadlibrary is not thread-safe
4. **Memory:** Large simulations may need MATLAB memory management attention

---

## Alternative: Python Bridge

For users with both MATLAB and Python:

```matlab
% Use MATLAB's Python interface
py.importlib.import_module('icarus');
sim = py.icarus.Simulator('config.yaml');
sim.stage();
sim.step();
alt = double(sim.get('Vehicle.Nav.altitude'));
```

This avoids separate MATLAB bindings but requires Python setup.
