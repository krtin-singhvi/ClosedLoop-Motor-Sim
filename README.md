# PID Motor Controller

A small simulation of a DC motor controlled by a PID controller. The
simulation applies a voltage control signal to a motor model, records the
response, and can print the recorded samples to the console or write them to
a CSV file.

## Features

- Proportional, integral, and derivative controller components
- Output voltage clamping (with integral anti-windup)
- First-order motor speed simulation using explicit Euler integration
- Scheduled load torque and friction changes
- Console and CSV output implementations

## Project layout

```text
.
├── main.cpp
├── Makefile
├── components/
│   ├── components.*       # Clamp and Comparator
│   ├── controller.*       # PID + clamp controller composition and anti-windup
│   ├── motor.*            # Motor dynamics
│   └── pid_components.*   # P, I, and D components
├── simulation/
│   ├── device.*           # High-level simulation wrapper
│   ├── simulator.*        # Simulation loop and variations
│   ├── simulationresult.* # Recorded samples
│   └── variation.*        # Scheduled motor changes
└── output/
    ├── output.h           # Output interface (abstract)
    ├── consoleoutput.*    # console output
    └── csvoutput.*        # CSV output
```

## Build and run

Requirements:

- GNU Make
- A compiler with C++17 support, such as `g++`

Build the executable:

```bash
make
```

Run the example configured in `main.cpp`:

```bash
./main
```

Remove generated object files, dependency files, and the executable:

```bash
make clean
```

The current example creates a `Device` with these parameters, in order:

```cpp
Device device(
    Kp, Ki, Kd,
    maxVoltage, minVoltage,
    inertia, friction, motorConstant,
    targetSpeed, timeStep, totalTime
);
```

The values currently used by `main.cpp` are:

| Parameter | Value | Meaning |
|---|---:|---|
| `Kp` | `10.0` | Proportional gain |
| `Ki` | `4.0` | Integral gain |
| `Kd` | `0.01` | Derivative gain |
| `maxVoltage` | `200.0` | Upper Voltage limit to motor|
| `minVoltage` | `-200.0` | Lower Voltage limit to motor|
| `inertia` | `0.01` | Motor inertia `J` |
| `friction` | `0.1` | Motor friction coefficient `b` |
| `motorConstant` | `0.01` | Motor torque constant `K` |
| `targetSpeed` | `20` | Desired angular speed |
| `timeStep` | `0.01` | Simulation step in seconds |
| `totalTime` | `25.0` | Simulation duration in seconds |

`main.cpp` sends the resulting samples to `ConsoleOutput`. CSV output can be
added with:

```cpp
#include "csvoutput.h"

CSVOutput csv("results.csv");
csv.output(device.getResult());
```

## Simulation model

The motor state is its angular speed `w`. For each simulation step, the motor
uses:

```text
J * dw/dt = K * V - b * w - T
```

where:

- `J` is inertia
- `b` is the friction coefficient
- `K` is the motor constant
- `V` is the controller voltage input
- `T` is the external load torque

The implementation uses explicit Euler integration:

```text
w(next) = w + (K * V - b * w - T) * dt / J
```

At each step, the simulator:

1. Applies any variations whose scheduled time has been reached.
2. Reads the current motor speed.
3. Computes the error as `targetSpeed - actualSpeed`.
4. Sends the error to Controller to compute control voltage.
5. Records a `SimulationData` sample.
6. Advances the motor by one time step.

The recorded sample contains:

| Field | Description |
|---|---|
| `time` | Current simulation time |
| `targetSpeed` | Requested speed |
| `actualSpeed` | Motor speed before the current update |
| `error` | Target speed minus actual speed |
| `controlInput` | Clamped voltage applied for the next update |

## Controller

`Controller` combines the four component classes:

```text
P = Kp * error
I = Ki * integral(error * dt)
D = Kd * derivative(error)
Clamp
```

## Variations

Variations are added to a `Simulator` and are processed in time order:

```cpp
simulator.addVariation(Variation(
    5.0,   // scheduled time in seconds
    0.05,  // load torque change
    0.0    // friction change
));
```

Load and friction values are additive. The motor prevents its load torque and
friction coefficient from becoming negative.

## Output formats

`ConsoleOutput` prints a table containing time, target speed, actual speed,
error, and control input.

`CSVOutput` writes the following header and one row per sample:

```text
Time,TargetSpeed,ActualSpeed,Error,ControlInput
```
