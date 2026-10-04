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
│   ├── simulator.*        # Simulation loop and variations, owns motor, controller, and other components.
│   ├── simulationresult.* # Recorded samples
│   └──variation.*         # Scheduled motor changes
|   
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

The current example creates a `Simulator` with these parameters, in order:

```cpp
Simulator sim(
    Kp, Ki, Kd,
    maxVoltage, minVoltage,
    inertia, friction, motorConstant,
    targetSpeed, timeStep, totalTime
);
```

The values currently used by `main.cpp` are:

| Parameter | Value | Meaning |
|---|---:|---|
| `Kp` | `5.0` | Proportional gain |
| `Ki` | `2.0` | Integral gain |
| `Kd` | `0.01` | Derivative gain |  
| `maxVoltage` | `200.0` | Upper Voltage limit to motor|
| `minVoltage` | `-200.0` | Lower Voltage limit to motor|
| `inertia` | `0.01` | Motor inertia `J` |
| `friction` | `0.05` | Motor friction coefficient `b` |
| `motorConstant` | `0.1` | Motor torque constant `K` |
| `targetSpeed` | `20` | Desired angular speed |
| `timeStep` | `0.01` | Simulation step in seconds |
| `totalTime` | `15.0` | Simulation duration in seconds |

(The values can be tuned by the user too, but may cause oscillatory/ unstable behaviour, which is expected for bad values for PID and motor constant)

```cpp
#include "csvoutput.h"

CSVOutput csv("results.csv");
csv.output(sim.returnResult());
```
## Simulation Loop (each step)

1. **Apply variations** — if any `Variation` is scheduled at or before the current time, add its load and friction changes to the motor.
2. **Read speed** — get the motor's current angular speed.
3. **Compute error** — `error = targetSpeed - actualSpeed`.
4. **Compute control** — pass the error through the PID controller to get a clamped voltage.
5. **Record sample** — store `{time, targetSpeed, actualSpeed, error, controlInput}` in `SimulationResult`.
6. **Update motor** — apply the control voltage and advance the motor's speed by one time step.
7. **Advance time** — `currentTime += dt`.

### Step Count

```cpp
int totalSteps = (int)(totalTime / dt) + 1;
```

With `totalTime = 25.0` and `dt = 0.01`, this produces **2501 samples** (time 0.00 through 25.00).

---

### Anti-Windup

**The problem:** When the output is saturated (clamped), the integral term keeps accumulating error even though the system can't respond to a larger signal. When the error eventually reverses, the accumulated integral causes a delayed, excessive response — this is called **integral windup**.

**The solution in this codebase:** Before updating the integral, the controller checks:
1. Would the tentative output `(P + I_old + D)` be clamped?
2. Does the error have the same sign as the tentative output?

If **both** conditions are true, the integral is frozen — `I.compute(0, dt)` is called instead of `I.compute(error, dt)`. This prevents the integral from growing further in the direction that's already saturated.

See: [controller.cpp](components/controller.cpp)

---

## Output formats

`ConsoleOutput` prints a table containing time, target speed, actual speed,
error, and control input.

`CSVOutput` writes the following header and one row per sample:

```text
Time,TargetSpeed,ActualSpeed,Error,ControlInput
```
