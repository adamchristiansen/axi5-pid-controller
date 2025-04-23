# AXI5-Stream PID Controller

An [AXI4/5-Stream][axi5] [PID controller][pid] implemented in VHDL-2008.

## How to use

Simply drop [`pid_controller.vhd`](./pid_controller.vhd) into your project and instantiate the PID
controller component.

```vhdl
my_pid_controller: entity work.pid_controller
generic map (
    DATA_WIDTH => ...,      -- natural (default: 16).
    DATA_RADIX => ...,      -- natural (default: 0).
    K_WIDTH => ...,         -- natural (default: `DATA_WIDTH`).
    K_RADIX => ...,         -- natural (default: `DATA_RADIX`).
    INTEGRATOR_WIDTH => ... -- natural: (default `DATA_WIDTH`).
)
port map (
    -- Interface global signals.
    aclk => ...,
    aresetn => ...,
    -- Error signal input (AXI4/5-Stream).
    s_axis_e_tdata => ...,
    s_axis_e_tvalid => ...,
    -- Control variable output (AXI4/5-Stream).
    m_axis_u_tdata => ...,
    m_axis_u_tvalid => ...,
    -- PID coefficients.
    kp => ...,
    ki => ...,
    kd => ...
);
```

## Documentation

> [!TIP]
> This information can also be found in [`pid_controller.vhd`](./pid_controller.vhd).

This controller implements the following system, where `e(t)` is the input error signal and
`u(t)` is the computed control variable that feeds back to the system. The PID coefficients
`kp`, `ki`, and `kd` define the loop.

```
        +---> kp * e(t) --------+
        |                       |
e(t) ---+---> ki * ∫e(t) dt --> Σ ---> u(t)
        |                       |
        +---> kd * e(t)/dt -----+
```

The PID controller is used in the following way, where `r(t)` is the setpoint and `y(t)` is
the process variable. The error signal fed to the PID controller is generally computed in the
same way from the setpoint and process variable (often as the difference).

 ```
             e(t)                u(t)
 s(t) ---> Σ ---> PID Controller ---> Process --+-> y(t)
           |                                    |
           +------------------------------------+
 ```

The PID controller accepts `e(t)` rather than `s(t)`, so it is possible to generate error
signals using more sophisticated approaches.

### Generics

- `DATA_WIDTH`: Width in bits of the input and output AXI4/5-Streams.
- `DATA_RADIX`: Radix position in bits of the fixed-point input and output AXI4/5-Streams.
- `K_WIDTH`: Width in bits of the input `kp`, `ki`, and `kd` coefficients.
- `K_RADIX`: Radix position in bits of the input `kp`, `ki`, and `kd` coefficients.
- `INTEGRATOR_WIDTH`: Width in bits of the internal integrator. Can be used to set the
  integrator width larger than `DATA_WIDTH` to prevent early saturation of the integrator.

### Ports

- `aclk`: Global AXI4/5-Stream clock.
- `aresetn`: Global active-low AXI4/5-Stream reset.
- `s_axis_e_*`: AXI4/5-Stream for the error signal input as a signed fixed-point integer.
- `m_axis_u_*`: AXI4/5-Stream for the control variable output as a signed fixed-point integer.
- `kp`: Proportional coefficient as a signed fixed-point integer.
- `ki`: Integral coefficient as a signed fixed-point integer.
- `kd`: Derivative coefficient as a signed fixed-point integer.

## Testing

The makefile has the following recipes:

- `make`: Alias for `make build`.
- `make build`: Analyze and elaborate the source and test files.
- `make test`: Run the testbench.
- `make wave`: Display testbench waveforms.
- `make clean`: Remove all build artifacts.

## Requirements

- [GHDL][ghdl] is used for building and testing.
- [GTKWave][gtkwave] is used for viewing waveforms.

## License

This project is distributed under the terms of the MIT License. The license text can be found in
the [`LICENSE`](./LICENSE) file of this repository.

[axi5]: https://developer.arm.com/documentation/ihi0022/latest
[ghdl]: https://ghdl.github.io/ghdl/
[gtkwave]: https://gtkwave.sourceforge.net/
[pid]: https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller
