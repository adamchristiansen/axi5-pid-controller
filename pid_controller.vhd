-- SPDX-License-Identifier: MIT
-- Copyright (c) 2025 Adam Christiansen

library ieee;
use ieee.fixed_float_types.all;
use ieee.fixed_pkg.all;
use ieee.numeric_std.all;
use ieee.std_logic_1164.all;

--- A PID controller.
---
--- This controller implements the following system, where `e(t)` is the input error signal and
--- `u(t)` is the computed control variable that feeds back to the system. The PID coefficients
--- `kp`, `ki`, and `kd` define the loop.
---
--- ```
---         +---> kp * e(t) --------+
---         |                       |
--- e(t) ---+---> ki * ∫e(t) dt --> Σ ---> u(t)
---         |                       |
---         +---> kd * e(t)/dt -----+
--- ```
---
--- The PID controller is used in the following way, where `r(t)` is the setpoint and `y(t)` is
--- the process variable. The error signal fed to the PID controller is generally computed in the
--- same way from the setpoint and process variable (often as the difference).
---
---  ```
---              e(t)                u(t)
---  s(t) ---> Σ ---> PID Controller ---> Process --+-> y(t)
---            |                                    |
---            +------------------------------------+
---  ```
---
--- The PID controller accepts `e(t)` rather than `s(t)`, so it is possible to generate error
--- signals using more sophisticated approaches.
---
--- # Generics
---
--- - `DATA_WIDTH`: Width in bits of the input and output AXI4/5-Streams.
--- - `DATA_RADIX`: Radix position in bits of the fixed-point input and output AXI4/5-Streams.
--- - `K_WIDTH`: Width in bits of the input `kp`, `ki`, and `kd` coefficients.
--- - `K_RADIX`: Radix position in bits of the input `kp`, `ki`, and `kd` coefficients.
--- - `INTEGRATOR_WIDTH`: Width in bits of the internal integrator. Can be used to set the
---   integrator width larger than `DATA_WIDTH` to prevent early saturation of the integrator.
---
--- # Ports
---
--- - `aclk`: Global AXI4/5-Stream clock.
--- - `aresetn`: Global active-low AXI4/5-Stream reset.
--- - `s_axis_e_*`: AXI4/5-Stream for the error signal input as a signed fixed-point integer.
--- - `m_axis_u_*`: AXI4/5-Stream for the control variable output as a signed fixed-point integer.
--- - `kp`: Proportional coefficient as a signed fixed-point integer.
--- - `ki`: Integral coefficient as a signed fixed-point integer.
--- - `kd`: Derivative coefficient as a signed fixed-point integer.
entity pid_controller is
generic (
    DATA_WIDTH: natural := 16;
    DATA_RADIX: natural := 0;
    K_WIDTH: natural := DATA_WIDTH;
    K_RADIX: natural := DATA_RADIX;
    INTEGRATOR_WIDTH: natural := DATA_WIDTH
);
port (
    aclk: in std_logic;
    aresetn: in std_logic;
    -- Input signal.
    s_axis_e_tdata: in std_logic_vector(DATA_WIDTH - 1 downto 0);
    s_axis_e_tvalid: in std_logic;
    -- Output signal.
    m_axis_u_tdata: out std_logic_vector(DATA_WIDTH - 1 downto 0);
    m_axis_u_tvalid: out std_logic;
    -- PID coefficients.
    kp: in std_logic_vector(K_WIDTH - 1 downto 0);
    ki: in std_logic_vector(K_WIDTH - 1 downto 0);
    kd: in std_logic_vector(K_WIDTH - 1 downto 0)
);
end pid_controller;

architecture behavioral of pid_controller is
    --- Stage 0: Capture one point.
    signal s0_seen: boolean;
    signal s0_valid: boolean;
    signal s0_e: sfixed(DATA_WIDTH - DATA_RADIX - 1 downto -DATA_RADIX);
    signal s0_e_prev: sfixed(DATA_WIDTH - DATA_RADIX - 1 downto -DATA_RADIX);
    signal s0_kp: sfixed(K_WIDTH - K_RADIX - 1 downto -K_RADIX);
    signal s0_ki: sfixed(K_WIDTH - K_RADIX - 1 downto -K_RADIX);
    signal s0_kd: sfixed(K_WIDTH - K_RADIX - 1 downto -K_RADIX);
    -- Stage 1: Integrate and differentiate.
    signal s1_valid: boolean;
    signal s1_p: sfixed(DATA_WIDTH - DATA_RADIX - 1 downto -DATA_RADIX);
    signal s1_i: sfixed(INTEGRATOR_WIDTH - DATA_RADIX - 1 downto -DATA_RADIX);
    signal s1_d: sfixed(DATA_WIDTH - DATA_RADIX - 1 downto -DATA_RADIX);
    -- Stage 2: Multiply PID coefficients.
    signal s2_valid: boolean;
    signal s2_p: sfixed(DATA_WIDTH - DATA_RADIX - 1 downto -DATA_RADIX);
    signal s2_i: sfixed(DATA_WIDTH - DATA_RADIX - 1 downto -DATA_RADIX);
    signal s2_d: sfixed(DATA_WIDTH - DATA_RADIX - 1 downto -DATA_RADIX);
    -- Stage 3: Sum P and I terms.
    signal s3_valid: boolean;
    signal s3_pi: sfixed(DATA_WIDTH - DATA_RADIX - 1 downto -DATA_RADIX);
    signal s3_d: sfixed(DATA_WIDTH - DATA_RADIX - 1 downto -DATA_RADIX);

    --- A helper function that resizes `arg` to match `size_res` with consistent rounding and
    --- overflow semantics.
    ---
    --- All arithmetic operations in the PID controller should saturate. This function simply wraps
    --- `ieee.fixed_pkg.resize` for `sfixed` to ensure consistent behavior for all resizes.
    ---
    --- # Arguments
    ---
    --- - `arg`: The `sfixed` to resize.
    --- - `size_res`: The new size is equal to the size of this value.
    function resize_consistent(
        arg: sfixed;
        constant size_res: sfixed
    ) return sfixed is
    begin
        return resize(
            arg,
            left_index => size_res'high,
            right_index => size_res'low,
            overflow_style => fixed_saturate,
            round_style => fixed_truncate);
    end function;
begin

-- Assert that generics are valid.
assert DATA_WIDTH mod 8 = 0
    report "DATA_WIDTH must be a multiple of 8"
    severity failure;
assert DATA_RADIX <= DATA_WIDTH
    report "DATA_RADIX must be less than or equal to DATA_WIDTH"
    severity failure;
assert K_WIDTH > 0
    report "K_WIDTH must be greater than 0"
    severity failure;
assert K_RADIX <= K_WIDTH
    report "K_RADIX must be less than or equal to K_WIDTH"
    severity failure;
assert INTEGRATOR_WIDTH >= DATA_WIDTH
    report "INTEGRATOR_WIDTH must be greater than or equal to K_WIDTH"
    severity failure;

pid_p: process (aclk)
begin
    if rising_edge(aclk) then
        if aresetn = '0' then
            -- Stage 0: Setup.
            s0_seen   <= false;
            s0_valid  <= false;
            s0_e      <= (others => '0');
            s0_e_prev <= (others => '0');
            s0_kp     <= (others => '0');
            s0_ki     <= (others => '0');
            s0_kd     <= (others => '0');
            -- Stage 1: Integrate and differentiate.
            s1_valid <= false;
            s1_p     <= (others => '0');
            s1_i     <= (others => '0');
            s1_d     <= (others => '0');
            -- Stage 2: Multiply PID coefficients.
            s2_valid <= false;
            s2_p     <= (others => '0');
            s2_i     <= (others => '0');
            s2_d     <= (others => '0');
            -- Stage 3: Sum P and I terms.
            s3_valid <= false;
            s3_pi    <= (others => '0');
            s3_d     <= (others => '0');
            -- Outputs.
            m_axis_u_tdata  <= (others => '0');
            m_axis_u_tvalid <= '0';
        else
            -- Stage 0: Setup.
            if s_axis_e_tvalid = '1' then
                s0_seen   <= true;
                s0_e      <= to_sfixed(s_axis_e_tdata, s0_e);
                s0_e_prev <= s0_e when s0_seen else to_sfixed(s_axis_e_tdata, s0_e);
            end if;
            s0_valid <= s_axis_e_tvalid = '1';
            s0_kp    <= to_sfixed(kp, s0_kp);
            s0_ki    <= to_sfixed(ki, s0_ki);
            s0_kd    <= to_sfixed(kd, s0_kd);
            -- Stage 1: Integrate and differentiate.
            s1_valid <= s0_valid;
            s1_p     <= s0_e;
            s1_i     <= resize_consistent(s1_i + s0_e, s1_i);
            s1_d     <= resize_consistent(s0_e - s0_e_prev, s1_d);
            -- Stage 2: Multiply PID coefficients.
            s2_valid <= s1_valid;
            s2_p     <= resize_consistent(s0_kp * s1_p, s2_p);
            s2_i     <= resize_consistent(s0_ki * s1_i, s2_i);
            s2_d     <= resize_consistent(s0_kd * s1_d, s2_d);
            -- Stage 3: Sum P and I terms.
            s3_valid <= s2_valid;
            s3_pi    <= resize_consistent(s2_p + s2_i, s3_pi);
            s3_d     <= s2_d;
            -- Stage 4: Sum PI and D terms and set outputs.
            if s3_valid then
                m_axis_u_tdata <= to_slv(resize_consistent(s3_pi + s3_d, s3_pi));
                m_axis_u_tvalid <= '1';
            else
                m_axis_u_tdata  <= (others => '0');
                m_axis_u_tvalid <= '0';
            end if;
        end if;
    end if;
end process;

end behavioral;
