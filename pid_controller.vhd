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
    -- Inputs.
    signal e_fixed:  sfixed(DATA_WIDTH - DATA_RADIX - 1 downto -DATA_RADIX);
    signal kp_fixed: sfixed(K_WIDTH - K_RADIX - 1 downto -K_RADIX);
    signal ki_fixed: sfixed(K_WIDTH - K_RADIX - 1 downto -K_RADIX);
    signal kd_fixed: sfixed(K_WIDTH - K_RADIX - 1 downto -K_RADIX);

    -- Stage 1: Integrate and differentiate.
    signal s1_tvalid: std_logic;
    signal s1_p_tdata: sfixed(DATA_WIDTH - DATA_RADIX - 1 downto -DATA_RADIX);
    signal s1_i_tdata: sfixed(INTEGRATOR_WIDTH - DATA_RADIX - 1 downto -DATA_RADIX);
    signal s1_d_tdata: sfixed(DATA_WIDTH - DATA_RADIX - 1 downto -DATA_RADIX);
    signal s1_eprev_tdata: sfixed(DATA_WIDTH - DATA_RADIX - 1 downto -DATA_RADIX);
    signal s1_eprev_tvalid: std_logic;

    -- Stage 2: Multiply PID coefficients.
    signal s2_tvalid: std_logic;
    signal s2_p_tdata: sfixed(
        sfixed_high(s1_p_tdata'high, s1_p_tdata'low, '*', kp_fixed'high, kp_fixed'low)
        downto
        sfixed_low(s1_p_tdata'high, s1_p_tdata'low, '*', kp_fixed'high, kp_fixed'low));
    signal s2_i_tdata: sfixed(
        sfixed_high(s1_i_tdata'high, s1_i_tdata'low, '*', ki_fixed'high, ki_fixed'low)
        downto
        sfixed_low(s1_i_tdata'high, s1_i_tdata'low, '*', ki_fixed'high, ki_fixed'low));
    signal s2_d_tdata: sfixed(
        sfixed_high(s1_d_tdata'high, s1_d_tdata'low, '*', kd_fixed'high, kd_fixed'low)
        downto
        sfixed_low(s1_d_tdata'high, s1_d_tdata'low, '*', kd_fixed'high, kd_fixed'low));

    -- Stage 3a: Sum P and I terms.
    --
    -- NOTE: P and D from Stage 2 are the same size (see assertions).
    signal s3a_tvalid: std_logic;
    signal s3a_pd_tdata: sfixed(s2_p_tdata'high downto s2_p_tdata'low);
    signal s3a_i_tdata: sfixed(s2_i_tdata'high downto s2_i_tdata'low);

    -- Stage 3b: Sum PI and D terms.
    --
    -- NOTE: I is guaranteed to be the same size or larger than PD from Stage 3a (see assertions).
    signal s3b_tvalid: std_logic;
    signal s3b_pid_tdata: sfixed(s3a_i_tdata'high downto s3a_i_tdata'low);

    --- A helper function for saturation.
    ---
    --- All arithmetic operations in the PID controller should saturate. This function simply
    --- wraps `ieee.fixed_pkg.resize` for `sfixed` to ensure consistent behavior for all resizes.
    ---
    --- # Arguments
    ---
    --- - `arg`: The `sfixed` to resize.
    --- - `size_res`: The new size is equal to the size of this value.
    function resize_saturate(
        arg: UNRESOLVED_sfixed;
        constant size_res: sfixed
    ) return UNRESOLVED_sfixed is
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
assert K_RADIX <= K_WIDTH
    report "K_RADIX must be less than or equal to K_WIDTH"
    severity failure;
assert INTEGRATOR_WIDTH >= DATA_WIDTH
    report "INTEGRATOR_WIDTH must be greater than or equal to K_WIDTH"
    severity failure;

-- Assert guarantees on intermediate signals.
assert s2_p_tdata'high = s2_d_tdata'high
    report "Stage 3a: P and D must be the same size"
    severity failure;
assert s2_p_tdata'low = s2_d_tdata'low
    report "Stage 3a: P and D must be the same size"
    severity failure;
assert s3a_i_tdata'high >= s3a_pd_tdata'high
    report "Stage 3b: I must be at least the size of PD"
    severity failure;
assert s3a_i_tdata'low >= s3a_pd_tdata'low
    report "Stage 3b: I must be at least the size of PD"
    severity failure;

-- Inputs.
e_fixed  <= to_sfixed(s_axis_e_tdata, e_fixed);
kp_fixed <= to_sfixed(kp, kp_fixed);
ki_fixed <= to_sfixed(ki, ki_fixed);
kd_fixed <= to_sfixed(kd, kd_fixed);

-- Stage 1: Integrate and differentiate.
stage1_p: process (aclk)
begin
    if rising_edge(aclk) then
        if aresetn = '0' then
            s1_tvalid       <= '0';
            s1_p_tdata      <= (others => '0');
            s1_i_tdata      <= (others => '0');
            s1_d_tdata      <= (others => '0');
            s1_eprev_tdata  <= (others => '0');
            s1_eprev_tvalid <= '0';
        else
            -- The idea is to delay the error signal by 1 so that the derivative can be computed
            -- from its current and previous value.
            if s_axis_e_tvalid = '1' then
                s1_eprev_tdata  <= e_fixed;
                s1_eprev_tvalid <= s_axis_e_tvalid;
            end if;
            if s_axis_e_tvalid = '1' and s1_eprev_tvalid = '1' then
                s1_tvalid  <= '1';
                s1_p_tdata <= e_fixed;
                s1_i_tdata <= resize_saturate(e_fixed + s1_i_tdata, s1_i_tdata);
                s1_d_tdata <= resize_saturate(e_fixed - s1_eprev_tdata, s1_d_tdata);
            else
                s1_tvalid <= '0';
                -- It is important to to set the other values to zero in here, because the
                -- accumulator for the integrator must be preserved.
            end if;
        end if;
    end if;
end process;

-- Stage 2: Multiply PID coefficients.
stage2_p: process (aclk)
begin
    if rising_edge(aclk) then
        if aresetn = '0' or s1_tvalid /= '1' then
            s2_tvalid  <= '0';
            s2_p_tdata <= (others => '0');
            s2_i_tdata <= (others => '0');
            s2_d_tdata <= (others => '0');
        else
            s2_tvalid  <= '1';
            s2_p_tdata <= resize_saturate(kp_fixed * s1_p_tdata, s2_p_tdata);
            s2_i_tdata <= resize_saturate(ki_fixed * s1_i_tdata, s2_i_tdata);
            s2_d_tdata <= resize_saturate(kd_fixed * s1_d_tdata, s2_d_tdata);
        end if;
    end if;
end process;

-- Stage 3a: Sum P and I terms.
stage3a_p: process (aclk)
begin
    if rising_edge(aclk) then
        if aresetn = '0' or s2_tvalid /= '1' then
            s3a_tvalid   <= '0';
            s3a_pd_tdata <= (others => '0');
            s3a_i_tdata  <= (others => '0');
        else
            s3a_tvalid   <= '1';
            s3a_pd_tdata <= resize_saturate(s2_p_tdata + s2_d_tdata, s3a_pd_tdata);
            s3a_i_tdata  <= s2_i_tdata;
        end if;
    end if;
end process;

-- Stage 3b: Sum PI and D terms.
stage3b_p: process (aclk)
begin
    if rising_edge(aclk) then
        if aresetn = '0' or s3a_tvalid /= '1' then
            s3b_tvalid    <= '0';
            s3b_pid_tdata <= (others => '0');
        else
            s3b_tvalid    <= '1';
            s3b_pid_tdata <= resize_saturate(s3a_pd_tdata + s3a_i_tdata, s3b_pid_tdata);
        end if;
    end if;
end process;

-- Outputs.
m_axis_u_tdata  <= to_slv(s3b_pid_tdata(DATA_WIDTH - DATA_RADIX - 1 downto -DATA_RADIX));
m_axis_u_tvalid <= s3b_tvalid;

end behavioral;
