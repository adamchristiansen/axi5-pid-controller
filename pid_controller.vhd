-- SPDX-License-Identifier: MIT
-- Copyright (c) 2025 Adam Christiansen

library ieee;
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
--- - `DATA_RADIX: Radix position in bits of the fixed-point input and output AXI4/5-Streams.
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
    K_RADIX: natural := 0;
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
    -- Stage 1: Integrate and differentiate.
    constant S1_DATA_RADIX: natural := DATA_RADIX;
    signal s1_tvalid: std_logic;
    signal s1_p_tdata: signed(DATA_WIDTH - 1 downto 0);
    signal s1_i_tdata: signed(INTEGRATOR_WIDTH - 1 downto 0);
    signal s1_d_tdata: signed(DATA_WIDTH - 1 downto 0);
    signal s1_eprev_tdata: signed(DATA_WIDTH - 1 downto 0);
    signal s1_eprev_tvalid: std_logic;

    -- Stage 2: Multiply PID coefficients.
    constant S2_DATA_RADIX: natural := maximum(DATA_RADIX, K_RADIX);
    signal s2_tvalid: std_logic;
    signal s2_p_tdata: signed(s1_p_tdata'length + K_WIDTH - 1 downto 0);
    signal s2_i_tdata: signed(s1_i_tdata'length + K_WIDTH - 1 downto 0);
    signal s2_d_tdata: signed(s1_d_tdata'length + K_WIDTH - 1 downto 0);

    -- Stage 3a: Sum P and I terms.
    constant S3A_DATA_RADIX: natural := S2_DATA_RADIX;
    signal s3a_tvalid: std_logic;
    signal s3a_pd_tdata: signed(maximum(s2_p_tdata'length, s2_d_tdata'length) - 1 downto 0);
    signal s3a_i_tdata: signed(s2_i_tdata'length - 1 downto 0);

    -- Stage 3b: Sum PI and D terms.
    constant S3B_DATA_RADIX: natural := S3A_DATA_RADIX;
    signal s3b_tvalid: std_logic;
    signal s3b_pid_tdata: signed(maximum(s3a_pd_tdata'length, s3a_i_tdata'length) - 1 downto 0);

    --- Perform an arithmetic right shift.
    ---
    --- # Arguments
    ---
    --- - `a`: The value to shift.
    --- - `count`: The number of bits to shift. it is a right shift when positive and left shift
    ---   when negative.
    function rshift(a: signed; count: integer) return signed is
    begin
        if count > 0 then
            return shift_right(a, count);
        else
            return shift_left(a, -count);
        end if;
    end function;

    --- Change the radix of a fixed-point number.
    ---
    --- # Arguments
    --- - `a`: Original.
    --- - `a_radix`: Radix of original.
    --- - `r`: Return value storage.
    --- - `r_radix`: Radix of return value.
    ---
    --- The return value itself is passed as an argument so that its size can be determined from its
    --- attributes.
    function fixed_radix(
        a: signed; a_radix: natural;
        r: signed; r_radix: natural
    ) return signed is
        constant SHIFT_COUNT: integer := integer(a_radix) - integer(r_radix);
    begin
        return resize(rshift(a, SHIFT_COUNT), r'length);
    end function;

    --- Fixed-point addition where numbers are interpreted as signed.
    ---
    --- # Arguments
    --- - `a`: Left hand side.
    --- - `a_radix`: Radix of left hand side.
    --- - `b`: Right hand side.
    --- - `b_radix`: Radix of right hand side.
    --- - `r`: Return value storage.
    --- - `r_radix`: Radix of return value.
    ---
    --- The return value itself is passed as an argument so that its size can be determined from its
    --- attributes.
    function fixed_add(
        a: signed; a_radix: natural;
        b: signed; b_radix: natural;
        r: signed; r_radix: natural
    ) return signed is
        constant SHIFT_COUNT: integer := integer(maximum(a_radix, b_radix)) - integer(r_radix);
    begin
        return resize(rshift(a + b, SHIFT_COUNT), r'length);
    end function;

    --- Fixed-point subtraction where numbers are interpreted as signed.
    ---
    --- # Arguments
    --- - `a`: Left hand side.
    --- - `a_radix`: Radix of left hand side.
    --- - `b`: Right hand side.
    --- - `b_radix`: Radix of right hand side.
    --- - `r`: Return value storage.
    --- - `r_radix`: Radix of return value.
    ---
    --- The return value itself is passed as an argument so that its size can be determined from its
    --- attributes.
    function fixed_sub(
        a: signed; a_radix: natural;
        b: signed; b_radix: natural;
        r: signed; r_radix: natural
    ) return signed is
    begin
        return fixed_add(a, a_radix, -b, b_radix, r, r_radix);
    end function;

    --- Fixed-point multiplication where numbers are interpreted as signed.
    ---
    --- # Arguments
    --- - `a`: Left hand side.
    --- - `a_radix`: Radix of left hand side.
    --- - `b`: Right hand side.
    --- - `b_radix`: Radix of right hand side.
    --- - `r`: Return value storage.
    --- - `r_radix`: Radix of return value.
    ---
    --- The return value itself is passed as an argument so that its size can be determined from its
    --- attributes.
    function fixed_mul(
        a: signed; a_radix: natural;
        b: signed; b_radix: natural;
        r: signed; r_radix: natural
    ) return signed is
        constant SHIFT_COUNT: integer := integer(maximum(a_radix, b_radix)) - integer(r_radix);
    begin
        return resize(rshift(a * b, SHIFT_COUNT), r'length);
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
                s1_eprev_tdata  <= signed(s_axis_e_tdata);
                s1_eprev_tvalid <= s_axis_e_tvalid;
            end if;
            if s_axis_e_tvalid = '1' and s1_eprev_tvalid = '1' then
                s1_tvalid  <= '1';
                s1_p_tdata <= signed(s_axis_e_tdata);
                s1_i_tdata <= fixed_add(
                    signed(s_axis_e_tdata), DATA_RADIX,
                    s1_i_tdata, S1_DATA_RADIX,
                    s1_i_tdata, S1_DATA_RADIX);
                s1_d_tdata <= fixed_sub(
                    signed(s_axis_e_tdata), DATA_RADIX,
                    s1_eprev_tdata, DATA_RADIX,
                    s1_d_tdata, S1_DATA_RADIX);
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
            s2_p_tdata <= fixed_mul(
                signed(kp), K_RADIX,
                s1_p_tdata, S1_DATA_RADIX,
                s2_p_tdata, S2_DATA_RADIX);
            s2_i_tdata <= fixed_mul(
                signed(ki), K_RADIX,
                s1_i_tdata, S1_DATA_RADIX,
                s2_i_tdata, S2_DATA_RADIX);
            s2_d_tdata <= fixed_mul(
                signed(kd), K_RADIX,
                s1_d_tdata, S1_DATA_RADIX,
                s2_d_tdata, S2_DATA_RADIX);
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
            s3a_pd_tdata <= fixed_add(
                s2_p_tdata, S2_DATA_RADIX,
                s2_d_tdata, S2_DATA_RADIX,
                s3a_pd_tdata, S3A_DATA_RADIX);
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
            s3b_pid_tdata <= fixed_add(
                s3a_pd_tdata, S3A_DATA_RADIX,
                s3a_i_tdata, S3A_DATA_RADIX,
                s3b_pid_tdata, S3B_DATA_RADIX);
        end if;
    end if;
end process;

-- Outputs.
m_axis_u_tdata  <= std_logic_vector(fixed_radix(
    s3b_pid_tdata, S3B_DATA_RADIX,
    signed(m_axis_u_tdata), DATA_RADIX));
m_axis_u_tvalid <= s3b_tvalid;

end behavioral;
