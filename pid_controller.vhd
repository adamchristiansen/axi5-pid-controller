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
--- - `K_RADIX: Radix position in bits of the input `kp`, `ki`, and `kd` coefficients.
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
    K_WIDTH: natural := 16;
    K_RADIX: natural := 0
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
    constant S1_P_DATA_WIDTH: natural := DATA_WIDTH;
    constant S1_I_DATA_WIDTH: natural := DATA_WIDTH;
    constant S1_D_DATA_WIDTH: natural := DATA_WIDTH;
    constant S1_DATA_RADIX: natural := DATA_RADIX;
    signal s1_tvalid: std_logic;
    signal s1_p_tdata: std_logic_vector(S1_P_DATA_WIDTH - 1 downto 0);
    signal s1_i_tdata: std_logic_vector(S1_I_DATA_WIDTH - 1 downto 0);
    signal s1_d_tdata: std_logic_vector(S1_D_DATA_WIDTH - 1 downto 0);
    signal s1_eprev_tdata: std_logic_vector(DATA_WIDTH - 1 downto 0);
    signal s1_eprev_tvalid: std_logic;

    -- Stage 2: Multiply PID coefficients.
    constant S2_P_DATA_WIDTH: natural := S1_P_DATA_WIDTH + K_WIDTH;
    constant S2_I_DATA_WIDTH: natural := S1_I_DATA_WIDTH + K_WIDTH;
    constant S2_D_DATA_WIDTH: natural := S1_D_DATA_WIDTH + K_WIDTH;
    constant S2_DATA_RADIX: natural := maximum(DATA_RADIX, K_RADIX);
    signal s2_tvalid: std_logic;
    signal s2_p_tdata: std_logic_vector(S2_P_DATA_WIDTH - 1 downto 0);
    signal s2_i_tdata: std_logic_vector(S2_I_DATA_WIDTH - 1 downto 0);
    signal s2_d_tdata: std_logic_vector(S2_D_DATA_WIDTH - 1 downto 0);

    -- Stage 3a: Sum P and I terms.
    signal s3a_tvalid: std_logic;
    constant S3A_PI_DATA_WIDTH: natural := maximum(S2_P_DATA_WIDTH, S2_I_DATA_WIDTH);
    constant S3A_D_DATA_WIDTH: natural := S2_D_DATA_WIDTH;
    constant S3A_DATA_RADIX: natural := S2_DATA_RADIX;
    signal s3a_pi_tdata: std_logic_vector(S3A_PI_DATA_WIDTH - 1 downto 0);
    signal s3a_d_tdata: std_logic_vector(S3A_D_DATA_WIDTH - 1 downto 0);

    -- Stage 3b: Sum PI and D terms.
    constant S3B_PID_DATA_WIDTH: natural := S3A_PI_DATA_WIDTH;
    constant S3B_DATA_RADIX: natural := S3A_DATA_RADIX;
    signal s3b_tvalid: std_logic;
    signal s3b_pid_tdata: std_logic_vector(S3B_PID_DATA_WIDTH - 1 downto 0);

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
        a: std_logic_vector; a_radix: natural;
        r: std_logic_vector; r_radix: natural
    ) return std_logic_vector is
        variable right_shift: integer := integer(a_radix) - integer(r_radix);
    begin
        -- TODO: What if the shift is negative? Does there need to be an if/else or does
        --       shift_right() appropriately handle a negative shift?
        return std_logic_vector(resize(shift_right(signed(a), right_shift), r'length));
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
        a: std_logic_vector; a_radix: natural;
        b: std_logic_vector; b_radix: natural;
        r: std_logic_vector; r_radix: natural
    ) return std_logic_vector is
        variable right_shift: integer := integer(maximum(a_radix, b_radix)) - integer(r_radix);
    begin
        -- TODO: What if the shift is negative? Does there need to be an if/else or does
        --       shift_right() appropriately handle a negative shift?
        return std_logic_vector(resize(shift_right(signed(a) + signed(b), right_shift), r'length));
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
        a: std_logic_vector; a_radix: natural;
        b: std_logic_vector; b_radix: natural;
        r: std_logic_vector; r_radix: natural
    ) return std_logic_vector is
        variable right_shift: integer := integer(maximum(a_radix, b_radix)) - integer(r_radix);
    begin
        -- TODO: What if the shift is negative? Does there need to be an if/else or does
        --       shift_right() appropriately handle a negative shift?
        return std_logic_vector(resize(shift_right(signed(a) - signed(b), right_shift), r'length));
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
        a: std_logic_vector; a_radix: natural;
        b: std_logic_vector; b_radix: natural;
        r: std_logic_vector; r_radix: natural
    ) return std_logic_vector is
        variable right_shift: integer := integer(maximum(a_radix, b_radix)) - integer(r_radix);
    begin
        -- TODO: What if the shift is negative? Does there need to be an if/else or does
        --       shift_right() appropriately handle a negative shift?
        return std_logic_vector(resize(shift_right(signed(a) * signed(b), right_shift), r'length));
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
                s1_eprev_tdata  <= s_axis_e_tdata;
                s1_eprev_tvalid <= s_axis_e_tvalid;
            end if;
            if s_axis_e_tvalid = '1' and s1_eprev_tvalid = '1' then
                s1_tvalid  <= '1';
                s1_p_tdata <= s_axis_e_tdata;
                s1_i_tdata <= fixed_add(
                    s_axis_e_tdata, DATA_RADIX,
                    s1_i_tdata, S1_DATA_RADIX,
                    s1_i_tdata, S1_DATA_RADIX);
                s1_d_tdata <= fixed_sub(
                    s_axis_e_tdata, DATA_RADIX,
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
                kp, K_RADIX,
                s1_p_tdata, S1_DATA_RADIX,
                s2_p_tdata, S2_DATA_RADIX);
            s2_i_tdata <= fixed_mul(
                ki, K_RADIX,
                s1_i_tdata, S1_DATA_RADIX,
                s2_i_tdata, S2_DATA_RADIX);
            s2_d_tdata <= fixed_mul(
                kd, K_RADIX,
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
            s3a_pi_tdata <= (others => '0');
            s3a_d_tdata  <= (others => '0');
        else
            s3a_tvalid   <= '1';
            s3a_pi_tdata <= fixed_add(
                s2_p_tdata, S2_DATA_RADIX,
                s2_i_tdata, S2_DATA_RADIX,
                s3a_pi_tdata, S3A_DATA_RADIX);
            s3a_d_tdata  <= s2_d_tdata;
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
                s3a_pi_tdata, S3A_DATA_RADIX,
                s3a_d_tdata, S3A_DATA_RADIX,
                s3b_pid_tdata, S3B_DATA_RADIX);
        end if;
    end if;
end process;

-- Outputs.
m_axis_u_tdata  <= fixed_radix(s3b_pid_tdata, S3B_DATA_RADIX, m_axis_u_tdata, DATA_RADIX);
m_axis_u_tvalid <= s3b_tvalid;

end behavioral;
