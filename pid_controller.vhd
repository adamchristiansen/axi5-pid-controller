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
--- The PID controller is used in in the following way, where `r(t)` is the setpoint and `y(t)` is
--- the process variable. The error signal fed to the PID controller is generally computed in same
--- way from the setpoint and process variable (often as the difference).
---
---  ```
---              e(t)                u(t)
---  s(t) ---> Σ ---> PID Controller ---> Process --+-> y(t)
---            |                                    |
---            +------------------------------------+
---  ```
---
--- The PID controller accepts `e(t)` rather than `s(t)` so it is possible to generate error
--- signals using more spohisticated approaches.
---
--- # Generics
---
--- - `TDATA_WIDTH`: Width in bits of the input and output AXI4/5-Streams.
--- - `K_WIDTH`: Width in bits of the input `kp`, `ki`, and `kd` coefficients.
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
    TDATA_WIDTH: natural := 16;
    K_WIDTH: natural := TDATA_WIDTH
);
port(
    aclk: in std_logic;
    aresetn: in std_logic;
    -- Input signal.
    s_axis_e_tdata: in std_logic_vector(TDATA_WIDTH - 1 downto 0);
    s_axis_e_tvalid: in std_logic;
    -- Output signal.
    m_axis_u_tdata: out std_logic_vector(TDATA_WIDTH - 1 downto 0);
    m_axis_u_tvalid: out std_logic;
    -- PID coefficients.
    kp: in std_logic_vector(K_WIDTH - 1 downto 0);
    ki: in std_logic_vector(K_WIDTH - 1 downto 0);
    kd: in std_logic_vector(K_WIDTH - 1 downto 0)
);
end pid_controller;

architecture behavioral of pid_controller is

-- Stage 1: Integrate and differentiate.
signal s1_tvalid: std_logic;
signal s1p_tdata: std_logic_vector(TDATA_WIDTH - 1 downto 0);
signal s1i_tdata: std_logic_vector(TDATA_WIDTH - 1 downto 0);
signal s1d_tdata: std_logic_vector(TDATA_WIDTH - 1 downto 0);
signal s1eprev_tdata: std_logic_vector(TDATA_WIDTH - 1 downto 0);
signal s1eprev_tvalid: std_logic;

-- Stage 2: Multiply PID coefficients.
signal s2_tvalid: std_logic;
signal s2p_tdata: std_logic_vector(TDATA_WIDTH - 1 downto 0);
signal s2i_tdata: std_logic_vector(TDATA_WIDTH - 1 downto 0);
signal s2d_tdata: std_logic_vector(TDATA_WIDTH - 1 downto 0);

begin

-- Stage 1: Integrate and differentiate.
stage1_p: process (aclk)
begin
    if rising_edge(aclk) then
        if aresetn = '0' then
            s1_tvalid      <= '0';
            s1p_tdata      <= (others => '0');
            s1i_tdata      <= (others => '0');
            s1d_tdata      <= (others => '0');
            s1eprev_tdata  <= (others => '0');
            s1eprev_tvalid <= '0';
        else
            -- The idea is to delay the error signal by 1 so that the derivative can be computed
            -- from its current and previous value.
            if s_axis_e_tvalid = '1' then
                s1eprev_tdata  <= s_axis_e_tdata;
                s1eprev_tvalid <= s_axis_e_tvalid;
            end if;
            if s_axis_e_tvalid = '1' and s1eprev_tvalid = '1' then
                s1_tvalid <= '1';
                s1p_tdata <= s_axis_e_tdata;
                s1i_tdata <= std_logic_vector(signed(s_axis_e_tdata) + signed(s1i_tdata));
                s1d_tdata <= std_logic_vector(signed(s_axis_e_tdata) - signed(s1eprev_tdata));
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
        if aresetn = '0' or s1_tvalid = '0' then
            s2_tvalid <= '0';
            s2p_tdata <= (others => '0');
            s2i_tdata <= (others => '0');
            s2d_tdata <= (others => '0');
        else
            s2_tvalid <= '1';
            s2p_tdata <= std_logic_vector(resize(signed(kp) * signed(s1p_tdata), s2p_tdata'length));
            s2i_tdata <= std_logic_vector(resize(signed(ki) * signed(s1i_tdata), s2i_tdata'length));
            s2d_tdata <= std_logic_vector(resize(signed(kd) * signed(s1d_tdata), s2d_tdata'length));
        end if;
    end if;
end process;

-- Stage 3: Sum to produce control variable.
-- TODO

-- TODO: Connect output signals.
m_axis_u_tdata  <= (others => '0');
m_axis_u_tvalid <= '0';

end behavioral;
