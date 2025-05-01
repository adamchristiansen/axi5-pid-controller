-- SPDX-License-Identifier: MIT
-- Copyright (c) 2025 Adam Christiansen

use std.env.finish;

library ieee;
use ieee.fixed_pkg.all;
use ieee.math_real.all;
use ieee.std_logic_1164.all;

entity pid_controller_tb is
end pid_controller_tb;

architecture behaviour of pid_controller_tb
is
    constant CLK_PERIOD : time := 10 ns;

    --- PID.
    constant DATA_WIDTH: natural := 24;
    constant DATA_RADIX: natural := 10;
    constant K_WIDTH: natural := DATA_WIDTH;
    constant K_RADIX: natural := DATA_RADIX;
    constant INTEGRATOR_WIDTH: natural := DATA_WIDTH + 8;
    signal aclk: std_logic;
    signal aresetn: std_logic;
    signal s_axis_e_tdata: std_logic_vector(DATA_WIDTH - 1 downto 0);
    signal s_axis_e_tvalid: std_logic;
    signal m_axis_u_tdata: std_logic_vector(DATA_WIDTH - 1 downto 0);
    signal m_axis_u_tvalid: std_logic;
    signal kp: std_logic_vector(K_WIDTH - 1 downto 0);
    signal ki: std_logic_vector(K_WIDTH - 1 downto 0);
    signal kd: std_logic_vector(K_WIDTH - 1 downto 0);

    --- Change the PID coefficients and assert the reset.
    ---
    --- # Arguments
    ---
    --- - `p`: Value of the P coefficient.
    --- - `i`: Value of the I coefficient.
    --- - `d`: Value of the D coefficient.
    --- - `rn_sig`: Active low reset to write.
    --- - `kp_sig`: P coefficient to write.
    --- - `ki_sig`: I coefficient to write.
    --- - `kd_sig`: D coefficient to write.
    procedure coefficients(
        constant p: in real;
        constant i: in real;
        constant d: in real;
        signal rn_sig: out std_logic;
        signal kp_sig: out std_logic_vector(K_WIDTH - 1 downto 0);
        signal ki_sig: out std_logic_vector(K_WIDTH - 1 downto 0);
        signal kd_sig: out std_logic_vector(K_WIDTH - 1 downto 0)
    ) is
    begin
        rn_sig <= '0';
        kp_sig <= to_slv(to_sfixed(p, K_WIDTH - K_RADIX - 1, -K_RADIX));
        ki_sig <= to_slv(to_sfixed(i, K_WIDTH - K_RADIX - 1, -K_RADIX));
        kd_sig <= to_slv(to_sfixed(d, K_WIDTH - K_RADIX - 1, -K_RADIX));
        wait for 100 * CLK_PERIOD;
        rn_sig <= '1';
        wait for 400 * CLK_PERIOD;
    end procedure;

    --- This value is used to track the last 100 values of the error signal.
    type PrevE is array(0 to 99) of real;
    signal prev_e: PrevE := (others => 0.0);

    --- Push an error signal value onto the tracked values in `prev_e`.
    ---
    --- # Arguments
    ---
    --- - `e`: The error signal value to push.
    ---
    --- # Returns
    ---
    --- A new `PrevE` value containing the pushed value.
    impure function push(e: std_logic_vector) return PrevE is
        variable next_e: PrevE;
    begin
        next_e(next_e'low) := to_real(to_sfixed(e, DATA_WIDTH - DATA_RADIX - 1, -DATA_RADIX));
        for i in next_e'low to next_e'high - 1 loop
            next_e(i + 1) := prev_e(i);
        end loop;
        return next_e;
    end function;

    --- Find the mean of the tracked error signal values in `prev_e`.
    impure function mean return real is
        variable sum: real := 0.0;
    begin
        for i in prev_e'low to prev_e'high loop
            sum := sum + prev_e(i);
        end loop;
        return sum / real(prev_e'length);
    end function;

    --- Find the RMS of the tracked error signal values in `prev_e`.
    impure function rms return real is
        variable sum: real := 0.0;
        variable m: real := mean;
    begin
        for i in prev_e'low to prev_e'high loop
            sum := sum + (prev_e(i) - m) ** 2;
        end loop;
        return sqrt(sum / real(prev_e'length));
    end function;
begin

-- Clock.
aclk_p: process
begin
    aclk <= '1';
    wait for CLK_PERIOD / 2;
    aclk <= '0';
    prev_e <= push(s_axis_e_tdata);
    wait for CLK_PERIOD / 2;
end process;

-- Update the coefficients and assert a reset every time the are changed.
coefficients_p: process
begin
    -- Stable: approach slowly.
    coefficients(0.100, 0.030, 0.000, aresetn, kp, ki, kd);
    assert rms < 0.001 report "Did not reach setpoint" severity failure;
    -- Stable: approach near critical damping.
    coefficients(0.100, 0.100, 0.000, aresetn, kp, ki, kd);
    assert rms < 0.001 report "Did not reach setpoint" severity failure;
    -- Underdamped: Oscillate on the way to the setpoint.
    coefficients(0.100, 0.300, 0.000, aresetn, kp, ki, kd);
    assert rms < 0.001 report "Did not reach setpoint" severity failure;
    -- Overdamped: too slow to to reach the setpoint.
    coefficients(0.000, 0.003, 0.000, aresetn, kp, ki, kd);
    assert abs mean > 1.000 report "Got too close to the setpoint" severity failure;
    -- Unstable: P and D too large, oscillate about the setpoint.
    coefficients(0.500, 0.300, 0.175, aresetn, kp, ki, kd);
    assert rms > 0.500 report "Settled too close to the setpoint" severity failure;
    -- Unstable: P and D too large, oscillate about the setpoint and increase in amplitude.
    coefficients(0.110, 0.300, 0.395, aresetn, kp, ki, kd);
    assert rms > 5.000 report "Settled too close to the setpoint" severity failure;
    finish;
end process;

-- For the test, the error signal is computed from a constant setpoint from which the control
-- variable is subtracted.
s_axis_e_tdata <= to_slv(resize(
    to_sfixed(10.0, DATA_WIDTH - DATA_RADIX - 1, -DATA_RADIX)
    -
    to_sfixed(m_axis_u_tdata, DATA_WIDTH - DATA_RADIX - 1, -DATA_RADIX),
    left_index => DATA_WIDTH - DATA_RADIX - 1,
    right_index => -DATA_RADIX
));
s_axis_e_tvalid <= '1';

dut: entity work.pid_controller
generic map (
    DATA_WIDTH => DATA_WIDTH,
    DATA_RADIX => DATA_RADIX,
    K_WIDTH => K_WIDTH,
    K_RADIX => K_RADIX,
    INTEGRATOR_WIDTH => INTEGRATOR_WIDTH
)
port map (
    aclk => aclk,
    aresetn => aresetn,
    s_axis_e_tdata => s_axis_e_tdata,
    s_axis_e_tvalid => s_axis_e_tvalid,
    m_axis_u_tdata => m_axis_u_tdata,
    m_axis_u_tvalid => m_axis_u_tvalid,
    kp => kp,
    ki => ki,
    kd => kd
);

end behaviour;
