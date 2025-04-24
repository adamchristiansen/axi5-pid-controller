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
    constant PID_DATA_WIDTH: natural := 24;
    constant PID_DATA_RADIX: natural := 10;
    constant PID_K_WIDTH: natural := PID_DATA_WIDTH;
    constant PID_K_RADIX: natural := PID_DATA_RADIX;
    constant PID_INTEGRATOR_WIDTH: natural := PID_DATA_WIDTH + 8;
    signal pid_aclk: std_logic;
    signal pid_aresetn: std_logic;
    signal pid_s_axis_e_tdata: std_logic_vector(PID_DATA_WIDTH - 1 downto 0);
    signal pid_s_axis_e_tvalid: std_logic;
    signal pid_m_axis_u_tdata: std_logic_vector(PID_DATA_WIDTH - 1 downto 0);
    signal pid_m_axis_u_tvalid: std_logic;
    signal pid_kp: std_logic_vector(PID_K_WIDTH - 1 downto 0);
    signal pid_ki: std_logic_vector(PID_K_WIDTH - 1 downto 0);
    signal pid_kd: std_logic_vector(PID_K_WIDTH - 1 downto 0);

    --- Change the PID coefficients and assert the reset.
    ---
    --- # Arguments
    ---
    --- - `p`: Value of the P coefficient.
    --- - `i`: Value of the I coefficient.
    --- - `d`: Value of the D coefficient.
    --- - `rn`: Active low reset to write.
    --- - `kp`: P coefficient to write.
    --- - `ki`: I coefficient to write.
    --- - `kd`: D coefficient to write.
    procedure coefficients(
        constant p: in real;
        constant i: in real;
        constant d: in real;
        signal rn: out std_logic;
        signal kp: out std_logic_vector(PID_K_WIDTH - 1 downto 0);
        signal ki: out std_logic_vector(PID_K_WIDTH - 1 downto 0);
        signal kd: out std_logic_vector(PID_K_WIDTH - 1 downto 0)
    ) is
    begin
        rn <= '0';
        kp <= to_slv(to_sfixed(p, PID_K_WIDTH - PID_K_RADIX - 1, -PID_K_RADIX));
        ki <= to_slv(to_sfixed(i, PID_K_WIDTH - PID_K_RADIX - 1, -PID_K_RADIX));
        kd <= to_slv(to_sfixed(d, PID_K_WIDTH - PID_K_RADIX - 1, -PID_K_RADIX));
        wait for 100 * CLK_PERIOD;
        rn <= '1';
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
        next_e(next_e'low) := to_real(to_sfixed(e,
            PID_DATA_WIDTH - PID_DATA_RADIX - 1, -PID_DATA_RADIX));
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
    pid_aclk <= '0';
    wait for CLK_PERIOD / 2;
    pid_aclk <= '1';
    prev_e   <= push(pid_s_axis_e_tdata);
    wait for CLK_PERIOD / 2;
end process;

-- Update the coefficients and assert a reset every time the are changed.
coefficients_p: process
begin
    -- Stable: approach slowly.
    coefficients(0.010, 0.030, 0.000, pid_aresetn, pid_kp, pid_ki, pid_kd);
    assert rms < 0.001 report "Did not reach setpoint" severity failure;
    -- Stable: approach slowly near critical damping.
    coefficients(0.050, 0.110, 0.100, pid_aresetn, pid_kp, pid_ki, pid_kd);
    assert rms < 0.001 report "Did not reach setpoint" severity failure;
    -- Underdamped: Oscillate on the way to the setpoint.
    coefficients(0.100, 0.400, 0.100, pid_aresetn, pid_kp, pid_ki, pid_kd);
    assert rms < 0.001 report "Did not reach setpoint" severity failure;
    -- Overdamped: too slow to to reach the setpoint.
    coefficients(0.010, 0.003, 0.000, pid_aresetn, pid_kp, pid_ki, pid_kd);
    assert abs mean > 1.000 report "Got too close to the setpoint" severity failure;
    -- Unstable: P and D too large, oscillate about the setpoint.
    coefficients(0.400, 0.125, 0.300, pid_aresetn, pid_kp, pid_ki, pid_kd);
    assert rms > 0.500 report "Settled too close to the setpoint" severity failure;
    -- Unstable: D too large, oscillate about the setpoint and increase in amplitude.
    coefficients(0.100, 0.030, 0.49, pid_aresetn, pid_kp, pid_ki, pid_kd);
    assert rms > 1.000 report "Settled too close to the setpoint" severity failure;
    finish;
end process;

-- For the test, the error signal is computed from a constant setpoint from which the control
-- variable is subtracted.
pid_s_axis_e_tdata <= to_slv(resize(
    to_sfixed(10.0, PID_DATA_WIDTH - PID_DATA_RADIX - 1, -PID_DATA_RADIX)
    -
    to_sfixed(pid_m_axis_u_tdata, PID_DATA_WIDTH - PID_DATA_RADIX - 1, -PID_DATA_RADIX),
    left_index => PID_DATA_WIDTH - PID_DATA_RADIX - 1,
    right_index => -PID_DATA_RADIX
));
pid_s_axis_e_tvalid <= '1';

dut: entity work.pid_controller
generic map (
    DATA_WIDTH => PID_DATA_WIDTH,
    DATA_RADIX => PID_DATA_RADIX,
    K_WIDTH => PID_K_WIDTH,
    K_RADIX => PID_K_RADIX,
    INTEGRATOR_WIDTH => PID_INTEGRATOR_WIDTH
)
port map (
    aclk => pid_aclk,
    aresetn => pid_aresetn,
    s_axis_e_tdata => pid_s_axis_e_tdata,
    s_axis_e_tvalid => pid_s_axis_e_tvalid,
    m_axis_u_tdata => pid_m_axis_u_tdata,
    m_axis_u_tvalid => pid_m_axis_u_tvalid,
    kp => pid_kp,
    ki => pid_ki,
    kd => pid_kd
);

end behaviour;
