-- SPDX-License-Identifier: MIT
-- Copyright (c) 2025 Adam Christiansen

use std.env.finish;

library ieee;
use ieee.std_logic_1164.all;

entity pid_controller_tb is
    port (
        aclk: out std_logic;
        aresetn: out std_logic
    );
end pid_controller_tb;

architecture behaviour of pid_controller_tb
is
    constant CLK_PERIOD : time := 10 ns;

    -- PID.
    constant PID_DATA_WIDTH: natural := 24;
    constant PID_DATA_RADIX: natural := 8;
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
begin

-- Terminate.
terminate_p: process
begin
    wait for 100 * CLK_PERIOD;
    finish;
end process;

-- Clock.
aclk_p: process
begin
    pid_aclk <= '0';
    wait for CLK_PERIOD / 2;
    pid_aclk <= '1';
    wait for CLK_PERIOD / 2;
end process;

-- Reset.
aresetn_p: process
begin
    pid_aresetn <= '0';
    wait for 5 * CLK_PERIOD / 2;
    pid_aresetn <= '1';
end process;

-- TODO: Drive these inputs.
pid_s_axis_e_tdata  <= (others => '0');
pid_s_axis_e_tvalid <= '0';
pid_kp              <= (others => '0');
pid_ki              <= (others => '0');
pid_kd              <= (others => '0');

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

-- Signals to monitor.
aclk    <= pid_aclk;
aresetn <= pid_aresetn;

end behaviour;
