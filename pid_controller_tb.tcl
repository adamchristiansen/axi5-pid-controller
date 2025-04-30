# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Adam Christiansen

# These must match the testbench.
set width 24
set radix 10

# Indices for the sugnal buses.
set upper [expr $width - 1]
set lower 0

# Signal names.
set aclk            "top.pid_controller_tb.aclk"
set aresetn         "top.pid_controller_tb.aresetn"
set s_axis_e_tdata  "top.pid_controller_tb.s_axis_e_tdata\[$upper:$lower\]"
set s_axis_e_tvalid "top.pid_controller_tb.s_axis_e_tvalid"
set m_axis_u_tdata  "top.pid_controller_tb.m_axis_u_tdata\[$upper:$lower\]"
set m_axis_u_tvalid "top.pid_controller_tb.m_axis_u_tvalid"
set kp              "top.pid_controller_tb.kp\[$upper:$lower\]"
set ki              "top.pid_controller_tb.ki\[$upper:$lower\]"
set kd              "top.pid_controller_tb.kd\[$upper:$lower\]"

# Add signals.
gtkwave::/Edit/Insert_Comment "Clock & Reset"
gtkwave::addSignalsFromList [list $aclk $aresetn]
gtkwave::/Edit/Insert_Blank
gtkwave::/Edit/Insert_Comment "Error Signal"
gtkwave::addSignalsFromList [list $s_axis_e_tdata $s_axis_e_tvalid]
gtkwave::/Edit/Insert_Blank
gtkwave::/Edit/Insert_Comment "Control Variable"
gtkwave::addSignalsFromList [list $m_axis_u_tdata $m_axis_u_tvalid]
gtkwave::/Edit/Insert_Blank
gtkwave::/Edit/Insert_Comment "PID Coefficients"
gtkwave::addSignalsFromList [list $kp $ki $kd]

# Set all of the listed waveforms to fixed-point.
#
# Loop over each waveform instead of highlighting all and manipulating them as a batch. Some
# commands do not work as a batch and instead only modify the most recently highlighted waveform.
foreach name [list $s_axis_e_tdata $m_axis_u_tdata $kp $ki $kd] {
    gtkwave::highlightSignalsFromList [list $name]
    gtkwave::/Edit/Data_Format/Signed_Decimal
    gtkwave::/Edit/Data_Format/Fixed_Point_Shift/Specify $radix
    gtkwave::/Edit/Data_Format/Fixed_Point_Shift/On
    gtkwave::setTraceHighlightFromNameMatch $name off
    gtkwave::unhighlightSignalsFromList [list $name]
}

# Set all of the listed waveforms as analog.
#
# Loop over each waveform instead of highlighting all and manipulating them as a batch. Some
# commands do not work as a batch and instead only modify the most recently highlighted waveform.
foreach name [list $s_axis_e_tdata $m_axis_u_tdata] {
    gtkwave::highlightSignalsFromList [list $name]
    gtkwave::/Edit/Data_Format/Analog/Step
    gtkwave::/Edit/Insert_Analog_Height_Extension
    gtkwave::/Edit/Insert_Analog_Height_Extension
    gtkwave::/Edit/Insert_Analog_Height_Extension
    gtkwave::unhighlightSignalsFromList [list $name]
}

# View settings.
gtkwave::nop
gtkwave::/Time/Zoom/Zoom_Full
