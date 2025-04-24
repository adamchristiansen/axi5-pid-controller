# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Adam Christiansen

# These must match the testbench.
set width 24
set radix 10

# Indices for the sugnal buses.
set upper [expr $width - 1]
set lower 0

# Signal names.
set pid_aclk            "top.pid_controller_tb.pid_aclk"
set pid_aresetn         "top.pid_controller_tb.pid_aresetn"
set pid_s_axis_e_tdata  "top.pid_controller_tb.pid_s_axis_e_tdata\[$upper:$lower\]"
set pid_s_axis_e_tvalid "top.pid_controller_tb.pid_s_axis_e_tvalid"
set pid_m_axis_u_tdata  "top.pid_controller_tb.pid_m_axis_u_tdata\[$upper:$lower\]"
set pid_m_axis_u_tvalid "top.pid_controller_tb.pid_m_axis_u_tvalid"
set pid_kp              "top.pid_controller_tb.pid_kp\[$upper:$lower\]"
set pid_ki              "top.pid_controller_tb.pid_ki\[$upper:$lower\]"
set pid_kd              "top.pid_controller_tb.pid_kd\[$upper:$lower\]"

# Add signals.
gtkwave::/Edit/Insert_Comment "Clock & Reset"
gtkwave::addSignalsFromList [list $pid_aclk $pid_aresetn]
gtkwave::/Edit/Insert_Blank
gtkwave::/Edit/Insert_Comment "Error Signal"
gtkwave::addSignalsFromList [list $pid_s_axis_e_tdata $pid_s_axis_e_tvalid]
gtkwave::/Edit/Insert_Blank
gtkwave::/Edit/Insert_Comment "Control Variable"
gtkwave::addSignalsFromList [list $pid_m_axis_u_tdata $pid_m_axis_u_tvalid]
gtkwave::/Edit/Insert_Blank
gtkwave::/Edit/Insert_Comment "PID Coefficients"
gtkwave::addSignalsFromList [list $pid_kp $pid_ki $pid_kd]

# Set all of the listed waveforms to fixed-point.
#
# Loop over each waveform instead of highlighting all and manipulating them as a batch. Some
# commands do not work as a batch and instead only modify the most recently highlighted waveform.
foreach name [list $pid_s_axis_e_tdata $pid_m_axis_u_tdata $pid_kp $pid_ki $pid_kd] {
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
foreach name [list $pid_s_axis_e_tdata $pid_m_axis_u_tdata] {
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
