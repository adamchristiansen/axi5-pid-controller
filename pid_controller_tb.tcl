# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Adam Christiansen

# Add signals.
gtkwave::/Edit/Insert_Comment "Clock & Reset"
gtkwave::addSignalsFromList { "pid_aclk" }
gtkwave::addSignalsFromList { "pid_aresetn" }
gtkwave::/Edit/Insert_Blank
gtkwave::/Edit/Insert_Comment "Error Signal"
gtkwave::addSignalsFromList { "pid_s_axis_e_tdata\[23:0\]" }
gtkwave::addSignalsFromList { "pid_s_axis_e_tvalid" }
gtkwave::/Edit/Insert_Blank
gtkwave::/Edit/Insert_Comment "Control Variable"
gtkwave::addSignalsFromList { "pid_m_axis_u_tdata\[23:0\]" }
gtkwave::addSignalsFromList { "pid_m_axis_u_tvalid" }
gtkwave::/Edit/Insert_Blank
gtkwave::/Edit/Insert_Comment "PID Coefficients"
gtkwave::addSignalsFromList { "pid_kp\[23:0\]" }
gtkwave::addSignalsFromList { "pid_ki\[23:0\]" }
gtkwave::addSignalsFromList { "pid_kd\[23:0\]" }

# Set all of the listed waveforms to fixed-point.
#
# Loop over each waveform instead of highlighting all and manipulating them as a batch. Some
# commands do not work as a batch and instead only modify the most recently highlighted waveform.
foreach name {
    "top.pid_controller_tb.pid_s_axis_e_tdata\[23:0\]"
    "top.pid_controller_tb.pid_m_axis_u_tdata\[23:0\]"
    "top.pid_controller_tb.pid_kp\[23:0\]"
    "top.pid_controller_tb.pid_ki\[23:0\]"
    "top.pid_controller_tb.pid_kd\[23:0\]"
} {
    gtkwave::highlightSignalsFromList [ list $name ]
    gtkwave::/Edit/Data_Format/Signed_Decimal
    gtkwave::/Edit/Data_Format/Fixed_Point_Shift/Specify 8
    gtkwave::/Edit/Data_Format/Fixed_Point_Shift/On
    gtkwave::setTraceHighlightFromNameMatch $name off
    gtkwave::unhighlightSignalsFromList [ list $name ]
}

# Set all of the listed waveforms as analog.
#
# Loop over each waveform instead of highlighting all and manipulating them as a batch. Some
# commands do not work as a batch and instead only modify the most recently highlighted waveform.
foreach name {
    "top.pid_controller_tb.pid_s_axis_e_tdata\[23:0\]"
    "top.pid_controller_tb.pid_m_axis_u_tdata\[23:0\]"
} {
    gtkwave::highlightSignalsFromList [ list $name ]
    gtkwave::/Edit/Data_Format/Analog/Step
    gtkwave::/Edit/Insert_Analog_Height_Extension
    gtkwave::/Edit/Insert_Analog_Height_Extension
    gtkwave::/Edit/Insert_Analog_Height_Extension
    gtkwave::unhighlightSignalsFromList [ list $name ]
}

# View settings.
gtkwave::nop
gtkwave::/Time/Zoom/Zoom_Full
