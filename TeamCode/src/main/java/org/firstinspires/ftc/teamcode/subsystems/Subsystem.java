package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Map;

public abstract class Subsystem {
	FtcDashboard dash;

	public abstract Map<String, Object> update();

	public abstract void init(HardwareMap hwMap, FtcDashboard dash);
}
