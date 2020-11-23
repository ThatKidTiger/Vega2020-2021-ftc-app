package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Map;

public abstract class Subsystem {
	public abstract Map<String, Object> update();

	public abstract void init(HardwareMap hwMap);
}
