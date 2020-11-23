package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Map;

public class Launcher extends Subsystem {
    private DcMotor launcher;

    public Launcher() {

    }

    @Override
    public Map<String, Object> update() {
        return null;
    }

    public void init(HardwareMap hwMap) {
        launcher = hwMap.get(DcMotor.class, "launcher");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void spinToVel(double velocity) {
        launcher.setPower(velocity);
    }
}
