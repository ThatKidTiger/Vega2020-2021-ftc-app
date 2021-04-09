package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.hardware.BotConstants.LAUNCH_CONSTANTS;

import java.util.HashMap;
import java.util.Map;

public class Intake extends Subsystem {
    public static final String TAG = "Intake";
    private DcMotorEx intake;

    public Intake() {

    }

    @Override
    public Map<String, Object> update() {
        Map<String, Object> updates = new HashMap<>();
        return updates;
    }

    public void init(HardwareMap hwMap) {
        intake = hwMap.get(DcMotorEx.class, "intake");
    }

    public void spinToVel(double velocity) {
        intake.setPower(velocity);
    }
}
