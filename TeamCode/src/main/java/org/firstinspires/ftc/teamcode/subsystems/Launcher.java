package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.qualcomm.hardware.motors.GoBILDA5201Series;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.hardware.BotConstants.LAUNCH_CONSTANTS;
import org.firstinspires.ftc.teamcode.hardware.BotConstants.LAUNCH_CONSTANTS.*;

import java.util.HashMap;
import java.util.Map;

public class Launcher extends Subsystem {
    public static final String TAG = "Launcher";
    private DcMotorEx launcher;
    private PIDFCoefficients coefficients = new PIDFCoefficients(LAUNCH_CONSTANTS.P, LAUNCH_CONSTANTS.I, LAUNCH_CONSTANTS.D, 0);

    public Launcher() {

    }

    @Override
    public Map<String, Object> update() {
        Map<String, Object> updates = new HashMap<>();

        updates.put("Launcher Vel", launcher.getVelocity());
        return updates;
    }

    public void init(HardwareMap hwMap) {
        launcher = hwMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setVelocityPIDFCoefficients(LAUNCH_CONSTANTS.P, LAUNCH_CONSTANTS.I, LAUNCH_CONSTANTS.D, LAUNCH_CONSTANTS.F);
        launcher.setPositionPIDFCoefficients(5.0);
    }

    public void spinToVel(double velocity) {
        launcher.setPower(velocity);
    }
}
