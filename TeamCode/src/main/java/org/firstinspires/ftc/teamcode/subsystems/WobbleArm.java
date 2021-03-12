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

public class WobbleArm extends Subsystem {
    public static final String TAG = "WobbleArm";
    private DcMotorEx wobbleArm;
    private int targetPosition = 0;

    public WobbleArm() {

    }

    @Override
    public Map<String, Object> update() {
        Map<String, Object> updates = new HashMap<>();
        updates.put("Position", wobbleArm.getCurrentPosition());
        updates.put("TargetPosition", targetPosition);
        return updates;
    }

    public void init(HardwareMap hwMap) {
        wobbleArm = hwMap.get(DcMotorEx.class, "wobbleArm");
        wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleArm.setTargetPositionTolerance(5);
    }

    public void wobbleUp() {
        wobbleArm.setTargetPosition(-120);
        targetPosition = -120;
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArm.setPower(-0.5);
    }

    public void wobbleDown() {
        targetPosition = 0;
        wobbleArm.setTargetPosition(0);
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArm.setPower(0.5);
    }
}
