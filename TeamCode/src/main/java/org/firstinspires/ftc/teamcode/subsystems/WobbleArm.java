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

    public WobbleArm() {

    }

    @Override
    public Map<String, Object> update() {
        Map<String, Object> updates = new HashMap<>();
        updates.put("Position", wobbleArm.getCurrentPosition());
        updates.put("TargetPosition", 112);
        return updates;
    }

    public void init(HardwareMap hwMap) {
        wobbleArm = hwMap.get(DcMotorEx.class, "wobbleArm");
        wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleArm.setTargetPositionTolerance(10);
    }

    public void wobbleUp() {
        wobbleArm.setTargetPosition(90);
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArm.setPower(-0.5);
        while(wobbleArm.isBusy()) {

        }
        wobbleArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobbleArm.setPower(0);
    }

    public void wobbleDown() {
        wobbleArm.setTargetPosition(222);
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArm.setPower(0.5);
        while(wobbleArm.isBusy()) {

        }
        wobbleArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobbleArm.setPower(0);
    }
}
