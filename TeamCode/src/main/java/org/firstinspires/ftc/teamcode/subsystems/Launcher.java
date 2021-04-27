package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.motors.GoBILDA5201Series;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;
import java.util.Map;

public class Launcher extends Subsystem {
    public static final String TAG = "Launcher";

    public DcMotorEx launcher;
    private Servo flicker;

    private PIDFCoefficients coefficients = new PIDFCoefficients(LAUNCH_CONSTANTS.P, LAUNCH_CONSTANTS.I, LAUNCH_CONSTANTS.D, 0);

    private double highSpeed = -1340;
    private double powerSpeed = -1240;

    private double velocity = powerSpeed;

    public Launcher() {

    }

    @Override
    public Map<String, Object> update() {
        launcher.setVelocityPIDFCoefficients(LAUNCH_CONSTANTS.P, LAUNCH_CONSTANTS.I, LAUNCH_CONSTANTS.D, LAUNCH_CONSTANTS.F);
        launcher.setPositionPIDFCoefficients(LAUNCH_CONSTANTS.PP);

        Map<String, Object> updates = new HashMap<>();

        updates.put("Launcher Vel", launcher.getVelocity());
        return updates;
    }

    public void init(HardwareMap hwMap, FtcDashboard dash) {
        this.dash = dash;

        launcher = hwMap.get(DcMotorEx.class, "launcher");
        flicker = hwMap.get(Servo.class, "flicker");

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setVelocityPIDFCoefficients(LAUNCH_CONSTANTS.P, LAUNCH_CONSTANTS.I, LAUNCH_CONSTANTS.D, LAUNCH_CONSTANTS.F);
        launcher.setPositionPIDFCoefficients(LAUNCH_CONSTANTS.PP);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetFlicker();
    }

    public void spinUp() {
        launcher.setVelocity(velocity);
    }

    public void shoot() {
        flicker.setPosition(0.25);
    }

    public void resetFlicker() {
        flicker.setPosition(0);
    }

    public void setHighSpeed() {
        velocity = highSpeed;
    }

    public void setPowerSpeed() {
        velocity = powerSpeed;
    }

    public void stopMotor() {
        launcher.setVelocity(0);
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    public double getVelocity() {
        return launcher.getVelocity();
    }

    public double getTargetVelocity() {
        return velocity;
    }

    public boolean atTargetVelocity() {
        return Math.abs(velocity - launcher.getVelocity()) < 10;
    }

    @Config
    public enum LAUNCH_CONSTANTS {;
        public static double P = 1;
        public static double I = .3;
        public static double D = 0;
        public static double F = 13.65;
        public static double PP = 5;
    }
}


