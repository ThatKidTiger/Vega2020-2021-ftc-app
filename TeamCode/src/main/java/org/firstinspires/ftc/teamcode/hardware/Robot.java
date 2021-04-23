package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.stormbots.MiniPID;
import com.sun.source.doctree.StartElementTree;

import org.firstinspires.ftc.teamcode.subsystems.IMU;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleArm;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static android.os.SystemClock.sleep;

/*
Todo: Run all systems as state machines, with update commands as transition states
Todo: pass the dashboard object reference to robot, and then retrieve the various subsystem updates
 */

public class Robot extends Subsystem {
    public static final String TAG = "Robot";

    ElapsedTime runtime = new ElapsedTime();

    ArrayList<Subsystem> subsystems = new ArrayList<>();
    public IMU imu = new IMU();
    public Launcher launcher = new Launcher();
    public MecanumDrive drive = new MecanumDrive();
    public Intake intake = new Intake();
    public WobbleArm wobble = new WobbleArm();

    FtcDashboard dash;

    private double reference = 0;
    private double currentX = 0;
    private double currentY = 0;

    /* Constructor */
    public Robot() {
        subsystems.add(drive);
        subsystems.add(imu);
        subsystems.add(launcher);
        subsystems.add(intake);
        subsystems.add(wobble);
    }

    public Map<String, Object> update() {
        Map<String, Object> updates = new HashMap<>();
        for(Subsystem subsystem : subsystems) {
            Map<String, Object> updatePacket = subsystem.update();
            if (updatePacket != null) {
                updates.putAll(updatePacket);
            }
        }

        //might wanna add to telemetry in the opmode as well
        updates.put("Reference", reference);
        updates.put("CurrentX", currentX);
        updates.put("CurrentY", currentY);

        return updates;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap, FtcDashboard dash) {
        this.dash = dash;

        for(Subsystem subsystem : subsystems) {
            subsystem.init(hwMap, dash);
        }
        runtime.startTime();
    }

    //region Auton methods
    public void forwardByDistance(double distance) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.startTime();

        double power;
        double target;
        double profilePower;

        MiniPID pid = new MiniPID(DRIVE_CONSTANTS.STRAIGHT_PID);

        List<Double> wheelPositions = drive.getWheelPositions();
        double x = (wheelPositions.get(0) + wheelPositions.get(1)) / 2;

        target = distance + x;

        reference = target;

        while(Math.abs(target - x) > 0.05) {
            currentX = x;

            TelemetryPacket packet = new TelemetryPacket();
            packet.putAll(update());
            dash.sendTelemetryPacket(packet);

            wheelPositions = drive.getWheelPositions();
            x = (wheelPositions.get(0) + wheelPositions.get(1)) / 2;

            power = pid.getOutput(x, target);

            Log.d("PID Output", "" + power);

            double travelled = x - target + distance;
            Log.d("Travelled" , "" + travelled);
            if(travelled <= 0.25 * distance) {
                profilePower = Math.abs(power) * (travelled / (0.25 * distance));
                Log.d("Phase", "Start");
            } else if(travelled >= 0.75 * distance) {
                profilePower = Math.abs(power) * ((distance - travelled) / (0.25 * distance));
                Log.d("Phase", "End");
            } else {
                profilePower = 1;
                Log.d("Phase", "Middle");
            }
            Log.d("Profile" , "" + profilePower);

            if(profilePower == 0) {
                power *= 0.2 / Math.abs(power);
            } else if(Math.abs(power) > Math.abs(profilePower)) {
                power *= Math.abs(profilePower)/Math.abs(power);
            }

            if(Math.abs(power) < 0.2) {
                power *= 0.2/Math.abs(power);
            }

            Log.d("Power", "" + -power);
            Log.d("Error", "" + (target-x));
            double[] powers = {-power, -power, -power, -power};
            drive.setMotorPowers(powers);
        }
        stop();
        Log.d(TAG, "Time to Complete Forward Movement: " + runtime.milliseconds());
    }

    public void strafebyDistance(double distance) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.startTime();

        double backAdjust = 1.1;

        double power;
        double target;
        double profilePower;

        MiniPID strafePID = new MiniPID(DRIVE_CONSTANTS.STRAFE_PID);

        List<Double> wheelPositions = drive.getWheelPositions();
        double y = wheelPositions.get(2);

        target = distance + y;

        reference = target;

        while(Math.abs(distance - y) > 0.05) {
            currentY = y;

            TelemetryPacket packet = new TelemetryPacket();
            packet.putAll(update());
            dash.sendTelemetryPacket(packet);

            wheelPositions = drive.getWheelPositions();
            y = wheelPositions.get(2);

            power = strafePID.getOutput(y, target);

            double travelled = y - target + distance;
            if(travelled <= 0.25 * distance) {
                profilePower = Math.abs(power) * (travelled / (0.25 * distance));
                Log.d("Phase", "Start");
            } else if(travelled >= 0.75 * distance) {
                profilePower = Math.abs(power) * ((distance - travelled) / (0.25 * distance));
                Log.d("Phase", "End");
            } else {
                profilePower = 1;
                Log.d("Phase", "Middle");
            }

            if(profilePower == 0) {
                power *= 0.2 / Math.abs(power);
            } else if(Math.abs(power) > Math.abs(profilePower)) {
                power *= Math.abs(profilePower)/Math.abs(power);
            }

            if(Math.abs(power) < 0.2) {
                power *= 0.2/Math.abs(power);
            }

            double[] powers = {power, -power * backAdjust, power * backAdjust, -power};
            drive.setMotorPowers(powers);
        }
        stop();
        Log.d(TAG, "Time to Complete strafe: " + runtime.milliseconds());
    }

    public void rotateByPower(double power) {
        Log.d(TAG, String.format("Rotating " + (power < 0 ? "counterclockwise" : "clockwise") + " at power %f.2", Math.abs(power)));
        double[] powers = {power, power, -power, -power};
        drive.setMotorPowers(powers);
    }

    //uses PID controller to precisely turn a certain number of degrees
    public void rotateByAngle(double degrees) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.startTime();

        reference = degrees;

        MiniPID imuPID = new MiniPID(DRIVE_CONSTANTS.ROT_PID);
        double leftPower, rightPower, temppower;

        imu.resetAngularDistance();
        sleep(250);

        double lastAngle = 0;
        double angle = imu.getAngularDistance();

        //rotates until the imu returns that the robot is within a margin of error
        while(Math.abs(degrees - angle) > 1 || Math.abs(degrees - lastAngle) > 1) {
            lastAngle = angle;
            angle = imu.getAngularDistance();
            Log.d(TAG, "" + angle);
            temppower = imuPID.getOutput(angle, degrees);

            if(Math.abs(temppower) > 0.8) {
                temppower *= 0.8/Math.abs(temppower);
            }

            if(Math.abs(temppower) < 0.2) {
                temppower *= 0.2 / Math.abs(temppower);
            }

            if(angle == 0) {
                temppower = Math.signum(degrees) * 0.2;
            }

            leftPower = temppower;
            rightPower = -temppower;

            double[] powers = {leftPower, leftPower, rightPower, rightPower};
            drive.setMotorPowers(powers);
        }
        stop();
        Log.d(TAG, "Time to Complete Rotation: " + runtime.milliseconds());
    }

    public void stop() {
        double[] powers = {0, 0, 0, 0};
        drive.setMotorPowers(powers);
    }

    @Config
    public enum DRIVE_CONSTANTS {;
        public static PIDCoefficients ROT_PID = new PIDCoefficients(0.0375, 0, 0.01);
        public static PIDCoefficients STRAIGHT_PID = new PIDCoefficients(0.2, 0, 0);
        public static PIDCoefficients STRAFE_PID = new PIDCoefficients(0.0375, 0, 0.01);
    }
}
