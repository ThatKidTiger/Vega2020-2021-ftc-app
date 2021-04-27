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
        updates.put("Travelled", 0);
        updates.put("PID Power", 0);
        updates.put("Profile", 0);
        updates.put("FinalPower", 0);

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

        double adjust = DRIVE_CONSTANTS.straightAdjust;

        MiniPID pid = new MiniPID(DRIVE_CONSTANTS.STRAIGHT_PID);

        List<Double> wheelPositions = drive.getWheelPositions();
        double x = (wheelPositions.get(0) + wheelPositions.get(1)) / 2;
        double lastX = x;

        target = distance + x;

        reference = target;

        while(Math.abs(target - x) > 0.075 || Math.abs(target - lastX) > 0.1 && runtime.seconds() < 2.9) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.putAll(update());
            currentX = x;

            lastX = x;

            wheelPositions = drive.getWheelPositions();
            x = (wheelPositions.get(0) + wheelPositions.get(1)) / 2;

            power = pid.getOutput(x, target);
            packet.put("PID Power", power);

            Log.d("PID Output", "" + power);

            double travelled = x - target + distance;
            packet.put("Travelled", travelled);
            Log.d("Travelled" , "" + travelled);
            if(Math.abs(travelled) <= 0.25 * Math.abs(distance)) {
                profilePower = (travelled / (0.25 * distance));
                Log.d("Phase", "Start");
            } else if(Math.abs(travelled) >= 0.25 * Math.abs(distance)) {
                profilePower = (Math.abs(distance - travelled) / (0.75 * distance));
                Log.d("Phase", "End");
            } else {
                profilePower = 1;
                Log.d("Phase", "Middle");
            }
            Log.d("Profile" , "" + profilePower);
            packet.put("Profile", profilePower);

            if(profilePower == 0) {
                power *= 0.15/ Math.abs(power);
            } else if(Math.abs(power) > Math.abs(profilePower)) {
                power *= Math.abs(profilePower)/Math.abs(power);
            }

            if(Math.abs(power) < 0.15) {
                power *= 0.15/Math.abs(power);
            }

            if(Math.abs(power * adjust) > 0.8) {
                power *= 0.8/(Math.abs(power) * adjust);
            }

            packet.put("FinalPower", power);
            dash.sendTelemetryPacket(packet);

            Log.d("Power", "" + -power);
            Log.d("Error", "" + (target-x));
            double[] powers = {-power * adjust, -power * adjust, -power, -power};
            drive.setMotorPowers(powers);
        }
        stop();
        Log.d(TAG, "Time to Complete Forward Movement: " + runtime.milliseconds());
    }

    public void strafebyDistance(double distance) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.startTime();

        double backAdjust = DRIVE_CONSTANTS.strafeAdjust;

        double power;
        double target;
        double profilePower;

        MiniPID strafePID = new MiniPID(DRIVE_CONSTANTS.STRAFE_PID);

        List<Double> wheelPositions = drive.getWheelPositions();
        double y = wheelPositions.get(2);
        double lastY = y;

        target = distance + y;

        reference = target;

        while(Math.abs(target - y) > 0.05 || Math.abs(target - lastY) > 0.1 && runtime.seconds() < 2.9) {
            currentY = y;

            lastY = y;

            TelemetryPacket packet = new TelemetryPacket();
            packet.putAll(update());
            dash.sendTelemetryPacket(packet);

            wheelPositions = drive.getWheelPositions();
            y = wheelPositions.get(2);

            power = strafePID.getOutput(y, target);

            double travelled = y - target + distance;
            if(Math.abs(travelled) <= 0.25 * Math.abs(distance)) {
                profilePower = (Math.abs(travelled) / (0.25 * distance));
            } else if(Math.abs(travelled) > 0.25 * Math.abs(distance)) {
                profilePower = (Math.abs(distance - travelled) / (0.75 * distance));
            } else {
                profilePower = 1;
            }

            if(profilePower == 0) {
                power *= 0.2 / Math.abs(power);
            } else if(Math.abs(power) > Math.abs(profilePower)) {
                power *= Math.abs(profilePower)/Math.abs(power);
            }

            if(Math.abs(power) < 0.18) {
                power *= 0.18/Math.abs(power);
            }

            if(Math.abs(power * backAdjust) > 0.8) {
                power *= 0.8/(Math.abs(power) * backAdjust);
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
        public static PIDCoefficients STRAIGHT_PID = new PIDCoefficients(0.2, 0, 0.15);
        public static PIDCoefficients STRAFE_PID = new PIDCoefficients(0.15, 0, 0.1);
        public static double straightAdjust = 1.04;
        public static double strafeAdjust = 1.2;
    }
}
