package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
import static org.firstinspires.ftc.teamcode.hardware.BotConstants.IMU_CONSTANTS.ROT_MAX;
import static org.firstinspires.ftc.teamcode.hardware.BotConstants.IMU_CONSTANTS.ROT_MIN;
import static org.firstinspires.ftc.teamcode.hardware.BotConstants.IMU_CONSTANTS.ROT_PID;
import static org.firstinspires.ftc.teamcode.hardware.BotConstants.IMU_CONSTANTS.STRAFE_PID;
import static org.firstinspires.ftc.teamcode.hardware.BotConstants.IMU_CONSTANTS.STRAIGHT_PID;

/*
Todo: implement color sensor methods
Todo: Integrate all subsystems into the robot class
Todo: Run all systems as state machines, with update commands as transition states
Todo: pass the dashboard object reference to robot, and then retrieve the various subsystem updates
 */

public class Robot extends Subsystem {
    public static final String TAG = "Robot";

    ElapsedTime runtime = new ElapsedTime();

    ArrayList<Subsystem> subsystems = new ArrayList<>();
    private IMU imu = new IMU();
    public Launcher launcher = new Launcher();
    public MecanumDrive drive = new MecanumDrive();
    public Intake intake = new Intake();

    FtcDashboard dashboard;

    private double speed = 1;

    private double reference = 0;
    private double currentX = 0;

    /* Constructor */
    public Robot() {
        subsystems.add(drive);
        subsystems.add(imu);
        subsystems.add(launcher);
        dashboard = FtcDashboard.getInstance();
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
        updates.put("launchSpeed", speed);
        updates.put("Reference", reference);
        updates.put("Distance", currentX);

        return updates;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        for(Subsystem subsystem : subsystems) {
            subsystem.init(hwMap);
        }
        runtime.startTime();
    }

    //region Auton methods
    public void forwardByDistance(double distance) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.startTime();

        MiniPID drivePID = new MiniPID(STRAIGHT_PID.p, STRAIGHT_PID.i, STRAIGHT_PID.d);
        double power;

        List<Double> wheelPositions = drive.getWheelPositions();
        double x = (wheelPositions.get(0) + wheelPositions.get(1)) / 2;
        double initialX = x;

        reference = distance;

        while(Math.abs(distance - x) > 0.2) {
            currentX = x;
            TelemetryPacket packet = new TelemetryPacket();
            packet.putAll(update());
            dashboard.sendTelemetryPacket(packet);

            wheelPositions = drive.getWheelPositions();
            x = (wheelPositions.get(0) + wheelPositions.get(1)) / 2;
            power = drivePID.getOutput(x - initialX, distance - initialX);

            if(Math.abs(power) < 0.12) {
                power *= 0.12/Math.abs(power);
            }

            double[] powers = {-power, -power, -power, -power};
            drive.setMotorPowers(powers);
        }
        stop();
        Log.d(TAG, "Time to Complete Forward Movement: " + runtime.milliseconds());
    }

    public void strafebyDistance(double distance) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.startTime();
        reference = distance;

        MiniPID strafePID = new MiniPID(STRAFE_PID.p, STRAFE_PID.i, STRAFE_PID.d);
        double power;

        List<Double> wheelPositions = drive.getWheelPositions();
        double y = wheelPositions.get(2);
        double initialY = y;

        distance += y;

        while(Math.abs(distance - y) > 0.2) {
            update();
            wheelPositions = drive.getWheelPositions();
            y = wheelPositions.get(2);

            power = strafePID.getOutput(y - initialY, distance - initialY);

            double[] powers = {power, -power, power, -power};
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

        MiniPID imuPID = new MiniPID(ROT_PID.p, ROT_PID.i, ROT_PID.d);
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

            if(Math.abs(temppower) < ROT_MIN) {
                temppower *= ROT_MIN/Math.abs(temppower);
            } else if(Math.abs(temppower) > ROT_MAX) {
                temppower *= ROT_MAX/Math.abs(temppower);
            }

            if(angle == 0) {
                temppower = Math.signum(degrees) * ROT_MIN;
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

    public void decreaseLaunchSpeed() {
        speed -= 0.05;
    }

    public void increaseLaunchSpeed() {
        speed += 0.05;
    }

    public void spinUp() {
        launcher.spinToVel(speed);
    }

    public void spinDown() {
        launcher.spinToVel(0);
    }

    //endregion
}
