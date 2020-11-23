package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.stormbots.MiniPID;

import org.firstinspires.ftc.teamcode.subsystems.IMU;
import org.firstinspires.ftc.teamcode.subsystems.HDrive;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.hardware.BotConstants.IMU_CONSTANTS.ROT_MAX;
import static org.firstinspires.ftc.teamcode.hardware.BotConstants.IMU_CONSTANTS.ROT_MIN;
import static org.firstinspires.ftc.teamcode.hardware.BotConstants.IMU_CONSTANTS.ROT_PID;

/*
Todo: implement color sensor methods
Todo: Integrate all subsystems into the robot class
Todo: Run all systems as state machines, with update commands as transition states
Todo: pass the dashboard object reference to robot, and then retrieve the various subsystem updates
 */

public class Robot {
    public static final String TAG = "Robot";
    ElapsedTime runtime = new ElapsedTime();

    ArrayList<Subsystem> subsystems = new ArrayList<Subsystem>();
    private IMU imu = new IMU();
    public Launcher launcher = new Launcher();
    public HDrive drive = new HDrive();

    FtcDashboard dashboard;
    TelemetryPacket packet;

    /* Constructor */
    public Robot() {
        subsystems.add(drive);
        subsystems.add(imu);
        subsystems.add(launcher);
        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();
    }

    public void update() {
        Map<String, Object> updates = new HashMap<>();
        for(Subsystem subsystem : subsystems) {
            Map<String, Object> updatePacket = subsystem.update();
            if (updatePacket != null) {
                updates.putAll(updatePacket);
            }
        }
        packet.putAll(updates);
        dashboard.sendTelemetryPacket(packet);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        for(Subsystem subsystem : subsystems) {
            subsystem.init(hwMap);
        }
        runtime.startTime();
    }

    //region Auton methods
    public void forward(double power) {
        Log.d(TAG, String.format("Moving " + (power > 0 ? "forward" : "backward") + " at power %f.2", Math.abs(power)));
        double[] powers = {power, power, 0};
        drive.setMotorPowers(powers);
    }

    public void strafe(double power) {
        Log.d(TAG, String.format("Strafing " + (power > 0 ? "right" : "left") + " at power %f.2", Math.abs(power)));
        double[] powers = {0, 0, power};
        drive.setMotorPowers(powers);
    }

    public void rotateByPower(double power) {
        Log.d(TAG, String.format("Rotating " + (power < 0 ? "counterclockwise" : "clockwise") + " at power %f.2", Math.abs(power)));
        double[] powers = {power, -power, 0};
        drive.setMotorPowers(powers);
    }

    //uses PID controller to precisely turn a certain number of degrees
    public void rotateByAngle(double degrees) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.startTime();
        MiniPID imuPID = new MiniPID(ROT_PID.p, ROT_PID.i, ROT_PID.d);
        double leftPower, rightPower, temppower;

        imu.resetAngularDistance();
        sleep(250);

        double lastAngle = 0;
        double angle = imu.getAngularDistance();

        //rotates until the imu returns that the robot is within a margin of error
        while(Math.abs(degrees - angle) > 0.1 || Math.abs(degrees - lastAngle) > 0.1) {
            packet.put("Reference", degrees);
            update();
            lastAngle = angle;
            angle = imu.getAngularDistance();
            temppower = imuPID.getOutput(angle, degrees);

            if(Math.abs(temppower) < ROT_MIN) {
                temppower *= ROT_MIN/Math.abs(temppower);
            } else if(Math.abs(temppower) > ROT_MAX) {
                temppower *= ROT_MAX/Math.abs(temppower);
            }

            if(angle == 0) {
                temppower = Math.signum(degrees) * ROT_MIN;
            }

            leftPower = -temppower;
            rightPower = temppower;

            double[] powers = {leftPower, rightPower, 0};
            drive.setMotorPowers(powers);
        }
        stop();
        Log.d(TAG, "Time to Complete Rotation: " + runtime.milliseconds());
    }

    public void stop() {
        double[] powers = {0, 0, 0};
        drive.setMotorPowers(powers);
    }
    //endregion
}
