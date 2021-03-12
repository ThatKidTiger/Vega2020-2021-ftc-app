package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.subsystems.GamepadController;

import java.util.Map;

@TeleOp(name="VegaOpMode", group="VegaBot")
public class VegaOpMode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

	private GamepadController controllers;
    private Robot robot = new Robot();

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private TelemetryPacket packet = new TelemetryPacket();

    private boolean bPressed = false;
    private boolean spinning = false;

    private boolean ddownPressed = false;
    private boolean dupPressed = false;

    @Override
    public void init() {
        runtime.startTime();
        robot.init(hardwareMap);

        controllers = new GamepadController(gamepad1, gamepad2);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    //Todo: figure out how to consolidate telemetry packets for dashboard updates per-loop, probably by returning a mega packet from the robot class.
    @Override
    public void loop() {
        Map<String, Object> updatePacket = robot.update();

        runtime.reset();

        robot.drive.setMotorPowers(controllers.getDrivePowers());

        if(gamepad1.a) {
            robot.forwardByDistance(4);
        }

        if(gamepad1.x) {
            robot.wobbleUp();
        }

        if(gamepad1.y) {
            robot.wobbleDown();
        }

        if(gamepad1.b) {
            if(bPressed != true) {
                spinning = !spinning;
            }
            bPressed = true;
        } else {
            bPressed = false;
        }

        if(spinning) {
            robot.spinUp();
        } else {
            robot.spinDown();
        }

        if(gamepad1.dpad_down) {
            if(!ddownPressed) {
                robot.decreaseLaunchSpeed();
            }
            ddownPressed = true;
        } else {
            ddownPressed = false;
        }

        if(gamepad1.dpad_up) {
            if(!dupPressed) {
                robot.increaseLaunchSpeed();
            }
            dupPressed = true;
        } else {
            dupPressed = false;
        }

        /*
        Todo: add remaining subsystems
         */

        updatePacket.put("Time: ", runtime.milliseconds());
        //packet.put("IMU_CONSTANTS Calib", robot.imu.getCalibrationStatus().toString());
        //packet.put("IMU_CONSTANTS Calib", robot.imu.getCalibrationStatus().toString());
        packet.putAll(updatePacket);
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
    }
}