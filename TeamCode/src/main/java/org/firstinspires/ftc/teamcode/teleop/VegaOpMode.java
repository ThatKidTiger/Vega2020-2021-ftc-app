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

@TeleOp(name="VegaOpMode", group="VegaBot")
public class VegaOpMode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

	private GamepadController controllers;
    private Robot robot = new Robot();

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private TelemetryPacket packet = new TelemetryPacket();

    private DistanceSensor dist;

    private boolean bPressed = false;
    private boolean spinning = false;

    @Override
    public void init() {
        runtime.startTime();
        robot.init(hardwareMap);

        controllers = new GamepadController(gamepad1, gamepad2);
        dist = hardwareMap.get(DistanceSensor.class, "dist");
    }

    @Override
    public void start() {
        runtime.reset();
    }

    //Todo: figure out how to consolidate telemetry packets for dashboard updates per-loop, probably by returning a mega packet from the robot class.
    @Override
    public void loop() {
        runtime.reset();

        robot.drive.setMotorPowers(controllers.getDrivePowers());

        if(gamepad1.a) {
            robot.rotateByAngle(90);
        }

        if(gamepad1.b && !bPressed) {
            bPressed = true;

            if(spinning) {
                robot.spinDown();
            } else {
                robot.spinUp();
            }

            spinning = !spinning;
        }

        bPressed = gamepad1.b;

        if(gamepad1.dpad_down) {
            robot.decreaseLaunchSpeed();
        } else if(gamepad1.dpad_up) {
            robot.increaseLaunchSpeed();
        }

        /*
        Todo: add remaining subsystems
         */

        packet.put("Time: ", runtime.milliseconds());
        //packet.put("IMU_CONSTANTS Calib", robot.imu.getCalibrationStatus().toString());
        //packet.put("IMU_CONSTANTS Calib", robot.imu.getCalibrationStatus().toString());
        //packet.put("Distance(cm): ", robot.distance.getDistance(DistanceUnit.CM));
        //packet.put("Top Distance(cm): ", robot.topdistance.getDistance(DistanceUnit.CM));
        //packet.put("Angle: ", getOrientation());
        telemetry.addData("Distance: ", "%f", dist.getDistance(DistanceUnit.CM));
        //telemetry.addData("Left R,G,B,A: ", "%d %d %d %d", colSen.red(), colSen.green(), colSen.blue(), colSen.alpha());
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
    }
}