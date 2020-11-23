package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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
        runtime.reset();

        robot.drive.setMotorPowers(controllers.getDrivePowers());

        if(gamepad1.a) {
            robot.rotateByAngle(90);
        }

        if(gamepad1.b) {
            robot.launcher.spinToVel(0.75);
        } else {
            robot.launcher.spinToVel(0);
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
        //telemetry.addData("Left R,G,B,A: ", "%d %d %d %d", robot.colLeft.red(), robot.colLeft.green(), robot.colLeft.blue(), robot.colLeft.alpha());
        //telemetry.addData("Right R,G,B,A: ", "%d, %d, %d, %d", robot.colRight.red(), robot.colRight.green(), robot.colRight.blue(), robot.colRight.alpha());
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
    }
}