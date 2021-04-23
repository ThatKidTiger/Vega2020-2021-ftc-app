package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.subsystems.GamepadController;

import java.util.Map;

@TeleOp(name="VegaOpMode", group="VegaBot")
public class VegaOpMode extends OpMode
{
    @Config
    public enum TEST_DISTANCE {;
        public static int inches = 12;
    }

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

	private GamepadController controllers;
    private Robot robot = new Robot();

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private TelemetryPacket packet = new TelemetryPacket();

    private boolean bPressed = false;
    private boolean spinning = false;

    private boolean leftBumperPressed = false;
    private boolean rightBumperPressed = false;

    private boolean yPressed = false;
    private boolean servoExtended = false;

    private boolean xPressed = false;
    private boolean wobbleClosed = false;

    @Override
    public void init() {
        runtime.startTime();
        robot.init(hardwareMap, dashboard);

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

        if(gamepad1.x) {
            robot.forwardByDistance(TEST_DISTANCE.inches);
        }

        /*if(gamepad1.x) {
            robot.strafebyDistance(7.5);
        }*/

        if(gamepad1.a) {
            robot.intake.spinToVel(-1);
        } else {
            robot.intake.spinToVel(0);
        }

        if(gamepad1.y) {
            if(yPressed != true) {
                servoExtended = !servoExtended;
            }
            yPressed = true;
        } else {
            yPressed = false;
        }

        if(servoExtended) {
            robot.launcher.shoot();
        } else {
            robot.launcher.resetFlicker();
        }

        if(gamepad1.x) {
            if(xPressed != true) {
                wobbleClosed = !wobbleClosed;
            }
            xPressed = true;
        } else {
            xPressed = false;
        }

        if(wobbleClosed) {
            robot.wobble.wobbleClose();
        } else {
            robot.wobble.wobbleOpen();
        }

        if(gamepad1.b) {
            if(bPressed != true) {
                spinning = !spinning;
                robot.launcher.resetFlicker();
                servoExtended = false;
            }
            bPressed = true;
        } else {
            bPressed = false;
        }

        if(spinning) {
            robot.launcher.spinUp();
        } else {
            robot.launcher.stopMotor();
        }

        if(gamepad1.left_bumper) {
            if(!leftBumperPressed) {
                //robot.decreaseLaunchSpeed();
                robot.launcher.setPowerSpeed();
            }
            leftBumperPressed = true;
        } else {
            leftBumperPressed = false;
        }

        if(gamepad1.right_bumper) {
            if(!rightBumperPressed) {
                //robot.increaseLaunchSpeed();
                robot.launcher.setHighSpeed();
            }
            rightBumperPressed = true;
        } else {
            rightBumperPressed = false;
        }

        if(gamepad1.left_trigger > 0) {
            robot.wobble.wobbleUp();
        }

        if(gamepad1.right_trigger > 0) {
            robot.wobble.wobbleDown();
        }

        /*
        Todo: add remaining subsystems
         */

        updatePacket.put("Time: ", runtime.milliseconds());
        telemetry.addData("Z: ", robot.imu.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.update();
        //packet.put("IMU_CONSTANTS Calib", robot.imu.getCalibrationStatus().toString());
        //packet.put("IMU_CONSTANTS Calib", robot.imu.getCalibrationStatus().toString());
        packet.putAll(updatePacket);
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
    }
}