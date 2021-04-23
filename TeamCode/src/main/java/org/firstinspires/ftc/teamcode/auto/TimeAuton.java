package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Autonomous
public class TimeAuton extends LinearOpMode
{
    Robot robot = new Robot();
    FtcDashboard dash = FtcDashboard.getInstance();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, dash);

        waitForStart();

        robot.drive.setMotorPowers(new double [] {-0.5, -0.5, -0.5, -0.5});

        sleep(2000);

        robot.drive.setMotorPowers(new double [] {0, 0, 0, 0});

        robot.launcher.setHighSpeed();
        robot.launcher.spinUp();
        sleep(3000);
        for(int i = 0; i < 5; i++) {
            robot.launcher.shoot();
            /*if(isStopRequested()) {
                robot.launcher.resetFlicker();
            }*/
            sleep(1500);
            robot.launcher.resetFlicker();
            sleep(1500);
        }

        robot.launcher.stopMotor();

        robot.drive.setMotorPowers(new double [] {-0.5, -0.5, -0.5, -0.5});

        sleep(800);

        robot.drive.setMotorPowers(new double [] {0, 0, 0, 0});
    }
}
