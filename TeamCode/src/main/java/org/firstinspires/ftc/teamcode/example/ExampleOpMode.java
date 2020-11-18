package org.firstinspires.ftc.teamcode.example;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.GamepadController;


//informs the compiler to make this teleop accessible in the app selection screen
@TeleOp(name="ExampleOpMode", group="VegaBot")
public class ExampleOpMode extends OpMode {
	private Robot robot = new Robot();

	//what happens when you press init (before the timer starts)
	@Override
	public void init() {
		robot.init(hardwareMap);
	}

	//what happens when you press start (when the timer starts)
	@Override
	public void start() {
	}

	//loop that repeats throughout the teleop game period
	@Override
	public void loop() {
		//region basic skid-steer setup
		//skid-steer setup on left stick
		double drive = gamepad1.left_stick_y;
		double turn = gamepad1.left_stick_x;

		//makes sure the motor power proportions are preserved
		double FL = drive - turn;
		double BL = drive - turn;
		double FR = drive + turn;
		double BR = drive + turn;

		double maxf = Math.max(Math.abs(FL), Math.abs(FR));
		double maxb = Math.max(Math.abs(BL), Math.abs(BR));
		double max = Math.max(maxf, maxb);
		if (max > 1)
		{
			FL /= max;
			BL /= max;
			FR /= max;
			BR /= max;
		}

		//region mecanum setup
		/*
		//skid-steer setup on left stick
		double drive = gamepad1.left_stick_y;
		double turn = gamepad1.left_stick_x;

		//holonomic / strafe functions are on the right stick
		double strafe = gamepad1.right_stick_x;

		//if the right stick is pressed sideways only (or close enough) strafe
		if(Math.abs(drive) < 0.1) {
			//remember to make some kind of point here about hiding implementation and limiting powers
			robot.drive.strafe(turn);
		}

		//makes sure the motor power proportions are preserved
		double FL = drive - turn - strafe;
		double BL = drive - turn + strafe;
		double FR = drive + turn + strafe;
		double BR = drive + turn - strafe;

		double maxf = Math.max(Math.abs(FL), Math.abs(FR));
		double maxb = Math.max(Math.abs(BL), Math.abs(BR));
		double max = Math.max(maxf, maxb);
		if (max > 1)
		{
			FL /= max;
			BL /= max;
			FR /= max;
			BR /= max;
		}

		robot.drive.setMotorPowers(FL, BL, BR, FR);
		*/
		//endregion
	}
}
