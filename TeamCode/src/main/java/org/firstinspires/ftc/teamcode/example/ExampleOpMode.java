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
		//you can put all this in a subsystem too GamepadController or something like that
		/*
		//because this code tends to get written alot you can separate it into its own subsystem
		double drive = gamepad1.left_stick_y;
		double turn = gamepad1.left_stick_x;

		//makes sure the motor power proportions are preserved
		double FL = drive + turn;
		double BL = drive + turn;
		double FR = drive - turn;
		double BR = drive - turn;

		double max = Math.max(Math.abs(FL), Math.abs(FR));;
		if (max > 1) {
			FL /= max;
			BL /= max;
			FR /= max;
			BR /= max;
		}

		robot.drive.setMotorPowers(FL, BL, FR, BR);

		 */
		//endregion

		//region mecanum / x-drive setup

		//the code for controlling the drive train also changes with different drive trains, another reason
		//you could choose to make its own class.
		//skid-steer setup on left stick
		double drive = gamepad1.left_stick_y;
		double turn = gamepad1.left_stick_x;

		//holonomic / strafe functions are on the right stick
		double strafe = gamepad1.right_stick_x;

		//if the right stick is pressed sideways only (or close enough) strafe
		if(Math.abs(drive) < 0.1 && Math.abs(strafe) > 0.1) {
			//remember to make some kind of point here about hiding implementation and limiting powers
			robot.drive.strafe(strafe);
		}

		//makes sure the motor power proportions are preserved
		//depends how you align your wheels(very important for mecanum)
		//in this scenario FR and BR motors have right-vectored wheels
		//draw both and demonstrate interchangeability
		double FL = drive - turn + strafe;
		double BL = drive - turn - strafe;
		double FR = drive + turn - strafe;
		double BR = drive + turn + strafe;

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
		//endregion

		//region h-drive
		/*
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
		double L = drive + turn;
		double R = drive - turn;
		double S = strafe;

		double maxb = Math.max(Math.abs(L), Math.abs(R));
		double max = Math.max(Math.abs(S), maxb);
		if (max > 1)
		{
			L /= max;
			R /= max;
			S /= max;
		}

		robot.drive.setMotorPowers(L, R, S);
		*/
		//endregion
	}
}
