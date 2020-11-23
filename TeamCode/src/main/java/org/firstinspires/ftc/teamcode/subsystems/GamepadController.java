package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

/*
Motor Arrangement
		Front
		  ^
		  |
	0			3



	1			2
 */
public class GamepadController {
	private Gamepad gamepad1;
	private Gamepad gamepad2;

	/*
	Todo: add support for controller configurations
	Todo: separate constructor for custom configs
	 */
	public GamepadController(Gamepad gamepad1, Gamepad gamepad2) {
		this.gamepad1 = gamepad1;
		this.gamepad2 = gamepad2;
	}

	public double[] getDrivePowers() {
		double drive = gamepad1.left_stick_y;
		double turn = gamepad1.left_stick_x;

		double strafe = gamepad1.right_stick_x;

		double left = drive - turn;
		double right = drive + turn;

		double max = Math.max(Math.abs(left), Math.abs(right));
		if (max > 1)
		{
			left /= max;
			right /= max;
		}

		return new double[] {-left, -right, -strafe};
	}
}
