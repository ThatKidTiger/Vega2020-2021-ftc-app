package org.firstinspires.ftc.teamcode.example;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain implements Subsystem{
	private static final String TAG = "MecanumDrive";
	/*
	Motor Arrangement
			Front
			  ^
			  |
		0			3



		1			2
	 */

	//hardware object references
	private DcMotor frontLeft;
	private DcMotor backLeft;
	private DcMotor backRight;
	private DcMotor frontRight;

	public Drivetrain() {

	}

	//initialization, retrieves necessary hardware references and sets initial values
	@Override
	public void init(HardwareMap hwMap) {
		frontLeft = hwMap.get(DcMotor.class, "frontLeft"); //<- the names that we are searching for
		backLeft = hwMap.get(DcMotor.class, "backLeft");
		backRight = hwMap.get(DcMotor.class, "backRight");
		frontRight = hwMap.get(DcMotor.class, "frontRight");

		//make motors all spin forward at positive power
		backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		Log.d(TAG, "Initialization Complete");
	}

	//most fundamental, basic functionality - "set the motor powers"
	public void setMotorPowers(double fL, double bL, double bR, double fR) {
		frontLeft.setPower(fL);
		backLeft.setPower(bL);
		backRight.setPower(bR);
		frontRight.setPower(fR);
	}

	//when adding a function to a subsystem, ask yourself, is this going to be
	//implemented differently somewhere else?
	//is strafe always going to have the same functionality (input -> output) no matter what?
	public void strafe(double power) {
		Log.d(TAG, String.format("Strafing " + (power > 0 ? "right" : "left") + " at power %f.2", Math.abs(power)));
		setMotorPowers(power, -power, power, -power);
	}
}
