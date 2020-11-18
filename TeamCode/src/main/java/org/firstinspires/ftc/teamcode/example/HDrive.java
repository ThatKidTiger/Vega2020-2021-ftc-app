package org.firstinspires.ftc.teamcode.example;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HDrive implements Subsystem{
	private static final String TAG = "HDrive";
	/*
	Motor Arrangement
			Front
			  ^
			  |


			  2

		0			1
	 */

	//hardware object references
	private DcMotor right;
	private DcMotor left;
	private DcMotor strafe;

	public HDrive() {

	}

	//initialization, retrieves necessary hardware references and sets initial values
	@Override
	public void init(HardwareMap hwMap) {
		right = hwMap.get(DcMotor.class, "frontLeft"); //<- the names that we are searching for
		left = hwMap.get(DcMotor.class, "backLeft");
		strafe = hwMap.get(DcMotor.class, "backRight");

		//make motors all spin forward at positive power
		left.setDirection(DcMotorSimple.Direction.REVERSE);
		Log.d(TAG, "Initialization Complete");
	}

	//most fundamental, basic functionality - "set the motor powers"
	public void setMotorPowers(double l, double r, double s) {
		left.setPower(l);
		right.setPower(r);
		strafe.setPower(s);
	}

	//when adding a function to a subsystem, ask yourself, is this going to be
	//implemented differently somewhere else?
	//is strafe always going to have the same functionality (input -> output) no matter what?
	public void strafe(double power) {
		Log.d(TAG, String.format("Strafing " + (power > 0 ? "right" : "left") + " at power %f.2", Math.abs(power)));
		strafe.setPower(power);
	}
}
