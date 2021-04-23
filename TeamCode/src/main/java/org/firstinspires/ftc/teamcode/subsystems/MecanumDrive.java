	package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/*
Todo: Read mecanum kinematic analysis
Todo: Set conventional wheel directions and figure out proper power negation
Todo: Implement rotateToOrientation command
*/

public class MecanumDrive extends Subsystem {
	private static final String TAG = "MecanumDrive";

	public static double TICKS_PER_REV = 8192;
	public static double WHEEL_RADIUS = .689; // in
	public static double GEAR_RATIO = 1;

	private ElapsedTime timer = new ElapsedTime();

	private double lastTime = 0;

	private double[] lastPositions = new double[] {0,0,0};
 	/*
	Motor Arrangement
			Front
			  ^
			  |
		3			1



		0			2
	 */

	//hardware object references
	private DcMotorEx[] motors = new DcMotorEx[4];

	//map to update telemetry stats to
	private HashMap<String, Object> updates = new HashMap<>();

	//names to search the hardware map for
	private String[] motorNames = {"frontLeft", "backLeft", "backRight", "frontRight"};

	private Encoder leftEncoder;
	private Encoder rightEncoder;
	private Encoder frontEncoder;

	@Override
	public Map<String, Object> update() {
		for(int i = 0; i < motors.length; i++) {
			updates.put(motorNames[i], motors[i].getVelocity());
		}

		updates.put("leftEncoder", encoderTicksToInches(leftEncoder.getCurrentPosition()));
		updates.put("rightEncoder", encoderTicksToInches(rightEncoder.getCurrentPosition()));
		updates.put("frontEncoder", encoderTicksToInches(frontEncoder.getCurrentPosition()));

		return updates;
	}

	public MecanumDrive() {

	}

	@Override
	public void init(HardwareMap hwMap, FtcDashboard dash) {
		this.dash = dash;

		for(int i = 0; i < 4; i++) {
			motors[i] = hwMap.get(DcMotorEx.class, motorNames[i]);
		}

		for(DcMotorEx motor : motors) {
			motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		}

		motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
		motors[1].setDirection(DcMotorSimple.Direction.REVERSE);

		leftEncoder = new Encoder(hwMap.get(DcMotorEx.class, "backLeft"));
		rightEncoder = new Encoder(hwMap.get(DcMotorEx.class, "frontRight"));
		frontEncoder = new Encoder(hwMap.get(DcMotorEx.class, "backRight"));

		rightEncoder.setDirection(Encoder.Direction.REVERSE);

		timer.startTime();

		Log.d(TAG, "Initialization Complete");
	}

	public void setMotorPowers(double[] powers) {
		for(int i = 0; i < motors.length; i++) {
			motors[i].setPower(-powers[i]);
		}

		updates.put("FL", powers[0]);
		updates.put("BL", powers[1]);
		updates.put("BR", powers[2]);
		updates.put("FR", powers[3]);
	}

	public static double encoderTicksToInches(double ticks) {
		return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
	}

	@NonNull
	public List<Double> getWheelPositions() {
		lastPositions[0] = encoderTicksToInches(leftEncoder.getCurrentPosition());
		lastPositions[1] = encoderTicksToInches(leftEncoder.getCurrentPosition());
		lastPositions[2] = encoderTicksToInches(leftEncoder.getCurrentPosition());
		return Arrays.asList(
			lastPositions[0],
			lastPositions[1],
			lastPositions[2]
		);
	}

	public List<Double> getWheelVelocities() {
		double time = timer.milliseconds();
		double[] previous = lastPositions.clone();
		getWheelPositions();
		return Arrays.asList(
			lastPositions[0] - previous[0] / ((time - lastTime) / 1000),
			lastPositions[1] - previous[1] / ((time - lastTime) / 1000),
			lastPositions[2] - previous[2] / ((time - lastTime) / 1000)
		);
	}
}
