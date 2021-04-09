	package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/*
Todo: Document
Todo: Read mecanum kinematic analysis
Todo: Find out why the motor powers need to be reversed
Todo: Implement rotateToOrientation command
*/

//deprecated, use roadrunner mecanum class
public class MecanumDrive extends Subsystem {
	public static double TICKS_PER_REV = 8192;
	public static double WHEEL_RADIUS = .689; // in
	public static double GEAR_RATIO = 1;

	private static final String TAG = "MecanumDrive";

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

		updates.put("leftEncoder", leftEncoder.getCurrentPosition());
		updates.put("rightEncoder", rightEncoder.getCurrentPosition());
		updates.put("frontEncoder", frontEncoder.getCurrentPosition());

		return updates;
	}

	public MecanumDrive() {

	}

	@Override
	public void init(HardwareMap hwMap) {
		for(int i = 0; i < 4; i++) {
			motors[i] = hwMap.get(DcMotorEx.class, motorNames[i]);
		}

		motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
		motors[1].setDirection(DcMotorSimple.Direction.REVERSE);

		leftEncoder = new Encoder(hwMap.get(DcMotorEx.class, "backLeft"));
		rightEncoder = new Encoder(hwMap.get(DcMotorEx.class, "frontRight"));
		frontEncoder = new Encoder(hwMap.get(DcMotorEx.class, "backRight"));

		leftEncoder.setDirection(Encoder.Direction.REVERSE);
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

	//return to this after understanding kinematic analysis
	/*converting unit circle gamepad input into polar coordinates to power + direction
	r = (sqrt(arccos(x)^2 + arcsin(y)^2), theta = arcsin(y)
	public void drivewithGamepad(double x, double y) {
		double r = sqrt(pow(acos(x), 2) + pow(asin(y),2 ));
		double theta = asin(y);
	}

	public void drivewithVelocity() {

	}
	*/
}
