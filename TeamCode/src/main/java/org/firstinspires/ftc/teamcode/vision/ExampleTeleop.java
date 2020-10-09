/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
@Disabled
public class ExampleTeleop extends OpMode
{
	// Declare OpMode members.
	ElapsedTime runtime = new ElapsedTime();

	//example straightforward hardware reference retrieval
	DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
	DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
	DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
	DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");

	DcMotor gripper = hardwareMap.get(DcMotor.class, "gripper");

	Servo exampleServo = hardwareMap.get(Servo.class, "exampleServo");

	CRServo exampleCRServo = hardwareMap.get(CRServo.class, "exampleCRServo");

	ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colSen");

	Rev2mDistanceSensor distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distSen");
	/*
	 * Code to run ONCE when the driver hits INIT
	 */
	@Override
	public void init() {
		runtime.startTime();
		telemetry.addData("Status", "Initialized");

		frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
		backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

		// Tell the driver that initialization is complete.
		telemetry.addData("Status", "Initialized");
	}

	/*
	 * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
	 */
	@Override
	public void init_loop() {
	}

	/*
	 * Code to run ONCE when the driver hits PLAY
	 */
	@Override
	public void start() {
	}

	/*
	 * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
	 */
	@Override
	public void loop() {
		//boolean vs double controller input
		//basic arcade drive layout
		double power = gamepad1.left_stick_y;
		double turn = gamepad1.left_stick_x;

		frontLeft.setPower(power + turn);
		backLeft.setPower(power + turn);
		frontRight.setPower(power - turn);
		backRight.setPower(power - turn);

		if(gamepad1.a) {
			exampleServo.setPosition(90);
		} else {
			exampleServo.setPosition(0);
		}

		exampleCRServo.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

		if(distanceSensor.getDistance(DistanceUnit.CM) < 5) {
			frontLeft.setPower(0);
			backLeft.setPower(0);
			frontRight.setPower(0);
			backRight.setPower(0);
		}

		colorSensor.red();
		colorSensor.green();
		colorSensor.blue();
		colorSensor.alpha();
		colorSensor.argb();

		if(colorSensor.red() > 200) {
			runtime.reset();
			gripper.setPower(0.5);
			while(runtime.seconds() < 1) {}
			gripper.setPower(0);
		}
	}

	/*
	 * Code to run ONCE after the driver hits STOP
	 */
	@Override
	public void stop() {
	}

}
