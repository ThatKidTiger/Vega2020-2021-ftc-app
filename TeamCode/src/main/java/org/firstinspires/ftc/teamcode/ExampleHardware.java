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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ExampleHardware
{

    /*TO.DO:
    *Write Subsystem Implementation with inspiration from kyle and Noah
    * Pass the hardwaremap object to the individual subsystems for motor identification
    * and object initialization
    *
    * Bulk handle subsystems with the handler, update the subsystems in bulk
     */
    /* Public OpMode members. */
    public DcMotor backLeft; // hub 3 port 0
    public DcMotor backRight; // hub 3 port 1
    public DcMotor frontLeft; // hub 3 port 3
    public DcMotor frontRight; //hub 3 port 2
    public DcMotor latch; //hub 2 port 0
    public DcMotor lift; //hub 2 port 1
    public DcMotor gripper; //hub 2 port 2
    public BNO055IMU imu;
    public ColorSensor colLeft; //hub 2 port 0
    public ColorSensor colRight; //hub 2 port 2
    public DistanceSensor distance; //hub 2 port 1
    public DistanceSensor topdistance; //hub 3 port 0

    /* local OpMode members. */
    HardwareMap hwMap =  null;

    /* Constructor */
    public ExampleHardware() {}

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");

        // Set all drive motors to run without encoders.
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //reverse left drive motors
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        latch = hwMap.get(DcMotor.class, "latch");
        lift = hwMap.get(DcMotor.class, "lift");

        /*lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setDirection(DcMotor.Direction.REVERSE);*/

        gripper = hwMap.get(DcMotor.class, "gripper");

        //define sensors
        imu = hwMap.get(BNO055IMU.class, "imu");
        colLeft = hwMap.get(ColorSensor.class, "colLeft");
        colRight = hwMap.get(ColorSensor.class, "colRight");
        distance = hwMap.get(DistanceSensor.class, "distance");
        topdistance = hwMap.get(DistanceSensor.class, "topdistance");

        //Configure IMU_CONSTANTS Sensor
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.calibrationDataFile = "IMUCalibration.json";

        imu.initialize(parameters);

        // Set all motors to zero power
        backLeft.setPower(0); //hub 1 0
        backRight.setPower(0); //hub 1 2
        frontLeft.setPower(0); //hub 1 1
        frontRight.setPower(0); //hub 1 3
    }
}

