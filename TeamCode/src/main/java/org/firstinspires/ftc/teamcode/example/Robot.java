package org.firstinspires.ftc.teamcode.example;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

//this class is your portal for interacting with all other hardware elements
//and also helps when we start to centralize telemetry updates and diagnostic information
public class Robot {
	public static final String TAG = "Robot";
	ElapsedTime runtime = new ElapsedTime();

	ArrayList<Subsystem> subsystems = new ArrayList<Subsystem>();
	public MecanumDrive drive = new MecanumDrive();

	/* Constructor */
	public Robot() {
		subsystems.add(drive);
	}

	/* Initialize standard Hardware interfaces */
	public void init(HardwareMap hwMap) {
		//when you have multiple classes that implement the subsystem interface, you can initialize them all at once
		for(Subsystem subsystem : subsystems) {
			subsystem.init(hwMap);
		}
		runtime.startTime();
		Log.d(TAG, "Initialization Complete");
	}
}
