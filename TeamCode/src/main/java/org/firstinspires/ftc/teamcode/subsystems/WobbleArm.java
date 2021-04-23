package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;
import java.util.Map;

public class WobbleArm extends Subsystem {
    public static final String TAG = "WobbleArm";

    public Servo wobbleArm;
    public Servo wobbleGrabber;

    public WobbleArm() {

    }

    @Override
    public Map<String, Object> update() {
        Map<String, Object> updates = new HashMap<>();
        updates.put("ServoPosition", wobbleArm.getPosition());
        return updates;
    }

    public void init(HardwareMap hwMap, FtcDashboard dash) {
        this.dash = dash;

        wobbleArm = hwMap.get(Servo.class, "wobbleArm");
        wobbleGrabber = hwMap.get(Servo.class, "wobbleGrabber");
    }

    public void wobbleUp() {
        wobbleArm.setPosition(0);
    }

    public void wobbleDown() {
        wobbleArm.setPosition(0.5);
    }

    public void wobbleClose() {
        wobbleGrabber.setPosition(0.9);
    }

    public void wobbleOpen() {
        wobbleGrabber.setPosition(0);
    }
}
