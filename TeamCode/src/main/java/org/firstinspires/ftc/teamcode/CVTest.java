package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.vision.ballIdentifier;
import org.firstinspires.ftc.teamcode.vision.stoneIdentifier;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvTrackerApiPipeline;

@TeleOp(name="CVTest", group="VegaBot")
public class CVTest extends OpMode {
	// Declare OpMode members.
	private ElapsedTime runtime = new ElapsedTime();
	ExampleHardware robot = new ExampleHardware();

	//cv fields
	OpenCvInternalCamera phoneCam;

	FtcDashboard dashboard = FtcDashboard.getInstance();
	TelemetryPacket packet = new TelemetryPacket();

	int openSoundID;

	@Override
	public void init() {
		runtime.startTime();
		openSoundID = hardwareMap.appContext.getResources().getIdentifier("open", "raw", hardwareMap.appContext.getPackageName());

		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
		phoneCam.openCameraDevice();

		OpenCvTrackerApiPipeline trackerApiPipeline = new OpenCvTrackerApiPipeline();
		stoneIdentifier tracker1 = new stoneIdentifier((new Scalar(235, 213, 52)));
		ballIdentifier tracker2 = new ballIdentifier(new Scalar(255, 255, 255));

		trackerApiPipeline.addTracker(tracker1);
		trackerApiPipeline.addTracker(tracker2);

		phoneCam.setPipeline(trackerApiPipeline);
		phoneCam.setFlashlightEnabled(true);
		phoneCam.setExposureCompensation(0);
		phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT, OpenCvInternalCamera.BufferMethod.DOUBLE);

		for(OpenCvInternalCamera.FrameTimingRange r : phoneCam.getFrameTimingRangesSupportedByHardware()) {
			if(r.max == 30 && r.min == 30) {
				phoneCam.setHardwareFrameTimingRange(r);
				break;
			}
		}
	}

	@Override
	public void start() {
		runtime.reset();
	}

	@Override
	public void loop() {
		int time = (int) runtime.seconds();
		if(time % 3 == 0) {
			SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, openSoundID);
		}
		packet.put("Frame Count", phoneCam.getFrameCount());
		packet.put("FPS", String.format("%.2f", phoneCam.getFps()));
		packet.put("Total frame time ms", phoneCam.getTotalFrameTimeMs());
		packet.put("Pipeline time ms", phoneCam.getPipelineTimeMs());
		packet.put("Overhead time ms", phoneCam.getOverheadTimeMs());
		dashboard.sendTelemetryPacket(packet);
	}

	@Override
	public void stop() {
		SoundPlayer.getInstance().close();
	}
}