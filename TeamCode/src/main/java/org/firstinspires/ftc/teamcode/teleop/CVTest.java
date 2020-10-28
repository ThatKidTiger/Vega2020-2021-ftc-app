package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ExampleHardware;
import org.firstinspires.ftc.teamcode.vision.ballIdentifier;
import org.firstinspires.ftc.teamcode.vision.stoneIdentifier;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvTracker;
import org.openftc.easyopencv.OpenCvTrackerApiPipeline;

import static java.lang.Thread.sleep;

@TeleOp(name="CVTest", group="VegaBot")
public class CVTest extends LinearOpMode
{
	OpenCvCamera phoneCam;
	OpenCvTrackerApiPipeline trackerApiPipeline;
	FtcDashboard dashboard = FtcDashboard.getInstance();
	TelemetryPacket packet = new TelemetryPacket();

	@Override
	public void runOpMode() throws InterruptedException {

		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
		phoneCam.openCameraDevice();

		/**
		 * Create an instance of the {@link OpenCvTrackerApiPipeline}
		 * pipeline (included with EasyOpenCV), and tell the camera
		 * to use it.
		 */
		trackerApiPipeline = new OpenCvTrackerApiPipeline();
		phoneCam.setPipeline(trackerApiPipeline);

		phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

		/*
		 * Create some trackers we want to run
		 */
		stoneIdentifier tracker1 = new stoneIdentifier((new Scalar(235, 213, 52)));
		//ballIdentifier tracker2 = new ballIdentifier(new Scalar(255, 255, 255));
		//UselessColorBoxDrawingTracker tracker1 = new UselessColorBoxDrawingTracker((new Scalar(235, 213, 52)));
		//UselessColorBoxDrawingTracker tracker2 = new UselessColorBoxDrawingTracker(new Scalar(255, 255, 255));

		/*
		 * Add those trackers to the pipeline. All trackers added to the
		 * trackerApiPipeline will be run upon receipt of a frame from the
		 * camera. Note: the trackerApiPipeline will handle switching
		 * the viewport view on tap between the output of each of the trackers
		 * for you.
		 */
		trackerApiPipeline.addTracker(tracker1);
		//trackerApiPipeline.addTracker(tracker2);

		waitForStart();

		while (opModeIsActive())
		{
			/*
			 * If you later want to stop running a tracker on each frame,
			 * you can remove it from the trackerApiPipeline like so:
			 */
			//trackerApiPipeline.removeTracker(tracker1);
			packet.put("Frame Count", phoneCam.getFrameCount());
			packet.put("FPS", String.format("%.2f", phoneCam.getFps()));
			packet.put("Total frame time ms", phoneCam.getTotalFrameTimeMs());
			packet.put("Pipeline time ms", phoneCam.getPipelineTimeMs());
			packet.put("Overhead time ms", phoneCam.getOverheadTimeMs());
			dashboard.sendTelemetryPacket(packet);
			sleep(100);
		}
	}

	class UselessColorBoxDrawingTracker extends OpenCvTracker
	{
		Scalar color;

		UselessColorBoxDrawingTracker(Scalar color)
		{
			this.color = color;
		}

		@Override
		public Mat processFrame(Mat input)
		{
			Imgproc.rectangle(
					input,
					new Point(
							input.cols()/4,
							input.rows()/4),
					new Point(
							input.cols()*(3f/4f),
							input.rows()*(3f/4f)),
					color, 4);

			return input;
		}
	}
}