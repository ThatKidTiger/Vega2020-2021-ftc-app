package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
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

		RingPipeline ringPipeline = new RingPipeline();
		phoneCam.setPipeline(ringPipeline);

		phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

		phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
		{
			@Override
			public void onOpened()
			{
				phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
			}
		});

		waitForStart();

		while (opModeIsActive())
		{

			packet.put("Frame Count", phoneCam.getFrameCount());
			packet.put("FPS", String.format("%.2f", phoneCam.getFps()));
			packet.put("Total frame time ms", phoneCam.getTotalFrameTimeMs());
			packet.put("Pipeline time ms", phoneCam.getPipelineTimeMs());
			packet.put("Overhead time ms", phoneCam.getOverheadTimeMs());
			telemetry.addData("Ring Configuration", ringPipeline.getAnalysis().name());
			telemetry.addData("Average", ringPipeline.getMin());
			telemetry.update();
			dashboard.sendTelemetryPacket(packet);
			sleep(50);
		}
	}

	public static class RingPipeline extends OpenCvPipeline
	{
		public enum RingConfig
		{
			ZERO,
			ONE,
			FOUR
		}

		static final Scalar BLUE = new Scalar(0, 0, 255);
		static final Scalar GREEN = new Scalar(0, 255, 0);


		static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(109,178);
		static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(109,98);
		static final int REGION1_WIDTH = 100;
		static final int REGION1_HEIGHT = 40;
		static final int REGION2_WIDTH = 100;
		static final int REGION2_HEIGHT = 120;


		Point region1_pointA = new Point(
				REGION1_TOPLEFT_ANCHOR_POINT.x,
				REGION1_TOPLEFT_ANCHOR_POINT.y);
		Point region1_pointB = new Point(
				REGION1_TOPLEFT_ANCHOR_POINT.x + REGION1_WIDTH,
				REGION1_TOPLEFT_ANCHOR_POINT.y + REGION1_HEIGHT);
		Point region2_pointA = new Point(
				REGION2_TOPLEFT_ANCHOR_POINT.x,
				REGION2_TOPLEFT_ANCHOR_POINT.y);
		Point region2_pointB = new Point(
				REGION2_TOPLEFT_ANCHOR_POINT.x + REGION2_WIDTH,
				REGION2_TOPLEFT_ANCHOR_POINT.y + REGION2_HEIGHT);


		Mat region1_Cb, region2_Cb;
		Mat YCrCb = new Mat();
		Mat Cb = new Mat();
		int avg1, avg2;

		private volatile int min = 0;
		private volatile RingConfig config = RingConfig.ZERO;

		void inputToCb(Mat input)
		{
			Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
			Core.extractChannel(YCrCb, Cb, 2);
		}

		@Override
		public void init(Mat firstFrame)
		{

			inputToCb(firstFrame);


			region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
			region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
		}

		@Override
		public Mat processFrame(Mat input)
		{
			/*
			 * Overview of what we're doing:
			 *
			 * We first convert to YCrCb color space, from RGB color space.
			 * Why do we do this? Well, in the RGB color space, chroma and
			 * luma are intertwined. In YCrCb, chroma and luma are separated.
			 * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
			 * are Y, the luma channel (which essentially just a B&W image), the
			 * Cr channel, which records the difference from red, and the Cb channel,
			 * which records the difference from blue. Because chroma and luma are
			 * not related in YCrCb, vision code written to look for certain values
			 * in the Cr/Cb channels will not be severely affected by differing
			 * light intensity, since that difference would most likely just be
			 * reflected in the Y channel.
			 *
			 * After we've converted to YCrCb, we extract just the 2nd channel, the
			 * Cb channel. We do this because stones are bright yellow and contrast
			 * STRONGLY on the Cb channel against everything else, including SkyStones
			 * (because SkyStones have a black label).
			 *
			 * We then take the average pixel value of 3 different regions on that Cb
			 * channel, one positioned over each stone. The brightest of the 3 regions
			 * is where we assume the SkyStone to be, since the normal stones show up
			 * extremely darkly.
			 *
			 * We also draw rectangles on the screen showing where the sample regions
			 * are, as well as drawing a solid rectangle over top the sample region
			 * we believe is on top of the SkyStone.
			 *
			 * In order for this whole process to work correctly, each sample region
			 * should be positioned in the center of each of the first 3 stones, and
			 * be small enough such that only the stone is sampled, and not any of the
			 * surroundings.
			 */

			/*
			 * Get the Cb channel of the input frame after conversion to YCrCb
			 */
			inputToCb(input);

			/*
			 * Compute the average pixel value of each submat region. We're
			 * taking the average of a single channel buffer, so the value
			 * we need is at index 0. We could have also taken the average
			 * pixel value of the 3-channel image, and referenced the value
			 * at index 2 here.
			 */
			avg1 = (int) Core.mean(region1_Cb).val[0];
			avg2 = (int) Core.mean(region2_Cb).val[0];


			Imgproc.rectangle(
					input, // Buffer to draw on
					region1_pointA, // First point which defines the rectangle
					region1_pointB, // Second point which defines the rectangle
					BLUE, // The color the rectangle is drawn in
					2); // Thickness of the rectangle lines


			Imgproc.rectangle(
					input, // Buffer to draw on
					region2_pointA, // First point which defines the rectangle
					region2_pointB, // Second point which defines the rectangle
					BLUE, // The color the rectangle is drawn in
					2); // Thickness of the rectangle lines


			min = Math.min(avg1, avg2);

			/*
			 * Now that we found the max, we actually need to go and
			 * figure out which sample region that value was from
			 */
			if(min > 95) {
				config = RingConfig.ZERO;
			}
			else if(min == avg1) // Was it from region 1?
			{
				config = RingConfig.ONE; // Record our analysis

				/*
				 * Draw a solid rectangle on top of the chosen region.
				 * Simply a visual aid. Serves no functional purpose.
				 */
				Imgproc.rectangle(
						input, // Buffer to draw on
						region1_pointA, // First point which defines the rectangle
						region1_pointB, // Second point which defines the rectangle
						GREEN, // The color the rectangle is drawn in
						-1); // Negative thickness means solid fill
			}
			else if(min == avg2) // Was it from region 2?
			{
				config = RingConfig.FOUR; // Record our analysis

				/*
				 * Draw a solid rectangle on top of the chosen region.
				 * Simply a visual aid. Serves no functional purpose.
				 */
				Imgproc.rectangle(
						input, // Buffer to draw on
						region2_pointA, // First point which defines the rectangle
						region2_pointB, // Second point which defines the rectangle
						GREEN, // The color the rectangle is drawn in
						-1); // Negative thickness means solid fill
			}

			/*
			 * Render the 'input' buffer to the viewport. But note this is not
			 * simply rendering the raw camera feed, because we called functions
			 * to add some annotations to this buffer earlier up.
			 */
			return input;
		}

		/*
		 * Call this from the OpMode thread to obtain the latest analysis
		 */
		public RingConfig getAnalysis()
		{
			return config;
		}

		public int getMin() {
			return min;
		}
	}
}