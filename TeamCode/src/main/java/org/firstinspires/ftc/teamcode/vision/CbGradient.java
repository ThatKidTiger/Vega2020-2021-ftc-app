package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class CbGradient extends OpenCvPipeline {
    @Config
    public enum CV_THRESH {;
        public static int lowBound = 75;
        public static int highBound = 100;
        public static int blur = 9;
        public static int threshold = 1;
        public static int sigX = 9;
    }

    Scalar color;
    Mat kernel = Mat.ones(3,3, CvType.CV_32F);
    Mat rotated = new Mat();
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    Mat average = new Mat();
    Mat thresh = new Mat();
    Mat close = new Mat();
    Mat cannyOutput = new Mat();
    Mat hierarchy = new Mat();
    Mat gradient = new Mat();
    //Mat drawing = new Mat();

    public CbGradient(Scalar color) {
        this.color = color;
    }

    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }

    public void init(Mat firstFrame)
    {
        inputToCb(firstFrame);
    }

    @Override
    public Mat processFrame(Mat input) {
        //Core.rotate(input, rotated, Core.ROTATE_180);

        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);

        Core.extractChannel(YCrCb, Cb, 2);

        //Imgproc.GaussianBlur(Cb, average, new Size(blur , blur), sigX);

        Core.inRange(Cb, new Scalar(CV_THRESH.lowBound),new Scalar(CV_THRESH.highBound), thresh);

        Core.bitwise_not(thresh, thresh);

        //gradient.setTo(new Scalar(0), thresh);

        Imgproc.applyColorMap(Cb, gradient, Imgproc.COLORMAP_JET);

        //gradient.setTo(new Scalar(0), thresh);

        return gradient;

        /*Imgproc.morphologyEx(thresh, close, Imgproc.MORPH_CLOSE, kernel);

        Imgproc.Canny(close, cannyOutput, CV_THRESH.threshold, CV_THRESH.threshold * 3);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(cannyOutput, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);*/

        /*Map<MatOfPoint, Double> contourmap = new HashMap<MatOfPoint, Double>();
        for(int i = 0; i < contours.size(); i++) {
            contourmap.put(contours.get(i), Imgproc.contourArea(contours.get(i)));
        }*/

        /*double maxVal = 0;
        int maxValIdx = 0;

        //drawing = Mat.zeros(cannyOutput.size(), CvType.CV_8UC3);

        if(contours.size() > 0) {
            for(int contourIdx = 0; contourIdx < contours.size(); contourIdx++) {
                double contourArea = Imgproc.contourArea(contours.get(contourIdx));
                if(maxVal < contourArea) {
                    maxVal = contourArea;
                    maxValIdx = contourIdx;
                }
            }

            Imgproc.drawContours(rotated, contours, maxValIdx, color, 2, Imgproc.LINE_8, hierarchy, 0, new Point());
            Rect bounds = Imgproc.boundingRect(contours.get(maxValIdx));
            Imgproc.rectangle(rotated, bounds, color, 2);
        }

        return rotated;*/
    }
}
