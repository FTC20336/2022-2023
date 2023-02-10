package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

//@Disabled
@Config
 public class BBBDetector_Contour_GPT2 extends OpenCvPipeline{

    private double width;

    public BBBDetector_Contour_GPT2(int width) {
        this.width = width;
    }

    @Override
    public Mat processFrame(Mat input) {
        // "Mat" stands for matrix, which is basically the image that the detector will process
        // the input matrix is the image coming from the camera
        // the function will return a matrix to be drawn on your phone's screen

        // The detector detects regular stones. The camera fits two stones.
        // If it finds one regular stone then the other must be the skystone.
        // If both are regular stones, it returns NONE to tell the robot to keep looking

        // Make a working copy of the input matrix in HSV


        Mat hsv = new Mat();
        Mat yellow = new Mat();
        Mat gray = new Mat();

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
        Core.inRange(hsv, new Scalar(22, 93, 0), new Scalar(45, 255, 255), yellow);
        if(!yellow.empty()) {
            Imgproc.cvtColor(yellow, gray, Imgproc.COLOR_BGR2GRAY);
        }




      /*
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(gray, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            MatOfPoint2f approx = new MatOfPoint2f();
            Imgproc.approxPolyDP(contour2f, approx, 5, true);

            Point[] points = approx.toArray();
            Mat line = new Mat();
            Imgproc.fitLine(new MatOfPoint2f(points), line, Imgproc.DIST_L2, 0, 0.01, 0.01);

            double vx = line.get(0, 0)[0];
            double vy = line.get(1, 0)[0];
            double x = line.get(2, 0)[0];
            double y = line.get(3, 0)[0];

            double angle = Math.atan2(vy, vx) * 180 / Math.PI;
            Imgproc.putText(gray, angle + "", new Point(100, 100), 3, 4, new Scalar(0, 0, 0), 3);
            Imgproc.line(gray, points[0], points[1], new Scalar(255,255, 255), 6);
        }
*/

        return yellow; // return the mat with rectangles drawn
    }

 }