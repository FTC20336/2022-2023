package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.*;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.*;

import java.util.*;

//@Disabled
@Config
 public class BBBDetector_Contour extends OpenCvPipeline{
    enum SkystoneLocation {
        LEFT,
        RIGHT,
        NONE
    }
    // We create a HSV range for yellow to detect regular stones
    // NOTE: In OpenCV's implementation,
    // Hue values are half the real value
    public static double h1 = 15;
    public static double h2 = 30;
    public static double sat1 = 100;
    public static double sat2 = 255;
    public static double v1 = 0;
    public static double v2 = 255;
    public org.firstinspires.ftc.robotcore.external.Telemetry telemetry;

    private int width; // width of the image
    SkystoneLocation location;

    Mat mat = new Mat();

    public Scalar lowHSV = new Scalar(h1, sat1, v1); // lower bound HSV for yellow
    public Scalar highHSV = new Scalar(h2, sat2, v2); // higher bound HSV for yellow
    Mat thresh = new Mat();

    double[] lowThresh = {h1, sat1, v1};
    double[] highThresh = {h2, sat2, v2};

    /**
     *
     * @param width The width of the image (check your camera)
     */
    public BBBDetector_Contour(int width) {
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

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);


        // if something is wrong, we assume there's no skystone
        if (mat.empty()) {
            location = SkystoneLocation.NONE;
            return input;
        }


        lowThresh[0] = h1;
        lowThresh[1] = sat1;
        lowThresh[2] = v1;

        highThresh[0] = h2;
        highThresh[1] = sat2;
        highThresh[2] = v2;

        lowHSV.set(lowThresh);
        highHSV.set(highThresh);




        // We'll get a black and white image. The white regions represent the regular stones.
        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
        Core.inRange(mat, lowHSV, highHSV, thresh);

        // Use Canny Edge Detection to find edges
        // you might have to tune the thresholds for hysteresis
        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges, 100, 300);

        // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
        // Oftentimes the edges are disconnected. findContours connects these edges.
        // We then find the bounding rectangles of those contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }

        // Iterate and check whether the bounding boxes
        // cover left and/or right side of the image
        double left_x = 0.25 * width;
        double right_x = 0.75 * width;
        boolean left = false; // true if regular stone found on the left side
        boolean right = false; // "" "" on the right side

        Mat edgeSource = new Mat();
        Imgproc.drawContours(edgeSource, contours, -1, new Scalar(255, 0, 0), 3);

        Mat lines = new Mat();
        Imgproc.HoughLinesP(edges, lines, 1, Math.PI/180, 50, 50, 10);

        /*
        for (int i = 0; i != boundRect.length; i++) {
            if (boundRect[i].x < left_x)
                left = true;
            if (boundRect[i].x + boundRect[i].width > right_x)
                right = true;

            // draw red bounding rectangles on mat
            // the mat has been converted to HSV so we need to use HSV as well
            Imgproc.rectangle(thresh, boundRect[i], new Scalar(126, 100, 100), 6);

        }
         */

        /*
        for(int i = 0; i < lines.rows(); i++) {
            double rho = lines.get(i, 0) [0], theta = lines.get(i, 0) [1];

            double a = Math.cos(theta), b = Math.sin(theta);
            double x0 = a * rho, y0 = b * rho;

            Point pt1 = new Point(Math.round(x0 + 1000 * (-b)), Math.round(y0 + 1000 * (a)));
            Point pt2 = new Point(Math.round(x0 - 1000 * (-b)), Math.round(y0 - 1000 * (a)));
            Imgproc.line(thresh, pt1, pt2, new Scalar(0, 0, 255), 3, Imgproc.LINE_AA, 0);
        }

         */

        //List points = new ArrayList<>();

        for(int i = 0; i < lines.rows(); i++) {
            double [] l = lines.get(i, 0);
            //List lineCoords = new ArrayList<>();
            //lineCoords.add(l[0]);
            //lineCoords.add(l[1]);
            //lineCoords.add(l[2]);
            //lineCoords.add(l[3]);

            //points.add(lineCoords);


            Imgproc.line(mat, new Point(l[0], l[1]), new Point(l[2], l[3]), new Scalar(0, 255, 255), 5, Imgproc.LINE_AA, 0);
        }








        /*List l1 = (List) points.get(0);
        double l1p0 = (double) l1.get(0);
        double l1p1 = (double) l1.get(1);
        double l1p2 = (double) l1.get(2);
        double l1p3 = (double) l1.get(3);


        Imgproc.line(mat, new Point(l1p0, l1p1), new Point(l1p2, l1p3), new Scalar(0, 0, 255), 3, Imgproc.LINE_AA, 0);

        List l2 = (List) points.get(points.size() - 1);
        double l2p0 = (double) l2.get(0);
        double l2p1 = (double) l2.get(1);
        double l2p2 = (double) l2.get(2);
        double l2p3 = (double) l2.get(3);


        Imgproc.line(mat, new Point(l2p0, l2p1), new Point(l2p2, l2p3), new Scalar(0, 0, 255), 3, Imgproc.LINE_AA, 0);*/



        // if there is no yellow regions on a side
        // that side should be a Skystone
        if (!left) location = SkystoneLocation.LEFT;
        else if (!right) location = SkystoneLocation.RIGHT;
            // if both are true, then there's no Skystone in front.
            // since our team's camera can only detect two at a time
            // we will need to scan the next 2 stones
        else location = SkystoneLocation.NONE;

        return mat; // return the mat with rectangles drawn
    }

 }