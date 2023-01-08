package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.sql.Array;
import java.util.ArrayList;
import java.util.List;

import java.util.*;


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
    public static double h1 = 22;
    public static double h2 = 45;
    public static double sat1 = 93;
    public static double sat2 = 255;
    public static double v1 = 0;
    public static double v2 = 255;

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
/*
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
        for (int i = 0; i != boundRect.length; i++) {
            if (boundRect[i].x < left_x)
                left = true;
            if (boundRect[i].x + boundRect[i].width > right_x)
                right = true;

            // draw red bounding rectangles on mat
            // the mat has been converted to HSV so we need to use HSV as well
            Imgproc.rectangle(mat, boundRect[i], new Scalar(0.5, 100, 100), 6);
        }

        // if there is no yellow regions on a side
        // that side should be a Skystone
        if (!left) location = SkystoneLocation.LEFT;
        else if (!right) location = SkystoneLocation.RIGHT;
            // if both are true, then there's no Skystone in front.
            // since our team's camera can only detect two at a time
            // we will need to scan the next 2 stones
        else location = SkystoneLocation.NONE;
*/
        return thresh; // return the mat with rectangles drawn
    }

 }