package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

//@Disabled
@Config
 public class BBBDetector_Contour_Pole extends OpenCvPipeline{

    private double width;

    public static double L1=15;
    public static double L2=93;
    public static double L3=0;
    public static double H1=45;
    public static double H2=255;
    public static double H3=255;

    double cy=0;
    int maxContourId = -1;
    double maxContourArea=0;
    private Rect myrec;

    static final Scalar ORANGE = new Scalar(255, 100, 0);

    private double yellow_x_centroid;
    private double yellowWidth;

    Point region1_pointA; //= new Point(RX-RW/2,RY-RH/2);
    Point region1_pointB;// = new Point(RX+RW/2,RY+RH/2);
    /*
     * Working variables
     */

    Mat region1_H;
    Mat hsv = new Mat();
    Mat hsv2 = new Mat();
    Mat yellow = new Mat();
    Mat gray = new Mat();


    public BBBDetector_Contour_Pole(int width) {
        this.width = width;
    }

    @Override
    public void init(Mat firstFrame)
    {
            /*
            Region to look at to based on the left top point and bottom right point
         */
        region1_pointA = new Point(0,300);
        region1_pointB = new Point(1200,400);

        /*
         * We need to call this in order to make sure the 'Cb'
         * object is initialized, so that the submats we make
         * will still be linked to it on subsequent frames. (If
         * the object were to only be initialized in processFrame,
         * then the submats would become delinked because the backing
         * buffer would be re-allocated the first time a real frame
         * was crunched)
         */
        Imgproc.cvtColor(firstFrame, hsv, Imgproc.COLOR_RGB2HSV);

        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any changes to the child affect the parent, and the
         * reverse also holds true.
         */

        region1_H = hsv.submat(new Rect(region1_pointA, region1_pointB));

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
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Look for yellow in the region_H.. dunp into yellow matrix (image in yellow will be white if it found yellow.. black if not)
        Core.inRange(region1_H, new Scalar(L1, L2, L3), new Scalar(H1, H2, H3), yellow);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(yellow, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Reset Center of mass to 0
        yellow_x_centroid=0;
        double cy=0;

        //Reset maximum area contour Id and value. We set to -1 in case we don't find any contours
        maxContourId = -1;
        maxContourArea=0;

            //Uncomment to draw all contours on the yellow mat.. won't align with the input mat
       // Imgproc.drawContours(yellow, contours,-1,new Scalar(255,255, 255), 6);
        //Uncomment to see how many contour we found
        //Imgproc.putText(input,  String.valueOf(contours.size()), new Point(200,200), Imgproc.FONT_HERSHEY_SIMPLEX, 4, new Scalar(255, 255, 255), 2);

        // Go thru all contours and if the area is bigger than the previous one, then use as the biggest one
        for (int i = 0 ; i< contours.size(); i++) {
            if (Imgproc.contourArea(contours.get(i)) > maxContourArea) {
                maxContourId = i;
                maxContourArea = Imgproc.contourArea(contours.get(i));
            }
        }

        // if we did find a contour with big area.. calculate the center of mass.
    if(maxContourId!= -1) {
        Moments moment = new Moments();
        moment = Imgproc.moments(contours.get(maxContourId), true);
        if (moment.m00 != 0) {
            yellow_x_centroid = (int) (moment.get_m10() / moment.get_m00()+0) ;
            cy = (int) (moment.get_m01() / moment.get_m00() +300) ;
        }
        myrec = Imgproc.boundingRect(contours.get(maxContourId));

        Imgproc.circle(input, new Point(yellow_x_centroid, cy), 10, ORANGE, 3);

    }

        Imgproc.rectangle(input, new Rect(region1_pointA, region1_pointB), ORANGE, 3);

        return input; // return the mat with rectangles drawn
    }


    public double getPosition() {
        return yellow_x_centroid;

    }
    public double getWidth() {
        return myrec.width;

    }
}
