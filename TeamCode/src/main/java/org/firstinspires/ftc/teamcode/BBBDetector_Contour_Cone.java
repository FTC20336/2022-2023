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
 public class BBBDetector_Contour_Cone extends OpenCvPipeline{

    private double width;
    private double height;
    public static int targetX;
    private int targetDiameter;

    // Blue Cone
    public static double H1=90; //90
    public static double S1=50; //50
    public static double V1=70; //70
    public static double H2=128; //128
    public static double S2=255; //255
    public static double V2=255; //255
/*
    // Red Cone
    public static double H1r=0; //15
    public static double S1r=175; //93
    public static double V1r=20; //70
    public static double H2r=10; //45
    public static double S2r=255; //255
    public static double V2r=255; //255

    // Red Cone2
    public static double H1r2=170; //15
    public static double S1r2=175; //93
    public static double V1r2=20; //70
    public static double H2r2=180; //45
    public static double S2r2=255; //255
    public static double V2r2=255; //255
*/

    public static double regionPercentWidth = 1; // we assume the region is centered with the camera
    public static int regionY1 = 0; // Top of rectangle of the region to check
    public static int regionPixelHeight = 600;

    private int regionX1= (int) (this.width/2-(this.width/2*regionPercentWidth));
    private int regionX2= (int) (this.width/2+(this.width/2*regionPercentWidth));

    double cy=0;
    int maxContourId = -1;
    double maxContourArea=0;
    private Rect myrec;
    private Rect myRecSmall;
    private Rect myBoundSmall;

    static final Scalar ORANGE = new Scalar(255, 100, 0);
    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar CYAN = new Scalar(0,255, 255);



    private double colorXCentroid;
    private double colorWidth;
    private double[] vals = new double[4];

    //Region to look at
    Point region1_pointA ;
    Point region1_pointB ;

    Mat region1_H;
    Mat regionConeTop;
    Mat hsv = new Mat();
    Mat yellow = new Mat();



    public BBBDetector_Contour_Cone(int width, int height, int targetX, int targetDiameter) {
        this.width = width;
        this.height = height;
        this.targetX = targetX;
        this.targetDiameter = targetDiameter;
    }

    @Override
    public void init(Mat firstFrame)
    {
            /*
            Region to look at to based on the left top point and bottom right point
         */

        regionX1= (int) (this.width/2-(this.width/2*regionPercentWidth));
        regionX2= (int) (this.width/2+(this.width/2*regionPercentWidth));

        region1_pointA = new Point(regionX1,regionY1);
        region1_pointB = new Point(regionX2,regionY1+regionPixelHeight);

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
        // the function will return a matrix to be drawn on your driver hub's screen

        // The detector detects a yellow object (pole/junction) in a narrow band
        // Contours are made for each region found and the biggest one is kept
        // The centroid of the contour is found to locate the center of the pole
        // Using the centroid location, we can determine where the pole is located in respect to the robot
        // A boundary box is create around that region and the width is used to estimate the distance to the camera


        // Create an HSV image using input
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Look for yellow in the region_H.. dump into yellow matrix (image in yellow will be white if it found yellow.. black if not)
        Core.inRange(region1_H, new Scalar(H1, S1, V1), new Scalar(H2, S2, V2), yellow);

        // Find all the contours of Yellow stuff
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(yellow, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Reset Center of mass to 0
        colorXCentroid=-1;
        double cy=0;

        //Reset maximum area contour Id and value. We set to -1 in case we don't find any contours
        maxContourId = -1;
        maxContourArea=0;

        //Uncomment to draw all contours on the yellow mat.. won't align with the input mat
        // Imgproc.drawContours(yellow, contours,-1,new Scalar(255,255, 255), 6);
        //Uncomment to see how many contour we found
        //Imgproc.putText(input,  String.valueOf(contours.size()), new Point(200,200), Imgproc.FONT_HERSHEY_SIMPLEX, 4, new Scalar(255, 255, 255), 2);

        // Go thru all contours and if the area is bigger than the current biggest one, then use this new id as  the biggest one
        for (int i = 0 ; i< contours.size(); i++) {
            if (Imgproc.contourArea(contours.get(i)) > maxContourArea) {
                maxContourId = i;
                maxContourArea = Imgproc.contourArea(contours.get(i));
            }
        }

        // if we did find a contour with big area.. calculate the center of mass.
        //https://docs.opencv.org/4.x/dd/d49/tutorial_py_contour_features.html
    if(maxContourId!= -1) {
        // Create bounding box around biggest Area, this will be used to recalculate the width of the Cone at the top
        // hence it's distance
        myrec = Imgproc.boundingRect(contours.get(maxContourId));
       vals[0] = (double)(myrec.x+regionX1);
       vals[1] = (double)(myrec.y+regionY1);
       vals[2] = (double)(myrec.width);
       vals[3] = (double)(myrec.height);
       myrec.set(vals); // Adding the offset because of the region_H

        Rect myRecSmall = new Rect(myrec.x, myrec.y ,myrec.width,35 );
        regionConeTop = hsv.submat(myRecSmall);
        Core.inRange(regionConeTop, new Scalar(H1, S1, V1), new Scalar(H2, S2, V2), yellow);


        // Contour for the top of the cone
        List<MatOfPoint> contoursSmall = new ArrayList<>();
        Mat hierarchySmall = new Mat();
        Imgproc.findContours(yellow, contoursSmall, hierarchySmall, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Reset Center of mass to 0
        colorXCentroid=-1;
        double cy2=0;

        //Reset maximum area contour Id and value. We set to -1 in case we don't find any contours
        maxContourId = -1;
        maxContourArea=0;

        //Uncomment to draw all contours on the yellow mat.. won't align with the input mat
        // Imgproc.drawContours(yellow, contours,-1,new Scalar(255,255, 255), 6);
        //Uncomment to see how many contour we found
        //Imgproc.putText(input,  String.valueOf(contours.size()), new Point(200,200), Imgproc.FONT_HERSHEY_SIMPLEX, 4, new Scalar(255, 255, 255), 2);

        // Go thru all contours and if the area is bigger than the current biggest one, then use this new id as  the biggest one
        for (int i = 0 ; i< contoursSmall.size(); i++) {
            if (Imgproc.contourArea(contoursSmall.get(i)) > maxContourArea) {
                maxContourId = i;
                maxContourArea = Imgproc.contourArea(contoursSmall.get(i));
            }
        }

        // if we did find a contour with big area.. calculate the center of mass.
        //https://docs.opencv.org/4.x/dd/d49/tutorial_py_contour_features.html
        if(maxContourId!= -1) {
            Moments momentSmall = new Moments();
            momentSmall = Imgproc.moments(contoursSmall.get(maxContourId), true);
            if (momentSmall.m00 != 0) {
                // We add the 'regionX1' value because the center of mass location origin is the corner of the region
                // We need to know the centroid from the original image size.
                colorXCentroid = (int) (momentSmall.get_m10() / momentSmall.get_m00() + myRecSmall.x);
                cy2 = (int) (momentSmall.get_m01() / momentSmall.get_m00() + myRecSmall.y);
            }

            // Create bounding box around biggest Area, this will be used to calculate the width of the pol
            // hence it's distance
            myBoundSmall = Imgproc.boundingRect(contoursSmall.get(maxContourId));
            vals[0] = (double) (myBoundSmall.x + regionX1+myRecSmall.x);
            vals[1] = (double) (myBoundSmall.y + regionY1+myRecSmall.y);
            vals[2] = (double) (myBoundSmall.width);
            vals[3] = (double) (myBoundSmall.height);
            myBoundSmall.set(vals); // Adding the offset because of the region_H


            //Draw a circle at the center of mass of the yellow region
            Imgproc.circle(input, new Point(colorXCentroid, cy), 10, CYAN, -1);
            Imgproc.rectangle(input, myrec, BLUE, 5);
            Imgproc.rectangle(input, myBoundSmall, CYAN, 7);
        }
    }
        // Draw rectangle around the region we checked
        Imgproc.rectangle(input, new Rect(region1_pointA, region1_pointB), ORANGE, 3);
        // Draw line of the target.. center of Claw
        Imgproc.line(input, new Point( targetX, 0 ), new Point(targetX, this.height), GREEN, 5);
        Imgproc.line(input, new Point( targetX-targetDiameter/2, 0 ), new Point(targetX-targetDiameter/2, this.height), GREEN, 2,4);
        Imgproc.line(input, new Point( targetX+targetDiameter/2, 0 ), new Point(targetX+targetDiameter/2, this.height), GREEN, 2,4);

        return input; // return the mat with rectangles drawn
    }


    public int getPosition() {
        return (int) colorXCentroid;

    }
    public double getWidth() {
    // Distance approximation
    // Excel says Polynomial Equation: Distance (inches)= y = 7E-05x2 - 0.0616x + 15.581
        if (!myBoundSmall.empty()) {
            if (myBoundSmall.width != 0) {
                return (-.00003 * myBoundSmall.width * myBoundSmall.width - .0498 * myBoundSmall.width + 21.398);
            } else {
                return -1;
            }
        }
        return -1;
    }

    public double getWidthpix() {
        if (!myBoundSmall.empty()){
            if (myBoundSmall.width != 0) {
                return myBoundSmall.width;
            } else {
                return -1;
            }
        }
        return -1;
    }
}
