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
 public class BBBDetector_Contour_Pole_Cone extends OpenCvPipeline{


    public enum conecolor
    {
        RED,
        BLUE
    }
    private conecolor coneColor;

    private double width;
    private double height;
    public static int targetX;
    private int targetTolerance;

    // Yellow Color for Pole
    public static double H1y=15; //15
    public static double S1y=90; //93
    public static double V1y=70; //70
    public static double H2y=45; //45
    public static double S2y=255; //255
    public static double V2y=255; //255

    // Blue Cone
    public static double H1b=90; //15
    public static double S1b=50; //93
    public static double V1b=120; //70
    public static double H2b=128; //45
    public static double S2b=255; //255
    public static double V2b=255; //255

    // Red Cone
    public static double H1r=0; //15
    public static double S1r=80; //93
    public static double V1r=20; //70
    public static double H2r=10; //45
    public static double S2r=255; //255
    public static double V2r=255; //255

    // Red Cone2
    public static double H1r2=160; //15
    public static double S1r2=80; //93
    public static double V1r2=20; //70
    public static double H2r2=180; //45
    public static double S2r2=255; //255
    public static double V2r2=255; //255
    

    public static double poleRegionPercentWidth = 1; // we assume the region is centered with the camera
    public static int poleRegionY1 = 300; // Top of rectangle of the region to check
    public static int poleRegionPixelHeight = 25;

    private int poleRegionX1; // = (int) (this.width/2-(this.width/2*poleRegionPercentWidth));
    private int poleRegionX2; // = (int) (this.width/2+(this.width/2*poleRegionPercentWidth));

    // Initially look at the top 600 pixels on the screen
    public static double coneRegionPercentWidth = 1; // we assume the region is centered with the camera
    public static int coneRegionY1 = 0; // Top of rectangle of the region to check
    public static int coneRegionPixelHeight = 700;

    private int coneRegionX1; // = (int) (this.width/2-(this.width/2*poleRegionPercentWidth));
    private int coneRegionX2; // = (int) (this.width/2+(this.width/2*poleRegionPercentWidth));


    int poleMaxContourId = -1;
    double poleMaxContourArea=0;

    int coneMaxContourId = -1;
    double coneMaxContourArea=0;

    // Create Rectangle to make sure we have rectangles in case we don't detect color
    private Rect poleSizingBox = new Rect();
    private Rect coneSizingBox = new Rect();


    static final Scalar ORANGE = new Scalar(255, 100, 0);
    static final Scalar YELLOW = new Scalar(255, 255, 0);
    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar CYAN = new Scalar(0,255, 255);



    private double poleCentroidX;
    double poleCentroidY=0;
    private double coneCentroidX;
    double coneCentroidY=0;

    private double poleWidth;
    private double[] poleVals = new double[4];

    private double coneWidth;
    private double[] coneVals = new double[4];

    //Region Points to look at for Pole and Cone
    Point poleRegionP1 ;
    Point poleRegionP2 ;

    Point coneRegionP1 ;
    Point coneRegionP2 ;

    
    Mat poleRegion;
    Mat coneRegion;
    Mat coneRegionTop;

    Mat hsv = new Mat();
    Mat poleMat = new Mat();
    Mat coneMat = new Mat();
    Mat coneMat2 = new Mat();



    public BBBDetector_Contour_Pole_Cone(int width, int height, int targetX, int targetTolerance, conecolor coneColor) {
        this.width = width;
        this.height = height;
        this.targetX = targetX;
        this.targetTolerance = targetTolerance;
        this.coneColor = coneColor;
    }

    @Override
    public void init(Mat firstFrame)
    {
            /*
            Region to look at to based on the left top point and bottom right point
         */

        poleRegionX1= (int) (this.width/2-(this.width/2*poleRegionPercentWidth));
        poleRegionX2= (int) (this.width/2+(this.width/2*poleRegionPercentWidth));

        poleRegionP1 = new Point(poleRegionX1,poleRegionY1);
        poleRegionP2 = new Point(poleRegionX2,poleRegionY1+poleRegionPixelHeight);

        coneRegionX1= (int) (this.width/2-(this.width/2*coneRegionPercentWidth));
        coneRegionX2= (int) (this.width/2+(this.width/2*coneRegionPercentWidth));

        coneRegionP1 = new Point(coneRegionX1,coneRegionY1);
        coneRegionP2 = new Point(coneRegionX2,coneRegionY1+coneRegionPixelHeight);

        

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

        poleRegion = hsv.submat(new Rect(poleRegionP1, poleRegionP2));
        coneRegion = hsv.submat(new Rect(coneRegionP1, coneRegionP2));

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

        // Process All the POLE
        // Look for yellow in the region_H.. dump into yellow matrix (image in yellow will be white if it found yellow.. black if not)
        Core.inRange(poleRegion, new Scalar(H1y, S1y, V1y), new Scalar(H2y, S2y, V2y), poleMat);

        // Find all the poleContours of pole stuff
        List<MatOfPoint> poleContours = new ArrayList<>();
        Mat poleHierarchy = new Mat();

        // Find contours of all the yellow in the image
        Imgproc.findContours(poleMat, poleContours, poleHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Reset Center of mass to 0
        poleCentroidX=-1; // Set to -1 so we know we haven't seen a yellow pole
        poleCentroidY=0;

        //Reset maximum area contour Id and value. We set to -1 in case we don't find any poleContours
        poleMaxContourId = -1;
        poleMaxContourArea=0;

        // Go thru all poleContours and Cone Contours and if the area is bigger than the current biggest one, then use this new id as  the biggest one
        for (int i = 0 ; i< poleContours.size(); i++) {
            if (Imgproc.contourArea(poleContours.get(i)) > poleMaxContourArea) {
                poleMaxContourId = i;
                poleMaxContourArea = Imgproc.contourArea(poleContours.get(i));
            }
        }

        // if we did find a contour with big area.. calculate the center of mass.
        //https://docs.opencv.org/4.x/dd/d49/tutorial_py_contour_features.html
    if(poleMaxContourId!= -1) {
        Moments moment = new Moments();
        moment = Imgproc.moments(poleContours.get(poleMaxContourId), true);
        if (moment.m00 != 0) {
            // We add the 'poleRegionX1' value because the center of mass location origin is the corner of the region
            // We need to know the centroid from the original image size.
            poleCentroidX = (int) (moment.get_m10() / moment.get_m00() +poleRegionX1) ;
            poleCentroidY = (int) (moment.get_m01() / moment.get_m00() +poleRegionY1) ;
        }

        // Create bounding box around biggest Area, this will be used to calculate the width of the pol
        // hence it's distance
        poleSizingBox = Imgproc.boundingRect(poleContours.get(poleMaxContourId));
        poleVals[0] = (double)(poleSizingBox.x+poleRegionX1);
        poleVals[1] = (double)(poleSizingBox.y+poleRegionY1);
        poleVals[2] = (double)(poleSizingBox.width);
        poleVals[3] = (double)(poleSizingBox.height);
        poleSizingBox.set(poleVals); // Adding the offset because of the region_H

        //Draw a circle at the center of mass of the yellow region
        Imgproc.circle(input, new Point(poleCentroidX, poleCentroidY), 10, CYAN, -1);

        //Draw a BLue box around the Biggest Contour we found and that we calculated CENTROID FROM
        Imgproc.rectangle(input, poleSizingBox, CYAN, 7);

        // Draw rectangle around the region we checked
        Imgproc.rectangle(input, new Rect(poleRegionP1, poleRegionP2), ORANGE, 3);

    }


    // Draw Where we check Cone
        // Draw rectangle around the region we checked
        Imgproc.rectangle(input, new Rect(coneRegionP1, coneRegionP2), RED, 3);

        if(coneColor == conecolor.RED){
            Core.inRange(coneRegion, new Scalar(H1r, S1r, V1r), new Scalar(H2r, S2r, V2r), coneMat);
            Core.inRange(coneRegion, new Scalar(H1r2, S1r2, V1r2), new Scalar(H2r2, S2r2, V2r2), coneMat2);
            Core.addWeighted( coneMat, 1, coneMat2,1,0,coneMat);

        }else {
            Core.inRange(coneRegion, new Scalar(H1b, S1b, V1b), new Scalar(H2b, S2b, V2b), coneMat);
        }

        // Reset Center of mass to 0
        coneCentroidX=-1; // Set to -1 so we know we haven't seen a yellow pole
        coneCentroidY=0;

        coneMaxContourId = -1;
        coneMaxContourArea=0;

        // Find all the poleContours of pole stuff
        List<MatOfPoint> coneContours = new ArrayList<>();
        Mat coneHierarchy = new Mat();
        Imgproc.findContours(coneMat, coneContours, coneHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Go thru all contours colored (red or blue).. find the biggest area
        for (int i = 0 ; i< coneContours.size(); i++) {
            if (Imgproc.contourArea(coneContours.get(i)) > coneMaxContourArea) {
                coneMaxContourId = i;
                coneMaxContourArea = Imgproc.contourArea(coneContours.get(i));
            }
        }
 /*       Imgproc.putText (
                input,                          // Matrix obj of the image
                "issue: "+ String.valueOf(coneMaxContourId),          // Text to be added
                new Point (100,200),               // point
                3,      // front face
                2,                               // front scale
                RED,             // Scalar object for color
                2                                // Thickness
        );
*/

        // if we did find a contour with big area.. calculate the center of mass.
        //https://docs.opencv.org/4.x/dd/d49/tutorial_py_contour_features.html
        if(coneMaxContourId!= -1) {
            /*Moments momentCone = new Moments();
            momentCone = Imgproc.moments(coneContours.get(coneMaxContourId), true);


            if (momentCone.m00 != 0) {
                // We add the 'poleRegionX1' value because the center of mass location origin is the corner of the region
                // We need to know the centroid from the original image size.
                coneCentroidX = (int) (momentCone.get_m10() / momentCone.get_m00() + coneRegionX1);
                coneCentroidY = (int) (momentCone.get_m01() / momentCone.get_m00() + coneRegionY1);
            }
*/
            coneSizingBox = Imgproc.boundingRect(coneContours.get(coneMaxContourId));
            coneVals[0] = (double) (coneSizingBox.x + coneRegionX1);
            coneVals[1] = (double) (coneSizingBox.y + coneRegionY1);
            coneVals[2] = (double) (coneSizingBox.width);
            coneVals[3] = (double) (coneSizingBox.height);
            coneSizingBox.set(coneVals); // Adding the offset because of the region_H
            coneCentroidX = coneSizingBox.x + coneSizingBox.width/2;
            coneCentroidY = coneSizingBox.y + coneSizingBox.height/2;

                    //Draw a circle at the center of mass of the yellow region
            Imgproc.circle(input, new Point(coneCentroidX, coneCentroidY), 10, YELLOW, 4);
            Imgproc.rectangle(input, coneSizingBox, YELLOW, 5);

        }

        // Draw line of the target.. center of Claw
        Imgproc.line(input, new Point( targetX, 0 ), new Point(targetX, this.height), GREEN, 5);
        Imgproc.line(input, new Point( targetX-targetTolerance/2, 0 ), new Point(targetX-targetTolerance/2, this.height), GREEN, 2,4);
        Imgproc.line(input, new Point( targetX+targetTolerance/2, 0 ), new Point(targetX+targetTolerance/2, this.height), GREEN, 2,4);

        return input; // return the mat with rectangles drawn
    }


    public int getPolePositionPixels() {
        return (int) poleCentroidX;

    }

    public double getPoleDistanceInches() {
        // Distance approximation
        // Excel says Polynomial Equation: Distance (inches)= y = 7E-05x2 - 0.0616x + 15.581
        if (!poleSizingBox.empty()) {
            if (poleSizingBox.width != 0) {
                return (.00007 * poleSizingBox.width * poleSizingBox.width - .0616 * poleSizingBox.width + 15.581);
            } else {
                return -1;
            }
            }
            return -1;
        }


    public double getPoleDistancePixel() {
        if (!poleSizingBox.empty()) {
            if (poleSizingBox.width != 0) {
                return poleSizingBox.width;
            } else {
                return -1;
            }
        }
        return -1;
    }



    public int getConePositionPixels() {
        return (int) coneCentroidX;

    }

    public double getConePositionInches() {

        return  (coneCentroidX/25);

    }

    public double getConeDistanceInches() {
        // Distance approximation
        // Excel says Polynomial Equation: Distance (inches)= y = 4E-05x2 - 0.0713x + 33.854
        if (!coneSizingBox.empty()) {
            if (coneSizingBox.width != 0) {
                return (.00004 * coneSizingBox.width * coneSizingBox.width - .0713 * coneSizingBox.width + 33.854);
            } else {
                return -1;
            }
        }
        return -1;
    }


    public double getConeDistancePixel() {
        if (!coneSizingBox.empty()) {
            if (coneSizingBox.width != 0) {
                return coneSizingBox.width;
            } else {
                return -1;
            }
        }
        return -1;
    }
}
