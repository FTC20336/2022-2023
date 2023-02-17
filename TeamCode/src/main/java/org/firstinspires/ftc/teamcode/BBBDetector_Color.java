package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

//@Disabled
@Config
 public class BBBDetector_Color extends OpenCvPipeline{


    public static double RX = 950;
    public static double RY = 150; // Distance in pixels from the top
    public static double RW=50;
    public static double RH=50;

     public int dummy;
        /*
         * An enum to define the team element position
         */
        public enum ElementPosition
        {
            LEFT,
            CENTER,
            RIGHT
        }

        // This will get the value of the H channel
         public int Hval;

        /*
         * Some color constants
         */
        static final Scalar ORANGE = new Scalar(255, 100, 0);

        //Position 1 :  Color Range on HSV Chart
        static final int    Pos1min = 1;
        static final int    Pos1max = 47;
        static final String Pos1str = "Yellow - Left";

        //Position 2 :  Color Range on HSV Chart
        static final int    Pos2min = Pos1max+1;
        static final int    Pos2max = 93;
        static final String Pos2str = "Green - Center";

        //Position 3 :  Color Range on HSV Chart
        static final int    Pos3min = Pos2max+1;
        static final int    Pos3max = 125;
        static final String Pos3str = "Blue - Right";

     Point region1_pointA; //= new Point(RX-RW/2,RY-RH/2);
     Point region1_pointB;// = new Point(RX+RW/2,RY+RH/2);
        /*
         * Working variables
         */

        Mat region1_H;
        Mat H = new Mat();
        Mat hsv = new Mat();


        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile ElementPosition position = ElementPosition.LEFT;


        /*
         * This function takes the RGB frame, converts to HSV,
         * and extracts the H channel to the 'H' variable
         */
        void inputToH(Mat input)
        {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Core.extractChannel(hsv, H, 0);
        }

     public BBBDetector_Color(double RX,double RY,double RW,double RH) {
       /*  this.RX = 875;
         this.RY = 340;
         this.RW = 50;
         this.RH = 50;*/

     }

     public void configureBorders(double RX, double RY, double RW, double RH) {
             this.RX = RX;
             this.RY = RY;
             this.RW = RW;
             this.RH = RH;
     }

        @Override
        public void init(Mat firstFrame)
        {
            /*
            Region to look at to based on the center point and width and height
         */
            region1_pointA = new Point(RX-RW/2,RY-RH/2);
            region1_pointB = new Point(RX+RW/2,RY+RH/2);

            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToH(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */

            region1_H = H.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {

             /*
             * Overview of what we're doing:
             *
             * We first convert to HSV color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined.
             * Check a small region of the frame and see in what range of color of H it falls in
             */

            /*
             * Get the H channel of the input frame after conversion to HSV
             */
            inputToH(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */

            Hval = (int) Core.mean(region1_H).val[0];

            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    ORANGE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            // Adding Text with the H Value
            Imgproc.putText (
                    input,                          // Matrix obj of the image
                    "H: "+ String.valueOf(Hval),          // Text to be added
                    new Point (RX ,RY),               // point
                    3,      // front face
                    2,                               // front scale
                    new Scalar(0, 0, 0),             // Scalar object for color
                    2                                // Thickness
            );

          if (Hval <= Pos1max && Hval >= Pos1min)
            {
                Imgproc.putText (
                        input,                          // Matrix obj of the image
                        Pos1str,          // Text to be added
                        new Point (RX ,RY+80),               // point
                        3,      // front face
                        2,                               // front scale
                        new Scalar(0, 0, 0),             // Scalar object for color
                        2                                // Thickness
                );
                position = ElementPosition.LEFT;
            }
          else if (Hval <= Pos2max && Hval >= Pos2min) {
              Imgproc.putText(
                      input,                          // Matrix obj of the image
                      Pos2str,          // Text to be added
                      new Point (RX ,RY+80),               // point
                      3,      // front face
                      2,                               // front scale
                      new Scalar(0, 0, 0),             // Scalar object for color
                      2                                // Thickness
              );
              position = ElementPosition.CENTER;
          }
          else if (Hval <= Pos3max && Hval >= Pos3min)
          {
              Imgproc.putText (
                      input,                          // Matrix obj of the image
                      Pos3str,          // Text to be added
                      new Point (RX ,RY+80),               // point
                      3,      // front face
                      2,                               // front scale
                      new Scalar(0, 0, 0),             // Scalar object for color
                      2                                // Thickness
              );
              position = ElementPosition.RIGHT;

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
        public BBBDetector_Color.ElementPosition getAnalysis()
        {
            return position;
        }


 }