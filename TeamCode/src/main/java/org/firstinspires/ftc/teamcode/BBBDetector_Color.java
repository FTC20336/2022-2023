package org.firstinspires.ftc.teamcode;
import org.opencv.core.CvType;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


 public class BBBDetector_Color extends OpenCvPipeline{

        /*
         * An enum to define the team element position
         */
        public enum ElementPosition
        {
            LEFT,
            CENTER,
            RIGHT
        }
        public int detect;
        // public int Yval;
       //  public int Cbval;
       //  public int Crval;
         public int Hval;
        /*
         * Some color constants
         */
        static final Scalar ORANGE = new Scalar(255, 100, 0);
        //static final Scalar GREEN = new Scalar(0, 255, 0);
        static final int    RED = 6;
        static final int    YELLOW = 33;
        static final int    GREEN = 78;

        // Green
    // public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 0.0, 0.0);
    // public static Scalar scalarUpperYCrCb = new Scalar(255.0, 120.0, 120.0);


        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(590,460);
        static final int REGION_WIDTH = 100;
        static final int REGION_HEIGHT = 200;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
      //  Mat region1_Cb;
      //  Mat region1_Cr;
      //  Mat region1_Y;
        Mat region1_H;
      //  Mat YCrCb = new Mat();
      //  Mat Cr = new Mat();
      //  Mat Y = new Mat();
      //  Mat Cb = new Mat();
        Mat H = new Mat();
      //  Mat processed = new Mat();
        Mat hsv = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile BBBDetector_Color.ElementPosition position = BBBDetector_Color.ElementPosition.LEFT;
        //private volatile int detect = 0;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
          //  Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            /*Core.extractChannel(YCrCb, Cb, 2);
            Core.extractChannel(YCrCb, Cr, 1);
            Core.extractChannel(YCrCb, Y, 0);*/
            Core.extractChannel(hsv, H, 0);
/*
            Core.inRange(YCrCb, scalarLowerYCrCb, scalarUpperYCrCb, processed);
            // Remove Noise
            Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_OPEN, new Mat());
            Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_CLOSE, new Mat());
            // GaussianBlur
            Imgproc.GaussianBlur(processed, processed, new Size(5.0, 15.0), 0.00);
            Core.bitwise_and(YCrCb, processed, new Mat());
*/
        }

        @Override
        public void init(Mat firstFrame)
        {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCb(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
           /* region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region1_Cr = Cr.submat(new Rect(region1_pointA, region1_pointB));
            region1_Y = Y.submat(new Rect(region1_pointA, region1_pointB));*/
            region1_H = H.submat(new Rect(region1_pointA, region1_pointB));
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

            //Cbval = (int) Core.mean(region1_Cb).val[0];
           // Crval = (int) Core.mean(region1_Cr).val[0];
           // Yval = (int) Core.mean(region1_Y).val[0];
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
            // Adding Text
           /* Imgproc.putText (
                    input,                          // Matrix obj of the image
                    "Cb: "+ String.valueOf(Cbval),          // Text to be added
                    new Point (590,400),               // point
                    3,      // front face
                    .75,                               // front scale
                    new Scalar(0, 0, 0),             // Scalar object for color
                    2                                // Thickness
            );
            Imgproc.putText (
                    input,                          // Matrix obj of the image
                    "Cr: "+ String.valueOf(Crval),          // Text to be added
                    new Point (590,420),               // point
                    3,      // front face
                    .75,                               // front scale
                    new Scalar(0, 0, 0),             // Scalar object for color
                    2                                // Thickness
            );
            Imgproc.putText (
                    input,                          // Matrix obj of the image
                    "Y: "+ String.valueOf(Yval),          // Text to be added
                    new Point (590,440),               // point
                    3,      // front face
                    .75,                               // front scale
                    new Scalar(0, 0, 0),             // Scalar object for color
                    2                                // Thickness
            );*/
            Imgproc.putText (
                    input,                          // Matrix obj of the image
                    "H: "+ String.valueOf(Hval),          // Text to be added
                    REGION1_TOPLEFT_ANCHOR_POINT,               // point
                    3,      // front face
                    .75,                               // front scale
                    new Scalar(0, 0, 0),             // Scalar object for color
                    2                                // Thickness
            );

          if (Hval < RED)
            {
                Imgproc.putText (
                        input,                          // Matrix obj of the image
                        "RED",          // Text to be added
                        new Point (590,440),               // point
                        3,      // front face
                        .75,                               // front scale
                        new Scalar(0, 0, 0),             // Scalar object for color
                        2                                // Thickness
                );
                position = ElementPosition.LEFT;
            }
          else if (Hval < YELLOW) {
              Imgproc.putText(
                      input,                          // Matrix obj of the image
                      "Yellow",          // Text to be added
                      new Point(590, 440),               // point
                      3,      // front face
                      .75,                               // front scale
                      new Scalar(0, 0, 0),             // Scalar object for color
                      2                                // Thickness
              );
          }
          else if (Hval < GREEN)
          {
              Imgproc.putText (
                      input,                          // Matrix obj of the image
                      "GREEN",          // Text to be added
                      new Point (590,440),               // point
                      3,      // front face
                      .75,                               // front scale
                      new Scalar(0, 0, 0),             // Scalar object for color
                      2                                // Thickness
              );

          }
/*
if hue_value < 5:
color = "RED"
elif hue_value < 22:
color = "ORANGE"
elif hue_value < 33:
color = "YELLOW"
elif hue_value < 78:
color = "GREEN"
elif hue_value < 131:
color = "BLUE"
elif hue_value < 170:
color = "VIOLET"

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