package net.polarbub.ftc.auto.vision.opencv;

import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Size;

import java.util.ArrayList;
import java.util.List;


public class FindNumSamples extends OpenCvPipeline {
    private Mat hsv = new Mat();
    private Mat cropped = new Mat();
    private Mat mask = new Mat();
    private Mat morphed = new Mat();
    private Mat blurred = new Mat();
    public int redCount = 0;
    public int yellowCount = 0;
    public int blueCount = 0;
    public int MIN_WIDTH = 20;
    public  int MAX_WIDTH = 700;
    public  int MIN_HEIGHT = 60;
    public  int MAX_HEIGHT = 900;
    public double MIN_ASPECT_RATIO = 0; // Height / Width
    public double MAX_ASPECT_RATIO = 2;


    @Override
    public Mat processFrame(Mat input) {
        // Convert to HSV color space
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Crop to bottom 5/8 of the image

        // Blur to reduce noise
       

        // Reset counts
        redCount = 0;
        yellowCount = 0;
        blueCount = 0;

        // Count and Highlight Red
        Scalar lowerRed1 = new Scalar(0, 100, 100);
        Scalar upperRed1 = new Scalar(10, 255, 255);
        Scalar lowerRed2 = new Scalar(160, 100, 100);
        Scalar upperRed2 = new Scalar(179, 255, 255);
        Core.inRange(blurred, lowerRed1, upperRed1, mask);
        Mat mask2 = new Mat();
        Core.inRange(blurred, lowerRed2, upperRed2, mask2);
        Core.bitwise_or(mask, mask2, mask);
        redCount = findAndHighlight(mask, input, new Scalar(255, 0, 0));

        // Count and Highlight Yellow
        Scalar lowerYellow = new Scalar(20, 100, 100);
        Scalar upperYellow = new Scalar(30, 255, 255);
        Core.inRange(blurred, lowerYellow, upperYellow, mask);
        yellowCount = findAndHighlight(mask, input, new Scalar(0, 255, 255));



        // Count and Highlight Blue
        Scalar lowerBlue =new Scalar(0, 0, 18);
        Scalar upperBlue = new Scalar(197, 203, 200);
        Core.inRange(blurred, lowerBlue, upperBlue, mask);
        blueCount = findAndHighlight(mask, input, new Scalar(0, 0, 255));

        // Draw rectangle on the cropped area for debugging

        return input;
    }

    private int findAndHighlight(Mat mask, Mat input, Scalar color) {
        // Morphological operations to separate overlapping samples
        Imgproc.erode(mask, morphed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)));
        Imgproc.dilate(morphed, morphed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)));

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(morphed, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        int count = 0;
        for (MatOfPoint contour : contours) {
            Rect rect = Imgproc.boundingRect(contour);

            double aspectRatio = (double) rect.height / rect.width;

            if (rect.width > MIN_WIDTH && rect.width < MAX_WIDTH &&
                    rect.height > MIN_HEIGHT && rect.height < MAX_HEIGHT &&
                    aspectRatio > MIN_ASPECT_RATIO && aspectRatio < MAX_ASPECT_RATIO) {

                // Adjust rectangle to original image coordinates
                Rect adjustedRect = new Rect(
                        rect.x,
                        rect.y,
                        rect.width,
                        rect.height
                );
                Imgproc.rectangle(input, adjustedRect, color, 2);
                count++;
            }
        }
        return count;
    }



public int getRedCount() {
    return redCount;
}

public int getYellowCount() {
    return yellowCount;
}

public int getBlueCount() {
    return blueCount;
}


}
