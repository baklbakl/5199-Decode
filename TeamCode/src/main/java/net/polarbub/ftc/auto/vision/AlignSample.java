package net.polarbub.ftc.auto.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class AlignSample extends OpenCvPipeline {
    private Mat hsvMat = new Mat();
    private Mat mask = new Mat();
    private Mat hierarchy = new Mat();
    public Point objectCenter = new Point();
    public double frameWidth;
    private double frameHeight;

    private static final Scalar LOWER_RED = new Scalar(0, 120, 70);
    private static final Scalar UPPER_RED = new Scalar(10, 255, 255);

    private static final double CENTER_THRESHOLD = 20.0; // Threshold for alignment

    @Override
    public Mat processFrame(Mat input) {
        // Get frame dimensions
        frameWidth = input.width();
        frameHeight = input.height();
        Point frameCenter = new Point(frameWidth / 2, frameHeight / 2);

        // Convert to HSV
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Threshold for blue color
        Core.inRange(hsvMat, LOWER_RED, UPPER_RED, mask);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        List<MatOfPoint> filteredRects = new ArrayList<>();
        for (MatOfPoint contour : contours) {
            if (Imgproc.contourArea(contour) >= 50) {
                Rect boundingRect = Imgproc.boundingRect(contour);
                if(boundingRect.width > 10 && boundingRect.height > 10) {
                    filteredRects.add(contour);
                }
            }
        }

        if (!filteredRects.isEmpty()) {
            // Find largest contour (assume it's the red object)
            MatOfPoint largestContour = filteredRects.stream()
                    .max(Comparator.comparingDouble(Imgproc::contourArea))
                    .orElse(null);

            if (largestContour != null) {
                // Get bounding box and center of the object
                Rect boundingBox = Imgproc.boundingRect(largestContour);
                objectCenter = new Point(
                        boundingBox.x + boundingBox.width / 2.0,
                        boundingBox.y + boundingBox.height / 2.0
                );

                // Draw bounding box and center point on the frame
                Imgproc.rectangle(input, boundingBox, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, objectCenter, 5, new Scalar(255, 0, 0), -1);

            }
        }

        // Draw the center of the frame
//        Imgproc.circle(input, new Point(frameWidth / 2, frameHeight / 2), 5, new Scalar(0, 0, 255), -1);

        return input; // Return the modified frame
    }

    private static final double ALIGNMENT_THRESHOLD = 1; // Inches
    public static double CAMERA_OFFSET = 80.70904545;

    public boolean isCentered() {
        double correctedOffset = getSmoothedXOffset() - CAMERA_OFFSET;
        return Math.abs(correctedOffset) < 1;
    }

    public double getXOffset() {
        // Calculate offset of object from the center of the camera view
        return (objectCenter != null ? objectCenter.x - frameWidth / 2 : 0);
    }

    private final int SMOOTHING_WINDOW = 5; // Number of frames to smooth over
    private final Queue<Double> offsetHistory = new LinkedList<>();
    private double smoothedOffset;

    public double getSmoothedXOffset() {
        double rawOffset = (objectCenter != null ? objectCenter.x - frameWidth / 2 : 0);

        // Add the raw offset to the history
        if (offsetHistory.size() >= SMOOTHING_WINDOW) {
            offsetHistory.poll(); // Remove oldest value
        }
        offsetHistory.add(rawOffset);

        // Compute the average
        smoothedOffset = offsetHistory.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
        return smoothedOffset;
    }

}