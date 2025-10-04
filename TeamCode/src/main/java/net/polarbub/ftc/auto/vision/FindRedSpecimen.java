package net.polarbub.ftc.auto.vision;

//import net.polarbub.ftc.lib.Pair;
import android.util.Pair;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class FindRedSpecimen extends OpenCvPipeline {

    // HSV color range for red and yellow
    private static final Scalar LOWER_RED = new Scalar(0, 120, 70);
    private static final Scalar UPPER_RED = new Scalar(10, 255, 255);
    private static final Scalar LOWER_YELLOW = new Scalar(20, 100, 100);
    private static final Scalar UPPER_YELLOW = new Scalar(30, 255, 255);

    // Real-world width of the object in centimeters (adjust to match your object)
    private static final double REAL_WIDTH = 10.0;

    // Calibrated focal length (experimentally determined)
    private static final double FOCAL_LENGTH = 700.0;

    // Matrices for processing
    private Mat hsvMat = new Mat();
    private Mat mask = new Mat();
    private Mat hierarchy = new Mat();

    private List<Rect> nonOverlappingRects;

    @Override
    public Mat processFrame(Mat input) {
        // Convert the image from RGB to HSV
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Mask for red
        Mat redMask = new Mat();
        Core.inRange(hsvMat, LOWER_RED, UPPER_RED, redMask);

        // Mask for yellow
        Mat yellowMask = new Mat();
        Core.inRange(hsvMat, LOWER_YELLOW, UPPER_YELLOW, yellowMask);

        // Combine red and yellow masks
        Core.add(redMask, yellowMask, mask);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);


        List<Rect> boundingRects = new ArrayList<>();
        for (MatOfPoint contour : contours) {
            if (Imgproc.contourArea(contour) >= 100) {
                Rect boundingRect = Imgproc.boundingRect(contour);
                boundingRects.add(boundingRect);
            }



        }

        // Step 4: Detect overlapping objects and filter them out
        nonOverlappingRects = getNonOverlappingObjects(boundingRects, input);

        int objectNumber = 0;
        // Step 5: Process non-overlapping objects (e.g., highlight them or calculate positions)
        for (Rect rect : nonOverlappingRects) {
            Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(0, 255, 0), 1); // Draw non-overlapping in green

            double objectWidthPx = rect.width;

            // Calculate approximate distance to the object based on its perceived width
            double distance = (REAL_WIDTH * FOCAL_LENGTH) / objectWidthPx;

            // Convert object width in pixels to centimeters using the distance
            double objectWidthCm = (objectWidthPx * distance) / FOCAL_LENGTH;

            double objectHeightCm = (rect.height * distance) / FOCAL_LENGTH;

            Point topLeft = rect.tl();
            Point topRight = new Point(rect.x + rect.width, rect.y);
            Point bottomRight = rect.br();
            Point bottomLeft = new Point(rect.x, rect.y + rect.height);

            // Create a list of points representing the rectangle's outline
            List<Point> points = Arrays.asList(topLeft, topRight, bottomRight, bottomLeft, topLeft);

            // Convert the list to a MatOfPoint
            MatOfPoint matOfPoints = new MatOfPoint();
            matOfPoints.fromList(points);

            // Draw the outline using polylines
            Imgproc.polylines(input, Arrays.asList(matOfPoints), true, new Scalar(0, 255, 0), 2);

            // Draw the unique number and measurements for this object
            Point labelPoint = new Point(rect.x + rect.width / 2, rect.y + rect.height / 2);
            Imgproc.putText(input, String.format("#%d", objectNumber), labelPoint,
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, new Scalar(0, 0, 0), 2);
//            Imgproc.putText(input, String.format("Distance: %.2f cm", distance),
//                    new Point(boundingRect.x, boundingRect.y - 40),
//                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 2);
//            Imgproc.putText(input, String.format("Width: %.2f cm", objectWidthCm),
//                    new Point(boundingRect.x, boundingRect.y - 60),
//                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 2);
//            Imgproc.putText(input, String.format("Height: %.2f cm", objectHeightCm),
//                    new Point(boundingRect.x, boundingRect.y - 80),
//                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 2);

            // Increment object number
            objectNumber++;
        }
        // Create a mask to overlay detected objects on the dimmed background

        // Initialize object counter


        // Combine dimmed background and objects using the mask

        // Release resources
        redMask.release();
        yellowMask.release();

        return input;
    }

    private List<Rect> getNonOverlappingObjects(List<Rect> boundingRects, Mat input) {
        List<Pair<Rect, Rect>> overlappingPairs = detectOverlappingObjects(boundingRects);

        // Create a set of overlapping rectangles to filter out
        Set<Rect> overlappingRects = new HashSet<>();
        for (Pair<Rect, Rect> pair : overlappingPairs) {
            overlappingRects.add(pair.first);
            overlappingRects.add(pair.second);
        }

        // Keep only rectangles that are not in the overlapping set
        List<Rect> nonOverlappingRects = new ArrayList<>();
        for (Rect rect : boundingRects) {
            if (!overlappingRects.contains(rect)) {
                boolean isNearGround = (rect.y + rect.height) >= (input.height() - 50);
                nonOverlappingRects.add(rect);
            }
        }

        return nonOverlappingRects;
    }

    private List<Pair<Rect, Rect>> detectOverlappingObjects(List<Rect> boundingRects) {
        List<Pair<Rect, Rect>> overlappingPairs = new ArrayList<>();
        for (int i = 0; i < boundingRects.size(); i++) {
            for (int j = i + 1; j < boundingRects.size(); j++) {
                Rect rect1 = boundingRects.get(i);
                Rect rect2 = boundingRects.get(j);
                if (rectanglesOverlap(rect1, rect2)) {
                    overlappingPairs.add(new Pair<>(rect1, rect2));
                }
            }
        }
        return overlappingPairs;
    }

    private static final int BUFFER_DISTANCE = -22; // Adjust based on needs (in pixels)

    private boolean rectanglesOverlap(Rect rect1, Rect rect2) {
        return !(rect1.x + rect1.width + BUFFER_DISTANCE < rect2.x ||   // rect1's right edge + buffer is to the left of rect2
                rect2.x + rect2.width + BUFFER_DISTANCE < rect1.x ||   // rect2's right edge + buffer is to the left of rect1
                rect1.y + rect1.height + BUFFER_DISTANCE < rect2.y ||  // rect1's bottom edge + buffer is above rect2
                rect2.y + rect2.height + BUFFER_DISTANCE < rect1.y);   // rect2's bottom edge + buffer is above rect1
    }


}