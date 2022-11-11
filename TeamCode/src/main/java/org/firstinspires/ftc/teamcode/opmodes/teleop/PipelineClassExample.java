package org.firstinspires.ftc.teamcode.opmodes.teleop;




import org.opencv.core.*;
import org.opencv.imgproc.*;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PipelineClassExample extends OpenCvPipeline {


    public static double[] colorAtMiddleRect = new double[3];


    private int width; // width of the image


    /**
     * @param width The width of the image (check your camera)
     */
    public PipelineClassExample(int width) {
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
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // if something is wrong, we assume there's no skystone


        // We create a HSV range for yellow to detect regular stones
        // NOTE: In OpenCV's implementation,
        // Hue values are half the real value
        Scalar lowHSV = new Scalar(36, 50, 70); // lower bound HSV for yellow
        Scalar highHSV = new Scalar(89, 255, 255); // higher bound HSV for yellow
        Mat thresh = new Mat();

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

        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }

        // Iterate and check whether the bounding boxes
        // cover left and/or right side of the image
        try {
            double left_x = 0.25 * width;
            double right_x = 0.75 * width;
            Rect rectangleToDraw = null;
            double maxArea = -1;
            for (int i = 0; i != boundRect.length; i++) {
                if (boundRect[i].area() > maxArea) {
                    maxArea = boundRect[i].area();
                    rectangleToDraw = boundRect[i];
                }

            }
            if (rectangleToDraw != null) {
                Imgproc.rectangle(input, rectangleToDraw, new Scalar(0.5, 76.9, 89.8));


                double middleX = rectangleToDraw.x + (rectangleToDraw.width / 2.0);
                double middleY = rectangleToDraw.y + (rectangleToDraw.height / 2.0);

                colorAtMiddleRect = input.get((int) middleY, (int) middleX);
            }
        } finally {

        }


        return input; // return the mat with rectangles drawn
    }

    public static String getColorAtMiddleRect() {

        if (colorAtMiddleRect[0] > colorAtMiddleRect[2] && colorAtMiddleRect[1] > colorAtMiddleRect[2]) {
            return "mid";
        }
        else if (colorAtMiddleRect[0] > colorAtMiddleRect[1] && colorAtMiddleRect[0] > colorAtMiddleRect[2]) {
            return "right";
        }

        else {//if (colorAtMiddleRect[2] > colorAtMiddleRect[1] && colorAtMiddleRect[2] > colorAtMiddleRect[0]) {
            return "left";
        }


    }

}