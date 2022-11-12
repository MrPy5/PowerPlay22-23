package org.firstinspires.ftc.teamcode.opmodes.teleop;




import android.graphics.Bitmap;
import android.graphics.Color;

import org.opencv.android.Utils;
import org.opencv.core.*;
import org.opencv.imgproc.*;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PipelineClassExample extends OpenCvPipeline {


    public static double[] colorAtMiddleRect = new double[3];


    private int width; // width of the image
    public int height;
    public static double redArea = 0;
    public static double blueArea = 0;
    public static double greenArea = 0;
    public static Rect rectangleToDrawGreen;
    public static double[] colorAtCenter = new double[3];
    public static Mat matGreen;
    public static Mat edgesGreen;
    public static Mat threshGreen;
    public static Mat hierarchyGreen;



    /**
     * @param width The width of the image (check your camera)
     */
    public PipelineClassExample(int width, int height) {
        this.width = width;
        this.height = height;
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

        matGreen = new Mat();
        Imgproc.cvtColor(input, matGreen, Imgproc.COLOR_RGB2HSV);

        // if something is wrong, we assume there's no skystone


        // We create a HSV range for yellow to detect regular stones
        // NOTE: In OpenCV's implementation,
        // Hue values are half the real value
        Scalar lowHSVGreen = new Scalar(36, 50, 40); // lower bound HSV for yellow
        Scalar highHSVGreen = new Scalar(89, 255, 255); // higher bound HSV for yellow
        threshGreen = new Mat();

        // We'll get a black and white image. The white regions represent the regular stones.
        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
        Core.inRange(matGreen, lowHSVGreen, highHSVGreen, threshGreen);

        // Use Canny Edge Detection to find edges
        // you might have to tune the thresholds for hysteresis
        try {
        edgesGreen = new Mat();
        Imgproc.Canny(threshGreen, edgesGreen, 100, 300);

        // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
        // Oftentimes the edges are disconnected. findContours connects these edges.
        // We then find the bounding rectangles of those contours
        List<MatOfPoint> contoursGreen = new ArrayList<>();
        hierarchyGreen = new Mat();
        Imgproc.findContours(edgesGreen, contoursGreen, hierarchyGreen, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint2f[] contoursPolyGreen = new MatOfPoint2f[contoursGreen.size()];
        Rect[] boundRectGreen = new Rect[contoursGreen.size()];
        for (int i = 0; i < contoursGreen.size(); i++) {
            contoursPolyGreen[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contoursGreen.get(i).toArray()), contoursPolyGreen[i], 3, true);
            boundRectGreen[i] = Imgproc.boundingRect(new MatOfPoint(contoursPolyGreen[i].toArray()));
        }

        // Iterate and check whether the bounding boxes
        // cover left and/or right side of the image

        rectangleToDrawGreen = null;
        double maxArea = -1;
        for (int i = 0; i != boundRectGreen.length; i++) {
            if (boundRectGreen[i].area() > maxArea) {
                maxArea = boundRectGreen[i].area();
                rectangleToDrawGreen = boundRectGreen[i];
            }

        }
        if (rectangleToDrawGreen != null) {
            Imgproc.rectangle(input, rectangleToDrawGreen, new Scalar(0,255,0));
            greenArea = rectangleToDrawGreen.area();

        }
        else {
            greenArea = 0;
        }
        } catch (Exception e) {
            e.printStackTrace();
        }

        //colorAtCenter = input.get(rectangleToDrawGreen.x + (rectangleToDrawGreen.width / 2), rectangleToDrawGreen.y + (rectangleToDrawGreen.height / 2));
        //Mat cropedMat = new Mat(matGreen, rect);
        /*if (rectangleToDrawGreen != null) {
            return new Mat(input, rectangleToDrawGreen);
        }
        else {*/
            return input;
        //}
        /*
        // Make a working copy of the input matrix in HSV
        Mat matBlue = new Mat();
        Imgproc.cvtColor(input, matBlue, Imgproc.COLOR_RGB2HSV);

        // if something is wrong, we assume there's no skystone


        // We create a HSV range for yellow to detect regular stones
        // NOTE: In OpenCV's implementation,
        // Hue values are half the real value
        Scalar lowHSVBlue = new Scalar(90, 50, 70); // lower bound HSV for yellow
        Scalar highHSVBlue = new Scalar(128, 255, 255); // higher bound HSV for yellow
        Mat threshBlue = new Mat();

        // We'll get a black and white image. The white regions represent the regular stones.
        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
        Core.inRange(matBlue, lowHSVBlue, highHSVBlue, threshBlue);

        // Use Canny Edge Detection to find edges
        // you might have to tune the thresholds for hysteresis
        Mat edgesBlue = new Mat();
        Imgproc.Canny(threshBlue, edgesBlue, 100, 300);

        // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
        // Oftentimes the edges are disconnected. findContours connects these edges.
        // We then find the bounding rectangles of those contours
        List<MatOfPoint> contoursBlue = new ArrayList<>();
        Mat hierarchyBlue = new Mat();
        Imgproc.findContours(edgesBlue, contoursBlue, hierarchyBlue, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint2f[] contoursPolyBlue = new MatOfPoint2f[contoursBlue.size()];
        Rect[] boundRectBlue = new Rect[contoursBlue.size()];
        for (int i = 0; i < contoursBlue.size(); i++) {
            contoursPolyBlue[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contoursBlue.get(i).toArray()), contoursPolyBlue[i], 3, true);
            boundRectBlue[i] = Imgproc.boundingRect(new MatOfPoint(contoursPolyBlue[i].toArray()));
        }

        // Iterate and check whether the bounding boxes
        // cover left and/or right side of the image
        try {

            Rect rectangleToDrawBlue = null;
            double maxArea = -1;
            for (int i = 0; i != boundRectBlue.length; i++) {
                if (boundRectBlue[i].area() > maxArea) {
                    maxArea = boundRectBlue[i].area();
                    rectangleToDrawBlue = boundRectBlue[i];
                }

            }
            if (rectangleToDrawBlue != null) {
                Imgproc.rectangle(input, rectangleToDrawBlue, new Scalar(0,0,255));
                blueArea = rectangleToDrawBlue.area();

            }
            else {
                blueArea = 0;
            }
        } finally {
        }
      

        // Make a working copy of the input matrix in HSV
        Mat matRed = new Mat();
        Imgproc.cvtColor(input, matRed, Imgproc.COLOR_RGB2HSV);

        // if something is wrong, we assume there's no skystone


        // We create a HSV range for yellow to detect regular stones
        // NOTE: In OpenCV's implementation,
        // Hue values are half the real value
        Scalar lowHSVRed = new Scalar(0, 50, 70); // lower bound HSV for yellow
        Scalar highHSVRed = new Scalar(30, 255, 255); // higher bound HSV for yellow
        Mat threshRed = new Mat();

        // We'll get a black and white image. The white regions represent the regular stones.
        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
        Core.inRange(matRed, lowHSVRed, highHSVRed, threshRed);

        // Use Canny Edge Detection to find edges
        // you might have to tune the thresholds for hysteresis
        Mat edgesRed = new Mat();
        Imgproc.Canny(threshRed, edgesRed, 100, 300);

        // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
        // Oftentimes the edges are disconnected. findContours connects these edges.
        // We then find the bounding rectangles of those contours
        List<MatOfPoint> contoursRed = new ArrayList<>();
        Mat hierarchyRed = new Mat();
        Imgproc.findContours(edgesRed, contoursRed, hierarchyRed, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint2f[] contoursPolyRed = new MatOfPoint2f[contoursRed.size()];
        Rect[] boundRectRed = new Rect[contoursRed.size()];
        for (int i = 0; i < contoursRed.size(); i++) {
            contoursPolyRed[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contoursRed.get(i).toArray()), contoursPolyRed[i], 3, true);
            boundRectRed[i] = Imgproc.boundingRect(new MatOfPoint(contoursPolyRed[i].toArray()));
        }

        // Iterate and check whether the bounding boxes
        // cover left and/or right side of the image
        try {
            Rect rectangleToDrawRed = null;
            double maxArea = -1;
            for (int i = 0; i != boundRectRed.length; i++) {
                if (boundRectRed[i].area() > maxArea) {
                    maxArea = boundRectRed[i].area();
                    rectangleToDrawRed = boundRectRed[i];
                }

            }
            if (rectangleToDrawRed != null) {
                Imgproc.rectangle(input, rectangleToDrawRed, new Scalar(255,0,0));

                redArea = rectangleToDrawRed.area();
            }
            else {
                redArea = 0;
            }
        } finally {

        }

        return input;

         */
    }

    



    public static double[] getColorAtMiddleRect() {

        /*if (redArea > blueArea && redArea > greenArea) {
            return "right";
        }
        else if (blueArea > greenArea && blueArea > redArea) {
            return "left";
        }

        else if (greenArea > redArea && greenArea > blueArea){//if (colorAtMiddleRect[2] > colorAtMiddleRect[1] && colorAtMiddleRect[2] > colorAtMiddleRect[0]) {
            return "mid";
        }
        else {
            return "none";
        }*/
        double[] color = new double[3];
        if (rectangleToDrawGreen != null) {
            Rect rect = rectangleToDrawGreen;
            Mat cropedMat = new Mat(matGreen, rect);

            Bitmap Cropedimage = Bitmap.createBitmap(cropedMat.cols(), cropedMat.rows(), Bitmap.Config.ARGB_8888);

            Utils.matToBitmap(cropedMat, Cropedimage);

            Bitmap bitmap = Cropedimage;
            double redBucket = 0;
            double greenBucket = 0;
            double blueBucket = 0;
            double pixelCount = 0;

            for (int y = 0; y < bitmap.getHeight(); y++) {
                for (int x = 0; x < bitmap.getWidth(); x++) {
                    int c = bitmap.getPixel(x, y);

                    pixelCount++;
                    redBucket += Color.red(c);
                    greenBucket += Color.green(c);
                    blueBucket += Color.blue(c);
                    // does alpha matter?
                }
            }

            color[0] = redBucket / pixelCount;
            color[1] = greenBucket / pixelCount;
            color[2] = blueBucket / pixelCount;
        }
        return color;

    }

}