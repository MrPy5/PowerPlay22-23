package org.firstinspires.ftc.teamcode.opmodes.teleop;



import static org.firstinspires.ftc.teamcode.opmodes.teleop.Constants.*;


import android.graphics.Bitmap;
import org.firstinspires.ftc.teamcode.opmodes.teleop.Constants.*;

import org.opencv.android.Utils;
import org.opencv.core.*;
import org.opencv.imgproc.*;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class PipelineClassExample extends OpenCvPipeline
{

    private Mat blurInput = new Mat();
    private Mat blurOutput = new Mat();
    private Mat hsvThresholdOutput = new Mat();
    private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<>();
    private Mat findContoursOutputMat = new Mat();
    private Mat finalContourOutputMat = new Mat();

    private int largestX, largestY;
    private double largestArea;

    public PipelineClassExample() {


        largestX = -1;
        largestY = -1;
        largestArea = -1;
    }

    @Override
    public Mat processFrame(Mat input)
    {
        // Step Blur0 (stage 1):
        blurInput = input;
        BlurType blurType = BlurType.get(BLUR);
        double blurRadius = BLUR_RADIUS;
        blur(blurInput, blurType, blurRadius, blurOutput);

        // Step HSV_Threshold0  (stage 2):
        Mat hsvThresholdInput = blurOutput;
        double[] hsvThresholdHue = {HUE_MIN, HUE_MAX};
        double[] hsvThresholdSaturation = {SATURATION_MIN, SATURATION_MAX};
        double[] hsvThresholdValue = {VALUE_MIN, VALUE_MAX};
        hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

        // Step Find_Contours0 (stage 3):
        Mat findContoursInput = hsvThresholdOutput;
        boolean findContoursExternalOnly = false;
        findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);
        findContoursOutputMat = input.clone();
        for(int i = 0; i < findContoursOutput.size(); i++) {
            Imgproc.drawContours(findContoursOutputMat, findContoursOutput, i, new Scalar(255, 255, 255), 2);
        }

        // Finding largest contour (stage 4):
        finalContourOutputMat = input.clone();
        largestArea = -1;
        largestX = -1;
        largestY = -1;
        int largestContourIndex = -1;
        for(int i = 0; i < findContoursOutput.size(); i++) {
            MatOfPoint contour = findContoursOutput.get(i);
            double contourArea = Imgproc.contourArea(contour);
            if(contourArea > MIN_CONTOUR_AREA && contourArea > largestArea) {
                Moments p = Imgproc.moments(contour, false);
                int x = (int) (p.get_m10() / p.get_m00());
                int y = (int) (p.get_m01() / p.get_m00());

                largestContourIndex = i;
                largestX = x;
                largestY = y;
                largestArea = contourArea;
            }
        }
        if(largestContourIndex != -1)
            Imgproc.drawContours(finalContourOutputMat, findContoursOutput, largestContourIndex, new Scalar(255, 255, 255), 2);

        handleDashboard();

        return input;
    }

    public int[] getPosition() {
        return new int[] {largestX, largestY};
    }

    private void handleDashboard() {

    }

    private void sendMatToDashboard(Mat input) {
        Bitmap bitmap = Bitmap.createBitmap(input.width(), input.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(input, bitmap);

    }


    enum BlurType{
        BOX("Box Blur"), GAUSSIAN("Gaussian Blur"), MEDIAN("Median Filter"),
        BILATERAL("Bilateral Filter");

        private final String label;

        BlurType(String label) {
            this.label = label;
        }

        public static BlurType get(String type) {
            if (BILATERAL.label.equals(type)) {
                return BILATERAL;
            }
            else if (GAUSSIAN.label.equals(type)) {
                return GAUSSIAN;
            }
            else if (MEDIAN.label.equals(type)) {
                return MEDIAN;
            }
            else {
                return BOX;
            }
        }

        @Override
        public String toString() {
            return this.label;
        }
    }

    private void blur(Mat input, BlurType type, double doubleRadius,
                      Mat output) {
        int radius = (int)(doubleRadius + 0.5);
        int kernelSize;
        switch(type){
            case BOX:
                kernelSize = 2 * radius + 1;
                Imgproc.blur(input, output, new Size(kernelSize, kernelSize));
                break;
            case GAUSSIAN:
                kernelSize = 6 * radius + 1;
                Imgproc.GaussianBlur(input,output, new Size(kernelSize, kernelSize), radius);
                break;
            case MEDIAN:
                kernelSize = 2 * radius + 1;
                Imgproc.medianBlur(input, output, kernelSize);
                break;
            case BILATERAL:
                Imgproc.bilateralFilter(input, output, -1, radius, radius);
                break;
        }
    }

    private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
                              Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
        Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
                new Scalar(hue[1], sat[1], val[1]), out);
    }

    private void findContours(Mat input, boolean externalOnly,
                              ArrayList<MatOfPoint> contours) {
        Mat hierarchy = new Mat();
        contours.clear();
        int mode;
        if (externalOnly) {
            mode = Imgproc.RETR_EXTERNAL;
        }
        else {
            mode = Imgproc.RETR_LIST;
        }
        int method = Imgproc.CHAIN_APPROX_SIMPLE;
        Imgproc.findContours(input, contours, hierarchy, mode, method);
    }
}