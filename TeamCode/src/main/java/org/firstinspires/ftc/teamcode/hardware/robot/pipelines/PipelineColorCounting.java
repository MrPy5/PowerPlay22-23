package org.firstinspires.ftc.teamcode.hardware.robot.pipelines;




import android.graphics.Bitmap;
import android.graphics.Color;

import org.opencv.android.Utils;
import org.opencv.core.*;
import org.opencv.imgproc.*;
import org.openftc.easyopencv.OpenCvPipeline;

public class PipelineColorCounting extends OpenCvPipeline {


    public static double[] colorAtMiddleRect = new double[3];


    private int width; // width of the image
    public int height;
    public static Rect rectangleToDrawScreen = null;
    public static Mat inputMat;
    public static String globalPosition = "null";
    public static double[] color;

    public PipelineColorCounting(int width, int height) {
        this.width = width;
        this.height = height;
    }

    @Override
    public Mat processFrame(Mat input) {

        if (globalPosition == "left") {
            rectangleToDrawScreen = new Rect(150, 15, 90, 120);
        }
        else {
            rectangleToDrawScreen = new Rect(170, 15, 90, 120);
        }

        Imgproc.rectangle(input, rectangleToDrawScreen, new Scalar(64,64,64), 10);
        Imgproc.putText(input, globalPosition,new Point(10,50), 1, 4, new Scalar(0,0,0));
        inputMat = input.clone();

        return input;

    }

    


    public static void updatePosition(String newPosition) {
        globalPosition = newPosition;
    }

    public static int getColorAtMiddleRect(String side) {


        color = new double[3];

        if (inputMat != null && rectangleToDrawScreen != null) {
            Mat workMat = new Mat(inputMat, rectangleToDrawScreen);
            Bitmap Cropedimage = Bitmap.createBitmap(workMat.cols(), workMat.rows(), Bitmap.Config.ARGB_8888);

            Utils.matToBitmap(workMat, Cropedimage);
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
        //always
        color[2] = color[2] *0.95;

        if (side == "red") {
            color[0] = color[0] * 0.94;
        }

        else {
            color[2] = color[2] * 0.94;
        }


        if (((color[0] / color[1]) * 100) > 104 && ((color[0] / color[2]) * 100) > 104) {
            return 3;
        }
        else if (((color[1] / color[2]) * 100) > 104 && ((color[1] / color[0]) * 100) > 104) {
            return 2;
        }
        else {
            return 1;
        }


    }

}