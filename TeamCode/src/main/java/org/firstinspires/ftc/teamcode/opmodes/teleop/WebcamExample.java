/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import java.util.ArrayList;
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
import java.util.List;

@TeleOp(name = "webcam2")

public class WebcamExample extends LinearOpMode
{
    OpenCvWebcam webcam;

    @Override
    public void runOpMode()
    {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        webcam.setPipeline(new PipelineClassExample(640, 360));

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */
            /*telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();*/

            /*
             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */
            if ((int) PipelineClassExample.getColorAtMiddleRect()[0] > (int) PipelineClassExample.getColorAtMiddleRect()[1] && (int) PipelineClassExample.getColorAtMiddleRect()[0] > (int) PipelineClassExample.getColorAtMiddleRect()[2]) {

                telemetry.addData("Color > ", "Red");
            }
    
            /*if ((int) PipelineClassExample.getColorAtMiddleRect()[1] > (int) PipelineClassExample.getColorAtMiddleRect()[2] && (int) PipelineClassExample.getColorAtMiddleRect()[1] > (int) PipelineClassExample.getColorAtMiddleRect()[0]) {
                telemetry.addData("Color > ", "Green");


            }*/

            if ((int) PipelineClassExample.getColorAtMiddleRect()[1] > (int) PipelineClassExample.getColorAtMiddleRect()[0] && (int) PipelineClassExample.getColorAtMiddleRect()[1] > (int) PipelineClassExample.getColorAtMiddleRect()[2]) {

                telemetry.addData("Color > ", "Green");
            }

            if ((int) PipelineClassExample.getColorAtMiddleRect()[2] > (int) PipelineClassExample.getColorAtMiddleRect()[1] && (int) PipelineClassExample.getColorAtMiddleRect()[2] > (int) PipelineClassExample.getColorAtMiddleRect()[0]) {

                telemetry.addData("Color > ", "Blue");
            }
            telemetry.addData("Color > ", (int) PipelineClassExample.getColorAtMiddleRect()[0] + " " + (int) PipelineClassExample.getColorAtMiddleRect()[1] + " " + (int) PipelineClassExample.getColorAtMiddleRect()[2]);

            telemetry.update();
            if(gamepad1.a)
            {
                /*
                 * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
                 * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
                 * if the reason you wish to stop the stream early is to switch use of the camera
                 * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                 * (commented out below), because according to the Android Camera API documentation:
                 *         "Your application should only have one Camera object active at a time for
                 *          a particular hardware camera."
                 *
                 * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
                 * but it doesn't hurt to call it anyway, if for no other reason than clarity.
                 *
                 * NB2: if you are stopping the camera stream to simply save some processing power
                 * (or battery power) for a short while when you do not need your vision pipeline,
                 * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
                 * it the next time you wish to activate your vision pipeline, which can take a bit of
                 * time. Of course, this comment is irrelevant in light of the use case described in
                 * the above "important note".
                 */

                webcam.closeCameraDevice();
            }

            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */

        }
    }

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    /*public class SamplePipeline extends OpenCvPipeline {
        Scalar BLUE_scalar= new Scalar(41, 240, 110);
        Scalar YELLOW_scalar = new Scalar(210,16,146);
        Scalar RED_scalar = new Scalar(82,90,240);

        // Pink, the default color                         Y      Cr     Cb    (Do not change Y)
        public Scalar scalarLowerYCrCbRed = new Scalar(0.0, 150.0, 120.0);
        public Scalar scalarUpperYCrCbRed = new Scalar(255.0, 255.0, 255.0);

        // Yellow, freight or ducks!
        //public Scalar scalarLowerYCrCbYellow = new Scalar(0.0, 100.0, 0.0);
        //public Scalar scalarUpperYCrCbYellow = new Scalar(255.0, 170.0, 120.0);

        // blue                                            Y      Cr     Cb
        public Scalar scalarLowerYCrCbBlue = new Scalar( 0.0, 0.0, 120.0);
        public Scalar scalarUpperYCrCbBlue = new Scalar(255.0, 120.0, 240.0);

        //Green
        public Scalar scalarLowerYCrCbGreen = new Scalar(  0.0, 0.0, 0.0);
        public Scalar scalarUpperYCrCbGreen = new Scalar(255.0, 120.0, 120.0);

     
        // Use this picture for you own color https://github.com/PinkToTheFuture/OpenCV_FreightFrenzy_2021-2022/blob/main/YCbCr.jpeg
        // Note that the Cr and Cb values range between 0-255. this means that the origin of the coordinate system is (128,128)

        // Volatile because accessed by OpMode without sync
        public volatile boolean error = false;
        public volatile Exception debug;

        private double borderLeftX = 0;     //fraction of pixels from the left side of the cam to skip
        private double borderRightX = 0;    //fraction of pixels from the right of the cam to skip
        private double borderTopY = 0;      //fraction of pixels from the top of the cam to skip
        private double borderBottomY = 0;   //fraction of pixels from the bottom of the cam to skip

        private int CAMERA_WIDTH = 640;
        private int CAMERA_HEIGHT = 360;

        private int loopCounter = 0;
        private int pLoopCounter = 0;

        private final Mat mat = new Mat();
        private final Mat processed = new Mat();

        private Rect maxRect = new Rect(600, 1, 1, 1);

        private double maxArea = 0;
        private boolean first = false;

        private final Object sync = new Object();

        public SamplePipeline(double borderLeftX, double borderRightX, double borderTopY, double borderBottomY) {
            this.borderLeftX = borderLeftX;
            this.borderRightX = borderRightX;
            this.borderTopY = borderTopY;
            this.borderBottomY = borderBottomY;
        }



        public void configureBorders(double borderLeftX, double borderRightX, double borderTopY, double borderBottomY) {
            this.borderLeftX = borderLeftX;
            this.borderRightX = borderRightX;
            this.borderTopY = borderTopY;
            this.borderBottomY = borderBottomY;
        }

        @Override
        public Mat processFrame(Mat input) {
            Mat red = input;
            Mat blue = input;
            Mat green = input;

            double[] blue_area = new double[3];


            double[] green_area = new double[3];

            CAMERA_WIDTH = input.width();
            CAMERA_HEIGHT = input.height();

            try {
                // Process Image
                Imgproc.cvtColor(blue, mat, Imgproc.COLOR_RGB2YCrCb);
                Core.inRange(mat, scalarLowerYCrCbBlue, scalarUpperYCrCbBlue, processed);
                // Remove Noise
                Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_OPEN, new Mat());
                Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_CLOSE, new Mat());
                // GaussianBlur
                Imgproc.GaussianBlur(processed, processed, new Size(5.0, 15.0), 0.00);
                // Find Contours
                List<MatOfPoint> contours = new ArrayList<>();
                Imgproc.findContours(processed, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

                // Draw Contours
                Imgproc.drawContours(blue, contours, -1, new Scalar(255, 0, 0));

                // Lock this up to prevent errors when outside threads access the max rect property.
                synchronized (sync) {
                    // Loop Through Contours
                    for (MatOfPoint contour : contours) {
                        Point[] contourArray = contour.toArray();

                        // Bound Rectangle if Contour is Large Enough
                        if (contourArray.length >= 15) {
                            MatOfPoint2f areaPoints = new MatOfPoint2f(contourArray);
                            Rect rect = Imgproc.boundingRect(areaPoints);

                            if (rect.area() > maxArea
                                    && rect.x + (rect.width / 2.0) > (borderLeftX * CAMERA_WIDTH)
                                    && rect.x + (rect.width / 2.0) < CAMERA_WIDTH - (borderRightX * CAMERA_WIDTH)
                                    && rect.y + (rect.height / 2.0) > (borderTopY * CAMERA_HEIGHT)
                                    && rect.y + (rect.height / 2.0) < CAMERA_HEIGHT - (borderBottomY * CAMERA_HEIGHT)

                                    || loopCounter - pLoopCounter > 6
                                    && rect.x + (rect.width / 2.0) > (borderLeftX * CAMERA_WIDTH)
                                    && rect.x + (rect.width / 2.0) < CAMERA_WIDTH - (borderRightX * CAMERA_WIDTH)
                                    && rect.y + (rect.height / 2.0) > (borderTopY * CAMERA_HEIGHT)
                                    && rect.y + (rect.height / 2.0) < CAMERA_HEIGHT - (borderBottomY * CAMERA_HEIGHT)
                            ) {
                                maxArea = rect.area();
                                maxRect = rect;
                                pLoopCounter++;
                                loopCounter = pLoopCounter;
                                first = true;
                            } else if (loopCounter - pLoopCounter > 10) {
                                maxArea = new Rect().area();
                                maxRect = new Rect();
                            }

                            areaPoints.release();
                        }
                        contour.release();
                    }
                    if (contours.isEmpty()) {
                        maxRect = new Rect(600, 1, 1, 1);
                    }
                }
                // Draw Rectangles If Area Is At Least 500
                if (first && maxRect.area() > 600 && getRectHeight() > getRectWidth()) {
                    Imgproc.rectangle(blue, maxRect, new Scalar(0, 0, 255), 2);
                }
                // Draw Borders
                Imgproc.rectangle(blue, new Rect(
                        (int) (borderLeftX * CAMERA_WIDTH),
                        (int) (borderTopY * CAMERA_HEIGHT),
                        (int) (CAMERA_WIDTH - (borderRightX * CAMERA_WIDTH) - (borderLeftX * CAMERA_WIDTH)),
                        (int) (CAMERA_HEIGHT - (borderBottomY * CAMERA_HEIGHT) - (borderTopY * CAMERA_HEIGHT))
                ), BLUE_scalar, 2);



                // Display Data
                //Imgproc.putText(blue,"Blue: " +  blue.get((int) getRectMidpointXY().y, (int) getRectMidpointXY().x)[0] + ", " +blue.get((int) getRectMidpointXY().y, (int) getRectMidpointXY().x)[1] +  ", " + blue.get((int) getRectMidpointXY().y, (int) getRectMidpointXY().x)[2], new Point(5, CAMERA_HEIGHT - 5), 0, 0.6, new Scalar(255, 255, 255), 2);
                blue_area = blue.get((int) getRectMidpointXY().y, (int) getRectMidpointXY().x);
                loopCounter++;
            } catch (Exception e) {
                debug = e;
                error = true;
            }

            try {
                // Process Image
                Imgproc.cvtColor(green, mat, Imgproc.COLOR_RGB2YCrCb);
                Core.inRange(mat, scalarLowerYCrCbGreen, scalarUpperYCrCbGreen, processed);
                // Remove Noise
                Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_OPEN, new Mat());
                Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_CLOSE, new Mat());
                // GaussianBlur
                Imgproc.GaussianBlur(processed, processed, new Size(5.0, 15.0), 0.00);
                // Find Contours
                List<MatOfPoint> contours = new ArrayList<>();
                Imgproc.findContours(processed, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

                // Draw Contours
                Imgproc.drawContours(green, contours, -1, new Scalar(255, 0, 0));

                // Lock this up to prevent errors when outside threads access the max rect property.
                synchronized (sync) {
                    // Loop Through Contours
                    for (MatOfPoint contour : contours) {
                        Point[] contourArray = contour.toArray();

                        // Bound Rectangle if Contour is Large Enough
                        if (contourArray.length >= 15) {
                            MatOfPoint2f areaPoints = new MatOfPoint2f(contourArray);
                            Rect rect = Imgproc.boundingRect(areaPoints);

                            if (rect.area() > maxArea
                                    && rect.x + (rect.width / 2.0) > (borderLeftX * CAMERA_WIDTH)
                                    && rect.x + (rect.width / 2.0) < CAMERA_WIDTH - (borderRightX * CAMERA_WIDTH)
                                    && rect.y + (rect.height / 2.0) > (borderTopY * CAMERA_HEIGHT)
                                    && rect.y + (rect.height / 2.0) < CAMERA_HEIGHT - (borderBottomY * CAMERA_HEIGHT)

                                    || loopCounter - pLoopCounter > 6
                                    && rect.x + (rect.width / 2.0) > (borderLeftX * CAMERA_WIDTH)
                                    && rect.x + (rect.width / 2.0) < CAMERA_WIDTH - (borderRightX * CAMERA_WIDTH)
                                    && rect.y + (rect.height / 2.0) > (borderTopY * CAMERA_HEIGHT)
                                    && rect.y + (rect.height / 2.0) < CAMERA_HEIGHT - (borderBottomY * CAMERA_HEIGHT)
                            ) {
                                maxArea = rect.area();
                                maxRect = rect;
                                pLoopCounter++;
                                loopCounter = pLoopCounter;
                                first = true;
                            } else if (loopCounter - pLoopCounter > 10) {
                                maxArea = new Rect().area();
                                maxRect = new Rect();
                            }

                            areaPoints.release();
                        }
                        contour.release();
                    }
                    if (contours.isEmpty()) {
                        maxRect = new Rect(600, 1, 1, 1);
                    }
                }
                // Draw Rectangles If Area Is At Least 500
                if (first && maxRect.area() > 600 && getRectHeight() > getRectWidth()) {
                    Imgproc.rectangle(green, maxRect, new Scalar(0, 255, 0), 2);
                }
                // Draw Borders
                Imgproc.rectangle(green, new Rect(
                        (int) (borderLeftX * CAMERA_WIDTH),
                        (int) (borderTopY * CAMERA_HEIGHT),
                        (int) (CAMERA_WIDTH - (borderRightX * CAMERA_WIDTH) - (borderLeftX * CAMERA_WIDTH)),
                        (int) (CAMERA_HEIGHT - (borderBottomY * CAMERA_HEIGHT) - (borderTopY * CAMERA_HEIGHT))
                ), BLUE_scalar, 2);



                // Display Data
                //Imgproc.putText(green,"Green: " +  green.get((int) getRectMidpointXY().y, (int) getRectMidpointXY().x)[0] + ", " +green.get((int) getRectMidpointXY().y, (int) getRectMidpointXY().x)[1] +  ", " + green.get((int) getRectMidpointXY().y, (int) getRectMidpointXY().x)[2], new Point(5, CAMERA_HEIGHT - 5), 0, 0.6, new Scalar(255, 255, 255), 2);
                green_area = green.get((int) getRectMidpointXY().y, (int) getRectMidpointXY().x);
                loopCounter++;
            } catch (Exception e) {
                debug = e;
                error = true;
            }

            telemetry.addData("Green", green_area[0] + " " + green_area[1] + " " + green_area[2]);
            telemetry.addData("Blue", blue_area[0] + " " + blue_area[1] + " " + blue_area[2]);
            /*if (blue_area[1] > blue_area[0] && blue_area[1] > blue_area[2]) {
                telemetry.addData("Green Cup", "");
            }
            else if (blue_area[2] > blue_area[0] && blue_area[2] > blue_area[1]) {
                telemetry.addData("Blue Cup", "");
            }
            else {
                telemetry.addData("Red Cup", "");
            }
            telemetry.update();*/
            //return blue;



        }

        /*
        public int getRectHeight() {
            synchronized (sync) {
                return maxRect.height;
            }
        }

        public int getRectWidth() {
            synchronized (sync) {
                return maxRect.width;
            }
        }

        public int getRectX() {
            synchronized (sync) {
                return maxRect.x;
            }
        }

        public int getRectY() {
            synchronized (sync) {
                return maxRect.y;
            }
        }

        public double getRectMidpointX() {
            synchronized (sync) {
                return getRectX() + (getRectWidth() / 2.0);
            }
        }

        public double getRectMidpointY() {
            synchronized (sync) {
                return getRectY() + (getRectHeight() / 2.0);
            }
        }

        public Point getRectMidpointXY() {
            synchronized (sync) {
                return new Point(getRectMidpointX(), getRectMidpointY());
            }
        }

        public double getAspectRatio() {
            synchronized (sync) {
                return getRectArea() / (CAMERA_HEIGHT * CAMERA_WIDTH);
            }
        }

        public double getRectArea() {
            synchronized (sync) {
                return maxRect.area();
            }
        }*/

