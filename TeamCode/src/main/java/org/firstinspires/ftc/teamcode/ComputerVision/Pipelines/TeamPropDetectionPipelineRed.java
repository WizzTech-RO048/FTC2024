package org.firstinspires.ftc.teamcode.ComputerVision.Pipelines;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeamPropDetectionPipelineRed extends OpenCvPipeline {
    private int location = 0;

    // -------- color ranges --------
    private Scalar lowerRed = new Scalar(0, 122, 141);
    private Scalar higherRed = new Scalar(255, 255, 255);

    // -------- created new canvases --------
    private Mat hsvMat = new Mat();
    private Mat binaryMat = new Mat();
    private Mat maskedInputMat = new Mat();

    // -------- shape of the input video --------
    private static int WIDTH = 864;
    private static int HEIGHT = 480;

    // -------- coordinates for the limits of the LEFT, MID and RIGHT ROIs --------
    private int line1 = WIDTH / 3 + 20;
    private int line2 = WIDTH / 3 * 2 +100;

    @Override
    public Mat processFrame(Mat input) {
        // -------- conversion from BGR to HSV --------
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat, lowerRed, higherRed, binaryMat);

        // -------- drawing lines for orientation --------
        Imgproc.line(binaryMat, new Point(line1, 0), new Point(line1, HEIGHT), new Scalar(97, 97, 97), 5);
        Imgproc.line(binaryMat, new Point(line2, 0), new Point(line2, HEIGHT), new Scalar(97, 97, 97), 5);

        // -------- creating for each ROI an image --------
        Mat left_roi = binaryMat.submat(new Rect(0, 0, line1, HEIGHT));
        Mat mid_roi = binaryMat.submat(new Rect(line1, 0, line2 - line1, HEIGHT));
        Mat right_roi = binaryMat.submat(new Rect(line2, 0, WIDTH - line2, HEIGHT));

        // -------- calculating the number of white pixels in each region --------
        double w1 = Core.countNonZero(left_roi);
        double w2 = Core.countNonZero(mid_roi);
        double w3 = Core.countNonZero(right_roi);

        // -------- writing the number of pixels in each region on the frame --------
        int FONT = Imgproc.FONT_HERSHEY_SIMPLEX;
        int FONT_SCALE = 1;
        Scalar COLOR = new Scalar(100, 100, 100);
        int THICKNESS = 3;

        Imgproc.putText(binaryMat, Double.toString(w1), new Point(10, 30), FONT, FONT_SCALE, COLOR, THICKNESS);
        Imgproc.putText(binaryMat, Double.toString(w2), new Point(line1+10, 30), FONT, FONT_SCALE, COLOR, THICKNESS);
        Imgproc.putText(binaryMat, Double.toString(w3), new Point(line2+10, 30), FONT, FONT_SCALE, COLOR, THICKNESS);

        // -------- finding the biggest area with white pixels --------
        double max_area = Math.max(Math.max(w1, w2), w3);
        if (max_area == w1 && w1 > 6000) {
            location = 1;
        } else if (max_area == w2 && w2 > 6000) {
            location = 2;
        } else if (max_area == w3 && w3 > 6000) {
            location = 3;
        } else {
            location = 0;
        }

        // -------- writing the detected position on the frame --------
        Imgproc.putText(binaryMat, Double.toString(location), new Point(WIDTH - 100, HEIGHT - 50), FONT, FONT_SCALE, COLOR, THICKNESS);

        return binaryMat;
    }

    public int getLocation() {
        return location;
    }
}
