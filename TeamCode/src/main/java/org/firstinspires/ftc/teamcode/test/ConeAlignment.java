package org.firstinspires.ftc.teamcode.test;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.outreachBot.Constants;
import org.opencv.core.Core;
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

@TeleOp(name = "Webcam Cone Alignment")
public class ConeAlignment extends OpMode {
    static final int STREAM_WIDTH = 1920; // modify for your camera
    static final int STREAM_HEIGHT = 1080; // modify for your camera
    OpenCvWebcam webcam;
    ConeAlignmentPipeline pipeline;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new ConeAlignmentPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                dashboard.startCameraStream(webcam, 0);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Failed", "");
                telemetry.update();
            }
        });

    }

    @Override
    public void loop() {
        telemetry.addData("Something:", pipeline.getMiddle());
        telemetry.update();

//        TelemetryPacket pack = new TelemetryPacket();
//        pack.put("Something:", pipeline.getMiddle());
//        dashboard.sendTelemetryPacket(pack);
    }


}

class ConeAlignmentPipeline extends OpenCvPipeline {
    Mat YCrCb = new Mat();
    Mat Cr = new Mat();
    static final int STREAM_WIDTH = 1920; // modify for your camera
    static final int STREAM_HEIGHT = 1080; // modify for your camera
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    int middle;

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Y channel to the 'Y' variable
     */
    void inputToCr(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        ArrayList<Mat> yCrCbChannels = new ArrayList(3);
        Core.split(YCrCb, yCrCbChannels);
        Cr = yCrCbChannels.get(1);
    }

    int[] detectRed() {
        boolean foundFirstRed = false;
        int left = 0;
        int right = 0;
        int targetLine = 540;
        for (int i = STREAM_HEIGHT - 1; i >= 0 && !foundFirstRed; i--) {
            for (int j = 0; j < STREAM_WIDTH; j++) {
//                if (input.at(Byte.class, i, j).getV4c().get_0() >= 210 && input.at(Byte.class, i, j).getV4c().get_1() <= 40 && input.at(Byte.class, i, j).getV4c().get_2() <= 40) {
                if (Cr.at(Byte.class, i, j).getV().byteValue() >= Constants.colorThresh) {
                    targetLine = i - 50;
                    foundFirstRed = true;
                    break;
                }
            }
        }


        TelemetryPacket pack = new TelemetryPacket();
        // pack.put("target:", targetLine);
        pack.put("Center Cr value:", Cr.at(Byte.class, STREAM_WIDTH / 2, STREAM_HEIGHT / 2).getV().byteValue());
        dashboard.sendTelemetryPacket(pack);

        for (int j = 0; j < STREAM_WIDTH - 3; j++) {
            if (Cr.at(Byte.class, targetLine, j).getV().byteValue() >= Constants.colorThresh && Cr.at(Byte.class, targetLine, j + 1).getV().byteValue() >= Constants.colorThresh &&Cr.at(Byte.class, targetLine, j + 2).getV().byteValue() >= Constants.colorThresh && Cr.at(Byte.class, targetLine, j + 3).getV().byteValue() >= Constants.colorThresh) {
                left = j;
                break;
            }
        }

        for (int j = STREAM_WIDTH - 1; j >= 3; j--) {
            if (Cr.at(Byte.class, targetLine, j).getV().byteValue() >= Constants.colorThresh && Cr.at(Byte.class, targetLine, j - 1).getV().byteValue() >= Constants.colorThresh && Cr.at(Byte.class, targetLine, j - 2).getV().byteValue() >= Constants.colorThresh && Cr.at(Byte.class, targetLine, j - 3).getV().byteValue() >= Constants.colorThresh) {
                right = j;
                break;
            }
        }
        pack.put("left:", left);
        pack.put("right:", right);
        dashboard.sendTelemetryPacket(pack);
        return new int[] {left, (left + right) / 2, right};
    }
    @Override
    public void init(Mat firstFrame) {
        inputToCr(firstFrame);
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToCr(input);
//        Cr.copyTo(input);
        int[] arr = detectRed();
        middle = arr[1];
        YCrCb.release(); // don't le`ak memory!
        Cr.release(); // don't leak memory!

        Imgproc.rectangle( // rings
                input, // Buffer to draw on
                new Point(middle, 0), // First point which defines the rectangle
                new Point(middle, STREAM_HEIGHT), // Second point which defines the rectangle
                new Scalar(0, 0, 255), // The color the rectangle is drawn in
                5); // Thickness of the rectangle lines
        Imgproc.rectangle( // rings
                input, // Buffer to draw on
                new Point(arr[0], 0), // First point which defines the rectangle
                new Point(arr[0], STREAM_HEIGHT), // Second point which defines the rectangle
                new Scalar(0, 0, 255), // The color the rectangle is drawn in
                5); // Thickness of the rectangle lines
        Imgproc.rectangle( // rings
                input, // Buffer to draw on
                new Point(arr[2], 0), // First point which defines the rectangle
                new Point(arr[2], STREAM_HEIGHT), // Second point which defines the rectangle
                new Scalar(0, 0, 255), // The color the rectangle is drawn in
                5); // Thickness of the rectangle lines
        return input;
    }

    public int getMiddle() {
        return middle;
    }

}
