/**
 * Testing the implementation of the apriltag detection.
 * */

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.ComputerVision.Pipelines.TeamPropDetectionPipelineBlue;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledFuture;

@Autonomous(name="Blue Audience Auto")
public class AutoBlueAudience extends LinearOpMode {
    OpenCvCamera camera;
    TeamPropDetectionPipelineBlue teamPropDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // ---- calibration for lens intrinsics, units are pixels ----
    // TODO: we need to do our own calibration for the camera, this one works for C920, 800x440
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.166;

    int detected_location;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Robot robot = new Robot(
                hardwareMap,
                telemetry,
                Executors.newScheduledThreadPool(1)
        );;

        ScheduledFuture<?> lastArmMove, lastSliderMove;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        teamPropDetectionPipeline = new TeamPropDetectionPipelineBlue();

        camera.setPipeline(teamPropDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(864, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            detected_location = teamPropDetectionPipeline.getLocation();
            telemetry.addData("TeamProp Location", detected_location);

            telemetry.update();
            sleep(20);
        }

        if (detected_location == 1) {
            TrajectorySequence Blue_Audience_Left = drive.trajectorySequenceBuilder(new Pose2d())
                    .lineToConstantHeading(new Vector2d(-29,0))
                    .lineToConstantHeading(new Vector2d(-20,0))
                    .splineToLinearHeading(new Pose2d(-29,19,Math.toRadians(-90)),Math.toRadians(0))
                    .forward(21)
                    .back(12)
                    .lineToConstantHeading(new Vector2d(-52,0))
                    .lineToConstantHeading(new Vector2d(-52,70))
                    //adauga outtake

                    //parcare
//                .lineToConstantHeading(new Vector2d(-4,34))
//                .lineToConstantHeading(new Vector2d(-4,40))
                    .build();

            if(isStopRequested()) return;
            drive.followTrajectorySequence(Blue_Audience_Left);



//            drive.followTrajectorySequence(forwardTrajectory);

        } else if (detected_location == 2) {
            // scenariul mid
            TrajectorySequence Blue_Audience_Middle = drive.trajectorySequenceBuilder(new Pose2d())
                    .lineToLinearHeading(new Pose2d(-34,0,Math.toRadians(-90)))
                    .forward(20)
                    .lineToLinearHeading(new Pose2d(-50,-20,Math.toRadians(-85)))
                    .build();

            if(isStopRequested()) return;
            drive.followTrajectorySequence(Blue_Audience_Middle);
//            drive.followTrajectorySequence(forwardTrajectory);

        } else if (detected_location == 3) {
            // scenariul right
            TrajectorySequence Blue_Audience_Right = drive.trajectorySequenceBuilder(new Pose2d())
                    .lineToConstantHeading(new Vector2d(-24,20))//pixel on spike
                    .lineToConstantHeading(new Vector2d(-2,20))
                    .lineToConstantHeading(new Vector2d(-5,-1))
                    .lineToConstantHeading(new Vector2d(-5,0))
                    .lineToConstantHeading(new Vector2d(-52,1))
                    .strafeRight(65)
                    .build();
            if(isStopRequested()) return;
            drive.followTrajectorySequence(Blue_Audience_Right);
//            drive.followTrajectorySequence(forwardTrajectory);

        }

        while(opModeIsActive()) { sleep(20); }
    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection tag) {
        Orientation rot = Orientation.getOrientation(tag.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID=%d", tag.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", tag.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", tag.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", tag.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
    }

}

