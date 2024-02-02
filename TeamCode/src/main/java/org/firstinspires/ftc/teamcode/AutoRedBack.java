/**
 * Testing the implementation of the apriltag detection.
 * */

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.ComputerVision.Pipelines.TeamPropDetectionPipelineRed;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledFuture;

import static org.firstinspires.ftc.teamcode.AutoRed.OptimizedStrafe;
import static org.firstinspires.ftc.teamcode.AutoRed.OptimizedStraight;

@Autonomous(name="Autonomous Red FTC 2024")
public class AutoRedBack extends LinearOpMode {
    OpenCvCamera camera;
    TeamPropDetectionPipelineRed teamPropDetectionPipeline;

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
        teamPropDetectionPipeline = new TeamPropDetectionPipelineRed();

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

            lastArmMove = robot.arm.raiseArm(250, 1.0);

            telemetry.update();
            sleep(20);
        }

        if (detected_location == 1) {


            TrajectorySequence forwardTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(OptimizedStraight(24))
                    .turn(Math.toRadians(100))
                    .addTemporalMarker(() -> robot.gripper.rotateIntake(-1))
                    .waitSeconds(1)
                    .addTemporalMarker(() -> robot.gripper.rotateIntake(0))
                    .build();

            TrajectorySequence parkingTrajectory = drive.trajectorySequenceBuilder(forwardTrajectory.end())

                    .strafeRight(24)
                    .back(80)
                    .strafeLeft(15)
                    .addTemporalMarker(() -> robot.arm.raiseArm(835,1))
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> robot.arm.gripperReleasePos())
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> robot.gripper.openBarier())
                    .waitSeconds(0.5)
                    .strafeRight(OptimizedStrafe(10))
                    .addTemporalMarker(()->robot.arm.gripperInitialPos())
                    .waitSeconds(1)
                    .addTemporalMarker(() -> robot.arm.raiseArm(0,1))
                    .addTemporalMarker(() -> robot.gripper.closeBarier())
                    .build();

            if(isStopRequested()) return;

            drive.followTrajectorySequence(forwardTrajectory);
            drive.followTrajectorySequence(parkingTrajectory);
        } else if (detected_location == 2) {
            // scenariul mid
            TrajectorySequence forwardTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(OptimizedStraight(48))
                    .turn(Math.toRadians(180))
                    .addTemporalMarker(() -> robot.gripper.rotateIntake(-1))
                    .waitSeconds(1)
                    .addTemporalMarker(() -> robot.gripper.rotateIntake(0))
                    .build();

            TrajectorySequence parkingTrajectory = drive.trajectorySequenceBuilder(forwardTrajectory.end())
                    .turn(Math.toRadians(-100))
                    .back(80)
                    .strafeLeft(15)
                    .addTemporalMarker(() -> robot.arm.raiseArm(835,1))
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> robot.arm.gripperReleasePos())
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> robot.gripper.openBarier())
                    .waitSeconds(0.5)
                    .strafeRight(OptimizedStrafe(10))
                    .addTemporalMarker(()->robot.arm.gripperInitialPos())
                    .waitSeconds(1)
                    .addTemporalMarker(() -> robot.arm.raiseArm(0,1))
                    .addTemporalMarker(() -> robot.gripper.closeBarier())
                    .build();

            if(isStopRequested()) return;

            drive.followTrajectorySequence(forwardTrajectory);

            drive.followTrajectorySequence(parkingTrajectory);


        } else if (detected_location == 3) {
            TrajectorySequence forwardTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(OptimizedStraight(24))
                    .turn(Math.toRadians(-100))
                    .addTemporalMarker(() -> robot.gripper.rotateIntake(-1))
                    .waitSeconds(1)
                    .addTemporalMarker(() -> robot.gripper.rotateIntake(0))
                    .build();

            TrajectorySequence parkingTrajectory = drive.trajectorySequenceBuilder(forwardTrajectory.end())
                    .strafeLeft(24)
                    .turn(Math.toRadians(180))
                    .back(80)
                    .strafeLeft(15)
                    .addTemporalMarker(() -> robot.arm.raiseArm(835,1))
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> robot.arm.gripperReleasePos())
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> robot.gripper.openBarier())
                    .waitSeconds(0.5)
                    .strafeRight(OptimizedStrafe(10))
                    .addTemporalMarker(()->robot.arm.gripperInitialPos())
                    .waitSeconds(1)
                    .addTemporalMarker(() -> robot.arm.raiseArm(0,1))
                    .addTemporalMarker(() -> robot.gripper.closeBarier())
                    .build();

            if(isStopRequested()) return;

            drive.followTrajectorySequence(forwardTrajectory);

            drive.followTrajectorySequence(parkingTrajectory);

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

