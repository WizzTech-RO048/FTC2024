

/**
 * Testing the implementation of the apriltag detection.
 * */

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
<<<<<<< HEAD
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
=======
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
>>>>>>> 37e3892 (full autonomie)
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
<<<<<<< HEAD
import org.firstinspires.ftc.teamcode.ComputerVision.Pipelines.TeamPropDetectionPipeline;
<<<<<<< HEAD
=======
=======
import org.firstinspires.ftc.teamcode.ComputerVision.Pipelines.TeamPropDetectionPipelineRed;
>>>>>>> a08ee6d (Added different pipelines for different alliances)
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
>>>>>>> 37e3892 (full autonomie)
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

<<<<<<< HEAD
@Autonomous(name="Autonomous FTC 2024")
public class Auto extends LinearOpMode {

=======
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledFuture;

@Autonomous(name="Autonomous Blue FTC 2024")
public class AutoRed extends LinearOpMode {
>>>>>>> 37e3892 (full autonomie)
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
    //ratio for optimized movement
    public static double ratioStrafe = (60.0/24.0)*0.94;
    public static double  ratioStraight = (60/48)*1.3;

    int detected_location;

    @Override
    public void runOpMode() {
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

<<<<<<< HEAD
=======
            //  lastArmMove = robot.arm.raiseArm(250, 1.0);

>>>>>>> 37e3892 (full autonomie)
            telemetry.update();
            sleep(20);
        }

        if (detected_location == 1) {
            // scenariul left
<<<<<<< HEAD
        } else if (detected_location == 2) {
            // scenariul mid
        } else if (detected_location == 3) {
            // scenariul right
        }
=======
            TrajectorySequence forwardTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(OptimizedStraight(24))
                    .turn(Math.toRadians(100))
                    .addTemporalMarker(() -> robot.gripper.rotateIntake(-1))
                    .waitSeconds(1)
                    .addTemporalMarker(() -> robot.gripper.rotateIntake(0))
                    .build();

            TrajectorySequence parkingTrajectory = drive.trajectorySequenceBuilder(forwardTrajectory.end())
                    .back(OptimizedStraight(35))
                    .addTemporalMarker(() -> robot.arm.raiseArm(835,1))
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> robot.arm.gripperReleasePos())
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> robot.gripper.openBarier())
                    .waitSeconds(0.5)
                    .strafeLeft(OptimizedStrafe(20))
                    .addTemporalMarker(()->robot.arm.gripperInitialPos())
                    .waitSeconds(1)
                    .addTemporalMarker(() -> robot.gripper.closeBarier())
                    .addTemporalMarker(() -> robot.arm.raiseArm(0,1))
                    .build();

            if(isStopRequested()) return;

            drive.followTrajectorySequence(forwardTrajectory);
            drive.followTrajectorySequence(parkingTrajectory);


        } else if (detected_location == 2) {
            // scenariul mid
            TrajectorySequence forwardTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(OptimizedStraight(24))
                    .addTemporalMarker(() -> robot.gripper.rotateIntake(-1))
                    .waitSeconds(1)
                    .addTemporalMarker(() -> robot.gripper.rotateIntake(0))
                    .build();

            TrajectorySequence parkingTrajectory = drive.trajectorySequenceBuilder(forwardTrajectory.end())
                    .back(OptimizedStraight(35))
                    .addTemporalMarker(() -> robot.arm.raiseArm(835,1))
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> robot.arm.gripperReleasePos())
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> robot.gripper.openBarier())
                    .waitSeconds(0.5)
                    .strafeLeft(OptimizedStrafe(20))
                    .addTemporalMarker(()->robot.arm.gripperInitialPos())
                    .waitSeconds(1)
                    .addTemporalMarker(() -> robot.arm.raiseArm(0,1))
                    .addTemporalMarker(() -> robot.gripper.closeBarier())
                    .build();

            if(isStopRequested()) return;

            drive.followTrajectorySequence(forwardTrajectory);
            drive.turn(Math.toRadians(100));
            drive.followTrajectorySequence(parkingTrajectory);


        } else if (detected_location == 3) {
            // scenariul right
            TrajectorySequence purplepixel = drive.trajectorySequenceBuilder(new Pose2d())
                    .strafeRight(OptimizedStrafe(24))
                    .forward(OptimizedStraight(24))
                    .turn(Math.toRadians(100))
                    .addTemporalMarker(() -> robot.gripper.rotateIntake(-1))
                    .waitSeconds(1)
                    .addTemporalMarker(() -> robot.gripper.rotateIntake(0))
                    .build();

            TrajectorySequence parkingTrajectory = drive.trajectorySequenceBuilder(purplepixel.end())
                    .back(OptimizedStraight(10))
                    .addTemporalMarker(() -> robot.arm.raiseArm(835,1))
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> robot.arm.gripperReleasePos())
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> robot.gripper.openBarier())
                    .waitSeconds(0.5)
                    .strafeLeft(OptimizedStrafe(20))
                    .addTemporalMarker(()->robot.arm.gripperInitialPos())
                    .waitSeconds(1)
                    .addTemporalMarker(() -> robot.arm.raiseArm(0,1))
                    .addTemporalMarker(() -> robot.gripper.closeBarier())
                    .build();

            if(isStopRequested()) return;

            drive.followTrajectorySequence(purplepixel);
            drive.followTrajectorySequence(parkingTrajectory);
        }

>>>>>>> 37e3892 (full autonomie)
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

    public static double OptimizedStrafe(double x)
    {

        return  x*ratioStrafe;
    }
    public static double OptimizedStraight(double x){

        return x*ratioStraight;
    }
}

