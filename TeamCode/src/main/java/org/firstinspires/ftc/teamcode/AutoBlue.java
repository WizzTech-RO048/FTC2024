/**
 * Testing the implementation of the apriltag detection.
 * */

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.ComputerVision.Pipelines.TeamPropDetectionPipelineBlue;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledFuture;

@Autonomous(name="Autonomous Blue FTC 2024")
public class AutoBlue extends LinearOpMode {
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

            lastArmMove = robot.arm.raiseArm(250, 1.0);

            telemetry.update();
            sleep(20);
        }

        if (detected_location == 1) {
            // scenariul left
            Trajectory forwardTrajectory = drive.trajectoryBuilder(new Pose2d())
                    .forward(40)
                    .build();

            Trajectory leftTrajectory = drive.trajectoryBuilder(forwardTrajectory.end())
                    .back(60)
                    .build();

            Trajectory forward = drive.trajectoryBuilder(leftTrajectory.end())
                    .forward(10)
                    .build();

            Trajectory parkTrajectory = drive.trajectoryBuilder(forward.end())
                    .strafeRight(40)
                    .build();

            if(isStopRequested()) return;
            drive.followTrajectory(forwardTrajectory);
            lastArmMove = robot.arm.raiseArm(250, 1.0);
            robot.gripper.rotateIntake(-0.5);
            drive.turn(-1.9);
            drive.followTrajectory(leftTrajectory);
            lastArmMove = robot.arm.raiseArm(835, 1.0);
//            lastSliderMove = robot.slider.raiseSlider(600, 1.0);
            robot.arm.gripperReleasePos();
            robot.gripper.openBarier();
            drive.wait(5);
            drive.followTrajectory(forward);
            drive.followTrajectory(parkTrajectory);
//            lastSliderMove = robot.slider.raiseSlider(0, 1.0);
            lastArmMove = robot.arm.raiseArm(250, 1.0);
            lastArmMove = robot.arm.raiseArm(0, 1.0);


        } else if (detected_location == 2) {
            // scenariul mid
            Trajectory forwardTrajectory = drive.trajectoryBuilder(new Pose2d())
                    .forward(40)
                    .build();

            Trajectory leftTrajectory = drive.trajectoryBuilder(forwardTrajectory.end())
                    .back(55)
                    .build();

            Trajectory forward = drive.trajectoryBuilder(leftTrajectory.end())
                    .forward(10)
                    .build();

            Trajectory parkTrajectory = drive.trajectoryBuilder(forward.end())
                    .strafeRight(40)
                    .build();

            if(isStopRequested()) return;
            drive.followTrajectory(forwardTrajectory);
            lastArmMove = robot.arm.raiseArm(250, 1.0);
            robot.gripper.rotateIntake(-0.5);
            drive.turn(-1.9);
            drive.followTrajectory(leftTrajectory);
            lastArmMove = robot.arm.raiseArm(835, 1.0);
//            lastSliderMove = robot.slider.raiseSlider(600, 1.0);
            robot.arm.gripperReleasePos();
            robot.gripper.openBarier();
            drive.followTrajectory(forward);
            drive.followTrajectory(parkTrajectory);
//            lastSliderMove = robot.slider.raiseSlider(0, 1.0);
            lastArmMove = robot.arm.raiseArm(250, 1.0);
            lastArmMove = robot.arm.raiseArm(0, 1.0);

        } else if (detected_location == 3) {
            // scenariul right

            Trajectory forwardTrajectory = drive.trajectoryBuilder(new Pose2d())
                    .forward(40)
                    .build();

            Trajectory leftTrajectory = drive.trajectoryBuilder(forwardTrajectory.end())
                    .back(60)
                    .build();

            Trajectory forward = drive.trajectoryBuilder(leftTrajectory.end())
                    .forward(10)
                    .build();

            Trajectory parkTrajectory = drive.trajectoryBuilder(forward.end())
                    .strafeRight(40)
                    .build();

            if(isStopRequested()) return;
            drive.followTrajectory(forwardTrajectory);
            lastArmMove = robot.arm.raiseArm(250, 1.0);
            robot.gripper.rotateIntake(-0.5);
            drive.turn(-1.9);
            drive.followTrajectory(leftTrajectory);
            lastArmMove = robot.arm.raiseArm(835, 1.0);
//            lastSliderMove = robot.slider.raiseSlider(600, 1.0);
            robot.arm.gripperReleasePos();
            robot.gripper.openBarier();
            drive.wait(5);
            drive.followTrajectory(forward);
            drive.followTrajectory(parkTrajectory);
//            lastSliderMove = robot.slider.raiseSlider(0, 1.0);
            lastArmMove = robot.arm.raiseArm(250, 1.0);
            lastArmMove = robot.arm.raiseArm(0, 1.0);
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
