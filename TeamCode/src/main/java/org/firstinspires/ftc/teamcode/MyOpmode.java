package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.security.acl.Group;
import java.util.Vector;
import java.util.concurrent.Executors;

@Config
@Autonomous(group = "drive")
public class MyOpmode extends LinearOpMode {
    private Robot robot;
    public static double X_BackDrop = -26,Y_BackDrop = 34;  ///pt start langa Backdrop
    @Override
    public void runOpMode() {
        robot = new Robot(
                hardwareMap,
                telemetry,
                Executors.newScheduledThreadPool(1)
        );
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


//        TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(new Pose2d())//bodo
//                .back(20)
//                .turn(Math.toRadians(45))
//                .forward(10)
//                .turn(Math.toRadians(-135))
//                .back(50)
//                .strafeRight(24)
//                .build();
//        TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
//                .back(17)
//                .back(2)
//                .turn(Math.toRadians(45))
//                .back(10)
//                .build();////merge pt stanga pixel
//        TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
//                .back(29.2)
//                .forward(26)
//                .strafeLeft(20)
//                .turn(Math.toRadians(-90))
//                .build();//blue side backboard middle scenario

        ////-------------Backdrop-------------

        TrajectorySequence Red_BackDrop_Right = drive.trajectorySequenceBuilder(new Pose2d())//testata si merge
                .lineToConstantHeading(new Vector2d(-19,15.5))//pixel on spike
                .lineToConstantHeading(new Vector2d(-5,15.5))
                .splineToLinearHeading(new Pose2d(-26, 34, Math.toRadians(-90)), Math.toRadians(0))//adauga outtake
                .lineToConstantHeading(new Vector2d(-4,34))//parcare
                .lineToConstantHeading(new Vector2d(-4,40))
                .build();
        TrajectorySequence Red_BackDrop_Middle = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(-29,0))
                .lineToConstantHeading(new Vector2d(-20,0))
                .splineToLinearHeading(new Pose2d(-26, 34, Math.toRadians(-90)), Math.toRadians(0))//adauga outtake
                .lineToConstantHeading(new Vector2d(-4,34))//parcare
                .lineToConstantHeading(new Vector2d(-4,40))
                .build();
        TrajectorySequence Red_BackDrop_Left = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(-17,0))
                .splineToLinearHeading(new Pose2d(-22,-6,Math.toRadians(75)),Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(-20,-4))
                .splineToLinearHeading(new Pose2d(-26, 34, Math.toRadians(-90)), Math.toRadians(0))
                //adauga outtake

                //parcare
                .lineToConstantHeading(new Vector2d(-4,34))
                .lineToConstantHeading(new Vector2d(-4,40))
                .build();

        ///// ---------Audience-----------

//        TrajectorySequence Red_Audience_Right = drive.trajectorySequenceBuilder(new Pose2d())
//                .lineToConstantHeading(new Vector2d(-15,0))
//                .splineToLinearHeading(new Pose2d(-22,8.5,Math.toRadians(-75)),Math.toRadians(0))
//                .forward(12)
//                .lineToLinearHeading(new Pose2d(-57,4,Math.toRadians(-90)))
//                .back(69)
//                .strafeLeft(10)
//                .build();
        TrajectorySequence Red_Audience_Right = drive.trajectorySequenceBuilder(new Pose2d())
               // .lineToConstantHeading(new Vector2d(-23,0))
               // .lineToConstantHeading(new Vector2d(-10,0))
                .back(24)
                .turn(Math.toRadians(-90), 2,2)
                .lineToConstantHeading(new Vector2d(-23 , 6))
                .lineTo(new Vector2d(-30, -10))
                .lineToConstantHeading(new Vector2d(-60 , 8))
                .lineToConstantHeading(new Vector2d(-60 , 78))
                .lineToConstantHeading(new Vector2d(-45 , 78))
//                .splineToLinearHeading(new Pose2d(-23,-10,Math.toRadians(-90)),Math.toRadians(0))
//                .lineToConstantHeading(new Vector2d(-23 , 18))
//                .lineToConstantHeading(new Vector2d(-23,0))
//                .strafeRight(18)
                .build();
        TrajectorySequence Red_Audience_Middle = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(-29,0))
                .lineToConstantHeading(new Vector2d(-15,0))
                .lineToConstantHeading(new Vector2d(-15,-18))
                .lineToLinearHeading(new Pose2d(-50,-18,Math.toRadians(-90)))
                .back(90)
                .lineToConstantHeading(new Vector2d(-40,72))
                //aduauga outtake
                .build();
        TrajectorySequence Red_Audience_Left =  drive.trajectorySequenceBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(-19,-9))//pixel on spike
                .lineToConstantHeading(new Vector2d(-5,-9))
                .lineToConstantHeading(new Vector2d(-5,-1.5))
                .lineToConstantHeading(new Vector2d(-55,-2))
                .lineToConstantHeading(new Vector2d(-55,1))
                .strafeLeft(65)
                .lineToLinearHeading(new Pose2d(-35,78,Math.toRadians(-90)))
                .build();

            TrajectorySequence parkingTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
                    .addTemporalMarker(() -> robot.gripper.rotateIntake(-1))
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> robot.arm.raiseArm(835,1))
                    .waitSeconds(2)
                    .addTemporalMarker(() -> robot.gripper.rotateIntake(0))
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> robot.slider.raiseSlider(1000,1))
                    .waitSeconds(2)
                    .addTemporalMarker(() -> robot.arm.gripperReleasePos())
                    .waitSeconds(1.5)
                    .addTemporalMarker(() -> robot.gripper.openBarier())
                    .waitSeconds(1.5)
                    .addTemporalMarker(()->robot.arm.gripperInitialPos())
                    .waitSeconds(2)
                    .addTemporalMarker(() -> robot.gripper.closeBarier())
                    .waitSeconds(1)
                    .addTemporalMarker(() -> robot.slider.raiseSlider(0,1))
                    .waitSeconds(2)
                    .addTemporalMarker(() -> robot.arm.raiseArm(0,1))
                    .waitSeconds(2)
                    .build();




//        TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
//                .back(24)
//                .strafeRight(15)
//                .forward(5)
//                .build();


        waitForStart();

        if(isStopRequested()) return;
        drive.followTrajectorySequence(Red_Audience_Right);
        //drive.followTrajectorySequence(
    }
}