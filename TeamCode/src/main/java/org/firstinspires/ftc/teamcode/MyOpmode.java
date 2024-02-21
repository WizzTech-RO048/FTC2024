package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.security.acl.Group;

@Config
@Autonomous(group = "drive")
public class MyOpmode extends LinearOpMode {
    public static double X_BackDrop = -26,Y_BackDrop = 34;  ///pt start langa Backdrop
    @Override
    public void runOpMode() {
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
                .lineToConstantHeading(new Vector2d(-10,0))
                .splineToLinearHeading(new Pose2d(-22,-6,Math.toRadians(45)),Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(-20,-4))
                .splineToLinearHeading(new Pose2d(-26, 34, Math.toRadians(-135)), Math.toRadians(0))
                //adauga outtake

                //parcare
                .lineToConstantHeading(new Vector2d(-4,34))
                .lineToConstantHeading(new Vector2d(-4,40))
                .build();

        ///// ---------Audience-----------

        TrajectorySequence Red_Audience_Right = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(-10,0))
                .splineToLinearHeading(new Pose2d(-22,6,Math.toRadians(-45)),Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(-20,4))
                .lineToLinearHeading(new Pose2d(4,-40,Math.toRadians(-45)))
                .back(70)
                .strafeLeft(10)
                .build();
        TrajectorySequence Red_Audience_Middle = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(-29,0))
                .lineToConstantHeading(new Vector2d(-20,0))
                .splineToConstantHeading(new Vector2d(-40,-20),Math.toRadians(0))
                .strafeLeft(60)
                .back(96)
                //aduauga outtake
                .build();
        TrajectorySequence Red_Audience_Left =  drive.trajectorySequenceBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(-19,-15.5))//pixel on spike
                .lineToConstantHeading(new Vector2d(-5,-15.5))
                .splineToConstantHeading(new Vector2d(-26, 0), Math.toRadians(0))
                .strafeLeft(60)
                .build();






//        TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(new Pose2d())
//                .back(24)
//                .strafeRight(15)
//                .forward(5)
//                .build();


        waitForStart();

        if(isStopRequested()) return;
        drive.followTrajectorySequence(Red_Audience_Middle);
    }
}