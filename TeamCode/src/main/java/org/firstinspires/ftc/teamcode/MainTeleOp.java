package org.firstinspires.ftc.teamcode;

import android.os.Build;
import android.util.Pair;
import androidx.annotation.RequiresApi;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Robot.*;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledFuture;

@TeleOp(name="FTC2024")                                                                                                                                                                                                                                                             
public class MainTeleOp extends OpMode {

    private Robot robot;
    private Controller controller1;
    private SampleMecanumDrive drive;

    private int raise_value, arm_value;
    public double RAISE_POWER = 1.0;
    private boolean closed, gripper_released;
    private boolean sculatoare;
    private int gheare = 0;
    private int slider_level = 0;
    private ScheduledFuture<?> lastArmMove, lastSliderMove;
    private ScheduledFuture<?> lastRightLift, lastLeftLift;

    // TODO: add timer

    @Override
    public void init() {
        robot = new Robot(
                hardwareMap,
                telemetry,
                Executors.newScheduledThreadPool(1)
        );
        controller1 = new Controller(gamepad1);

        // --------- initializing the robot --------
//        robot.gripper.release();
        robot.gripper.defaultPickupPixelPos();

        gheare = 1;

        robot.plane.grabPlane();

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        robot.gripper.openBarier();
        closed = false;
        sculatoare = false;
        gripper_released = false;
        robot.lift.setDownPosition();
    }

    // ------ the emergency stop function ---------
//    public void stop() { robot.stopRobot(); }

    @Override
    public void loop() {
        controller1.update();

        // -------- controlling the robot movement ------
        // TOMICI: movement-ul ar trebui sa fie cel de roadrunner, nu am mai apucat sa il testez, dar ar trebui sa fie ok
        // (eventual doar pozitiile din joystick sa fie negative / pozitive)
        drive.setWeightedDrivePower(
                new Pose2d(
                        -controller1.left_stick_y, // gen astea negative / pozitive sau schimbate intre ele
                        -controller1.left_stick_x,
                        -controller1.right_stick_x
                )
        );

//        if (controller1.startButtonOnce()){
//            robot.plane.releasePlane();
//        }
//
//        if (controller1.leftBumper()) {
//            robot.arm.gripperReleasePos();
//            gripper_released = true;
//        } else if (controller1.rightBumper()) {
//            robot.arm.gripperInitialPos();
//            gripper_released = false;
//        }
//
//        if (controller1.dpadRightOnce()) {
//            if (closed == true) {
//                robot.gripper.openBarier();
//            } else {
//                robot.gripper.closeBarier();
//            }
//            closed = !closed;
//        }
//
//        if (controller1.dpadLeftOnce()) {
//            gheare = gheare + 1;
//            if (gheare % 3 == 0){
//                robot.gripper.leavePixels();
//            }else if (gheare % 3 == 1){
//                robot.gripper.defaultPickupPixelPos();
//            } else if (gheare % 3 == 2) {
//                robot.gripper.pickPixels();
//            }
//        }

        // pana aici sa scrii cod
        if(!Utils.isDone(lastArmMove) || !Utils.isDone(lastSliderMove)) {
            return ;
        }

        // ------- controlling the arm positions -----
        else if (controller1.YOnce()) {
            arm_value = 835;
//            armIsUp = true;
            lastArmMove = robot.arm.raiseArm(arm_value, RAISE_POWER);
        } else if (controller1.BOnce()) {
            arm_value = 750;
            lastArmMove = robot.arm.raiseArm(arm_value, RAISE_POWER);
        } else if (controller1.XOnce()) {
            arm_value = 250;
            lastArmMove = robot.arm.raiseArm(arm_value, RAISE_POWER);
        } else if (controller1.AOnce()) {
            arm_value = 0;
            lastArmMove = robot.arm.raiseArm(arm_value, RAISE_POWER);
        }

        if (controller1.leftBumper()) {
            robot.lift.setDownPosition();
            gripper_released = true;
        } else if (controller1.rightBumper()) {
            robot.lift.setUpPosition();
            gripper_released = false;
        }

        // ------- controlling the lift -----
        if(!Utils.isDone(lastRightLift) || !Utils.isDone(lastLeftLift)) {
            return ;
        } else if (controller1.YOnce()) {
            arm_value = 2000;
        }
        lastRightLift = robot.lift.liftUpLeft(arm_value, RAISE_POWER);
        lastLeftLift = robot.lift.liftUpRight(arm_value, RAISE_POWER);


        // ------- controlling the slider positions -----
//        else if (controller1.dpadUpOnce()) {
//            raise_value = 3000;
//            lastSliderMove = robot.slider.raiseSlider(raise_value, RAISE_POWER);
//        } else if (controller1.dpadDownOnce()) {
//            raise_value = 0;
//            lastSliderMove = robot.slider.raiseSlider(raise_value, RAISE_POWER);
//        }

        if (controller1.dpadUpOnce()) {
            if (slider_level < 5) {
                slider_level = slider_level + 1;
                if (slider_level == 1) {
                    slider_level = slider_level + 1;
                }
            }
            raise_value = 600 * slider_level;
            lastSliderMove = robot.slider.raiseSlider(raise_value, RAISE_POWER);
        } else if (controller1.dpadDownOnce() && gripper_released == false) {
            if (slider_level > 0) {
                slider_level = slider_level - 1;
            }
            raise_value = 600 * slider_level;
            lastSliderMove = robot.slider.raiseSlider(raise_value, RAISE_POWER);
        }

        // ------- printing the slider position -------
        telemetry.addData("Slider target value", raise_value);
        telemetry.addData("Slider position", robot.slider.getCurrentPositionSlider());
        telemetry.addLine("---------------------");
        telemetry.addData("Arm target value", arm_value);
        telemetry.addData("Arm position", robot.arm.getCurrentPositionArm());
        telemetry.addLine("---------------------");
        telemetry.addData("Lift target value", arm_value);
        telemetry.addData("lift position", robot.lift.getCurrentPositionArm());

        telemetry.update();

        // ------- moving the slider -------

        // ------- moving the arm -------
    }

}