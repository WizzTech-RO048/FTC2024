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
    private boolean closed;
    private boolean gheare;
    private ScheduledFuture<?> lastArmMove, lastSliderMove;

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

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.gripper.openBarier();
        closed = false;

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
        // faci tu troubleshooting la tot :)

//        drive.update();
        // de aici sa incepi sa scrii cod


        if (controller1.leftBumper()) {
            robot.arm.gripperReleasePos();
        } else if (controller1.rightBumper()) {
            robot.arm.gripperInitialPos();
        }

        if (controller1.dpadLeftOnce()) {
            if (closed == true) {
                robot.gripper.openBarier();
            } else {
                robot.gripper.closeBarier();
            }
            closed = !closed;
        }

        if (controller1.dpadRightOnce()) {
            if ( gheare==true){
                robot.gripper.leavePixels();
            }else{
                robot.gripper.pickPixels();
            }
            gheare = !gheare;
        }



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

        // ------- controlling the slider positions -----
        else if (controller1.dpadUpOnce()) {
            raise_value = 2500;
            lastSliderMove = robot.slider.raiseSlider(raise_value, RAISE_POWER);
        } else if (controller1.dpadDownOnce()) {
            raise_value = 0;
            lastSliderMove = robot.slider.raiseSlider(raise_value, RAISE_POWER);
        }

        // ------- printing the slider position -------
        telemetry.addData("Slider target value", raise_value);
        telemetry.addData("Slider position", robot.slider.getCurrentPositionSlider());
        telemetry.addLine("---------------------");
        telemetry.addData("Arm target value", arm_value);
        telemetry.addData("Arm position", robot.arm.getCurrentPositionArm());

        telemetry.update();

        // ------- moving the slider -------

        // ------- moving the arm -------
    }

}