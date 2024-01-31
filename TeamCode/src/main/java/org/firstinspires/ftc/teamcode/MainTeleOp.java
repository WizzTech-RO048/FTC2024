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
    private Controller controller1, controller2;
    private SampleMecanumDrive drive;

    private int raise_value, arm_value;
    public double RAISE_POWER = 1.0;
    private boolean closed, gripper_released;
    private boolean sculatoare;
    private int last_arm_position; // 0 - a, 1 - x, 2 - b, 3 - y
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
        controller2 = new Controller(gamepad2);

        robot.plane.grabPlane();

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        robot.gripper.openBarier();
        closed = false;
        sculatoare = false;
        last_arm_position = 0;
        gripper_released = false;
        robot.lift.setDownPosition();
//        robot.arm.gripperInitialPos();
        robot.gripper.rotateIntake(-0.3);
    }

    // ------ the emergency stop function ---------
//    public void stop() { robot.stopRobot(); }

    @Override
    public void loop() {
        controller1.update();
        controller2.update();


        // controller 1
        // - movement
        // - avion
        // - intake rotire - a primi
        // - lift

        // controller 2
        // - rotire reverse intake
        // - ridicare arm
        // - control slider
        // - cutie rotate
        // - bariera

        // --------- (BODO) movement general al robotului ---------
        drive.setWeightedDrivePower(
                new Pose2d(
                        -controller1.left_stick_y, // gen astea negative / pozitive sau schimbate intre ele
                        -controller1.left_stick_x,
                        -controller1.right_stick_x
                )
        );

        // --------- (BODO) lansare avion ---------
        if (controller1.dpadUpOnce()){
            robot.plane.releasePlane();
        }

        // --------- (BODO) intake primire ---------
        double rotation_speed1 = controller1.right_trigger - controller1.left_trigger;
        robot.gripper.rotateIntake(rotation_speed1);

        // --------- (BODO) control lift ---------
        if (controller1.leftBumper()) {
            robot.lift.setDownPosition();
            gripper_released = true;
        } else if (controller1.rightBumper()) {
            robot.lift.setUpPosition();
            gripper_released = false;
        }

        if(!Utils.isDone(lastRightLift) || !Utils.isDone(lastLeftLift)) {
            return ;
        } else if (controller1.YOnce()) {
            arm_value = 2500;
            lastRightLift = robot.lift.liftUpLeft(arm_value, RAISE_POWER);
            lastLeftLift = robot.lift.liftUpRight(arm_value, RAISE_POWER);
        }



        // ------- (BELE) controlling the arm positions -----
        if(!Utils.isDone(lastArmMove) || !Utils.isDone(lastSliderMove)) {
            return ;
        }

        else if (controller2.YOnce()) {
            arm_value = 835;

            if (last_arm_position == 0) {
                robot.arm.gripperSafety();
                robot.gripper.closeBarier();
            }

//            armIsUp = true;
            lastArmMove = robot.arm.raiseArm(arm_value, RAISE_POWER);
            last_arm_position = 3;
        } else if (controller2.BOnce()) {
            arm_value = 750;

            if (last_arm_position == 0) {
                robot.arm.gripperSafety();
                robot.gripper.closeBarier();
            }

            robot.arm.gripperSafety();
            robot.gripper.closeBarier();

            lastArmMove = robot.arm.raiseArm(arm_value, RAISE_POWER);
            last_arm_position = 2;
        } else if (controller2.XOnce()) {
            arm_value = 250;

            if (last_arm_position == 0) {
                robot.arm.gripperSafety();
                robot.gripper.closeBarier();
            }

            robot.arm.gripperSafety();
            robot.gripper.closeBarier();

            lastArmMove = robot.arm.raiseArm(arm_value, RAISE_POWER);
            last_arm_position = 1;
        } else if (controller2.AOnce()) {
            arm_value = 0;

            lastArmMove = robot.arm.raiseArm(arm_value, RAISE_POWER);
            last_arm_position = 0;

            robot.arm.gripperAfterArm();
            robot.gripper.openBarier();
        }

        // --------- (BELE) intake iesire ---------
//        double rotation_speed2 = controller2.right_trigger - controller2.left_trigger;
//        robot.gripper.rotateIntake(rotation_speed2);

        // ------- (BELE) basculare cutie intake -------
        if (controller2.dpadLeftOnce()) {
            if(gripper_released == true) {
                robot.arm.gripperInitialPos();
            } else {
                robot.arm.gripperReleasePos();
            }
            gripper_released = !gripper_released;
        }

        // ------- (BELE) controlare bariera -------
        if (controller2.dpadRightOnce()) {
            if (closed == true) {
                robot.gripper.openBarier();
            } else {
                robot.gripper.closeBarier();
            }
            closed = !closed;
        }

        // ------- (BELE) controlling the slider positions -----
        if (controller2.dpadUpOnce()) {
            if (slider_level < 5) {
                slider_level = slider_level + 1;
                if (slider_level == 1) {
                    slider_level = slider_level + 1;
                }
            }
            raise_value = 600 * slider_level;
            lastSliderMove = robot.slider.raiseSlider(raise_value, RAISE_POWER);
        } else if (controller2.dpadDownOnce() && gripper_released == false) {
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