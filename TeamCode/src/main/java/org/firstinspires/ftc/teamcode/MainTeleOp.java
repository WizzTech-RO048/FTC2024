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
//    private SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    private int raise_value, arm_value;
    public double RAISE_POWER = 1.0;
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
        robot.gripper.release();

//        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    // ------ the emergency stop function ---------
//    public void stop() { robot.stopRobot(); }

    @Override
    public void loop() {
        controller1.update();

        // -------- controlling the robot movement ------
        // TOMICI: movement-ul ar trebui sa fie cel de roadrunner, nu am mai apucat sa il testez, dar ar trebui sa fie ok
        // (eventual doar pozitiile din joystick sa fie negative / pozitive)
//        drive.setWeightedDrivePower(
//                new Pose2d(
//                        -controller1.left_stick_y, // gen astea negative / pozitive sau schimbate intre ele
//                        -controller1.left_stick_x,
//                        -controller1.right_stick_x
//                )
//        );
        // faci tu troubleshooting la tot :)

//        drive.update();

        if (controller1.rightBumper()) {
            robot.gripper.grab();
        } else if (controller1.leftBumperOnce()) {
            robot.gripper.release();
        }

        // ------- controlling the slider positions -----
        if(!Utils.isDone(lastSliderMove)) { return ; }
        else if (controller1.YOnce()) { raise_value = 3500; }
        else if (controller1.BOnce()) { raise_value = 2500; }
        else if (controller1.XOnce()) { raise_value = 1000; }
        else if (controller1.AOnce()) { raise_value = 0; }
        else { return ; }

        // TOMICI: nu te lua dupa asta, lucreaza doar cu sliderul, implementarea arm-ului o sa o facem cand termina
        // baietii la mecanica pentru ca am dat de ceva probleme in folosirea in paralel a slider-ului si a
        // arm-ului cand se afla in functiune
        // ------- controlling the arm positions -----
        if(!Utils.isDone(lastArmMove)) { return ; }
        else if (controller1.YOnce()) { arm_value = 1000; }
        else if (controller1.BOnce()) { arm_value = 750; }
        else if (controller1.XOnce()) { arm_value = 250; }
        else if (controller1.AOnce()) { arm_value = 0; }
        else { return ; }

        // ------- printing the slider position -------
        telemetry.addData("Slider target value", raise_value);
        telemetry.addData("Slider position", robot.slider.getCurrentPositionSlider());
        telemetry.addLine("---------------------");
        telemetry.addData("Arm target value", arm_value);
        telemetry.addData("Arm position", robot.arm.getCurrentPositionSlider());

        telemetry.update();

        // ------- moving the slider -------
        // lastSliderMove = robot.slider.raiseSlider(raise_value, RAISE_POWER);

        // ------- moving the arm -------
        lastArmMove = robot.arm.raiseArm(arm_value, RAISE_POWER);
    }

}