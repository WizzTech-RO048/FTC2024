package org.firstinspires.ftc.teamcode.Robot;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

public class Gripper {
    private final double GRAB_POSITION_1 = 0.5, GRAB_POSITION_2 = 1.0;
    private final double RELEASE_POSITION_1 = GRAB_POSITION_1+0.1;
    private final double RELEASE_POSITION_2 = GRAB_POSITION_2-0.1;

    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;

    private final Servo left_gripper, right_gripper;

    Gripper(@NonNull final Parameters parameters) {
        telemetry = Objects.requireNonNull(parameters.telemetry, "Telemetry object was not set");
        hardwareMap = Objects.requireNonNull(parameters.hardwareMap, "HardwareMap was not set");

        left_gripper = hardwareMap.get(Servo.class, "left_gripper");
        right_gripper = hardwareMap.get(Servo.class, "right_gripper");
    }

    public void grab() {
        left_gripper.setPosition(GRAB_POSITION_1);
        right_gripper.setPosition(GRAB_POSITION_2);
    }

    public void release() {
        left_gripper.setPosition(RELEASE_POSITION_1);
        right_gripper.setPosition(RELEASE_POSITION_2);
    }

    public static class Parameters {
        public HardwareMap hardwareMap;
        public Telemetry telemetry;
    }
}