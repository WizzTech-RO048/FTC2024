package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Autonomous")
public class Auto extends LinearOpMode {

    public enum Alliance {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }

    public enum StartPosition {
        AUDIENCE,
        BACKSTAGE
    }

    @Override
    public void waitForStart() {

    }


    @Override
    public void runOpMode() {

        while (!isStarted() && !isStopRequested()) {

        }
    }
}
