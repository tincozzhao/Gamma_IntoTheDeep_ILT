package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;


@Autonomous(name="encoderTest")
public class encoderTest extends AutoHardware {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware();
        // Motor encoder parameter
        double ticksPerInch = 31.3;
        double ticksPerDegree = 15.6;

        robot.init(hardwareMap);

        //reset encoder
        robot.setAutoDriveMotorMode();

        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            int forwardTicks = 1000;
            driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.6,
                    true, robot.yaw0);
        }


    }
    }