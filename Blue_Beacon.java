package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "Kip: Blue Beacon & Shoot", group = "Kip")
@Disabled
public class Blue_Beacon extends LinearOpMode {


    /* Declare OpMode members. */
    Kip Kip = new Kip();
    private ElapsedTime runtime = new ElapsedTime();
    ColorSensor beaconColor;
    ColorSensor lineColor;


    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.6;

    @Override
    public void runOpMode() throws InterruptedException {


        float hsvValues[] = {0F, 0F, 0F};

        Kip.init(hardwareMap);
        beaconColor = hardwareMap.colorSensor.get("beacon");
        lineColor = hardwareMap.colorSensor.get("line");

        I2cAddr beaconAddress = I2cAddr.create8bit(0x3a);
        I2cAddr lineAddress = I2cAddr.create8bit(0x3c);
        beaconColor.setI2cAddress(beaconAddress);
        lineColor.setI2cAddress(lineAddress);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        updateTelemetry(telemetry);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("LEFT POS",
                Kip.leftMotor2.getCurrentPosition());
        telemetry.addData("RIGHT POS", Kip.rightMotor1.getCurrentPosition());
        updateTelemetry(telemetry);
        sleep(2000);


        telemetry.addData("Encoders Reset", "Press Start");
        updateTelemetry(telemetry);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        resetEncoders(1000);
        encoderDrive(-DRIVE_SPEED, -24, -24, 4.0); // S1: Forward 70 Inches with 5 Sec timeout
        punchBall(1.0, 200);
        punchBall(1.0, 700);
        runCollector(0.7, 1500);
        punchBall(1.0, 250);
        punchBall(1.0, 700);
        driveWithoutEncoders(300);
        resetEncoders(1500);
        encoderDrive(TURN_SPEED, 8, -8, 5.0);
        encoderDrive(-TURN_SPEED, -8, 8, 5.0);
        resetEncoders(1000);
        encoderDrive(DRIVE_SPEED, 20, 20, 5.0);
        encoderDrive(DRIVE_SPEED, 4, -4, 5.0);
        driveTillWhiteLine(4, 4, 4);

        //encoderDrive(DRIVE_SPEED, -35, -35, 5.0);
        //turnTillWhiteLine(4, 4, 4);
        //resetEncoders(1500);
        //encoderDrive(DRIVE_SPEED, 0, 2, 5.0);



         // S3: Reverse 24 Inches with 4 Sec timeout

          // pause for servos to move

    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double Speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newLeftTarget1;
        int newRightTarget1;
        int newLeftTarget2;
        int newRightTarget2;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newRightTarget1 = Kip.rightMotor1.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightTarget2 = Kip.rightMotor2.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            Kip.rightMotor2.setTargetPosition(newRightTarget1);
            Kip.rightMotor1.setTargetPosition(newRightTarget2);

            telemetry.addData("targetright", newRightTarget1);
            updateTelemetry(telemetry);

            // Turn On RUN_TO_POSITION
            Kip.rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Kip.rightMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            Kip.rightMotor1.setPower(Math.abs(Speed));
            Kip.rightMotor2.setPower(Math.abs(Speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (Kip.rightMotor2.isBusy()
                     && Kip.rightMotor1.isBusy())) {

                Kip.leftMotor1.setPower(Speed);
                Kip.leftMotor2.setPower(Speed);

                // Display it for the driver.
                telemetry.addData("Left",
                        Kip.leftMotor2.getCurrentPosition());
                telemetry.addData("newRight", newRightTarget1);
                telemetry.addData("Right", Kip.rightMotor1.getCurrentPosition());
                updateTelemetry(telemetry);

                // Allow time for other processes to run.
            }

            // Stop all motion;
            Kip.leftMotor1.setPower(0);
            Kip.rightMotor1.setPower(0);
            Kip.leftMotor2.setPower(0);
            Kip.rightMotor2.setPower(0);

            // Turn off RUN_TO_POSITION
            Kip.rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Kip.rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

        public void driveTillWhiteLine(int blue, int red, int green) throws InterruptedException {

            Kip.leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Kip.rightMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Kip.leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Kip.rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while(lineColor.blue() < blue && lineColor.red() < red && lineColor.green() < green) {
                Kip.leftMotor1.setPower(-0.5);
                Kip.rightMotor1.setPower(-0.5);
                Kip.leftMotor2.setPower(-0.5);
                Kip.rightMotor2.setPower(-0.5);

                telemetry.addData("Red", lineColor.red());
                telemetry.addData("Blue", lineColor.blue());
                telemetry.addData("Green", lineColor.green());
                updateTelemetry(telemetry);
            }

            Kip.leftMotor1.setPower(0);
            Kip.rightMotor1.setPower(0);
            Kip.leftMotor2.setPower(0);
            Kip.rightMotor2.setPower(0);

            Kip.rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Kip.rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(500);
        }
        public void driveWithoutEncoders(int milliseconds) throws InterruptedException {

            Kip.leftMotor1.setPower(-0.4);
            Kip.rightMotor1.setPower(-0.4);
            Kip.leftMotor2.setPower(-0.4);
            Kip.rightMotor2.setPower(-0.4);

            sleep(milliseconds);

            Kip.leftMotor1.setPower(0);
            Kip.rightMotor1.setPower(0);
            Kip.leftMotor2.setPower(0);
            Kip.rightMotor2.setPower(0);
        }
        public void punchBall(double punchSpeed, int time) throws InterruptedException {

            Kip.punchMotor.setPower(punchSpeed);

            sleep(time);

            Kip.punchMotor.setPower(0);

            sleep(500);
        }
        public void runCollector(double collectorSpeed, int time) throws InterruptedException {

            Kip.collectorMotor.setPower(collectorSpeed);

            sleep(time);

            Kip.collectorMotor.setPower(0);

            sleep(500);
        }
        public void resetEncoders(int sleepTime) {

            Kip.rightMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Kip.rightMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(sleepTime);

            Kip.rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Kip.rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(500);
        }
    public void turnTillWhiteLine(int blue, int red, int green) throws InterruptedException {

        while(lineColor.blue() < blue && lineColor.red() < red && lineColor.green() < green) {
            Kip.leftMotor1.setPower(-0.2);
            Kip.rightMotor1.setPower(0.2);
            Kip.leftMotor2.setPower(-0.2);
            Kip.rightMotor2.setPower(0.2);

            telemetry.addData("Red", lineColor.red());
            telemetry.addData("Blue", lineColor.blue());
            telemetry.addData("Green", lineColor.green());
            updateTelemetry(telemetry);
        }

        Kip.leftMotor1.setPower(0);
        Kip.rightMotor1.setPower(0);
        Kip.leftMotor2.setPower(0);
        Kip.rightMotor2.setPower(0);

        sleep(500);
    }
    public void turnWithoutEncoders(double speed, int milliseconds) throws InterruptedException {

        Kip.leftMotor1.setPower(speed);
        Kip.rightMotor1.setPower(-speed);
        Kip.leftMotor2.setPower(speed);
        Kip.rightMotor2.setPower(-speed);

        sleep(milliseconds);

        Kip.leftMotor1.setPower(0);
        Kip.rightMotor1.setPower(0);
        Kip.leftMotor2.setPower(0);
        Kip.rightMotor2.setPower(0);
    }

    }

