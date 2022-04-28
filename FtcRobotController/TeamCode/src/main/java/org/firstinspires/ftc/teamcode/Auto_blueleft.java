package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Auto blueLeft", group="Auto")
//@Disabled
public class Auto_blueleft extends LinearOpMode {

    /* Declare OpMode members. */

    // Declare Mecanum Drive
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearRightDrive = null;
    private DcMotor spinner=null;
    private DcMotor armMotor = null;
    private CRServo intake = null;
    // DECLARATIONS GO ABOVE

    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // goBuilda Motor 537.7 PPR
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        // hardwareMap of Mecanum Drive
        frontLeftDrive = hardwareMap.get(DcMotor.class, "left_drive1");
        rearLeftDrive  = hardwareMap.get(DcMotor.class, "left_drive2");
        frontRightDrive = hardwareMap.get(DcMotor.class, "right_drive1");
        rearRightDrive = hardwareMap.get(DcMotor.class, "right_drive2");
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        intake = hardwareMap.get(CRServo.class, "intake");
        spinner = hardwareMap.get(DcMotor.class, "spinner");
        // REPLACE ABOVE.

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                frontLeftDrive.getCurrentPosition(),
                frontRightDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //Movement start here
        strafeDrive("left", 1100,1);//move right to aim at the Alliance Tower
        encoderDrive(1, -1,-1,5.0);
        armMotor.setPower(-5.0);//raise up arm
        sleep(600);
        armMotor.setPower(0);//stop raising arm
        encoderDrive(DRIVE_SPEED,2,2,5.0);
        intake.setPower(-1);//spit out box
        sleep(3000);
        intake.setPower(0);//stop intake
        encoderDrive(DRIVE_SPEED,-2,-2,5.0);
        strafeDrive("right",600 ,1);//move left to aim barrier
        armMotor.setPower(0.5);//raise arm down
        sleep(600);
        armMotor.setPower(0);//stop raising arm
        encoderDrive(1,5,5,5.0);
        encoderDrive(1, -90,-90,5.0);//back all the way into warehouse
        //done



        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void strafeDrive(String direction,
                            long time, double speed){
        if (direction.equals("right")){
            frontRightDrive.setPower(-speed);
            rearRightDrive.setPower(speed);
            frontLeftDrive.setPower(speed);
            rearLeftDrive.setPower(-speed);
            sleep(time);
            frontRightDrive.setPower(0);
            rearRightDrive.setPower(0);
            frontLeftDrive.setPower(0);
            rearLeftDrive.setPower(0);
        }
        if (direction.equals("left")){
            frontRightDrive.setPower(1);
            rearRightDrive.setPower(-1);
            frontLeftDrive.setPower(-1);
            rearLeftDrive.setPower(1);
            sleep(time);
            frontRightDrive.setPower(0);
            rearRightDrive.setPower(0);
            frontLeftDrive.setPower(0);
            rearLeftDrive.setPower(0);
        }
        sleep(1000);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newfrontLeftTarget;
        int newfrontRightTarget;
        int newrearLeftTarget;
        int newrearRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newfrontLeftTarget = frontLeftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newfrontRightTarget = frontRightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newrearLeftTarget = rearLeftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newrearRightTarget = rearRightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            frontLeftDrive.setTargetPosition(newfrontLeftTarget);
            frontRightDrive.setTargetPosition(newfrontRightTarget);
            rearLeftDrive.setTargetPosition(newrearLeftTarget);
            rearRightDrive.setTargetPosition(newrearRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            frontLeftDrive.setPower(Math.abs(speed));
            frontRightDrive.setPower(Math.abs(speed));
            rearLeftDrive.setPower(Math.abs(speed));
            rearRightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (frontLeftDrive.isBusy() && frontRightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newfrontLeftTarget,  newfrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        frontLeftDrive.getCurrentPosition(),
                        frontRightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            rearLeftDrive.setPower(0);
            rearRightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

              sleep(1000);   // optional pause after each move
        }
    }
}
