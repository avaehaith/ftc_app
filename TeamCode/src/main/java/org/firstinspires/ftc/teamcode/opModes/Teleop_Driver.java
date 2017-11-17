package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.ShelbyBot;
import org.firstinspires.ftc.teamcode.robot.TilerunnerGtoBot;
import org.firstinspires.ftc.teamcode.util.Input_Shaper;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;

@SuppressWarnings("unused")
@TeleOp(name="TeleopDriver", group="Tele")
//@Disabled
public class Teleop_Driver extends InitLinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initCommon(this, false, false, false, false);
        Input_Shaper ishaper = new Input_Shaper();
        DcMotor.ZeroPowerBehavior zeroPwr = DcMotor.ZeroPowerBehavior.BRAKE;
        boolean useSetVel = true;

        robot.setName(pmgr.getBotName());

        /* Initialize the hardware variables. */
        RobotLog.dd(TAG, "Initialize robot");
        robot.init(this);

        if (robot.numLmotors  > 0 &&
            robot.numRmotors  > 0)
        {
            RobotLog.dd(TAG, "Initialize drivetrain");
            dtrn.init(robot);

            dtrn.setRampUp(false);
            dtrn.setRampDown(false);
            robot.setDriveDir(ShelbyBot.DriveDir.INTAKE);
            RobotLog.dd(TAG, "Start Hdg %.2f", robot.autonEndHdg);

            lex = (DcMotorEx)(robot.leftMotors.get(0));
            rex = (DcMotorEx)(robot.rightMotors.get(0));
        }

        // Send telemetry message to signify robot waiting;
        dashboard.displayPrintf(0, "Hello Driver - I am %s", robot.getName());

        // Wait for the game to start (driver presses PLAY)
        while(!isStarted())
        {
            gpad1.update();
            gpad2.update();
            gpad1.log(1);
            gpad2.log(2);
            idle();
        }

        RobotLog.dd(TAG, "Telop_Driver starting");

        boolean toggle = false;
        boolean pActive = false;
        boolean eActive = false;

        if(robot.gripper != null)
        {
            //robot.gripper.setPosition(robot.GRIPPER_CLOSE_POS);
            robot.closeGripper();
        }

        if(robot.gpitch != null)
        {
            //robot.gpitch.setPosition(robot.GPITCH_UP_POS);
            robot.retractGpitch();
            pActive = true;
            currentPitchState = PitchState.PITCH_UP;
        }

        if(robot.elevMotor != null)
        {
            robot.elevMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.elevMotor.setPower(0.0);
            robot.elevMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            gpad1.update();
            gpad2.update();

            boolean toggle_run_mode   = gpad1.just_pressed(ManagedGamepad.Button.X);
            boolean invert_drive_dir  = gpad1.just_pressed(ManagedGamepad.Button.Y);

            boolean step_driveType    = gpad1.just_pressed(ManagedGamepad.Button.A);
            boolean toggle_float      = gpad1.just_pressed(ManagedGamepad.Button.B);
            boolean toggle_vel        = gpad1.just_pressed(ManagedGamepad.Button.R_BUMP);

            boolean left_step         = gpad1.pressed(ManagedGamepad.Button.D_LEFT);
            boolean right_step        = gpad1.pressed(ManagedGamepad.Button.D_RIGHT);
            boolean fwd_step          = gpad1.pressed(ManagedGamepad.Button.D_UP);
            boolean back_step         = gpad1.pressed(ManagedGamepad.Button.D_DOWN);

            boolean lowerElev         = gpad2.just_pressed(ManagedGamepad.Button.D_DOWN);
            boolean raiseElev         = gpad2.just_pressed(ManagedGamepad.Button.D_UP);

            boolean gripper_open_par  = gpad2.pressed(ManagedGamepad.Button.A);
            boolean gripper_open      = gpad2.pressed(ManagedGamepad.Button.B);
            boolean toggle_gpitch     = gpad2.just_pressed(ManagedGamepad.Button.X);
            boolean toggle_jflicker   = gpad2.just_pressed(ManagedGamepad.Button.Y);

            double raw_left     = -gpad1.value(ManagedGamepad.AnalogInput.L_STICK_Y);
            double raw_right    = -gpad1.value(ManagedGamepad.AnalogInput.R_STICK_Y);
            double raw_turn     =  gpad1.value(ManagedGamepad.AnalogInput.R_STICK_X);

            double elev         = -gpad2.value(ManagedGamepad.AnalogInput.R_STICK_Y);
            double pitch        = -gpad2.value(ManagedGamepad.AnalogInput.L_STICK_Y);

            DcMotor.ZeroPowerBehavior zeroPowBeh = DcMotor.ZeroPowerBehavior.UNKNOWN;

            if(robot.leftMotor  != null) zeroPowBeh = robot.leftMotor.getZeroPowerBehavior();

            // Run wheels in tank mode
            // (note: The joystick goes negative when pushed forwards, so negate it)

            dashboard.displayPrintf(0, "TMODE " + driveType);
            dashboard.displayPrintf(1, "L_IN %4.2f", raw_left);
            dashboard.displayPrintf(2, "R_IN %4.2f", raw_right);
            double shp_left  = ishaper.shape(raw_left,  0.1);
            double shp_right = ishaper.shape(raw_right, 0.1);
            double shp_turn  = ishaper.shape(raw_turn, 0.1);
            elev  = ishaper.shape(elev);

            double outPitch = Range.scale(pitch, -1.0, 1.0,
                    robot.GPITCH_MIN, robot.GPITCH_MAX);

            if(robot.GPITCH_DOWN_POS > robot.GPITCH_UP_POS)
                outPitch = 1.0 - outPitch;

            double arcadeTurnScale = 0.5;

            if(toggle_vel) useSetVel = !useSetVel;

            double step_dist = 2.0;
            double step_spd  = 0.4;
            double step_ang  = 5.0;

            double fHdg = robot.getGyroFhdg();
            double detail_speed = 0.14;

            double maxIPS = 40.0;
            double maxRPS = maxIPS/(4.0*Math.PI);
            double maxDPS = maxRPS*360.0;

            boolean useD = false;

            double left  = 0.0;
            double right = 0.0;
            double turn  = 0.0;

            if(fwd_step)
            {
                RobotLog.dd(TAG, "In fwd_step");
                left  = detail_speed;
                right = detail_speed;
                useD = true;
                //dtrn.driveDistanceLinear(step_dist, step_spd, Drivetrain.Direction.FORWARD);
                RobotLog.dd(TAG, "Done fwd_step");
            }
            else if(back_step)
            {
                RobotLog.dd(TAG, "In back_step");
                left  = -detail_speed;
                right = -detail_speed;
                useD = true;
                //dtrn.driveDistanceLinear(step_dist, step_spd, Drivetrain.Direction.REVERSE);
                RobotLog.dd(TAG, "Done back_step");
            }
            else if(left_step)
            {
                RobotLog.dd(TAG, "In left_step");
                left  = -detail_speed;
                right =  detail_speed;
                useD = true;
                //dtrn.setInitValues();
                //dtrn.ctrTurnToHeading(fHdg + step_ang, step_spd);
                RobotLog.dd(TAG, "Done left_step");
            }
            else if(right_step)
            {
                RobotLog.dd(TAG, "In right_step");
                left  =  detail_speed;
                right = -detail_speed;
                useD = true;
                //dtrn.setInitValues();
                //dtrn.ctrTurnToHeading(fHdg - step_ang, step_spd);
                RobotLog.dd(TAG, "Done right_step");
            }
            else
            {
                if(useSetVel)
                {
                    switch (driveType)
                    {
                        case TANK_DRIVE:
                            left  = raw_left;
                            right = raw_right;
                            break;

                        case ARCADE_DRIVE:
                            left  = raw_right;
                            right = left;
                            left  += raw_turn*arcadeTurnScale;
                            right -= raw_turn*arcadeTurnScale;
                    }
                }
                else
                {
                    switch (driveType)
                    {
                        case TANK_DRIVE:
                            left  = shp_left;
                            right = shp_right;
                            break;

                        case ARCADE_DRIVE:
                            left = shp_right;
                            right = left;
                            turn = (1 - Math.abs(left)) * turn;
                            if(turn < 0.95) turn *= arcadeTurnScale;
                            left  += turn;
                            right -= turn;
                            break;
                    }
                }

                double vmax = Math.max(Math.abs(left), Math.abs(right));
                if(vmax > 1.0)
                {
                    left  /= vmax;
                    right /= vmax;
                }
            }

            double out_left = left;
            double out_right = right;

            if(useSetVel)
            {
                out_left  = maxDPS*left;
                out_right = maxDPS*right;
                lex.setVelocity(out_left,  AngleUnit.DEGREES);
                rex.setVelocity(out_right, AngleUnit.DEGREES);
            }
            else
            {
                out_left  = left;
                out_right = right;
                robot.leftMotors.get(0).setPower(out_left);
                robot.rightMotors.get(0).setPower(out_right);
            }

            dashboard.displayPrintf(4, "TURN  %4.2f", turn);
            dashboard.displayPrintf(5, "L_OUT %4.2f", left);
            dashboard.displayPrintf(6, "R_OUT %4.2f", right);
            dashboard.displayPrintf(7, "L_CNT %d", robot.leftMotor.getCurrentPosition());
            dashboard.displayPrintf(8, "R_CNT %d", robot.rightMotor.getCurrentPosition());
            dashboard.displayPrintf(9, "DTYPE " + driveType);
            dashboard.displayPrintf(10,"D_DIR " + robot.getDriveDir());
            dashboard.displayPrintf(11,"Z_PWR " + zeroPwr);
            dashboard.displayPrintf(12,"RMODE " + robot.leftMotor.getMode());
            dashboard.displayPrintf(13,"L_DIR " + robot.leftMotor.getDirection());
            dashboard.displayPrintf(14,"R_DIR " + robot.rightMotor.getDirection());
            dashboard.displayPrintf(15,"SVEL " + useSetVel);

            if(Math.abs(elev) > 0.001)
            {
                eActive = false;
            }

            if(!eActive)
            {
                robot.elevMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                int minElev = robot.liftPositions.get(0) + 40;
                int maxElev = robot.liftPositions.get(robot.liftPositions.size() - 1) - 40;
                if(robot.elevMotor.getCurrentPosition() < minElev && elev < 0.0)
                    elev = 0.0;
                if(robot.elevMotor.getCurrentPosition() > maxElev && elev > 0.0)
                    elev = 0.0;
                robot.elevMotor.setPower(elev);
            }

            if (lowerElev && curElevIdx > 0)
            {
                curElevIdx--;
                robot.elevMotor.setTargetPosition(robot.liftPositions.get(curElevIdx));
                robot.elevMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elev = 0.55;
                robot.elevMotor.setPower(elev);
                eActive = true;
            } else if (raiseElev && curElevIdx < robot.liftPositions.size() - 1)
            {
                curElevIdx++;
                robot.elevMotor.setTargetPosition(robot.liftPositions.get(curElevIdx));
                robot.elevMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elev = 0.85;
                robot.elevMotor.setPower(elev);
                eActive = true;
            }

            if(toggle_float)
            {
                if(zeroPwr == DcMotor.ZeroPowerBehavior.BRAKE)
                    zeroPwr = DcMotor.ZeroPowerBehavior.FLOAT;
                else
                    zeroPwr = DcMotor.ZeroPowerBehavior.BRAKE;
                if(robot.leftMotor  != null) robot.leftMotor.setZeroPowerBehavior(zeroPwr);
                if(robot.rightMotor != null) robot.rightMotor.setZeroPowerBehavior(zeroPwr);
            }

            if(invert_drive_dir)
            {
                robot.invertDriveDir();
            }

            if(toggle_run_mode && robot.leftMotor != null && robot.rightMotor != null)
            {
                DcMotor.RunMode currMode = robot.leftMotor.getMode();
                if(currMode == DcMotor.RunMode.RUN_USING_ENCODER)
                {
                    dtrn.setMode(robot.leftMotors, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    dtrn.setMode(robot.rightMotors, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                else
                {
                    dtrn.setMode(robot.leftMotors, DcMotor.RunMode.RUN_USING_ENCODER);
                    dtrn.setMode(robot.rightMotors, DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }

            if(step_driveType && robot.leftMotor != null && robot.rightMotor != null)
            {
                switch(driveType)
                {
                    case TANK_DRIVE:
                        driveType = TELEOP_DRIVE_TYPE.ARCADE_DRIVE;
                        break;
                    case ARCADE_DRIVE:
                        driveType = TELEOP_DRIVE_TYPE.TANK_DRIVE;
                        break;
                }
            }

            // Gripper (a: Somewhat Open, b: all the way open, neither: closed)
            if (gripper_open)
            {
                //robot.gripper.setPosition(robot.GRIPPER_OPEN_POS);
                robot.openGripper();
            }
            else if (gripper_open_par)
            {
                //robot.gripper.setPosition(robot.GRIPPER_PARTIAL_POS);
                robot.partialGripper();
            }
            else
            {
                //robot.gripper.setPosition(robot.GRIPPER_CLOSE_POS);
                robot.closeGripper();
            }

            // Pitch (Gripper angle servo) (x: toggles between closed and open position)

            if (toggle_gpitch)
            {
                currentPitchState = (currentPitchState == PitchState.PITCH_UP) ?
                                            PitchState.PITCH_DOWN : PitchState.PITCH_UP;

                if (currentPitchState == PitchState.PITCH_UP)
                {
                    //robot.gpitch.setPosition(robot.GPITCH_UP_POS);
                    robot.retractGpitch();
                }
                else if (currentPitchState == PitchState.PITCH_DOWN)
                {
                    //robot.gpitch.setPosition(robot.GPITCH_DOWN_POS);
                    robot.deployGpitch();
                }
                pActive = true;
            }

            if (Math.abs(pitch) > 0.001)
            {
                pActive = false;
            }

            if(!pActive)
            {
                robot.gpitch.setPosition(outPitch);
            }

            // Jewel Flicker (y: toggles between up and down positions)
            if (toggle_jflicker)
            {
                currentFlickerState = (currentFlickerState == FlickerState.DOWN) ?
                                              FlickerState.UP : FlickerState.DOWN;

                if (currentFlickerState == FlickerState.DOWN)
                {
                    //robot.jflicker.setPosition(robot.JFLICKER_DOWN_POS);
                    robot.deployFlicker();
                }
                else if (currentFlickerState == FlickerState.UP)
                {
                    //robot.jflicker.setPosition(robot.JFLICKER_UP_POS);
                    robot.stowFlicker();
                }
            }

            // Pause for metronome tick.
            robot.waitForTick(10);
        }
    }

    private enum TELEOP_DRIVE_TYPE
    {
        TANK_DRIVE,
        ARCADE_DRIVE
    }

    private int curElevIdx = 0;

    private TELEOP_DRIVE_TYPE driveType = TELEOP_DRIVE_TYPE.ARCADE_DRIVE;

    public final double minPwr = 0.10;
    public final double minTrn = 0.10;

    private DcMotorEx lex = null;
    private DcMotorEx rex = null;

    private TilerunnerGtoBot robot = new TilerunnerGtoBot();
    private Drivetrain dtrn = new Drivetrain();

    enum PitchState {PITCH_UP, PITCH_DOWN };
    private PitchState currentPitchState = PitchState.PITCH_UP;

    private enum FlickerState { UP, DOWN }
    private FlickerState currentFlickerState = FlickerState.UP;

    private static final String TAG = "SJH_TD";
}