package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.PreferenceMgr;

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcValueMenu;

@Autonomous(name = "Auton Config", group = "0")
public class AutoSetupMenu extends InitLinearOpMode implements FtcMenu.MenuButtons {

    private final String TAG = "Auton Menu";

    //The autonomous menu settings using sharedpreferences
    private PreferenceMgr prfMgr = new PreferenceMgr();
    private String club;
    private String bot;
    private String allianceColor;
    private String startPosition;
    private String parkPosition;
    private int    delay;

    public AutoSetupMenu()
    {
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        initCommon(this, false, false, false, false);
        dashboard.displayPrintf(0, "Starting Menu System");
        setup();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive())
        {
            idle();
        }
    }


    private void setup()
    {
        prfMgr.readPrefs();
        getPrefs();
        dashboard.displayPrintf(0, "INITIALIZING - Please wait for Menu");
        doMenus();
        dashboard.displayPrintf(0, "COMPLETE - Settings Written");
    }

    private void getPrefs()
    {
        club          = prfMgr.getClubName();
        bot           = prfMgr.getBotName();
        allianceColor = prfMgr.getAllianceColor();
        startPosition = prfMgr.getStartPosition();
        parkPosition  = prfMgr.getParkPosition();
        delay         = prfMgr.getDelay();

        RobotLog.dd(TAG, "Default Config Values:");
        RobotLog.dd(TAG, "Club:     %s", club);
        RobotLog.dd(TAG, "Bot:      %s", bot);
        RobotLog.dd(TAG, "Alliance: %s", allianceColor);
        RobotLog.dd(TAG, "startPos: %s", startPosition);
        RobotLog.dd(TAG, "parkPos:  %s", parkPosition);
        RobotLog.dd(TAG, "delay:    %d", delay);
    }

    private String botNames[]       = {"GTO1", "GTO2", "MEC"};
    private String alliances[]      = {"RED", "BLUE"};
    private String startPositions[] = {"START_1", "START_2"};
    private String parkPositions[]  = {"TRI_TIP"};

    //
    // Implements FtcMenu.MenuButtons interface.
    //

    @Override
    public boolean isMenuUpButton()   { return gamepad1.dpad_up;}

    @Override
    public boolean isMenuDownButton() { return gamepad1.dpad_down; }

    @Override
    public boolean isMenuEnterButton() { return gamepad1.a; }

    @Override
    public boolean isMenuBackButton() { return gamepad1.dpad_left; }

    private void doMenus()
    {
        FtcChoiceMenu<String> botMenu
                = new FtcChoiceMenu<>("Bot:",      null,         this);
        FtcChoiceMenu<String> allianceMenu
                = new FtcChoiceMenu<>("Alliance:", botMenu,      this);
        FtcChoiceMenu<String> startPosMenu
                = new FtcChoiceMenu<>("Start:",    allianceMenu, this);
        FtcChoiceMenu<String> parkMenu
                = new FtcChoiceMenu<>("Park:",     startPosMenu, this);
        FtcValueMenu  delayMenu
                = new FtcValueMenu("Delay:",       parkMenu,     this,
                                          0.0, 20.0, 1.0, 0.0, "%5.2f");

        //
        // remember last saved settings and reorder the menu with last run settings as the defaults
        //

        botMenu.addChoice(bot, bot, allianceMenu);
        for(String str : botNames)
        {
            if(!bot.equals(str))
                botMenu.addChoice(str, str, allianceMenu);
        }

        allianceMenu.addChoice(allianceColor, allianceColor, startPosMenu);
        for(String str : alliances)
        {
            if(!allianceColor.equals(str))
                allianceMenu.addChoice(str, str, startPosMenu);
        }

        startPosMenu.addChoice(startPosition, startPosition, parkMenu);
        for(String str : startPositions)
        {
            if(!startPosition.equals(str))
                startPosMenu.addChoice(str, startPosition, parkMenu);
        }

        parkMenu.addChoice(parkPosition, parkPosition, delayMenu);
        for(String str : parkPositions)
        {
            if(!parkPosition.equals(str))
                parkMenu.addChoice(str, str, delayMenu);
        }

        //
        // Walk the menu tree starting with the strategy menu as the root
        // menu and get user choices.
        //
        FtcMenu.walkMenuTree(botMenu, this);

        //
        // Set choices variables.
        //
        startPosition = startPosMenu.getChoiceText(startPosMenu.getCurrentChoice());
        parkPosition  = parkMenu.getChoiceText(parkMenu.getCurrentChoice());
        allianceColor = allianceMenu.getChoiceText(allianceMenu.getCurrentChoice());
        bot           = botMenu.getChoiceText(botMenu.getCurrentChoice());
        delay = (int)delayMenu.getCurrentValue();

        prfMgr.setBotName(bot);
        prfMgr.setStartPosition(startPosition);
        prfMgr.setParkPosition(parkPosition);
        prfMgr.setAllianceColor(allianceColor);
        prfMgr.setDelay(delay);

        RobotLog.dd(TAG, "Writing Config Values:");
        RobotLog.dd(TAG, "Club:     %s", club);
        RobotLog.dd(TAG, "Bot:      %s", bot);
        RobotLog.dd(TAG, "Alliance: %s", allianceColor);
        RobotLog.dd(TAG, "startPos: %s", startPosition);
        RobotLog.dd(TAG, "parkPos:  %s", parkPosition);
        RobotLog.dd(TAG, "delay:    %d", delay);

        //write the options to sharedpreferences
        prfMgr.writePrefs();

        //read them back to ensure they were written
        getPrefs();

        int lnum = 1;
        dashboard.displayPrintf(lnum++, "Bot:      " + bot);
        dashboard.displayPrintf(lnum++, "Alliance: " + allianceColor);
        dashboard.displayPrintf(lnum++, "Start:    " + startPosition);
        dashboard.displayPrintf(lnum++, "Park:     " + parkPosition);
        dashboard.displayPrintf(lnum,   "Delay:    " + String.valueOf(delay));
    }
}
