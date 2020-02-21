package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.RunPath;
import frc.robot.commands.auto.RunPathBack;
import frc.robot.commands.auto.Turn;
import frc.robot.commands.auto.TurnAimShoot;
import frc.robot.commands.auto.autoDeCorrect;
import frc.robot.commands.auto.autoShoot;
import frc.robot.commands.auto.autoStoreValue;

public class AutoGroups extends RobotContainer{
    private static final Logger LOGGER = Logger.getLogger(AutoGroups.class.getName());
    


    public final static HashMap<String, SequentialCommandGroup> hm = new HashMap<String, SequentialCommandGroup>();

    public final static SequentialCommandGroup leftRook = new SequentialCommandGroup(
            new RunPathBack(leftArray1, rightArray1), new autoStoreValue(), new TurnAimShoot(), new autoShoot(3),
            new autoDeCorrect(), new Turn(leftTurnArray, rightTurnArray), new RunPath(leftArray2, rightArray2),
            new RunPathBack(leftArray2, rightArray2));

    public static void startAutoGroups() {
        hm.put("leftRook", leftRook);
    


    }

    public static SequentialCommandGroup getAutoGroup(String autoNameGroup) {
        return hm.get(autoNameGroup);
    }


}