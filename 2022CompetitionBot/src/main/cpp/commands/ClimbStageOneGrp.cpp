// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/WaitCommand.h>
#include "commands/ClimbStageOneGrp.h"
#include "commands/ClimberArmRaiseCmd.h"
#include "commands/ClimberArmLowerCmd.h"
#include "commands/ClimberArmsLatchReleaseCmd.h"

ClimbStageOneGrp::ClimbStageOneGrp(ClimberSub *climberSub) {
  // Required Initial Start State:
  //   1. robot facing front entrance of hangar, between medium and high rungs
  //   2. hook arms fully folded in (retracted) and pinned at lower position
  //   3. mast arm raised (extended) all the way
  //   4. mast arm shaft against medium rung (so mast will hook from rear of medium rung)

  // NOTES: 1. To fully lower mast arm pulling robot up from lowest to highest position
  //           takes roughly 1.4 seconds.
  //        2. Mast arm travel distance from its lowest (starting retracted) position to
  //           highest (extended) position is 21 inches.
  //        3. Mast arm travel distance from highest position down to securely grabbing the
  //           medium rung, but not really lifting the robot is 3 inches.
  //        4. Mast arm travel distance from highest position down to securely grabbing the
  //           high rung, and just barely lifting the hook arm hooks off the high rung
  //           taking all pressure off the hook arms is xxx cm.
  //        5. To fully unfold (extend) hook arms from completely folded in (retracted)
  //           position takes roughly 1 second.
  //        6. To fully fold in (retract) hook arms from completely unfolded (extended)
  //           position while hanging and swinging on hook arms takes roughly 1 second when
  //           unhooking from medium rung to hang and swing from high rung - the hook arms
  //           retract in 2 waves pausing momentarily between waves as they fight the swing.

  // Climb from medium to traversal rungs.
  AddCommands(

    // ---------------------------------------------------------------------------------------
    // STAGE 1
    // ---------------------------------------------------------------------------------------
    
    // 1. Partially lower (retract) mast arm only enough to secure the mast hook on the
    //    medium rung before unfolding (extending) hook arms (this helps avoid being
    //    bumped out of position while the hook arms extend):
    ClimberArmLowerCmd(climberSub, 85.0),
    // 2. Unfold (extend) hook arms:
    ClimberArmsLatchReleaseCmd(climberSub, false),
    frc2::WaitCommand(0.3_s),
    // 3. Fully lower (retract) mast arm, lifting robot as high as possible:
    ClimberArmLowerCmd(climberSub, 0.0),

    // Robot is now hanging by mast arm from medium rung.
    // Hook arms are ready to grab high rung from rear.
    // Robot is not swinging.

    // 4. Fold in (retract) hook arms:
    ClimberArmsLatchReleaseCmd(climberSub, true),
    frc2::WaitCommand(0.2_s),
    // 5. Partially raise (extend) mast arm to lower robot and unhook mast from medium rung:
    ClimberArmRaiseCmd(climberSub, 60.0),

    // Robot is now detached from medium rung.
    // Robot is swinging on high rung by hook arms toward the rear of hangar.
    // Mast arm is not fully extended yet.

    // 6. Let hook arms fully fold in (retract):
    frc2::WaitCommand(2.0_s),
    // 7. Fully raise (extend) mast arm:
    ClimberArmRaiseCmd(climberSub, 100.0),

    // Mast arm hook *MUST* end up on rear side of the high rung.
    // IF IT DOESN'T, "KILL EVERYTHING" AND RESTART AUTO CLIMB IMMEDIATELY TO RECOVER.

    // ---------------------------------------------------------------------------------------
    // STAGE 2
    // ---------------------------------------------------------------------------------------

    // Robot is perfectly level and hanging on high rung by hook arms.
    // Robot is swinging toward the rear of hangar.
    // Hook arms are folded in (retracted).
    // Mast arm is fully raised (extended).

    // 8. Unfold (extend) hook arms to press mast against rear side of high rung:
    ClimberArmsLatchReleaseCmd(climberSub, false),
    frc2::WaitCommand(0.2_s),
    // 9. Partially lower (retract) mast arm to raise robot just enough to lift hook arms off
    //    high rung, but not let them pop off (i.e. hook tip insides are still contacting front
    //    of high rung preventing pop off):
    ClimberArmLowerCmd(climberSub, 85.0),
    // 10. Reverse hook arm - fold them in (retract) to take unhook pressure off high rung to
    //     avoid aggressive unhook.
    ClimberArmsLatchReleaseCmd(climberSub, true),
    // 11. Partially lower (retract) mast arm to raise robot a little more so hook arm hooks
    //     are clear of high rung.
    ClimberArmLowerCmd(climberSub, 70.0),
    // 12. Fully lower (retract) mast arm to raise robot as high as possible.
    ClimberArmLowerCmd(climberSub, 0.0),
    
    // Once mast arm is fully lowered (retracted),
    //   - robot is as high as possible
    //   - robot is at its forward most part of swing
    //   - robot starts swinging back toward the rear
    // We wait until now to reverse the hook arms to minimize swing amplification.
    
    // 13. Reverse hook arm - unfold them (extend) to reach for traversal rung while robot
    //     swings toward rear (toward traversal rung).
    ClimberArmsLatchReleaseCmd(climberSub, false),
    // 14. Keep raising robot a few times to fight any slippage from gravity and aggressive
    //     swinging while giving the hook arms time to fully extend.
    frc2::WaitCommand(0.2_s),
    ClimberArmLowerCmd(climberSub, 0.0),
    frc2::WaitCommand(0.2_s),
    ClimberArmLowerCmd(climberSub, 0.0),

    // ---------------------------------------------------------------------------------------
    // STAGE 3
    // ---------------------------------------------------------------------------------------

    // Robot is aggressively swinging on high rung by mast arm - swinging toward rear.
    // Hook arms are fully extended or on the way to be.
    // We expect hook arm hooks to make contact with rear of traversal rung and abruptly stop
    // robot swing.

    // 15. Fold in (retract) hook arms:
    ClimberArmsLatchReleaseCmd(climberSub, true),
    frc2::WaitCommand(0.2_s),

    // Hook arms must end up on rear side of traversal rung.
    // IF NOT, LET AUTO CLIMB COMPLETE AND THEN MANUALLY COMPLETE CLIMB TO RECOVER.

    // 16. Partially raise (extend) mast arm to lower robot and unhook mast from high rung.
    ClimberArmRaiseCmd(climberSub, 30.0),

    // Robot is aggressively swinging on traversal rung by hook arms.
    // Hook arms are becoming fully folded in (retracted).
    // Robot should end up straight (perfectly level once swing stops).

    // 17. Fully lower (retract) mast arm, returning it to start position for next match.
    ClimberArmLowerCmd(climberSub, 0.0)

    // TODO: If this final auto stage does not consistently grab the traversal run without
    //       needing to add more waits and logic using the gyro to determine robot tilt,
    //       we *could* add a new ~2 second auto to do this instead:
    //         - unfold (extend) hook arms
    //         - fully lower (retract) mast arm raising robot as high as possible
    //         - partially lower (extend) mast arm to lower robot to 85% height
    //         - wait as necessary to ensure hook arms are fully extended
    //         - fully lower (retract) mast arm raising robot as high as possible
    //         - fold in (retract) hook arms to grab traversal rung (wait 0.2 seconds after)
    //         - partially raise (extend) mast arm to lower robot and unhook mast from high rung
    //         - fully lower (retract) mast arm, returning it to start position for next match
  );
}

