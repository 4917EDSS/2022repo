// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClimbStageOneGrp.h"
#include "commands/ClimberArmRaiseCmd.h"
#include "commands/ClimberArmLowerCmd.h"
#include "commands/ClimberArmsLatchReleaseCmd.h"

ClimbStageOneGrp::ClimbStageOneGrp(ClimberSub *climberSub) {
  // Required Initial Start State:
  //   1. robot facing front entrance of hangar, between medium and high rungs
  //   2. mast arm raised (extended) all the way
  //   3. mast arm shaft against medium rung (so mast will hook from rear of medium rung)

  // TODO: We need more granularity with delays (more tweaking options) and should move all
  //       delays out of the raise/lower and latch/release commands to make this easier to
  //       tweak and understand. Do this in steps as follows:
  //       1. Create a DelayCmd command that takes a double parameter named delaySeconds and
  //          simply completes once that amount of time has passed.
  //       2. Add calls to new DelayCmd wherever any of these command calls delay:
  //              ClimberArmsLatchReleaseCmd
  //              ClimberArmLowerCmd
  //              ClimberArmRaiseCmd
  //          Do this with the intent that *ALL* delays in those commands will be removed.
  //       3. Remove all delays in the above 3 existing commands - all start and end delays
  //          and all related parameters and member variables.
  //       4. Change ClimberArmRaiseCmd bool isPartial parameter to be a double parameter named
  //          raisePercentage and make the command determine the target climb height based on
  //          the requested percentage of kClimberArmMaxHeight. Also rename climberArmMaxHeight
  //          in ClimberArmRaiseCmd::IsFinished to ba nameed targetClimbHeight to better
  //          represent what it is.
  //          NOTES:
  //            * We currently divide by 3 which would be equivalent to passing in 33.333 for
  //              raisePercentage after these changes are made.
  //            * To fully raise, we would pass in 100.0 for raisePercentage.
  //       5. Change ClimberArmLowerCmd to add a double parameter named lowerPercentage and
  //          make the command determine the target height to lower to based on the requested
  //          percentage of kClimberArmMinHeight. Make this similar to ClimberArmRaiseCmd.
  //          NOTES:
  //            * We may need to partially lower the mast arm and this will let us.
  //            * To fully lower, we would pass in 100.0 for lowerPercentage.
  //       6. At this point, the code execution behaviour should be identical to what it was
  //          before these changes.
  //       7. Make changes as indicated by TODO comments below.
  //       8. Test and tweak delays and raise/lower percentages.

  // Climb from medium to traversal rungs.
  AddCommands(

    // ---------------------------------------------------------------------------------------
    // STAGE 1
    // ---------------------------------------------------------------------------------------
    
    // 1. Unfold (extend) hook arms - no delays:
    ClimberArmsLatchReleaseCmd(climberSub, false, false, false),
    // 2. Fully lower (retract) mast arm, lifting robot as high as possible:
    ClimberArmLowerCmd(climberSub),

    // Robot is now hanging by mast arm from medium rung.
    // Hook arms are ready to grab high rung from rear.
    // Robot is not swinging.

    // 3. Fold in (retract) hook arms - no delays:
    ClimberArmsLatchReleaseCmd(climberSub, true, false, false),
    // 4. Partially raise (extend) mast arm to lower robot and unhook mast from medium rung:
    ClimberArmRaiseCmd(climberSub, true),

    // Robot is now detached from medium rung.
    // Robot is swinging on high rung by hook arms.
    // Mast is not fully extended yet.

    // 5. Fold in (retract) hook arms - delay at end:
    ClimberArmsLatchReleaseCmd(climberSub, true, false, true),
    // 6. Fully raise (extend) mast arm => mast arm hook *MUST* end up on rear side of high rung:
    ClimberArmRaiseCmd(climberSub, false),

    // Attempt to recover from mast hook ending up on the wrong side (front) of high rung.
    // If we have this condition, the back of the mast hook will be rubbing against the front
    // of the high rung and that with a little time will help the swing slow down.
    // TODO: 7. Partially lower (retract) mast arm (e.g. maybe by 15%) making mast hook slightly
    //       lower than high rung.
    // TODO: 8. Fully raise (extend) mast arm and hope mast hook ends up on rear side of high rung.
    // NOTE: This may not really fix anything, but worth a try. Some alternatives to this:
    //           * If we break this group out into stages, the operator could manually lower
    //             and raise the mast arm as desired/required to get mast hook on the rear
    //             side of high rung and then manually continue with the next stage.
    //           * Use gyro (input from navx module?) to help trigger when to perform grabbing
    //             actions instead of purely relying on timing.

    // TODO: We *MAY* need to move the rest of this to a separate command groups and trigger
    //       them with separate buttons (or use shifts with same button) so we can control
    //       when the latter stages start and time them properly with the kind of swing we are
    //       dealing with. The intensity of the swing matters. An intense swing and where the
    //       robot is at during a swing cycle may negatively affect the ability of the mast
    //       hook to end up on the rear side of the high rung.

    // ---------------------------------------------------------------------------------------
    // STAGE 2
    // ---------------------------------------------------------------------------------------

    // 9. Delay at start - unfold (extend) hook arms to press mast against rear side of high rung:
    ClimberArmsLatchReleaseCmd(climberSub, false, true, false),
    // 10. Fully lower (retract) mast arm to lift robot and allow hook arms to pop off high rung:
    ClimberArmLowerCmd(climberSub),

    // Robot is swinging on high rung by mast arm.
    // Hook arms are fully extended or on the way to be.
    // We found that the aggressive unhook action can make the robot twist and hook arms not
    // approach the traversal rung evenly - one hook can properly end up on rear side, but
    // other hook can end up on the front side and our climb must be abruptly stopped when
    // that happens to avoid bend damage.
    // To avoid this we can try this:
    //    (a) Make step 10 above partially lower by 3-5% only - just enough to get the mast
    //        hook to lift the hook arms slightly off the high rung.
    //    (b) Add a command to reverse the hook arms to fold them in (retract) so they don't
    //        aggressively unhook.
    //    (c) Add a command to partially lower to around 15% so that hook arms' hooks are
    //        clear of high rung.
    //    (d) Add a command to fully unfold (extend) hook amrs.
    //    (e) Add a command to delay as necessary before performing step 11.
    // That removes the aggressive unhook and should avoid inducing wobble.
    // We may need to duplicate steps 7 and 8 next before performing step 11 or break the
    // rest of the commands below into stage 3 and rely on operator to manually fix hook
    // arm placement.

    // ---------------------------------------------------------------------------------------
    // STAGE 3
    // ---------------------------------------------------------------------------------------

    // 11. Fold in (retract) hook arms - no delays:
    ClimberArmsLatchReleaseCmd(climberSub, true, false, false)

    // TODO: 12. Partially raise (extend) mast arm to lower robot and unhook mast from high rung.
    
    // Robot is swinging on traversal rung by mast arm.
    // Hook arms are becoming fully folded in (retracted).
    // Robot should end up straight.

    // TODO: 13. Fully lower (retract) mast arm, returning it to start position for next match.
  );
}

