// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClimbStageOneGrp.h"
#include "commands/ClimberArmRaiseCmd.h"
#include "commands/ClimberArmLowerCmd.h"
#include "commands/ClimberArmsLatchReleaseCmd.h"

ClimbStageOneGrp::ClimbStageOneGrp(ClimberSub *climberSub) {
  // Required Initial Start State:
  //   1. robot facing front entrance of hangar, between high and traversal rungs
  //   2. mast arm raised (extended) all the way
  //   3. mast arm shaft against high rung (so mast will hook from rear of high rung)

  // Climb from high to traversal rungs.
  AddCommands(
    // Unfold (extend) hook arms - no delays:
    ClimberArmsLatchReleaseCmd(climberSub, false, false, false),
    // Fully lower (retract) mast arm, lifting robot as high as possible:
    ClimberArmLowerCmd(climberSub),

    // Robot is now hanging by mast arm from high rung.
    // Hook arms are ready to grab traversal rung from rear.
    // Robot is not swinging.

    // Fold in (retract) hook arms - no delays:
    ClimberArmsLatchReleaseCmd(climberSub, true, false, false),
    // Partially raise (extend) mast arm to lower robot and unhook mast from high rung:
    ClimberArmRaiseCmd(climberSub, true),

    // Robot is now detached from high rung.
    // Robot is swinging on traversal rung by hook arms.
    // Mast is not fully extended yet.

    // TODO: We likely need to move the rest of this to a separate command group and trigger
    //       it with a separate button (or shift with same button) so we can control when
    //       the second stage starts and time it properly with the kind of swing we are
    //       dealing with. The intensity of the swing matters. An intense swing and where the
    //       robot is at during a swing may negatively affect the ability of the mast hook to
    //       end up on the rear side of the traversal rung.

    // Fold in (retract) hook arms - delay at end:
    ClimberArmsLatchReleaseCmd(climberSub, true, false, true),
    // Fully raise (extend) mast => mast arm hook *MUST* end up on rear side of traversal rung:
    ClimberArmRaiseCmd(climberSub, false),
    // Delay at start - unfold (extend) hook arms to press mast against tranversal rung:
    ClimberArmsLatchReleaseCmd(climberSub, false, true, false),
    // Fully lower (retract) mast arm to lift robot and allow hook arms to pop off traversal rung:
    ClimberArmLowerCmd(climberSub),

    // Robot is slightly swinging on traversal rung by mast arm.
    // Hook arms are fully extended or on the way to be.

    // Fold in (retract) hook arms - no delays:
    ClimberArmsLatchReleaseCmd(climberSub, true, false, false)

    // Robot is barely swinging on traversal rung by mast arm.
    // Mast arm is being extended by gravity pull on robot (robot is slowly lowering).
    // Hook arms are fully retracted and will stop robot from lowering once hooks reach traversal rung.
  );
}

