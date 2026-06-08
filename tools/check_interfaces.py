#!/usr/bin/env python3
from pathlib import Path


REQUIRED = {
    "msg": [
        "WalkingState.msg",
        "AdapterStatus.msg",
        "SafetyState.msg",
        "RobotProfile.msg",
        "LocomotionCommand.msg",
        "BodyPoseCommand.msg",
        "Footstep.msg",
        "FootstepPlan.msg",
        "SemanticAction.msg",
    ],
    "srv": [
        "SetLocomotionMode.srv",
        "EmergencyStop.srv",
        "ClearFault.srv",
        "GetRobotProfile.srv",
    ],
    "action": [
        "ExecuteVelocity.action",
        "ExecuteFootstepPlan.action",
        "ExecuteBodyPose.action",
        "ExecuteSemanticAction.action",
    ],
}


def main() -> int:
    root = Path(__file__).resolve().parents[1] / "src" / "locomotion_ros2_msgs"
    missing = []
    for folder, names in REQUIRED.items():
        for name in names:
            path = root / folder / name
            if not path.exists():
                missing.append(str(path))
    if missing:
        print("Missing interfaces:")
        print("\n".join(missing))
        return 1
    print("locomotion_ros2 interfaces present")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
