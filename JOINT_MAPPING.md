# Joint Mapping Visualization

## SO-101 Leader to Piper Follower Joint Mapping

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          SO-101 LEADER ARM                              │
│                         (5 arm joints + gripper)                        │
└─────────────────────────────────────────────────────────────────────────┘

     Motor 1              Motor 2           Motor 3          Motor 4         Motor 5         Motor 6
  shoulder_pan        shoulder_lift      elbow_flex       wrist_flex      wrist_roll       gripper
      (base)          (shoulder)         (elbow)          (wrist)         (wrist)         (grip)
        │                  │                 │                │               │               │
        │                  │                 │                │               │               │
        ▼                  ▼                 ▼                ▼               ▼               ▼
  ┌──────────┐       ┌──────────┐     ┌──────────┐     ┌──────────┐   ┌──────────┐    ┌─────────┐
  │ Feetech  │       │ Feetech  │     │ Feetech  │     │ Feetech  │   │ Feetech  │    │Feetech  │
  │ STS3215  │       │ STS3215  │     │ STS3215  │     │ STS3215  │   │ STS3215  │    │ STS3215 │
  │  1/191   │       │  1/345   │     │  1/191   │     │  1/147   │   │  1/147   │    │  1/147  │
  └────┬─────┘       └────┬─────┘     └────┬─────┘     └────┬─────┘   └────┬─────┘    └────┬────┘
       │                  │                 │                │               │               │
       │ alias:           │ alias:          │ alias:         │ alias:        │ alias:        │
       │ "shoulder_pan"   │ "shoulder_lift" │ "elbow_flex"   │ "wrist_flex"  │ "wrist_roll"  │
       │                  │                 │                │               │               │
       │                  │                 │                │               │               │
       ├──────────────────┼─────────────────┼────────────────┼───────────────┼───────────────┤
       │                                                                                      │
       │                         LeRobot Joint Aliases                                       │
       │                         joint_aliases dict                                          │
       │                                                                                      │
       ├──────────────────┬─────────────────┬────────────────┬───────────────┬───────────────┤
       │                  │                 │                │               │               │
       ▼                  ▼                 ▼                ▼               ▼               ▼
  ┌─────────┐       ┌─────────┐      ┌─────────┐      ┌─────────┐    ┌─────────┐     ┌─────────┐
  │ joint_1 │       │ joint_2 │      │ joint_3 │      │ joint_5 │    │ joint_6 │     │ gripper │
  │  (pan)  │       │ (lift)  │      │ (flex)  │      │ (flex)  │    │ (roll)  │     │  (mm)   │
  └─────────┘       └─────────┘      └─────────┘      └─────────┘    └─────────┘     └─────────┘
       │                  │                 │         ┌─────────┐          │               │
       │                  │                 │         │ joint_4 │          │               │
       │                  │                 │         │   ⚠️    │          │               │
       │                  │                 │         │UNMAPPED │          │               │
       │                  │                 │         └─────────┘          │               │
       │                  │                 │                │             │               │
       ▼                  ▼                 ▼                ▼             ▼               ▼

┌─────────────────────────────────────────────────────────────────────────────────────────────┐
│                              PIPER FOLLOWER ARM                                             │
│                          (6 arm joints + gripper)                                           │
│                                                                                             │
│  Joint 1    Joint 2    Joint 3    Joint 4    Joint 5    Joint 6    Gripper                │
│  Base Pan   Shoulder   Elbow      Wrist?     Wrist      Wrist      Open/Close             │
│             Lift       Flex       Rotation?  Flex       Roll       (mm)                    │
│                                                                                             │
│  Controlled Controlled Controlled ⚠️ NOT     Controlled Controlled Controlled             │
│  by SO-101  by SO-101  by SO-101  CONTROLLED by SO-101  by SO-101  by SO-101              │
│                                    by SO-101                                                │
└─────────────────────────────────────────────────────────────────────────────────────────────┘

```

## Key Observations

### ✅ What Works
1. **5 arm joints + gripper** are successfully mapped and controlled
2. **Sign corrections** ensure movements match between leader and follower
3. **Coordinate transformations** handle degrees ↔ normalized ranges
4. **Bi-directional observation mirroring** allows reading positions by alias names

### ⚠️ Critical Consideration: joint_4

**Joint 4 Behavior:**
- **Current State**: Uncontrolled during teleoperation
- **Likely Function**: Wrist/forearm rotation (not present on SO-101)
- **During Operation**: Maintains last position or initialization position
- **Safety Concern**: Could be in unexpected/unsafe position

**Why This Design?**
- SO-101 has 5 arm DoF, Piper has 6 arm DoF
- Direct 1:1 mapping is not possible
- Choosing to skip joint_4 preserves the kinematic correspondence of other joints
- Alternative would be to map joint_4 to follow another joint (e.g., joint_3)

### 📊 Comparison Table

| Feature               | SO-101 Leader    | Piper Follower   | Mapping Status |
|-----------------------|------------------|------------------|----------------|
| Base/Shoulder Pan     | shoulder_pan     | joint_1          | ✅ Mapped      |
| Shoulder Lift         | shoulder_lift    | joint_2          | ✅ Mapped      |
| Elbow Flex            | elbow_flex       | joint_3          | ✅ Mapped      |
| Wrist/Forearm Rotation| ❌ Not present   | joint_4          | ⚠️ UNMAPPED    |
| Wrist Flex            | wrist_flex       | joint_5          | ✅ Mapped      |
| Wrist Roll            | wrist_roll       | joint_6          | ✅ Mapped      |
| Gripper               | gripper          | gripper          | ✅ Mapped      |
| **Total Control**     | **6 joints**     | **6 of 7 joints**| **~86%**       |

## Data Flow

### Observation Path (Follower → Leader)
```
Piper Hardware
    ↓ [CAN bus]
PiperSDKInterface.get_status_deg()
    ↓ [Convert from milli-degrees]
Piper.get_observation()
    ↓ [Apply sign corrections]
    ↓ [Convert to degrees or normalized]
    ↓ [Mirror to alias names]
Observation Dict {
    "joint_1.pos": float,
    "joint_2.pos": float,
    ...
    "shoulder_pan.pos": float,  # Alias
    "shoulder_lift.pos": float, # Alias
    ...
}
```

### Action Path (Leader → Follower)
```
SO-101 Leader reads positions
    ↓
Action Dict {
    "shoulder_pan.pos": float,
    "shoulder_lift.pos": float,
    ...
}
    ↓
Piper.send_action(action)
    ↓ [Look up alias mappings]
    ↓ [Convert from degrees or normalized]
    ↓ [Apply sign corrections]
    ↓ [Clamp to hardware limits]
    ↓ [joint_4 uses fallback: current obs]
PiperSDKInterface.set_joint_positions_deg()
    ↓ [Convert to milli-degrees]
    ↓ [CAN bus]
Piper Hardware executes
```

## Sign Corrections

```python
joint_signs = [-1, 1, 1, -1, 1, -1]
#              ↓   ↓  ↓   ↓   ↓   ↓
#            j1  j2 j3  j4  j5  j6
```

**Purpose**: Compensate for different motor mounting orientations so that:
- Positive leader movement → Positive follower movement
- Negative leader movement → Negative follower movement

**Applied symmetrically**:
- **Reading observations**: `obs_deg = hardware_deg * sign`
- **Sending actions**: `hardware_deg = oriented_deg * sign`

## Coordinate Systems

### Mode 1: Degrees (`use_degrees=True`)
```
Joints: Raw degrees (-180° to +180° or custom limits)
Gripper: Millimeters (0 mm to 10 mm)
```

### Mode 2: Normalized (`use_degrees=False`)
```
Joints: Percentage [-100, 100]
  -100 = minimum position
     0 = middle position
  +100 = maximum position

Gripper: Percentage [0, 100]
     0 = closed
   100 = fully open
```

**Conversion Formula** (example for joint at middle of range):
```python
# Degree to Normalized
normalized = (degrees - min_limit) / (max_limit - min_limit) * 200.0 - 100.0

# Normalized to Degree
degrees = min_limit + ((normalized + 100.0) / 200.0) * (max_limit - min_limit)
```

## Implementation Files

```
lerobot_robot_piper/
├── config_piper.py         # Configuration with joint_aliases
├── piper.py                # Main robot class with mapping logic
├── piper_sdk_interface.py  # Low-level SDK wrapper
├── __init__.py             # Package exports
└── README.md               # Usage documentation
```

## Related Resources

- **SO-101 Documentation**: [huggingface.co/docs/lerobot/so101](https://huggingface.co/docs/lerobot/so101)
- **Piper SDK**: [github.com/agilexrobotics/piper_sdk](https://github.com/agilexrobotics/piper_sdk)
- **LeRobot Framework**: [github.com/huggingface/lerobot](https://github.com/huggingface/lerobot)

---

**Diagram Version**: 1.0  
**Date**: December 5, 2024
