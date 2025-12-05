# SO-101 to Piper Arm Mapping Analysis

## Executive Summary

This document provides an in-depth analysis of how the `lerobot_robot_piper` repository implements the mapping between the SO-101 teleoperation arm (leader) and the Piper robot arm (follower) within the LeRobot framework.

## 1. SO-101 Arm Specifications

### Degrees of Freedom (DoF)
The **SO-101 arm has 6 degrees of freedom** plus a gripper, for a total of **7 controllable joints**:

1. **shoulder_pan** (Joint 1) - Base rotation
2. **shoulder_lift** (Joint 2) - Shoulder elevation  
3. **elbow_flex** (Joint 3) - Elbow flexion
4. **wrist_flex** (Joint 4) - Wrist flexion
5. **wrist_roll** (Joint 5) - Wrist rotation
6. **gripper** (Joint 6) - Gripper open/close

### Motor Configuration
From the [SO-101 documentation](https://huggingface.co/docs/lerobot/so101):
- All joints use Feetech STS3215 motors
- Different gear ratios for different joints to balance weight and ease of movement
- Leader arm uses 1/191, 1/345, and 1/147 gear ratios depending on the joint

### Communication
- Serial communication via USB (e.g., `/dev/ttyACM0`, `/dev/tty.usbmodem*`)
- Feetech motor protocol

## 2. Piper Arm Specifications

### Degrees of Freedom (DoF)
The **Piper arm has 6 degrees of freedom** plus a gripper, for a total of **7 controllable joints**:

1. **joint_1** - Base/shoulder pan
2. **joint_2** - Shoulder lift
3. **joint_3** - Elbow flex
4. **joint_4** - (Likely wrist/forearm rotation)
5. **joint_5** - Wrist flex
6. **joint_6** - Wrist roll
7. **gripper** - Gripper in mm

### Motor Configuration
From the [Piper SDK](https://github.com/agilexrobotics/piper_sdk):
- CAN bus interface (e.g., `can0`)
- Positions reported in thousandths of degrees (milli-degrees)
- Gripper position in 1/10000 mm units
- Motors indexed 1-6 (according to `piper_sdk_interface.py` line 70)

### Communication
- CAN bus protocol at 1Mbps bitrate
- Uses `piper_sdk` (C_PiperInterface_V2)

## 3. Joint Mapping Strategy

### 3.1 Direct Name-to-Index Mapping

The implementation in `config_piper.py` defines explicit joint aliases:

```python
joint_aliases: dict[str, str] = field(
    default_factory=lambda: {
        "shoulder_pan": "joint_1",
        "shoulder_lift": "joint_2",
        "elbow_flex": "joint_3",
        "wrist_flex": "joint_5",
        "wrist_roll": "joint_6",
    }
)
```

**Key observation**: The mapping **skips joint_4** of the Piper arm. This is a critical internal design decision.

### 3.2 Rationale for Skipping joint_4

Looking at the typical 6-DoF arm kinematics:
- SO-101 has: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll (5 joints + gripper)
- Piper has: joint_1 through joint_6 (6 joints + gripper)

**Analysis**: 
- SO-101 appears to have **5 arm joints** (not counting gripper)
- Piper has **6 arm joints** (not counting gripper)
- **joint_4 on Piper is likely a wrist/forearm rotation** that doesn't exist on SO-101
- The mapping deliberately omits control of joint_4 to maintain compatibility

This creates a **5-DoF effective control** of a 6-DoF arm, leaving joint_4 uncontrolled during teleoperation.

### 3.3 Joint Sign Corrections

The configuration applies sign flips to handle different motor orientations:

```python
joint_signs: list[int] = field(default_factory=lambda: [-1, 1, 1, -1, 1, -1])
```

This ensures that positive movements on the SO-101 leader correspond to positive movements on the Piper follower, despite potential differences in motor mounting orientations.

## 4. Implementation Details

### 4.1 Coordinate Transformations

The implementation supports two modes:

#### Degrees Mode (`use_degrees=True`)
- Raw degree values passed through
- Gripper in millimeters

#### Normalized Mode (`use_degrees=False`)  
- Joint positions mapped to [-100, 100] range
- Gripper mapped to [0, 100] range
- Formula: `value = (position - min) / (max - min) * 200 - 100`

### 4.2 Observation Mirroring

From `piper.py` lines 146-151:
```python
# Mirror joint values under alias names so teleop processors can access them easily
for alias, target in self.config.joint_aliases.items():
    target_key = f"{target}.pos"
    alias_key = f"{alias}.pos"
    if target_key in obs and alias_key not in obs:
        obs[alias_key] = obs[target_key]
```

This allows the teleoperation system to read positions using SO-101 joint names even though the underlying robot uses Piper joint names.

### 4.3 Action Mapping

From `piper.py` lines 199-210, actions can be specified using either:
- Native Piper joint names (`joint_1.pos`, `joint_2.pos`, etc.)
- SO-101 alias names (`shoulder_pan.pos`, `shoulder_lift.pos`, etc.)

The aliases override native names if both are present.

## 5. Identified Issues and Concerns

### 5.1 CRITICAL: Uncontrolled Joint (joint_4)

**Issue**: Joint 4 on the Piper arm is not mapped to any SO-101 joint.

**Impact**:
- During teleoperation, joint_4 will remain at its last position
- Could cause unexpected arm configurations
- Potential safety concern if joint_4 drifts

**Recommendation**: 
- Document this limitation clearly
- Consider fixing joint_4 at a neutral position during initialization
- Or add a separate control mechanism for joint_4

**Code Evidence**:
```python
# In piper.py, get_observation() line 130-132
for i, name in enumerate(self.config.joint_names, start=1):
    deg = status[f"joint_{i}.pos"] * self.config.joint_signs[i - 1]
    obs[f"{name}.pos"] = deg if self.config.use_degrees else deg_to_pct(deg, i - 1)
```

The observation includes all 6 joints (`joint_1` through `joint_6`), but the action mapping only controls 5 of them via aliases.

### 5.2 Potential Issue: Missing Gripper Range Configuration

**Code Location**: `piper_sdk_interface.py` lines 70-76

```python
self.min_pos = [pos.min_angle_limit / 10.0 for pos in angel_status.all_motor_angle_limit_max_spd.motor[1:7]] + [0.0]
self.max_pos = [pos.max_angle_limit / 10.0 for pos in angel_status.all_motor_angle_limit_max_spd.motor[1:7]] + [10.0]
```

**Issue**: Gripper limits are hardcoded as `[0.0, 10.0]` mm instead of being read from the SDK.

**Impact**: 
- May not match actual gripper range
- Could lead to clipping or unexpected behavior

**Recommendation**: Query actual gripper limits from the SDK if available.

### 5.3 Error Handling: Motor Limit Fallback

**Code Location**: `piper_sdk_interface.py` lines 72-76

```python
except Exception as e:
    log.warning("Could not read joint limits: %s", e)
    # sensible defaults to avoid crashes; keep lists length >=7
    self.min_pos = [-180.0] * 6 + [0.0]
    self.max_pos = [180.0] * 6 + [10.0]
```

**Issue**: Silent fallback to ±180° for all joints may be unsafe.

**Impact**: 
- Could allow movements beyond actual hardware limits
- Risk of damaging the robot

**Recommendation**: Fail initialization if limits cannot be read, or use more conservative defaults.

### 5.4 Coordinate System Consistency

**Code Location**: `piper.py` line 131

```python
deg = status[f"joint_{i}.pos"] * self.config.joint_signs[i - 1]
```

**Issue**: Sign corrections are applied symmetrically to both observations and actions, but there's no verification that this creates a consistent coordinate frame.

**Recommendation**: Add validation tests to ensure leader/follower positions match in home configuration.

### 5.5 Missing joint_4 in Action Processing

**Code Location**: `piper.py` lines 186-210

The action processing loop only processes joints that have entries in `name_to_idx`, which only includes the 6 Piper joints. The alias mapping then provides values for 5 of these joints. 

**Issue**: If joint_4 is not in the action dict and not mapped by an alias, it will use the observation fallback (line 191):

```python
raw = action.get(key, obs.get(key, 0.0))
```

This means joint_4 will maintain its current position, which is the intended behavior but not explicitly documented.

### 5.6 SDK Unit Conversions

**Code Location**: `piper_sdk_interface.py`

Multiple unit conversions between:
- SDK thousandths of degrees (milli-degrees)
- Degrees
- Gripper SDK units (1/10000 mm)
- Millimeters

**Issue**: Complex conversions increase risk of errors. The code uses:
- `angle * 1000.0` for degrees to SDK (line 98)
- `angle / 1000.0` for SDK to degrees (line 124)
- `gripper * 10000.0` for mm to SDK (line 144)
- `gripper / 10000.0` for SDK to mm (line 133)

**Recommendation**: Add unit tests for all conversions.

## 6. LeRobot Integration Architecture

### 6.1 Robot Class Hierarchy

```
lerobot.robots.Robot (base)
    └── Piper (lerobot_robot_piper.piper)
        ├── config: PiperConfig
        ├── _iface: PiperSDKInterface
        └── cameras: dict
```

### 6.2 Feature Definitions

**Observation Features** (`piper.py` lines 40-45):
- All 6 Piper joints as `joint_N.pos`
- Optional gripper as `gripper.pos`
- Camera feeds

**Action Features** (`piper.py` lines 47-52):
- SO-101 aliases as action keys
- Optional gripper

This asymmetry is intentional: observations use native Piper names, actions use SO-101 names for teleoperation compatibility.

## 7. Comparison with Other Implementations

### 7.1 Reference: lykycy123/lerobot-piper

The [lerobot-piper repository](https://github.com/lykycy123/lerobot-piper) provides another implementation approach. Key differences would help understand best practices.

### 7.2 SO-101 Follower Implementation

In the main lerobot repo, there's also an `SO101Follower` robot that uses the same 6-motor setup as the leader. This suggests the SO-101 ecosystem uses 6 motors total (5 arm + 1 gripper).

## 8. Recommendations

### 8.1 Critical Fixes

1. **Document joint_4 behavior**: Add clear documentation that joint_4 is not controlled
2. **Initialize joint_4 position**: Add code to set joint_4 to a safe neutral position during `connect()`
3. **Add configuration option**: Allow users to specify joint_4 behavior (fixed, follow joint_3, etc.)

### 8.2 Safety Improvements

1. **Strict limit enforcement**: Fail if motor limits cannot be read
2. **Add limit checking**: Validate actions against limits before sending
3. **Emergency stop**: Implement proper emergency stop in disconnect

### 8.3 Code Quality

1. **Add unit tests**: Test coordinate transformations, mappings, conversions
2. **Add integration tests**: Test with mock Piper SDK
3. **Improve error messages**: More specific error messages for common issues
4. **Add type hints**: Complete type hints throughout the codebase

### 8.4 Documentation

1. **Architecture diagram**: Visual representation of joint mapping
2. **Calibration guide**: Detailed calibration procedure with joint_4 considerations  
3. **Troubleshooting guide**: Common issues and solutions
4. **API documentation**: Complete API docs for all classes and methods

## 9. Conclusion

The `lerobot_robot_piper` implementation provides a functional bridge between the SO-101 teleoperation arm and the Piper robot arm. The key insight is that:

- **SO-101 effectively has 5 arm DoF + gripper** (6 total)
- **Piper has 6 arm DoF + gripper** (7 total)
- **The mapping uses 5 of Piper's 6 DoF**, leaving joint_4 uncontrolled

This is a **reasonable design decision** for teleoperation compatibility but should be:
1. Clearly documented
2. Handled safely (joint_4 should be positioned appropriately)
3. Made configurable for advanced users

The code is generally well-structured and follows LeRobot conventions, but would benefit from:
- More explicit handling of the joint_4 case
- Better error handling and validation
- Comprehensive testing
- Improved documentation

## 10. Technical Deep Dive: Code Flow

### 10.1 Initialization Flow

```
Piper.__init__()
    └── PiperConfig with joint_aliases
    
Piper.connect()
    └── PiperSDKInterface.__init__()
        ├── C_PiperInterface_V2(port)
        ├── ConnectPort()
        ├── Resume from teaching mode if needed
        ├── EnablePiper() with timeout
        ├── MotionCtrl_2(joint mode, 100% speed)
        └── GetAllMotorAngleLimitMaxSpd() → min_pos, max_pos
```

### 10.2 Observation Flow

```
Piper.get_observation()
    └── PiperSDKInterface.get_status_deg()
        ├── GetArmJointMsgs() → joint_state
        ├── GetArmGripperMsgs() → gripper_state
        └── Convert from SDK units (milli-degrees, 1/10000mm)
    
    For each joint 1-6:
        ├── Apply sign correction
        ├── Convert to degrees or normalized [-100,100]
        └── Store as "joint_N.pos"
    
    Mirror to aliases:
        └── Copy values to SO-101 names
```

### 10.3 Action Flow

```
Piper.send_action(action)
    Get current observation (for fallback)
    
    For each Piper joint:
        ├── Get value from action (or fallback to obs)
        ├── Apply coordinate transform (normalized → degrees)
        └── Store in oriented_deg dict
    
    Process aliases:
        └── Override oriented_deg with alias values if present
    
    For each joint:
        ├── Apply sign correction (oriented → hardware)
        ├── Clamp to hardware limits
        └── Append to joints_hw_deg list
    
    PiperSDKInterface.set_joint_positions_deg()
        ├── Convert to SDK units (* 1000)
        ├── JointCtrl(*j_ints)
        └── GripperCtrl(gripper_int) if provided
```

## 11. References

1. [SO-101 Documentation](https://huggingface.co/docs/lerobot/so101) - HuggingFace LeRobot
2. [Piper SDK](https://github.com/agilexrobotics/piper_sdk) - AgileX Robotics
3. [LeRobot Repository](https://github.com/huggingface/lerobot) - Main LeRobot framework
4. [SO-ARM100 Hardware](https://github.com/TheRobotStudio/SO-ARM100) - Bill of materials and assembly
5. [lerobot-piper](https://github.com/lykycy123/lerobot-piper) - Alternative implementation

---

**Document Version**: 1.0  
**Date**: December 5, 2024  
**Author**: Analysis of charithmu/lerobot_robot_piper repository
