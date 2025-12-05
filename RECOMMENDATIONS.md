# Implementation Recommendations for lerobot_robot_piper

## Overview

This document provides concrete code recommendations to address the issues identified in the ANALYSIS.md document.

## 1. Fix for Uncontrolled joint_4

### Issue
Joint 4 of the Piper arm is not mapped to any SO-101 joint, leaving it in an undefined state during teleoperation.

### Recommendation: Add joint_4 Configuration

#### Option A: Fixed Position (Recommended for initial implementation)

Add to `config_piper.py`:

```python
@RobotConfig.register_subclass("piper")
@dataclass
class PiperConfig(RobotConfig):
    # ... existing fields ...
    
    # Strategy for handling unmapped joint_4: "fixed", "follow_joint3", "manual"
    joint_4_mode: str = "fixed"
    # Position in degrees to hold joint_4 when in "fixed" mode
    joint_4_fixed_position: float = 0.0
```

Update `piper.py`:

```python
def connect(self, calibrate: bool = True) -> None:
    # ... existing code ...
    self.configure()
    
    # Handle joint_4 initialization
    if self.config.joint_4_mode == "fixed":
        self._initialize_joint_4()

def _initialize_joint_4(self) -> None:
    """Initialize joint_4 to a fixed safe position."""
    if self._iface is None:
        return
    
    logger.info(f"Initializing joint_4 to fixed position: {self.config.joint_4_fixed_position}°")
    
    # Read current positions
    current = self._iface.get_status_deg()
    
    # Create action with only joint_4 modified
    joints_deg = [current[f"joint_{i}.pos"] for i in range(1, 7)]
    joints_deg[3] = self.config.joint_4_fixed_position  # Index 3 = joint_4
    
    # Send position
    self._iface.set_joint_positions_deg(joints_deg, None)
```

#### Option B: Follow Adjacent Joint (Alternative)

```python
def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
    # ... existing code up to line 210 ...
    
    # Handle joint_4 strategy
    if self.config.joint_4_mode == "follow_joint3":
        # Make joint_4 follow joint_3 with optional offset
        joint_3_idx = 2  # joint_3 is index 2 in the list
        joint_4_idx = 3  # joint_4 is index 3
        joints_hw_deg[joint_4_idx] = joints_hw_deg[joint_3_idx] + self.config.joint_4_offset
        # Clamp to limits
        joints_hw_deg[joint_4_idx] = max(hw_min[joint_4_idx], 
                                         min(hw_max[joint_4_idx], 
                                             joints_hw_deg[joint_4_idx]))
    
    # ... rest of existing code ...
```

## 2. Improved Error Handling for Motor Limits

### Issue
Silent fallback to potentially unsafe default limits.

### Recommendation: Strict Validation

Update `piper_sdk_interface.py`:

```python
def __init__(self, port: str = "can0", enable_timeout: float = 5.0):
    # ... existing code up to line 66 ...
    
    # Get the min and max positions for each joint and gripper
    try:
        angel_status = self.piper.GetAllMotorAngleLimitMaxSpd()
        # Validate we got expected number of motors
        if len(angel_status.all_motor_angle_limit_max_spd.motor) < 7:
            raise RuntimeError(
                f"Expected at least 7 motors in limit response, got "
                f"{len(angel_status.all_motor_angle_limit_max_spd.motor)}"
            )
        
        # Extract limits (SDK motor list is 1-indexed)
        self.min_pos = [
            pos.min_angle_limit / 10.0 
            for pos in angel_status.all_motor_angle_limit_max_spd.motor[1:7]
        ]
        self.max_pos = [
            pos.max_angle_limit / 10.0 
            for pos in angel_status.all_motor_angle_limit_max_spd.motor[1:7]
        ]
        
        # Try to get gripper limits, fallback to sensible defaults for gripper only
        try:
            # Assuming motor[7] is gripper if it exists
            gripper_motor = angel_status.all_motor_angle_limit_max_spd.motor[7]
            self.min_pos.append(gripper_motor.min_angle_limit / 10000.0)  # Convert to mm
            self.max_pos.append(gripper_motor.max_angle_limit / 10000.0)
        except (IndexError, AttributeError):
            log.warning("Could not read gripper limits from SDK, using defaults [0.0, 10.0] mm")
            self.min_pos.append(0.0)
            self.max_pos.append(10.0)
        
        # Validate limits are reasonable
        for i, (min_val, max_val) in enumerate(zip(self.min_pos, self.max_pos)):
            if max_val <= min_val:
                raise RuntimeError(
                    f"Invalid limits for joint {i}: min={min_val}, max={max_val}"
                )
            if abs(max_val - min_val) > 360:  # Sanity check for joints
                if i < 6:  # Only for regular joints, not gripper
                    log.warning(
                        f"Joint {i} has unusually large range: "
                        f"{max_val - min_val} degrees"
                    )
        
        log.info(f"Successfully read motor limits: {self.min_pos} to {self.max_pos}")
        
    except Exception as e:
        log.error("Failed to read joint limits from SDK: %s", e)
        raise RuntimeError(
            "Cannot initialize Piper arm without valid joint limits. "
            "Please check robot connection and SDK installation."
        ) from e
```

## 3. Add Joint Limit Validation in send_action

### Issue
Actions are clamped but there's no warning when limits are exceeded.

### Recommendation: Add Validation and Warnings

Update `piper.py`:

```python
def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
    # ... existing code up to line 216 ...
    
    # Validate and clamp with warnings
    joints_hw_deg_clamped = []
    for idx, (name, deg_hw) in enumerate(zip(self.config.joint_names, joints_hw_deg)):
        original_deg = deg_hw
        deg_hw = max(hw_min[idx], min(hw_max[idx], deg_hw))
        
        if abs(deg_hw - original_deg) > 0.1:  # More than 0.1 degree clamping
            logger.warning(
                f"Action for {name} exceeded limits: requested {original_deg:.2f}°, "
                f"clamped to {deg_hw:.2f}° (limits: [{hw_min[idx]:.2f}, {hw_max[idx]:.2f}])"
            )
        
        joints_hw_deg_clamped.append(deg_hw)
    
    # ... continue with joints_hw_deg_clamped ...
```

## 4. Add Unit Tests

### Create tests/test_piper_coordinate_transforms.py

```python
import pytest
import numpy as np
from lerobot_robot_piper import PiperConfig, Piper


def test_degree_to_normalized_conversion():
    """Test that degree to normalized [-100, 100] conversion is correct."""
    config = PiperConfig(
        can_interface="can0",
        use_degrees=False,
    )
    
    # Mock limits: -180 to 180 degrees
    min_pos = [-180.0] * 6 + [0.0]
    max_pos = [180.0] * 6 + [10.0]
    
    # Test middle position
    deg = 0.0
    idx = 0
    pct = (deg - min_pos[idx]) / (max_pos[idx] - min_pos[idx]) * 200.0 - 100.0
    assert abs(pct - 0.0) < 0.01, "Middle position should be 0%"
    
    # Test min position
    deg = -180.0
    pct = (deg - min_pos[idx]) / (max_pos[idx] - min_pos[idx]) * 200.0 - 100.0
    assert abs(pct - (-100.0)) < 0.01, "Min position should be -100%"
    
    # Test max position
    deg = 180.0
    pct = (deg - min_pos[idx]) / (max_pos[idx] - min_pos[idx]) * 200.0 - 100.0
    assert abs(pct - 100.0) < 0.01, "Max position should be 100%"


def test_normalized_to_degree_conversion():
    """Test that normalized [-100, 100] to degree conversion is correct."""
    min_pos = [-180.0] * 6 + [0.0]
    max_pos = [180.0] * 6 + [10.0]
    idx = 0
    
    # Test middle position
    pct = 0.0
    p01 = (pct + 100.0) / 200.0
    deg = min_pos[idx] + p01 * (max_pos[idx] - min_pos[idx])
    assert abs(deg - 0.0) < 0.01, "0% should be 0 degrees"
    
    # Test min position
    pct = -100.0
    p01 = (pct + 100.0) / 200.0
    deg = min_pos[idx] + p01 * (max_pos[idx] - min_pos[idx])
    assert abs(deg - (-180.0)) < 0.01, "-100% should be -180 degrees"
    
    # Test max position
    pct = 100.0
    p01 = (pct + 100.0) / 200.0
    deg = min_pos[idx] + p01 * (max_pos[idx] - min_pos[idx])
    assert abs(deg - 180.0) < 0.01, "100% should be 180 degrees"


def test_sign_correction_symmetry():
    """Test that sign corrections are applied symmetrically."""
    config = PiperConfig(
        can_interface="can0",
        joint_signs=[-1, 1, 1, -1, 1, -1],
    )
    
    # Forward: oriented -> hardware
    oriented_deg = 45.0
    sign = -1
    hw_deg = oriented_deg * sign
    assert hw_deg == -45.0, "Sign should flip the value"
    
    # Backward: hardware -> oriented  
    hw_deg_read = -45.0
    oriented_deg_read = hw_deg_read * sign
    assert oriented_deg_read == 45.0, "Sign should flip back to original"


def test_joint_aliases_mapping():
    """Test that joint aliases map correctly."""
    config = PiperConfig(can_interface="can0")
    
    assert config.joint_aliases["shoulder_pan"] == "joint_1"
    assert config.joint_aliases["shoulder_lift"] == "joint_2"
    assert config.joint_aliases["elbow_flex"] == "joint_3"
    assert config.joint_aliases["wrist_flex"] == "joint_5"
    assert config.joint_aliases["wrist_roll"] == "joint_6"
    
    # Verify joint_4 is not mapped
    assert "joint_4" not in config.joint_aliases.values()


def test_gripper_conversion():
    """Test gripper mm to percent conversion."""
    g_min = 0.0
    g_max = 10.0
    
    # Test min
    mm = 0.0
    pct = (mm - g_min) / (g_max - g_min) * 100.0
    assert abs(pct - 0.0) < 0.01
    
    # Test max
    mm = 10.0
    pct = (mm - g_min) / (g_max - g_min) * 100.0
    assert abs(pct - 100.0) < 0.01
    
    # Test middle
    mm = 5.0
    pct = (mm - g_min) / (g_max - g_min) * 100.0
    assert abs(pct - 50.0) < 0.01
```

## 5. Add Integration Test with Mock SDK

### Create tests/test_piper_integration.py

```python
import pytest
from unittest.mock import Mock, MagicMock, patch
from lerobot_robot_piper import Piper, PiperConfig


@pytest.fixture
def mock_piper_sdk():
    """Create a mock Piper SDK for testing."""
    with patch('lerobot_robot_piper.piper_sdk_interface.C_PiperInterface_V2') as mock_sdk:
        # Setup mock responses
        mock_instance = MagicMock()
        mock_sdk.return_value = mock_instance
        
        # Mock ConnectPort
        mock_instance.ConnectPort.return_value = None
        
        # Mock GetArmStatus
        status_mock = MagicMock()
        status_mock.arm_status.motion_status = 0
        status_mock.arm_status.ctrl_mode = 0
        mock_instance.GetArmStatus.return_value = status_mock
        
        # Mock EnablePiper
        mock_instance.EnablePiper.return_value = True
        
        # Mock MotionCtrl_2
        mock_instance.MotionCtrl_2.return_value = None
        
        # Mock GetAllMotorAngleLimitMaxSpd
        limits_mock = MagicMock()
        motor_limits = []
        for i in range(8):  # 0-7, we use 1-7
            motor = MagicMock()
            motor.min_angle_limit = -1800  # -180 degrees * 10
            motor.max_angle_limit = 1800   # 180 degrees * 10
            motor_limits.append(motor)
        limits_mock.all_motor_angle_limit_max_spd.motor = motor_limits
        mock_instance.GetAllMotorAngleLimitMaxSpd.return_value = limits_mock
        
        # Mock GetArmJointMsgs
        joint_mock = MagicMock()
        joint_mock.joint_state.joint_1 = 0
        joint_mock.joint_state.joint_2 = 0
        joint_mock.joint_state.joint_3 = 0
        joint_mock.joint_state.joint_4 = 0
        joint_mock.joint_state.joint_5 = 0
        joint_mock.joint_state.joint_6 = 0
        mock_instance.GetArmJointMsgs.return_value = joint_mock
        
        # Mock GetArmGripperMsgs
        gripper_mock = MagicMock()
        gripper_mock.gripper_state.grippers_angle = 50000  # 5.0 mm * 10000
        mock_instance.GetArmGripperMsgs.return_value = gripper_mock
        
        # Mock JointCtrl and GripperCtrl
        mock_instance.JointCtrl = MagicMock()
        mock_instance.GripperCtrl = MagicMock()
        
        yield mock_instance


def test_piper_connect_disconnect(mock_piper_sdk):
    """Test basic connect and disconnect flow."""
    config = PiperConfig(can_interface="can0")
    piper = Piper(config)
    
    assert not piper.is_connected
    
    piper.connect(calibrate=False)
    assert piper.is_connected
    
    # Verify SDK was initialized
    mock_piper_sdk.ConnectPort.assert_called_once()
    mock_piper_sdk.EnablePiper.assert_called()
    
    piper.disconnect()
    assert not piper.is_connected


def test_piper_get_observation(mock_piper_sdk):
    """Test observation reading."""
    config = PiperConfig(can_interface="can0", use_degrees=True, include_gripper=True)
    piper = Piper(config)
    piper.connect(calibrate=False)
    
    obs = piper.get_observation()
    
    # Check all joints are present
    for i in range(1, 7):
        assert f"joint_{i}.pos" in obs
    
    # Check gripper
    assert "gripper.pos" in obs
    assert abs(obs["gripper.pos"] - 5.0) < 0.01  # We mocked 5.0 mm
    
    # Check aliases are mirrored
    assert "shoulder_pan.pos" in obs
    assert obs["shoulder_pan.pos"] == obs["joint_1.pos"]


def test_piper_send_action(mock_piper_sdk):
    """Test action sending."""
    config = PiperConfig(can_interface="can0", use_degrees=True)
    piper = Piper(config)
    piper.connect(calibrate=False)
    
    action = {
        "shoulder_pan.pos": 45.0,
        "shoulder_lift.pos": 30.0,
        "elbow_flex.pos": -20.0,
        "wrist_flex.pos": 10.0,
        "wrist_roll.pos": 0.0,
    }
    
    result = piper.send_action(action)
    
    # Verify JointCtrl was called
    mock_piper_sdk.JointCtrl.assert_called_once()
    
    # Check the values sent (with sign corrections)
    call_args = mock_piper_sdk.JointCtrl.call_args[0]
    # joint_1: 45 * -1 = -45, * 1000 = -45000
    # joint_2: 30 * 1 = 30, * 1000 = 30000
    # joint_3: -20 * 1 = -20, * 1000 = -20000
    assert call_args[0] == -45000
    assert call_args[1] == 30000
    assert call_args[2] == -20000
```

## 6. Improved Documentation

### Update README.md

Add a section on joint_4 behavior:

```markdown
## Important: Joint 4 Behavior

The Piper arm has 6 degrees of freedom (joints 1-6), but the SO-101 teleoperation arm only has 5 arm joints that map to the Piper. **Joint 4 on the Piper is not controlled during teleoperation.**

By default, joint_4 is held at a fixed position (0 degrees). You can configure this behavior:

```python
config = PiperConfig(
    can_interface="can0",
    joint_4_mode="fixed",  # Options: "fixed", "follow_joint3", "manual"
    joint_4_fixed_position=0.0,  # Degrees when in "fixed" mode
)
```

### Modes:
- **fixed**: Holds joint_4 at `joint_4_fixed_position` degrees
- **follow_joint3**: Makes joint_4 track joint_3 (useful for some arm configurations)
- **manual**: Does not initialize joint_4 (use only if you have another control mechanism)

Please ensure joint_4 is in a safe position before starting teleoperation.
```

## 7. Configuration File Example

Create `examples/piper_config_example.py`:

```python
from lerobot_robot_piper import PiperConfig
from lerobot.cameras.opencv import OpenCVCameraConfig

# Example 1: Basic configuration with normalized control
config_normalized = PiperConfig(
    can_interface="can0",
    bitrate=1_000_000,
    use_degrees=False,  # Use normalized [-100, 100] range
    include_gripper=True,
    joint_4_mode="fixed",
    joint_4_fixed_position=0.0,
)

# Example 2: Degree-based control with custom signs
config_degrees = PiperConfig(
    can_interface="can0",
    use_degrees=True,
    include_gripper=True,
    joint_signs=[-1, 1, 1, -1, 1, -1],  # Custom sign corrections
    cameras={
        "wrist": OpenCVCameraConfig(
            index_or_path=0,
            width=640,
            height=480,
            fps=30,
            fourcc="MJPG",
        )
    },
)

# Example 3: Advanced configuration with joint_4 tracking
config_advanced = PiperConfig(
    can_interface="can0",
    use_degrees=True,
    include_gripper=True,
    joint_4_mode="follow_joint3",  # Make joint_4 track joint_3
    joint_4_offset=0.0,  # Offset in degrees
    enable_timeout=10.0,  # Longer timeout for initialization
)
```

## 8. Safety Checklist

Before deployment, verify:

- [ ] Joint limits are correctly read from SDK
- [ ] joint_4 is initialized to a safe position
- [ ] Sign corrections match physical robot orientation
- [ ] Gripper limits are appropriate
- [ ] Emergency stop is functional
- [ ] Teleoperation dead-man switch is implemented (if applicable)
- [ ] Robot workspace limits are enforced
- [ ] Collision detection is enabled (if available)

## Implementation Priority

1. **High Priority** (Safety Critical):
   - Fix joint_4 initialization
   - Improve limit validation
   - Add action limit checking

2. **Medium Priority** (Stability):
   - Add unit tests
   - Improve error messages
   - Add configuration validation

3. **Low Priority** (Nice to Have):
   - Integration tests
   - Documentation improvements
   - Example configurations

---

**Document Version**: 1.0  
**Date**: December 5, 2024
