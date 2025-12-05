# Quick Reference Guide

## For Users: "I just want to use it"

**Start here**: [README.md](./README.md)

**Key things to know**:
- Piper arm has 6 arm joints, SO-101 has 5
- Joint 4 on Piper is NOT controlled by SO-101
- This is normal and intentional

**If something breaks**: Check [RECOMMENDATIONS.md](./RECOMMENDATIONS.md) Section 8 (Safety Checklist)

## For Developers: "I want to understand how it works"

**Start here**: [SUMMARY.md](./SUMMARY.md) → [JOINT_MAPPING.md](./JOINT_MAPPING.md)

**Deep dive**: [ANALYSIS.md](./ANALYSIS.md)

**Key concepts**:
1. Joint aliases map SO-101 names → Piper joint numbers
2. Sign corrections handle motor orientation differences
3. Two modes: degrees (raw) or normalized ([-100, 100])
4. Observation mirroring allows reading by alias names

## For Contributors: "I want to fix/improve it"

**Start here**: [RECOMMENDATIONS.md](./RECOMMENDATIONS.md)

**Priority order**:
1. **HIGH**: Fix joint_4 initialization (Section 1)
2. **HIGH**: Improve limit validation (Section 2)
3. **MEDIUM**: Add unit tests (Section 4)
4. **MEDIUM**: Improve error handling (Section 3)
5. **LOW**: Documentation improvements (Section 6)

**Before PR**:
- [ ] Add/run tests
- [ ] Update documentation
- [ ] Check safety considerations
- [ ] Test with hardware (if available)

## Documentation Map

```
README.md           ← Start: Installation & usage examples
    ↓
SUMMARY.md          ← Quick overview & answers to FAQs
    ↓
JOINT_MAPPING.md    ← Visual diagram of joint mapping
    ↓
ANALYSIS.md         ← Complete technical deep-dive
    ↓
RECOMMENDATIONS.md  ← Implementation guide for fixes
```

## Quick Answers

### Q: Why is joint_4 not controlled?
**A**: SO-101 has 5 arm joints, Piper has 6. Joint 4 is a wrist/forearm rotation that doesn't exist on SO-101.

### Q: Is this a bug?
**A**: No, it's an intentional design decision for teleoperation compatibility. However, it needs better handling (see RECOMMENDATIONS.md).

### Q: Can I control all 6 Piper joints?
**A**: Yes, if you bypass the SO-101 teleoperation and directly use Piper's action interface with native joint names.

### Q: What happens to joint_4 during teleoperation?
**A**: Currently: It maintains its last position. Recommended: Should be initialized to a safe position (not yet implemented).

### Q: Is it safe to use?
**A**: With precautions, yes. Key: Ensure joint_4 is in a safe position before starting. See RECOMMENDATIONS.md Section 8.

### Q: How do I run tests?
**A**: Tests are not yet implemented. See RECOMMENDATIONS.md Section 4-5 for test templates.

### Q: Which mode should I use: degrees or normalized?
**A**: 
- **Degrees**: More intuitive, matches physical angles
- **Normalized**: Better for neural networks, handles different robots

Most SO-101 examples use normalized mode (`use_degrees=False`).

### Q: What are the joint signs for?
**A**: They flip motor directions to compensate for different mounting orientations. Current values work for standard Piper configuration.

### Q: Can I customize the joint mapping?
**A**: Yes, edit `joint_aliases` in `PiperConfig`. But note: Changing this may break teleoperation compatibility.

## Code Snippets

### Basic Usage
```python
from lerobot_robot_piper import Piper, PiperConfig

config = PiperConfig(
    can_interface="can0",
    use_degrees=False,
    include_gripper=True,
)

robot = Piper(config)
robot.connect()

# Read current state
obs = robot.get_observation()
print(f"Joint 1: {obs['joint_1.pos']}")
print(f"Shoulder pan: {obs['shoulder_pan.pos']}")  # Same value as joint_1

# Send action (using SO-101 names)
action = {
    "shoulder_pan.pos": 0.0,
    "shoulder_lift.pos": 0.0,
    "elbow_flex.pos": 0.0,
    "wrist_flex.pos": 0.0,
    "wrist_roll.pos": 0.0,
    "gripper.pos": 50.0,
}
robot.send_action(action)

robot.disconnect()
```

### Read Joint Limits
```python
from lerobot_robot_piper import PiperConfig, Piper

config = PiperConfig(can_interface="can0")
robot = Piper(config)
robot.connect()

min_pos, max_pos = robot._get_hw_limits()
for i, (min_deg, max_deg) in enumerate(zip(min_pos[:6], max_pos[:6])):
    print(f"Joint {i+1}: {min_deg:.1f}° to {max_deg:.1f}°")
print(f"Gripper: {min_pos[6]:.1f}mm to {max_pos[6]:.1f}mm")

robot.disconnect()
```

### Check Connection
```python
from lerobot_robot_piper import Piper, PiperConfig

config = PiperConfig(can_interface="can0")
robot = Piper(config)

print(f"Connected: {robot.is_connected}")  # False

robot.connect()
print(f"Connected: {robot.is_connected}")  # True
print(f"Calibrated: {robot.is_calibrated}")  # True (always for Piper)

robot.disconnect()
print(f"Connected: {robot.is_connected}")  # False
```

## Common Issues & Solutions

### Issue: "C_PiperInterface_V2 not found"
**Solution**: Install piper_sdk: `pip install piper_sdk`

### Issue: "Failed to initialize Piper SDK"
**Solution**: 
1. Check CAN interface is up: `ip link show can0`
2. Bring up CAN: `sudo ip link set can0 up type can bitrate 1000000`
3. Check permissions: You may need root or add user to `dialout` group

### Issue: "Robot not moving"
**Solution**:
1. Check EnablePiper succeeded (check logs)
2. Verify robot is not in teaching mode
3. Check emergency stop is not engaged
4. Verify joint limits are reasonable

### Issue: "joint_4 in weird position"
**Solution**: This is the known issue. Manually position joint_4 before starting, or implement the fix from RECOMMENDATIONS.md Section 1.

### Issue: "Values are wrong/backwards"
**Solution**: 
1. Check `use_degrees` setting matches your expectations
2. Verify `joint_signs` configuration is correct for your robot
3. Check calibration if using normalized mode

## File Modification Guide

### To change joint mapping:
**File**: `lerobot_robot_piper/config_piper.py`
**Line**: ~19-27 (joint_aliases)

### To change sign corrections:
**File**: `lerobot_robot_piper/config_piper.py`
**Line**: ~17 (joint_signs)

### To add joint_4 handling:
**File**: `lerobot_robot_piper/piper.py`
**Section**: `connect()` method and `send_action()` method
**Reference**: RECOMMENDATIONS.md Section 1

### To improve error handling:
**File**: `lerobot_robot_piper/piper_sdk_interface.py`
**Section**: `__init__()` method, lines 66-76
**Reference**: RECOMMENDATIONS.md Section 2

### To add tests:
**Create**: `tests/test_piper_coordinate_transforms.py`
**Reference**: RECOMMENDATIONS.md Section 4

## Teleoperation Flow

```
1. Setup
   ├─ Start SO-101 leader arm
   ├─ Start Piper follower arm  
   └─ Connect both to LeRobot

2. Calibration (one-time)
   ├─ Calibrate SO-101 leader
   └─ Calibrate Piper follower (joint limits auto-read)

3. Teleoperation Loop
   ├─ SO-101 reads operator movements
   ├─ Movements → actions (with SO-101 joint names)
   ├─ LeRobot processes actions
   ├─ Actions → Piper via joint aliases
   ├─ Piper executes movements
   └─ Repeat

4. Recording (optional)
   ├─ Same as teleoperation
   └─ + Save observations/actions to dataset

5. Training (offline)
   └─ Train policy on recorded dataset

6. Deployment
   ├─ Load trained policy
   ├─ Policy generates actions
   ├─ Actions → Piper (same as teleoperation)
   └─ No leader arm needed
```

## Development Workflow

```
1. Fork/Clone repo
2. Read SUMMARY.md (10 min)
3. Read ANALYSIS.md relevant sections (30 min)
4. Identify issue/improvement (RECOMMENDATIONS.md)
5. Write test first (TDD)
6. Implement fix
7. Run tests
8. Update documentation
9. Create PR
```

## Resources

- **SO-101 Hardware**: https://github.com/TheRobotStudio/SO-ARM100
- **SO-101 Docs**: https://huggingface.co/docs/lerobot/so101
- **Piper SDK**: https://github.com/agilexrobotics/piper_sdk
- **LeRobot**: https://github.com/huggingface/lerobot
- **LeRobot Docs**: https://huggingface.co/docs/lerobot

## Support

- **Issues**: Open GitHub issue with detailed description
- **Questions**: Check SUMMARY.md FAQs first
- **Contributions**: Follow RECOMMENDATIONS.md priority guide
- **Discord**: LeRobot community (link in main repo)

---

**Quick Reference Version**: 1.0  
**Last Updated**: December 5, 2024  
**Status**: ✅ Complete
