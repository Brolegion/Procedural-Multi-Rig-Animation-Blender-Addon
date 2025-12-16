# properties.py
import bpy
from bpy.types import PropertyGroup
from bpy.props import (
    StringProperty, BoolProperty, IntProperty, FloatProperty, EnumProperty
)
from .translations import tr

def apply_creature_preset_callback(self, context):
    """Автоматически применяет набор параметров при выборе пресета в UI."""
    # Принудительная перерисовка UI
    try:
        if context and context.area:
            context.area.tag_redraw()
    except Exception:
        pass

    if self.creature_preset == 'CUSTOM':
        return

    presets = {
        'STEVE': {
            'step_height': 0.12, 'stride_angle': 0.35, 'floatiness': 0.6,
            'noise_amount': 0.02, 'push_profile': 'SMOOTH',
            'push_strength': 0.9, 'com_inertia': 0.6
        },
        'ZOMBIE': {
            'step_height': 0.06, 'stride_angle': 0.22, 'floatiness': 0.08,
            'noise_amount': 0.18, 'push_profile': 'SNAPPY',
            'push_strength': 0.4, 'com_inertia': 0.85
        },
        'SPIDER': {
            'step_height': 0.04, 'stride_angle': 0.28, 'floatiness': 0.0,
            'noise_amount': 0.05, 'push_profile': 'SUSTAIN',
            'push_strength': 0.6, 'com_inertia': 0.5
        },
        'RUNNER': {
            'step_height': 0.06, 'stride_angle': 0.45,
            'floatiness': 0.12, 'noise_amount': 0.01, 'push_profile': 'SNAPPY',
            'push_strength': 1.5, 'com_inertia': 0.3
        }
    }

    data = presets.get(self.creature_preset)
    if data:
        for attr, value in data.items():
            try:
                if hasattr(self, attr):
                    setattr(self, attr, value)
            except Exception as e:
                print(f"Error setting {attr}: {str(e)}")
                
def update_unit_category(self, context):
    """Автоматически устанавливает single_phase при изменении категории"""
    if self.unit_category == 'SIMPLE':
        self.single_phase = True
    else:
        self.single_phase = False
def update_vehicle_type(self, context):
    """Auto-sets defaults based on vehicle_type preset."""
    presets = {  # Кратко, как в анализе
        'CAR': {'num_axles': 2, 'steering_mode': 'FRONT', 'suspension_stiffness': 10.0, 'suspension_damping': 0.5},
        'BIKE': {'num_axles': 1, 'steering_mode': 'TILT', 'suspension_stiffness': 15.0, 'suspension_damping': 0.7},
        'TANK': {'num_axles': 2, 'steering_mode': 'TANK', 'suspension_stiffness': 8.0, 'suspension_damping': 0.4},
        'TRAIN': {'num_axles': 4, 'steering_mode': 'NONE', 'suspension_stiffness': 5.0, 'suspension_damping': 0.3},
        'MONO': {'num_axles': 1, 'steering_mode': 'BALANCE', 'suspension_stiffness': 20.0, 'suspension_damping': 0.8},
        'TRI': {'num_axles': 2, 'steering_mode': 'FRONT', 'suspension_stiffness': 12.0, 'suspension_damping': 0.6},
        'DRONE': {'num_axles': 1, 'steering_mode': 'TILT', 'suspension_stiffness': 25.0, 'suspension_damping': 0.9},
        'QUAD': {'num_axles': 2, 'steering_mode': 'ALL', 'suspension_stiffness': 8.0, 'suspension_damping': 0.4},
    }
    data = presets.get(self.vehicle_type, {})
    for attr, value in data.items():
        if hasattr(self, attr):
            setattr(self, attr, value)
    # Redraw UI
    if context and context.area:
        context.area.tag_redraw()

class PW_Settings(PropertyGroup):
    """Properties for Procedural Walk/Turn addon"""

    # frames / speed
    frame_start: IntProperty(name="Start", default=1, min=0)
    frame_end: IntProperty(name="End", default=60, min=1)
    frequency: FloatProperty(name="Cycles", default=2.0, min=0.0)

    # step params
    step_height: FloatProperty(name="Step Height", default=0.08, min=0.0)
    stride_angle: FloatProperty(name="Stride Angle (rad)", default=0.35, min=0.0)
    floatiness: FloatProperty(name="Floatiness", default=0.4, min=0.0, max=2.0)

    bone_name_mask: StringProperty(
        name="Bone Name Mask",
        default="leg;thigh;upleg;Left;Right;L;R",
        description="Tokens to find leg roots (used by automatic detection). Semi-colon separated."
    )

    unit_category: EnumProperty(
        name="Unit Category",
        items=[
            ('LIVING', 'Living / Humanoid', 'Animals, humanoids, complex skeletons'),
            ('MECHANICAL', 'Mechanical', 'Wheeled, tracked, flying vehicles / machines'),
            ('SIMPLE', 'Simple', 'One-bone or single-mesh units: slime, drone, ball'),
        ],
        default='LIVING',
        update=update_unit_category  # Добавлен callback
    )

    auto_detect_category: BoolProperty(
        name="Auto-detect Category",
        description="Try to detect category from selected object and switch UI accordingly",
        default=True
    )
    
    creature_preset: EnumProperty(
        name="Preset",
        items=[
            ('CUSTOM', 'Custom', 'Custom settings (no preset)'),
            ('STEVE', 'Steve', 'Floaty shuffle'),
            ('ZOMBIE', 'Zombie', 'Shamble'),
            ('SPIDER', 'Spider', 'Multi-leg'),
            ('RUNNER', 'Runner', 'Fast'),
        ],
        default='CUSTOM',
        update=apply_creature_preset_callback
    )

    # apply preset automatically on generate
    apply_preset_on_generate: BoolProperty(
        name="Apply Preset on Generate",
        description="Automatically apply the selected preset when generating animation",
        default=True
    )

    use_orbital: BoolProperty(name="Orbital Motion", default=True)
    use_invert: BoolProperty(name="Invert Joints", default=False)
    use_lag: BoolProperty(name="Lag / Delay", default=False)
    noise_amount: FloatProperty(name="Noise", default=0.0, min=0.0, max=1.0)

    left_leg_name: StringProperty(name="Left Bone (override)", default="")
    right_leg_name: StringProperty(name="Right Bone (override)", default="")

    forward_axis: EnumProperty(
        name="Forward Axis",
        items=[('X', 'X', ''), ('-X', '-X', ''), ('Y', 'Y', ''), ('-Y', '-Y', ''), ('Z', 'Z', ''), ('-Z', '-Z', '')],
        default='Y'
    )
    forward_coordinate_system: EnumProperty(
        name="Forward Coordinate System",
        description="Coordinate system for forward direction",
        items=[
            ('GLOBAL', 'Global', 'Use global coordinate system'),
            ('LOCAL', 'Local', 'Use local bone coordinate system')
        ],
        default='GLOBAL'
    )

    # Phasing / asymmetry
    single_phase: BoolProperty(
        name="Single Phase",
        description="Use single phase animation for simple units (drones, slimes, balls)",
        default=False
    )
    phase_offset: FloatProperty(
        name="Phase Offset (rad)",
        description="Phase offset between legs. π ≈ opposite phase, 0 — sync.",
        default=3.141592653589793,
        min=0.0,
        max=2.0 * 3.141592653589793
    )
    phase_noise: FloatProperty(
        name="Phase Noise",
        description="Small deterministic sinusoidal phase noise.",
        default=0.05,
        min=0.0,
        max=1.0
    )
    leg_phase_drift: FloatProperty(
        name="Phase Drift",
        description="Slow drift of phases over time.",
        default=0.0,
        min=0.0,
        max=0.2
    )

    left_amplitude: FloatProperty(
        name="Left Amplitude",
        description="Multiplier for left leg amplitude (1.0 = no change).",
        default=1.0,
        min=0.0,
        max=2.0
    )
    right_amplitude: FloatProperty(
        name="Right Amplitude",
        description="Multiplier for right leg amplitude (1.0 = no change).",
        default=1.0,
        min=0.0,
        max=2.0
    )

    com_lateral_amount: FloatProperty(
        name="COM Lateral Swing",
        description="Amplitude of pelvis lateral swing (meters).",
        default=0.03,
        min=0.0,
        max=0.5
    )

    # --- Arm Animation ---
    use_arm_animation: BoolProperty(
        name="Animate Arms",
        description="Enable procedural arm animation during walk/run",
        default=True
    )

    arm_name_mask: StringProperty(
        name="Arm Bone Mask",
        default="shoulder;upperarm;arm;clavicle",
        description="Semicolon-separated tokens to find arm bones"
    )

    arm_swing_amount: FloatProperty(
        name="Swing Amount",
        description="Forward/backward swing amplitude (radians)",
        default=0.5,
        min=0.0,
        max=2.0,
        subtype='ANGLE'
    )

    arm_lateral_amount: FloatProperty(
        name="Lateral Swing",
        description="Sideways swing amplitude",
        default=0.1,
        min=0.0,
        max=1.0
    )

    arm_elbow_bend: FloatProperty(
        name="Elbow Bend",
        description="Base elbow bend angle (0=straight, 1=90°)",
        default=0.4,
        min=0.0,
        max=1.0
    )

    arm_dynamic_bend: FloatProperty(
        name="Dynamic Bend",
        description="Extra elbow bend when swinging forward",
        default=0.3,
        min=0.0,
        max=1.0
    )

    arm_phase_offset: FloatProperty(
        name="Phase Offset",
        description="Phase offset relative to legs (π = opposite phase)",
        default=3.14159,
        min=0.0,
        max=6.28318,
        subtype='ANGLE'
    )

    arm_stiffness: FloatProperty(
        name="Arm Stiffness",
        description="How stiff the arm movement is",
        default=0.7,
        min=0.0,
        max=1.0
    )

    arm_swing_plane: EnumProperty(
        name="Swing Plane",
        description="Primary plane for arm swinging",
        items=[
            ('AUTO', 'Auto', 'Automatic detection based on rest pose'),
            ('SAGITTAL', 'Sagittal', 'Forward/backward (default for T-pose)'),
            ('FRONTAL', 'Frontal', 'Sideways (for certain rigs)'),
            ('COMBINED', 'Combined', 'Both forward and sideways')
        ],
        default='AUTO'
    )

    arm_upper_axis: EnumProperty(
        name="Upper Arm Axis",
        description="Primary rotation axis for upper arm",
        items=[
            ('AUTO', 'Auto', 'Automatic detection'),
            ('X', 'X', 'X axis'),
            ('Y', 'Y', 'Y axis'),
            ('Z', 'Z', 'Z axis')
        ],
        default='AUTO'
    )

    # Calibration
    calibration_enabled: BoolProperty(
        name="Enable Push Calibration",
        description="Enable numeric calibration push -> geometric forward",
        default=True
    )
    calibration_samples: IntProperty(
        name="Calibration Samples",
        description="Number of samples for calibration (more = finer).",
        default=128,
        min=16,
        max=2048
    )
    calibration_clamp_min: FloatProperty(name="Calibration Min", default=0.25, min=0.0, max=10.0)
    calibration_clamp_max: FloatProperty(name="Calibration Max", default=4.0, min=0.0, max=100.0)
    calibration_scale: FloatProperty(name="Calibration Scale", default=1.0, min=0.0, max=100.0)



    # IK offsets & rear retargeting
    ik_pole_forward: FloatProperty(
        name="IK Pole Forward",
        description="Forward lead for pole target as fraction of stride length (pushes knee forward)",
        default=0.2, min=-1.0, max=2.0
    )
    ik_pole_lateral: FloatProperty(
        name="IK Pole Lateral",
        description="Lateral offset multiplier for pole target (moves knee sideways)",
        default=0.25, min=0.0, max=2.0
    )
    ik_foot_back_offset: FloatProperty(
        name="IK Foot Back Offset",
        description="Offset (meters) to move foot target backward along -forward (for heel-plant)",
        default=0.0, min=-0.5, max=0.5
    )
    ik_interpolation_mode: EnumProperty(
        name="IK Interpolation",
        items=[
            ('LINEAR', 'Linear', 'Linear interpolation (current)'),
            ('CONSTANT_SPEED', 'Constant Speed', 'Move at constant speed'),
            ('BEZIER', 'Bezier Curve', 'Smooth acceleration and deceleration'),
            ('EXPONENTIAL', 'Exponential Smoothing', 'Smooth movement with exponential decay'),
            ('SPRING', 'Spring', 'Spring physics simulation'),
            ('STEP', 'Step', 'Move by fixed steps'),
            ('QUADRATIC', 'Quadratic Ease', 'Quadratic ease-in and ease-out')
        ],
        default='LINEAR'
    )

    ik_constant_speed: FloatProperty(
        name="Constant Speed",
        default=0.1,
        min=0.001,
        max=10.0,
        description="Speed in meters per frame"
    )

    ik_exponential_alpha: FloatProperty(
        name="Exponential Alpha",
        default=0.3,
        min=0.01,
        max=1.0,
        description="Smoothing factor for exponential interpolation"
    )

    ik_spring_stiffness: FloatProperty(
        name="Spring Stiffness",
        default=10.0,
        min=0.1,
        max=100.0,
        description="Spring stiffness coefficient"
    )

    ik_spring_damping: FloatProperty(
        name="Spring Damping",
        default=0.5,
        min=0.0,
        max=10.0,
        description="Spring damping coefficient"
    )

    ik_step_factor: FloatProperty(
        name="Step Factor",
        default=0.5,
        min=0.01,
        max=1.0,
        description="Fixed step size factor"
    )

    rear_copy_mode: EnumProperty(
        name="Rear Copy Mode",
        description="Copy front leg animation to additional leg pairs",
        items=[
            ('NONE', 'None', 'No rear retargeting'),
            ('SAME_SIDE', 'Same Side', 'Copy left->left, right->right'),
            ('OPPOSITE_SIDE', 'Opposite Side', 'Copy left->right, right->left')
        ],
        default='NONE'
    )

    # Foot selection mask and manual overrides for IK target bones
    foot_name_mask: StringProperty(
        name="Foot Name Mask",
        default="toe;foot;end;tip",
        description="Tokens (semi-colon separated) to prefer when auto-selecting final foot/toe bone."
    )

    ik_foot_override_left: StringProperty(
        name="IK Foot Left (override)",
        default="",
        description="Optional explicit bone name to use as IK target for the left leg. If set, takes priority over automatic selection."
    )
    ik_foot_override_right: StringProperty(
        name="IK Foot Right (override)",
        default="",
        description="Optional explicit bone name to use as IK target for the right leg. If set, takes priority over automatic selection."
    )

    ik_foot_override: StringProperty(
        name="IK Foot (global override)",
        default="",
        description="If set and specific left/right overrides are empty, this bone name will be used for both sides (use with caution)."
    )
    def update_use_ik(self, context):
        arm_obj = context.object
        if not arm_obj or arm_obj.type != 'ARMATURE':
            return
        settings = context.scene.pw_settings
        left_name = settings.left_leg_name or "thigh.L"
        right_name = settings.right_leg_name or "thigh.R"
        left_pose = arm_obj.pose.bones.get(left_name)
        right_pose = arm_obj.pose.bones.get(right_name)
        from .utils import remove_ik_constraints
        if not settings.use_ik:
            if left_pose:
                remove_ik_constraints(left_pose)
            if right_pose:
                remove_ik_constraints(right_pose)
            if settings.rear_copy_mode != 'NONE':
                from .utils import find_leg_pairs_by_name_or_space
                pairs = find_leg_pairs_by_name_or_space(arm_obj, settings.bone_name_mask)
                for l, r in pairs:
                    if l == left_name and r == right_name: continue
                    lp = arm_obj.pose.bones.get(l)
                    rp = arm_obj.pose.bones.get(r)
                    if lp: remove_ik_constraints(lp)
                    if rp: remove_ik_constraints(rp)
        # Если use_ik включен, IK constraints будут добавлены при генерации анимации

    use_ik: BoolProperty(
        name="Use IK for Legs",
        default=False,
        update=update_use_ik
    )
    ik_chain_count: IntProperty(name="IK Chain Count", default=2, min=1, max=5)
    ik_pole_bone_name: StringProperty(
        name="IK Pole Bone",
        default="",
        description="Name of bone to use as pole target (leave empty for auto)"
    )
    ik_invert_knee: BoolProperty(name="Invert Knee (flip pole angle)", default=False)

    # Center / root-motion
    center_enabled: BoolProperty(name="Enable Center COM", default=True)
    center_apply_mode: EnumProperty(
        name="Center Apply",
        items=[
            ('NONE', 'None (compute only)', ''),
            ('POSE_BONE', 'Apply to Pose Bone', ''),
            ('ARMATURE_OBJECT', 'Move Armature Object', ''),
        ],
        default='POSE_BONE'
    )
    jump_preserve_pose: BoolProperty(name="Save current pose", default=True)
    center_target_bone: StringProperty(name="Center Bone", default="pelvis")
    center_stride_scale: FloatProperty(name="Center Scale", default=1.0, min=0.0)
    center_preserve_height: BoolProperty(name="Preserve Rest Height", default=True)
    center_bob_amount: FloatProperty(name="Body Bob", default=0.25, min=0.0, max=2.0)
    center_only_compute: BoolProperty(name="Only Compute (no leg keys)", default=False)
    backward_bias: FloatProperty(name="Backward Bias", default=0.6, min=0.0, max=1.0)

    # Push profile
    push_strength: FloatProperty(name="Push Strength", default=0.9, min=0.0, max=2.0)
    push_profile: EnumProperty(
        name="Push Profile",
        items=[('SNAPPY', 'Snappy', ''), ('SMOOTH', 'Smooth', ''), ('SUSTAIN', 'Sustain', '')],
        default='SMOOTH'
    )
    push_aggressiveness: FloatProperty(name="Aggressiveness", default=0.6, min=0.0, max=1.5)
    com_inertia: FloatProperty(name="COM Inertia", default=0.6, min=0.0, max=0.98)
    com_vertical_mode: EnumProperty(
        name="Vertical Mode",
        items=[('SUM', 'Sum lifts', ''), ('STANCE', 'Stance-weighted', ''), ('NONE', 'None', '')],
        default='SUM'
    )

    # Добавим JUMP в animation_type
    animation_type: EnumProperty(
        name="Animation Type",
        items=[
            ('WALK', 'Walk', ''),
            ('DRONE', 'Drone', ''),
            ('IDLE', 'Idle', ''),
            ('ATTACK', 'Attack', ''),
            ('CUSTOM', 'Custom', ''),
            ('TURN', 'Turn', 'Procedural body turn'),
            ('FULL_BODY_SWING', 'Full Body Swing', 'Full body rotation without return'),
            ('JUMP', 'Jump', 'Procedural jump with squash/stretch')  # НОВОЕ
        ],
        default='WALK'
    )

    mech_unit_type: EnumProperty(
        name="Unit Type",
        items=[
            ('VEHICLE', 'Wheeled Vehicle', 'Car, tank, bike, monowheel, etc.'),
            ('DRONE', 'Wheeled Drone', 'Small UGV, robot'),
            ('HOVER', 'Hovercraft', 'Anti-gravity, maglev'),
            ('BOAT', 'Boat / Submarine', 'Water vehicle'),
            ('CUSTOM', 'Custom', 'Manual setup'),
        ],
        default='VEHICLE'
    )

    # Turn settings
    turn_angles: StringProperty(name="Turn Angles", default="35,5", description="Angle,speed per segment (e.g., '35,5;45,3')")
    turn_speed: FloatProperty(name="Turn Speed", default=5.0, min=0.1, max=10.0)
    turn_angle: FloatProperty(
        name="Turn Angle",
        default=35.0,
        min=-360.0,
        max=360.0,
        description="Angle in degrees (-360 to 360)"
    )
    spine_root_bone: StringProperty(name="Spine Root Bone", default="spine")
    spine_end_bone: StringProperty(name="End Spine Bone", default="neck")
    turn_bone_limits: StringProperty(name="Turn Bone Limits", default="", description="Bone:limit (deg) e.g., 'Spine:30;Neck:20'")

    # Humanoid override
    force_humanoid_override: BoolProperty(name="Override Skeleton Type", default=False)
    force_humanoid: BoolProperty(name="Force Humanoid", default=False)

    # UI language
    ui_language: EnumProperty(
        name="UI Language",
        items=[('AUTO', 'Auto', ''), ('EN', 'English', ''), ('RU', 'Русский', ''), ('DE', 'Deutsch', ''), ('ZH', '中文', '')],
        default='AUTO'
    )

    # Debug
    debug_pw: BoolProperty(name="Debug PW", default=True)

    # В класс PW_Settings добавим:

    # Turn interpolation and modulation settings
    turn_interpolation_mode: EnumProperty(
        name="Turn Interpolation",
        items=[
            ('BEZIER', 'Bezier', 'Smooth acceleration and deceleration'),
            ('LINEAR', 'Linear', 'Linear interpolation'),
            ('EASE_IN_OUT', 'Ease In Out', 'Ease in and out'),
            ('CONSTANT', 'Constant', 'Constant speed'),
            ('PHASE_MODULATED', 'Phase Modulated', 'Modulated by step phases')
        ],
        default='PHASE_MODULATED'
    )

    turn_inertia_in: FloatProperty(
        name="Turn Inertia In",
        description="Smoothness at the start of turn (0 = immediate, 1 = very slow start)",
        default=0.3,
        min=0.0,
        max=1.0
    )

    turn_inertia_out: FloatProperty(
        name="Turn Inertia Out", 
        description="Smoothness at the end of turn (0 = immediate stop, 1 = very slow stop)",
        default=0.3,
        min=0.0,
        max=1.0
    )

    turn_step_influence: FloatProperty(
        name="Step Influence",
        description="How much each step influences the rotation speed",
        default=0.5,
        min=0.0,
        max=2.0
    )

    turn_modulation_strength: FloatProperty(
        name="Turn Modulation Strength",
        description="Strength of step phase modulation on turn interpolation",
        default=0.5,
        min=0.0,
        max=2.0
    )

    turn_sensitivity: FloatProperty(
        name="Turn Sensitivity",
        description="Overall sensitivity of turn to steps",
        default=1.0,
        min=0.0,
        max=2.0
    )

    # Walk parameters that should also affect turns
    turn_step_height: FloatProperty(
        name="Step Height",
        default=0.08,
        min=0.0,
        description="Step height for turn animation"
    )

    turn_stride_angle: FloatProperty(
        name="Stride Angle",
        default=0.35,
        min=0.0,
        description="Stride angle for turn animation"
    )

    turn_floatiness: FloatProperty(
        name="Floatiness", 
        default=0.4,
        min=0.0,
        max=2.0,
        description="Floatiness for turn animation"
    )

    # Jump for armatures
    jump_crouch_factor: FloatProperty(
        name="Crouch Factor",
        description="How deep the crouch before jump (based on leg length)",
        default=1.0,
        min=0.0,
        max=2.0
    )

    jump_use_limb_ik: BoolProperty(
        name="Use IK for Jump",
        description="Use IK for legs during jump (otherwise FK)",
        default=True
    )

    jump_arm_swing: FloatProperty(
        name="Arm Swing",
        description="Arm swing amount during jump",
        default=1.0,
        min=0.0,
        max=2.0
    )

    jump_trail_effect: BoolProperty(
        name="Trail Effect",
        description="Add squash/stretch trail effect during jump",
        default=True
    )

    jump_secondary_motion: BoolProperty(
        name="Secondary Motion",
        description="Add secondary motion to spine and accessories",
        default=True
    )
    # -------------------------
    # Simple unit properties (idle, move, jump, dodge, damage)
    # Simple unit target selection
    simple_target_type: bpy.props.EnumProperty(
        name="Target Type",
        description="What to animate: the armature object or a specific bone",
        items=[
            ('OBJECT', "Object", "Animate the entire object"),
            ('BONE', "Bone", "Animate a specific bone in the armature")
        ],
        default='OBJECT'
    )

    simple_target_bone: bpy.props.StringProperty(
        name="Bone",
        description="Name of the bone to animate (if target type is Bone)",
        default=""
    )
    # -------------------------
    # Idle
    idle_amp: FloatProperty(name="Idle Amp (m)", default=0.08, min=0.0)
    idle_freq: FloatProperty(name="Idle Freq (cycles)", default=0.5, min=0.0)
    idle_rot_deg: FloatProperty(name="Idle Tilt (deg)", default=2.5, min=0.0)
    idle_stiffness: FloatProperty(name="Idle Stiffness", default=8.0, min=0.1)
    idle_damping: FloatProperty(name="Idle Damping", default=0.7, min=0.0)

    # В класс PW_Settings, после существующих свойств для idle, добавим:

    # --- Idle Noise Settings ---
    use_noise_idle: BoolProperty(
        name="Use Noise Idle",
        description="Add procedural noise to idle animation",
        default=True
    )

    idle_noise_seed: IntProperty(
        name="Seed",
        description="Seed for noise generator",
        default=12345,
        min=0
    )

    idle_noise_amp_vertical: FloatProperty(
        name="Vertical Amp",
        description="Vertical noise amplitude",
        default=0.08,
        min=0.0,
        max=1.0
    )

    idle_noise_amp_horizontal: FloatProperty(
        name="Horizontal Amp",
        description="Horizontal noise amplitude",
        default=0.04,
        min=0.0,
        max=1.0
    )

    idle_noise_amp_rotation: FloatProperty(
        name="Rotation Amp",
        description="Rotation noise amplitude (degrees)",
        default=3.0,
        min=0.0,
        max=45.0
    )

    idle_noise_frequency: FloatProperty(
        name="Noise Frequency",
        description="Noise frequency/speed",
        default=0.8,
        min=0.1,
        max=5.0
    )

    # Move / slide (canonical names)
    move_distance: FloatProperty(name="Move Distance", default=2.0, min=0.0)
    move_bob_amp: FloatProperty(name="Move Bob Amp", default=0.06, min=0.0)
    move_roll_deg: FloatProperty(name="Move Roll (deg)", default=6.0, min=0.0)
    move_stiffness: FloatProperty(name="Move Stiffness", default=18.0, min=0.0)
    move_damping: FloatProperty(name="Move Damping", default=0.7, min=0.0)
    move_accelerate_frac: FloatProperty(name="Move Accel Frac", default=0.15, min=0.0, max=1.0)
    move_decel_frac: FloatProperty(name="Move Decel Frac", default=0.15, min=0.0, max=1.0)

    # For compatibility with UI (aliases expected by ui.py)
    simple_move_distance: FloatProperty(
        name="Simple Move Distance",
        default=2.0,
        min=0.0,
        description="Alias for move_distance used by simple UI"
    )
    simple_move_accel_frac: FloatProperty(
        name="Simple Move Accel Frac",
        default=0.15,
        min=0.0,
        max=1.0
    )
    simple_move_decel_frac: FloatProperty(
        name="Simple Move Decel Frac",
        default=0.15,
        min=0.0,
        max=1.0
    )

    # Jump (canonical)
    # --- Mode Switch ---
    jump_physics_enabled: BoolProperty(
        name="Use Physics",
        description="Calculate trajectory based on mass and gravity",
        default=True
    )

    # --- Common / Target ---
    jump_height: FloatProperty(name="Jump Height", default=1.0, min=0.0, description="Target peak height of the jump")
    # Используем jump_height как целевую высоту для обоих режимов

    # --- Deformation ---
    jump_squash: FloatProperty(name="Jump Squash", default=0.12, min=0.0, max=1.0)
    jump_stretch: FloatProperty(name="Jump Stretch", default=0.08, min=0.0, max=1.0)

    # --- Physical Mode Params ---
    jump_gravity: FloatProperty(name="Gravity", default=9.81, min=0.1, max=100.0, description="m/s²")
    jump_mass: FloatProperty(name="Mass (kg)", default=50.0, min=1.0, max=1000.0)
    jump_landing_height: FloatProperty(name="Landing Offset", default=0.0, min=-10.0, max=10.0,
                                       description="Height difference for landing")
    jump_randomness: FloatProperty(name="Randomness", default=0.05, min=0.0, max=1.0)
    jump_tilt_amount: FloatProperty(name="Air Tilt", default=15.0, min=0.0, max=90.0,
                                    description="Pitch angle during jump (rigid body)")

    # --- Simple Mode Params (Legacy) ---
    jump_takeoff_frac: FloatProperty(name="Takeoff Phase", default=0.25, min=0.0, max=1.0)
    jump_hang_frac: FloatProperty(name="Hang Time", default=0.2, min=0.0, max=1.0)

    # UI Aliases (kept for compatibility if needed, but mapped to new logic where possible)
    simple_jump_height: FloatProperty(name="Simple Jump Height", default=1.0, min=0.0)
    simple_jump_takeoff_frac: FloatProperty(name="Simple Jump Takeoff Fraction", default=0.25, min=0.0, max=1.0)
    simple_jump_hang_frac: FloatProperty(name="Simple Jump Hang Fraction", default=0.18, min=0.0, max=1.0)

    # Dodge / Damage
    dodge_distance: FloatProperty(name="Dodge Distance", default=1.0, min=0.0)
    dodge_quick_frac: FloatProperty(name="Dodge Quick Fraction", default=0.25, min=0.0, max=1.0)

    # UI expects different names
    dodge_lateral_dist: FloatProperty(name="Dodge Lateral Dist", default=1.0, min=0.0)
    dodge_side: EnumProperty(
        name="Dodge Side",
        items=[('L', 'Left', ''), ('R', 'Right', '')],
        default='R'
    )

    damage_shake_amp: FloatProperty(name="Damage Shake Amp", default=0.06, min=0.0)
    damage_recoil_z: FloatProperty(name="Damage Recoil Z", default=0.12, min=0.0)
    damage_shake_freq: FloatProperty(name="Damage Shake Freq", default=20.0, min=0.0)

    # Turn settings expected by simple UI:
    turn_overshoot_deg: FloatProperty(name="Turn Overshoot Deg", default=6.0, min=0.0, max=90.0)
    turn_ease: FloatProperty(name="Turn Ease Fraction", default=0.85, min=0.01, max=0.99)

    # New animation parameters
    surprise_amp: bpy.props.FloatProperty(
        name="Surprise Amplitude",
        description="How high the unit jumps in surprise",
        default=0.3,
        min=0.0,
        max=2.0
    )

    panic_speed: bpy.props.FloatProperty(
        name="Panic Speed", 
        description="Speed of panic vibrations",
        default=15.0,
        min=1.0,
        max=30.0
    )

    shoot_recoil: bpy.props.FloatProperty(
        name="Shoot Recoil",
        description="How much the unit moves back when shooting",
        default=0.1,
        min=0.0,
        max=0.5
    )

    roll_rotations: bpy.props.IntProperty(
        name="Roll Rotations",
        description="How many full rotations during roll",
        default=2,
        min=1,
        max=5
    )

    # --- Основные параметры ---
    fly_amp: FloatProperty(
        name="Vertical Amplitude",
        description="Сила вертикального колебания",
        default=0.25,
        min=0.0, max=10.0
    )

    fly_lateral: FloatProperty(
        name="Lateral Amplitude",
        description="Боковое отклонение",
        default=0.15,
        min=0.0, max=10.0
    )

    fly_forward_dist: FloatProperty(
        name="Forward Distance",
        description="Общее расстояние полёта",
        default=3.0,
        min=0.0, max=1000.0
    )

    fly_speed: FloatProperty(
        name="Speed",
        description="Базовая скорость полёта (влияет на частоту шума и волн)",
        default=1.0,
        min=0.01, max=10.0
    )

    fly_octaves: IntProperty(
        name="Noise Octaves",
        description="Количество октав шума (детализация)",
        default=4,
        min=1, max=8
    )

    fly_seed: IntProperty(
        name="Seed",
        description="Сид для процедурного шума",
        default=12345,
        min=0, max=999999
    )

    # --- Ротация ---
    fly_bank_strength: FloatProperty(
        name="Bank Strength",
        description="Сила крена при поворотах",
        default=0.30,
        min=0.0, max=2.0
    )

    fly_pitch_strength: FloatProperty(
        name="Pitch Strength",
        description="Сила тангажа при наборе высоты",
        default=0.10,
        min=0.0, max=2.0
    )

    fly_yaw_strength: FloatProperty(
        name="Yaw Strength",
        description="Небольшие рыскания для реализма",
        default=0.045,
        min=0.0, max=2.0
    )

    # --- Масштабирование ---
    allow_scale_changes: BoolProperty(
        name="Breathing / Scale Motion",
        description="Позволять изменять масштаб в анимации полёта",
        default=True
    )

    fly_breath_amp: FloatProperty(
        name="Breath Amplitude",
        description="Сила дыхательного эффекта масштаба",
        default=0.03,
        min=0.0, max=1.0
    )
# В class PW_Settings добавить:

    vehicle_type: EnumProperty(
        name="Vehicle Type",
        items=[
            ('CUSTOM', 'Custom', 'Manual settings'),
            ('CAR', 'Car', 'Standard 4-wheel car'),
            ('BIKE', 'Bike/Moto', '2-wheel motorcycle'),
            ('TANK', 'Tank/UGV', 'Tracked or diff-turn vehicle'),
            ('TRAIN', 'Train/Tram', 'Fixed path multi-axle'),
            ('MONO', 'Mono-wheel', 'Single wheel bot/drone'),
            ('TRI', 'Trike', '3-wheel vehicle'),
            ('DRONE', 'Wheeled Drone', 'Small 2-wheel bot'),
            ('QUAD', 'Quad/ATV', '4-wheel all-terrain'),
        ],
        default='CUSTOM',
        update=update_vehicle_type  # Новый callback ниже
    )

    # Только для VEHICLE и DRONE — какой режим анимации
    vehicle_animation_mode: EnumProperty(
        name="Animation",
        description="What the vehicle should do",
        items=[
            ('DRIVE', 'Drive Forward', 'Acceleration → cruise → brake'),
            ('REVERSE', 'Reverse', 'Drive backward'),
            ('TURN_IN_PLACE', 'Turn in Place', 'Pivot without moving'),
            ('DRIFT', 'Drift / Slide', 'Power slide'),
            ('JUMP', 'Jump', 'Suspension bounce / boost'),
            ('IDLE', 'Idle', 'Subtle wobble on spot'),
            ('CIRCLE', 'Drive in Circle', 'Continuous turn'),
        ],
        default='DRIVE'
    )
    # Vehicle-specific
    vehicle_wheel_mask: StringProperty(
        name="Wheel Mask",
        default="wheel;tire;rim;front;rear;left;right",
        description="Tokens to find wheel bones; semi-colon separated"
    )
    vehicle_num_axles: IntProperty(name="Num Axles", default=2, min=1, max=10)
    vehicle_steering_mode: EnumProperty(
        name="Steering Mode",
        items=[('FRONT', 'Front', ''), ('ALL', 'All Wheels', ''), ('TANK', 'Tank Diff', ''), ('TILT', 'Tilt/Balance', ''), ('NONE', 'None', '')],
        default='FRONT'
    )
    vehicle_max_speed: FloatProperty(name="Max Speed (m/s)", default=5.0, min=0.0)
    vehicle_accel_time: FloatProperty(name="Accel Time (s)", default=3.0, min=0.01)
    vehicle_brake_time: FloatProperty(name="Brake Time (s)", default=2.0, min=0.01)
    vehicle_steering_max_angle: FloatProperty(name="Max Steer Angle (deg)", default=30.0, min=0.0, max=90.0)
    vehicle_suspension_stiffness: FloatProperty(name="Susp Stiffness", default=10.0, min=0.1)
    vehicle_suspension_damping: FloatProperty(name="Susp Damping", default=0.5, min=0.0, max=1.0)
    vehicle_suspension_rest: FloatProperty(name="Susp Rest Offset", default=0.0, min=-1.0, max=1.0)
    vehicle_suspension_travel: FloatProperty(name="Susp Travel", default=0.1, min=0.0)
    vehicle_roll_strength: FloatProperty(name="Roll Strength", default=0.1, min=0.0)
    vehicle_pitch_strength: FloatProperty(name="Pitch Strength", default=0.05, min=0.0)
    vehicle_noise_freq: FloatProperty(name="Bump Noise Freq", default=5.0, min=0.1)

# Новый callback (добавь после существующих)


def register():
    bpy.utils.register_class(PW_Settings)
    bpy.types.Scene.pw_settings = bpy.props.PointerProperty(type=PW_Settings)

def unregister():
    bpy.utils.unregister_class(PW_Settings)
    del bpy.types.Scene.pw_settings
