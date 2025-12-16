# ui.py
import bpy
from .translations import tr
from .operators import (
    PW_OT_PresetExport,
    PW_OT_PresetImport,
    PW_OT_Generate,
    PW_OT_AddTurnConfig,
    PW_OT_SetNegativeAngle,
    # simple operators
    PW_OT_GenerateSimpleIdle,
    PW_OT_GenerateSimpleSlide,
    PW_OT_GenerateSimpleTurn,
    PW_OT_GenerateSimpleJump,
    PW_OT_GenerateSimpleMove,
    PW_OT_GenerateSimpleDamage,
    PW_OT_GenerateSimpleDodge,
    # NEW simple operators
    PW_OT_GenerateSimpleSurprise,
    PW_OT_GenerateSimplePanic,
    PW_OT_GenerateSimpleShoot,
    PW_OT_GenerateSimpleRoll,
    PW_OT_GenerateSimpleFly,
)
from . import utils

# Master panel — category selector + auto-detect + shared frame fields + status
class PW_PT_Panel(bpy.types.Panel):
    bl_label = "Procedural Anim"
    bl_idname = "VIEW3D_PT_pw_master"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Procedural Anim"

    @classmethod
    def poll(cls, context):
        return True

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        settings = scene.pw_settings
        if settings is None:
            layout.label(text="Settings not initialized.")
            return

        sel = context.object

        # Auto-detect category if requested
        detected = None
        status_info = None
        if getattr(settings, "auto_detect_category", False):
            try:
                detected = utils.detect_unit_category(sel)
                settings.unit_category = detected
                # Автоматически устанавливаем single_phase для простых юнитов
                if detected == 'SIMPLE':
                    settings.single_phase = True
            except Exception:
                detected = settings.unit_category
        else:
            detected = settings.unit_category

        # Try to generate a helpful status string
        try:
            if sel is None:
                status_info = "no selection"
            else:
                if sel.type == 'ARMATURE' and getattr(sel, "data", None):
                    bcount = len(sel.data.bones)
                    status_info = f"{bcount} bones"
                else:
                    # check for wheel-like children or keyname hints
                    child_names = " ".join((ch.name or "").lower() for ch in getattr(sel, "children", []))
                    lower_name = (sel.name or "").lower()
                    wheels_found = any(tok in child_names for tok in ('wheel','tyre','tire','track','_wl','_wr'))
                    mech_hint = any(tok in lower_name for tok in ('wheel','car','truck','tank','track','tyre','tire','prop','rotor','blade'))
                    if wheels_found:
                        status_info = "wheel children"
                    elif mech_hint:
                        status_info = "mechanical name"
                    else:
                        status_info = "generic object"
        except Exception:
            status_info = "unknown"

        # Top row: category selector + auto-detect toggle
        row = layout.row()
        row.prop(settings, "unit_category", expand=True)
        row = layout.row()
        row.prop(settings, "auto_detect_category", text="Auto detect")

        layout.separator()

        # Shared common fields visible in master so user doesn't have to switch tabs
        box_common = layout.box()
        box_common.label(text="Common (shared) Settings")
        colc = box_common.column(align=True)
        colc.prop(settings, "frame_start", text="Start Frame")
        colc.prop(settings, "frame_end", text="End Frame")
        # frequency is common-ish; show it here too
        colc.prop(settings, "frequency", text="Frequency / Cycles")

        layout.separator()

        # Status line: auto-detected category + short info
        status_text = f"Auto-detected: {detected}" if detected else "Category: (unset)"
        if status_info:
            status_text += f" — {status_info}"
        layout.label(text=status_text)

        layout.separator()
        layout.label(text="Selected: " + (sel.name if sel else "None"))


# -----------------------
# Living / Humanoid panel
# -----------------------
class PW_PT_LivingPanel(bpy.types.Panel):
    bl_label = "Living / Humanoid"
    bl_idname = "VIEW3D_PT_pw_living"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Procedural Anim"
    bl_parent_id = "VIEW3D_PT_pw_master"

    @classmethod
    def poll(cls, context):
        s = getattr(context.scene, "pw_settings", None)
        return s is not None and s.unit_category == 'LIVING'

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        settings = scene.pw_settings
        lang = settings.ui_language if hasattr(settings, "ui_language") else 'AUTO'

        # Keep the same behaviour as original panel for living units:
        # selection of animation_type + import/export + detailed blocks
        row = layout.row()
        row.prop(settings, "animation_type", text=tr("Animation Type", lang))

        # Import/Export presets
        row = layout.row(align=True)
        row.operator("pw.export_preset", text=tr("Export Preset", lang))
        row.operator("pw.import_preset", text=tr("Import Preset", lang))

        layout.separator()

        # Now show sections depending on selected animation_type (same as original)
        if settings.animation_type == 'WALK':
            box = layout.box()
            box.label(text=tr("WALK", lang))

            # Preset selection + apply button
            row = box.row(align=True)
            row.prop(settings, "creature_preset", text=tr("Preset", lang))
            row.operator("pw.apply_preset", text="", icon='CHECKMARK')

            box.operator("pw.generate_walk", text=tr("Generate Animation", lang))

            box2 = layout.box()
            box2.label(text=tr("Generator Settings", lang))
            box2.prop(settings, "frame_start")
            box2.prop(settings, "frame_end")
            box2.prop(settings, "frequency")
            box2.prop(settings, "step_height")
            box2.prop(settings, "stride_angle")
            box2.prop(settings, "floatiness")
            box2.prop(settings, "use_orbital", text=tr("Orbital Motion", lang))
            box2.prop(settings, "use_invert", text=tr("Invert Joints", lang))
            box2.prop(settings, "use_lag", text=tr("Lag / Delay", lang))
            box2.prop(settings, "noise_amount", text=tr("Noise Amount", lang))
            box2.prop(settings, "apply_preset_on_generate", text=tr("Apply Preset on Generate", lang))

            box3 = layout.box()
            box3.label(text=tr("Leg Selection (overrides)", lang))
            box3.prop(settings, "left_leg_name", text=tr("Left Bone (override)", lang))
            box3.prop(settings, "right_leg_name", text=tr("Right Bone (override)", lang))
            box3.prop(settings, "bone_name_mask")

            box4 = layout.box()
            box4.label(text=tr("COM / Push", lang))
            box4.prop(settings, "push_strength", text=tr("Push Strength", lang))
            box4.prop(settings, "push_profile", text=tr("Push Profile", lang))
            box4.prop(settings, "push_aggressiveness", text=tr("Aggressiveness", lang))
            box4.prop(settings, "com_inertia", text=tr("COM Inertia", lang))
            box4.prop(settings, "com_vertical_mode")
            box4.prop(settings, "com_lateral_amount", text=tr("COM Lateral Swing", lang))

            box5 = layout.box()
            box5.label(text=tr("Forward / Center (COM)", lang))
            box5.prop(settings, "forward_axis", text=tr("Forward Axis", lang))
            box5.prop(settings, "center_enabled")
            box5.prop(settings, "center_apply_mode", text=tr("Center Apply", lang))
            box5.prop(settings, "center_target_bone")
            box5.prop(settings, "center_preserve_height")
            box5.prop(settings, "center_bob_amount")
            box5.prop(settings, "center_stride_scale")
            box5.prop(settings, "backward_bias")

            box6 = layout.box()
            box6.label(text=tr("Coordinate System", lang))
            box6.prop(settings, "forward_coordinate_system", text=tr("Coordinate System", lang))
            box6.prop(settings, "foot_name_mask", text="Foot Name Mask")
            box6.prop(settings, "ik_foot_override_left", text="IK Foot Left (override)")
            box6.prop(settings, "ik_foot_override_right", text="IK Foot Right (override)")
            box6.prop(settings, "ik_foot_override", text="IK Foot (global override)")
            box6.prop(settings, "rear_copy_mode", text=tr("Rear Copy Mode", lang))

            # Phasing / Asymmetry
            box7 = layout.box()
            box7.label(text=tr("Phasing / Asymmetry", lang))
            box7.prop(settings, "phase_offset", text=tr("Phase Offset", lang))
            box7.prop(settings, "phase_noise", text=tr("Phase Noise", lang))
            box7.prop(settings, "leg_phase_drift", text=tr("Phase Drift", lang))
            box7.prop(settings, "left_amplitude", text=tr("Left Amplitude", lang))
            box7.prop(settings, "right_amplitude", text=tr("Right Amplitude", lang))

            # В классе PW_PT_LivingPanel, в разделе WALK, после блока "Phasing / Asymmetry" добавить:

            # Arm Animation Settings
            box_arms = layout.box()
            box_arms.label(text="Arm Animation")
            box_arms.prop(settings, "use_arm_animation", text="Enable Arm Animation")

            if settings.use_arm_animation:
                col = box_arms.column(align=True)
                col.prop(settings, "arm_swing_amount", text="Swing Amount")
                col.prop(settings, "arm_lateral_amount", text="Lateral Swing")
                col.prop(settings, "arm_elbow_bend", text="Elbow Bend")
                col.prop(settings, "arm_dynamic_bend", text="Dynamic Bend")
                col.prop(settings, "arm_phase_offset", text="Phase Offset")
                col.prop(settings, "arm_stiffness", text="Stiffness")

                # Advanced settings
                box_advanced = col.box()
                box_advanced.label(text="Advanced")
                box_advanced.prop(settings, "arm_swing_plane", text="Swing Plane")
                box_advanced.prop(settings, "arm_upper_axis", text="Upper Arm Axis")
                box_advanced.prop(settings, "arm_name_mask", text="Bone Name Mask")

            # Calibration
            box8 = layout.box()
            box8.label(text=tr("Calibration", lang))
            box8.prop(settings, "calibration_enabled", text=tr("Enable Push Calibration", lang))
            box8.prop(settings, "calibration_samples")
            box8.prop(settings, "calibration_clamp_min")
            box8.prop(settings, "calibration_clamp_max")
            box8.prop(settings, "calibration_scale")

            # IK / legs block
            box_ik = layout.box()
            box_ik.label(text=tr("IK / Legs", lang))
            box_ik.prop(settings, "use_ik")
            if settings.use_ik:
                box_ik.prop(settings, "ik_chain_count")
                box_ik.prop(settings, "ik_pole_bone_name")
                box_ik.prop(settings, "ik_invert_knee")
                box_ik.prop(settings, "ik_pole_forward")
                box_ik.prop(settings, "ik_pole_lateral")
                box_ik.prop(settings, "ik_foot_back_offset")
                box_ik.prop(settings, "ik_interpolation_mode", text=tr("IK Interpolation", lang))
                if settings.ik_interpolation_mode == 'CONSTANT_SPEED':
                    box_ik.prop(settings, "ik_constant_speed")
                elif settings.ik_interpolation_mode == 'EXPONENTIAL':
                    box_ik.prop(settings, "ik_exponential_alpha")
                elif settings.ik_interpolation_mode == 'SPRING':
                    box_ik.prop(settings, "ik_spring_stiffness")
                    box_ik.prop(settings, "ik_spring_damping")
                elif settings.ik_interpolation_mode == 'STEP':
                    box_ik.prop(settings, "ik_step_factor")
            box_ik.prop(settings, "rear_copy_mode", text=tr("Rear Copy Mode", lang))



        elif settings.animation_type == 'TURN':
            box = layout.box()
            box.label(text=tr("TURN", lang))
            box.operator("pw.generate_walk", text=tr("Generate Animation", lang))

            box2 = layout.box()
            box2.label(text=tr("Turn Settings", lang))
            box2.prop(settings, "turn_speed", text=tr("Turn Speed", lang))
            row = box2.row()
            row.prop(settings, "turn_angle", text=tr("Turn Angle (deg)", lang))
            row = box2.row(align=True)
            op1 = row.operator("pw.set_negative_angle", text="-90")
            if op1:
                op1.angle = -90
            op2 = row.operator("pw.set_negative_angle", text="-180")
            if op2:
                op2.angle = -180
            op3 = row.operator("pw.set_negative_angle", text="-360")
            if op3:
                op3.angle = -360

            box3 = layout.box()
            box3.label(text=tr("Spine Chain Settings", lang))
            box3.prop(settings, "spine_root_bone", text=tr("Spine Root Bone", lang))
            box3.prop(settings, "spine_end_bone", text=tr("End Spine Bone", lang))
            box3.prop(settings, "turn_angles")
            box3.prop(settings, "turn_bone_limits")

            box4 = layout.box()
            box4.label(text=tr("Forward / Center (COM)", lang))
            box4.prop(settings, "forward_axis", text=tr("Forward Axis", lang))
            box4.prop(settings, "center_enabled")
            box4.prop(settings, "center_apply_mode", text=tr("Center Apply", lang))
            box4.prop(settings, "center_target_bone")
            box4.prop(settings, "center_preserve_height")
            box4.prop(settings, "center_bob_amount")

            box5 = layout.box()
            box5.label(text=tr("Coordinate System", lang))
            box5.prop(settings, "forward_coordinate_system", text=tr("Coordinate System", lang))

            box6 = layout.box()
            box6.prop(settings, "force_humanoid_override", text=tr("Override Skeleton Type", lang))
            if settings.force_humanoid_override:
                box6.prop(settings, "force_humanoid", text=tr("Force Humanoid", lang))

        elif settings.animation_type == 'JUMP':
            box = layout.box()
            box.label(text=tr("JUMP", lang))
            box.operator("pw.generate_walk", text=tr("Generate Jump Animation", lang))

            # --- Leg Selection (как в WALK) ---
            box_legs = layout.box()
            box_legs.label(text=tr("Leg Selection (overrides)", lang))
            box_legs.prop(settings, "left_leg_name", text=tr("Left Bone (override)", lang))
            box_legs.prop(settings, "right_leg_name", text=tr("Right Bone (override)", lang))
            box_legs.prop(settings, "jump_preserve_pose")
            box_legs.prop(settings, "bone_name_mask")

            # --- Jump Settings ---
            box2 = layout.box()
            box2.label(text=tr("Jump Settings", lang))
            box2.prop(settings, "jump_height", text=tr("Jump Height", lang))
            box2.prop(settings, "move_distance", text=tr("Jump Distance", lang))
            box2.prop(settings, "jump_crouch_factor", text=tr("Crouch Factor", lang))
            box2.prop(settings, "jump_takeoff_frac", text=tr("Takeoff Duration", lang))
            box2.prop(settings, "jump_hang_frac", text=tr("Hang Time", lang))

            # --- COM / Center ---
            box3 = layout.box()
            box3.label(text=tr("Center of Mass", lang))
            box3.prop(settings, "center_enabled")
            box3.prop(settings, "center_apply_mode", text=tr("Center Apply", lang))
            box3.prop(settings, "center_target_bone")
            box3.prop(settings, "center_preserve_height")
            box3.prop(settings, "center_bob_amount")

            # --- Arm Animation ---
            box4 = layout.box()
            box4.label(text=tr("Arm Animation", lang))
            box4.prop(settings, "use_arm_animation", text=tr("Animate Arms", lang))
            if settings.use_arm_animation:
                box4.prop(settings, "arm_swing_amount", text=tr("Swing Amount", lang))
                box4.prop(settings, "arm_phase_offset", text=tr("Phase Offset", lang))

            # --- IK / Legs (как в WALK) ---
            box_ik = layout.box()
            box_ik.label(text=tr("IK / Legs", lang))
            box_ik.prop(settings, "use_ik", text=tr("Use IK for Legs", lang))
            if settings.use_ik:
                box_ik.prop(settings, "ik_chain_count")
                box_ik.prop(settings, "ik_interpolation_mode", text=tr("IK Interpolation", lang))
                if settings.ik_interpolation_mode == 'CONSTANT_SPEED':
                    box_ik.prop(settings, "ik_constant_speed")
                elif settings.ik_interpolation_mode == 'EXPONENTIAL':
                    box_ik.prop(settings, "ik_exponential_alpha")
                elif settings.ik_interpolation_mode == 'SPRING':
                    box_ik.prop(settings, "ik_spring_stiffness")
                    box_ik.prop(settings, "ik_spring_damping")
                elif settings.ik_interpolation_mode == 'STEP':
                    box_ik.prop(settings, "ik_step_factor")

            # --- Squash and Stretch ---
            box5 = layout.box()
            box5.label(text=tr("Squash and Stretch", lang))
            box5.prop(settings, "jump_squash", text=tr("Squash", lang))
            box5.prop(settings, "jump_stretch", text=tr("Stretch", lang))
            box5.prop(settings, "jump_trail_effect", text=tr("Trail Effect", lang))
            box5.prop(settings, "jump_secondary_motion", text=tr("Secondary Motion", lang))
                
        elif settings.animation_type == 'FULL_BODY_SWING':
            box = layout.box()
            box.label(text=tr("FULL BODY SWING", lang))
            box.operator("pw.generate_walk", text=tr("Generate Animation", lang))

            box2 = layout.box()
            box2.label(text=tr("Swing Settings", lang))
            box2.prop(settings, "turn_speed", text=tr("Swing Speed", lang))
            row = box2.row()
            row.prop(settings, "turn_angle", text=tr("Swing Angle (deg)", lang))
            row = box2.row(align=True)
            op1 = row.operator("pw.set_negative_angle", text="-90")
            if op1:
                op1.angle = -90
            op2 = row.operator("pw.set_negative_angle", text="-180")
            if op2:
                op2.angle = -180
            op3 = row.operator("pw.set_negative_angle", text="-360")
            if op3:
                op3.angle = -360

            # Новые настройки интерполяции
            box_interp = layout.box()
            box_interp.label(text=tr("Turn Interpolation", lang))
            box_interp.prop(settings, "turn_interpolation_mode", text=tr("Interpolation Mode", lang))

            if settings.turn_interpolation_mode == 'PHASE_MODULATED':
                box_interp.prop(settings, "turn_step_influence", text=tr("Step Influence", lang))
                box_interp.prop(settings, "turn_modulation_strength", text=tr("Modulation Strength", lang))
                box_interp.prop(settings, "turn_sensitivity", text=tr("Sensitivity", lang))

            box_interp.prop(settings, "turn_inertia_in", text=tr("Inertia In", lang))
            box_interp.prop(settings, "turn_inertia_out", text=tr("Inertia Out", lang))

            # Настройки шага для поворота
            box_step = layout.box()
            box_step.label(text=tr("Step Settings for Turn", lang))
            box_step.prop(settings, "turn_step_height", text=tr("Step Height", lang))
            box_step.prop(settings, "turn_stride_angle", text=tr("Stride Angle", lang))
            box_step.prop(settings, "turn_floatiness", text=tr("Floatiness", lang))
            box_step.prop(settings, "frequency", text=tr("Frequency", lang))

            # Остальные существующие настройки...
            box3 = layout.box()
            box3.label(text=tr("Spine Chain Settings", lang))
            box3.prop(settings, "spine_root_bone", text=tr("Spine Root Bone", lang))
            box3.prop(settings, "spine_end_bone", text=tr("End Spine Bone", lang))
            box3.prop(settings, "turn_bone_limits")

            box4 = layout.box()
            box4.label(text=tr("Forward / Center (COM)", lang))
            box4.prop(settings, "forward_axis", text=tr("Forward Axis", lang))
            box4.prop(settings, "center_enabled")
            box4.prop(settings, "center_apply_mode", text=tr("Center Apply", lang))
            box4.prop(settings, "center_target_bone")
            box4.prop(settings, "center_preserve_height")
            box4.prop(settings, "center_bob_amount")

            box5 = layout.box()
            box5.label(text=tr("Coordinate System", lang))
            box5.prop(settings, "forward_coordinate_system", text=tr("Coordinate System", lang))

            box6 = layout.box()
            box6.prop(settings, "force_humanoid_override", text=tr("Override Skeleton Type", lang))
            if settings.force_humanoid_override:
                box6.prop(settings, "force_humanoid", text=tr("Force Humanoid", lang))

        else:
            # Fallback: show a compact set for other animation types
            box = layout.box()
            box.label(text=tr(settings.animation_type, settings.ui_language))
            box.operator("pw.generate_walk", text=tr("Generate Animation", lang))

            box2 = layout.box()
            box2.label(text=tr("Leg Selection (overrides)", lang))
            box2.prop(settings, "left_leg_name", text=tr("Left Bone (override)", lang))
            box2.prop(settings, "right_leg_name", text=tr("Right Bone (override)", lang))
            box2.prop(settings, "bone_name_mask")

            box3 = layout.box()
            box3.label(text=tr("Forward / Center (COM)", lang))
            box3.prop(settings, "forward_axis", text=tr("Forward Axis", lang))
            box3.prop(settings, "center_enabled")
            box3.prop(settings, "center_apply_mode", text=tr("Center Apply", lang))
            box3.prop(settings, "center_target_bone")
            box3.prop(settings, "center_preserve_height")
            box3.prop(settings, "center_bob_amount")
            box3.prop(settings, "backward_bias")

        layout.separator()
        layout.prop(settings, "ui_language", text="UI Lang")


# -----------------------
# Mechanical panel (основная)
# -----------------------
class PW_PT_MechPanel(bpy.types.Panel):
    bl_label = "Mechanical"
    bl_idname = "VIEW3D_PT_pw_mech"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Procedural Anim"
    bl_parent_id = "VIEW3D_PT_pw_master"

    @classmethod
    def poll(cls, context):
        s = getattr(context.scene, "pw_settings", None)
        return s is not None and s.unit_category == 'MECHANICAL'

    def draw(self, context):
        layout = self.layout
        s = context.scene.pw_settings

        # Главный селектор анимации для всех механических юнитов
        layout.prop(s, "mech_unit_type", text="Animation Type")

        # Быстрые кнопки генерации (можно оставить или убрать — по желанию)
        row = layout.row(align=True)
        row.scale_y = 1.5
        row.operator("pw.generate_mechanical", text="Generate", icon='PLAY')

        # Общие настройки (всегда видны в Mechanical)
        box = layout.box()
        box.label(text="Common Settings")
        col = box.column(align=True)
        col.prop(s, "frame_start")
        col.prop(s, "frame_end")
        col.prop(s, "frequency")
        col.prop(s, "forward_axis")

        # Показываем только если это колёсный транспорт
        if s.mech_unit_type in {'VEHICLE', 'DRONE'}:
            box = layout.box()
            box.label(text="Vehicle Animation", icon='AUTO')
            box.prop(s, "vehicle_animation_mode", text="Mode")

            # Кнопка генерации
            box.operator("pw.generate_mechanical", text="Generate", icon='PLAY').mode = "VEHICLE"

        elif s.mech_animation_type == 'FLY':
            layout.label(text="→ Fly settings in Simple tab", icon='INFO')


# -----------------------
# Подпанель Vehicle / Wheeled (только когда выбран VEHICLE или DRONE)
# -----------------------
class PW_PT_VehiclePanel(bpy.types.Panel):
    bl_label = "Vehicle / Wheeled"
    bl_idname = "VIEW3D_PT_pw_vehicle"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Procedural Anim"
    bl_parent_id = "VIEW3D_PT_pw_mech"

    @classmethod
    def poll(cls, context):
        s = context.scene.pw_settings
        return (s
                and s.unit_category == 'MECHANICAL'
                and getattr(s, 'mech_unit_type', '') in {'VEHICLE', 'DRONE'})

    def draw(self, context):
        layout = self.layout
        s = context.scene.pw_settings

        # Пресет типа транспорта
        box = layout.box()
        box.label(text="Vehicle Preset", icon='OUTLINER_OB_ARMATURE')
        box.prop(s, "vehicle_type", text="Type")

        # Поиск колёс
        box = layout.box()
        box.label(text="Wheel Detection")
        box.prop(s, "vehicle_wheel_mask", text="Name Mask")
        box.prop(s, "vehicle_num_axles", text="Number of Axles")

        # Управление
        box = layout.box()
        box.label(text="Steering & Speed")
        box.prop(s, "vehicle_steering_mode")
        box.prop(s, "vehicle_steering_max_angle")
        box.prop(s, "vehicle_max_speed")

        # Разгон / торможение
        box = layout.box()
        box.label(text="Acceleration & Braking")
        box.prop(s, "vehicle_accel_time")
        box.prop(s, "vehicle_brake_time")

        # Подвеска и эффекты
        box = layout.box()
        box.label(text="Suspension & Feel")
        box.prop(s, "vehicle_suspension_stiffness")
        box.prop(s, "vehicle_suspension_damping")
        box.prop(s, "vehicle_suspension_travel")
        box.prop(s, "vehicle_noise_freq", text="Road Bump Frequency")
        box.prop(s, "vehicle_roll_strength")
        box.prop(s, "vehicle_pitch_strength")

        # Кнопка генерации (дублирует главную, но удобно)
        layout.separator()
        row = layout.row()
        row.scale_y = 2.0
        row.operator("pw.generate_mechanical", text="Generate Vehicle Animation", icon='AUTO')
# -----------------------
# Simple panel
# -----------------------
class PW_PT_SimplePanel(bpy.types.Panel):
    bl_label = "Simple Units"
    bl_idname = "VIEW3D_PT_pw_simple"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Procedural Anim"
    bl_parent_id = "VIEW3D_PT_pw_master"

    @classmethod
    def poll(cls, context):
        s = getattr(context.scene, "pw_settings", None)
        return s is not None and s.unit_category == 'SIMPLE'

    def draw(self, context):
        layout = self.layout
        s = context.scene.pw_settings

        # Header: quick mode selectors
        row = layout.row(align=True)
        row.operator("pw.generate_simple_idle", text="Idle")
        row.operator("pw.generate_simple_slide", text="Slide")

        row = layout.row(align=True)
        row.operator("pw.generate_simple_turn", text="Turn")
        row.operator("pw.generate_simple_jump", text="Jump")

        row = layout.row(align=True)
        row.operator("pw.generate_simple_move", text="Move")
        row.operator("pw.generate_simple_damage", text="Damage")
        row.operator("pw.generate_simple_dodge", text="Dodge")

        # NEW ANIMATIONS
        row = layout.row(align=True)
        row.operator("pw.generate_simple_surprise", text="Surprise")
        row.operator("pw.generate_simple_panic", text="Panic")

        row = layout.row(align=True)
        row.operator("pw.generate_simple_shoot", text="Shoot")
        row.operator("pw.generate_simple_roll", text="Roll")
        row.operator("pw.generate_simple_fly", text="Fly")

        layout.separator()

        # TARGET SELECTION - NEW SECTION
        box_target = layout.box()
        box_target.label(text="Target Selection")
        box_target.prop(s, "simple_target_type", expand=True)
        if s.simple_target_type == 'BONE' and context.active_object and context.active_object.type == 'ARMATURE':
            box_target.prop_search(s, "simple_target_bone", context.active_object.pose, "bones", text="Bone")
        elif s.simple_target_type == 'BONE':
            box_target.label(text="Select an armature first", icon='ERROR')

        box = layout.box()
        box.label(text="Simple Settings (shared)")
        col = box.column(align=True)
        col.prop(s, "frame_start")
        col.prop(s, "frame_end")
        col.prop(s, "frequency")
        col.prop(s, "single_phase")
        col.prop(s, "center_bob_amount")
        col.prop(s, "allow_scale_changes", text="Allow Scale Changes")

        layout.separator()

        # Idle specific
        box_idle = layout.box()
        box_idle.label(text="Idle (subtle) Settings")
        box_idle.prop(s, "idle_amp", text="Vertical Amp")
        box_idle.prop(s, "idle_freq", text="Wobble Freq")
        box_idle.prop(s, "idle_rot_deg", text="Tilt Deg")
        box_idle.prop(s, "idle_stiffness", text="Spring Stiffness")
        box_idle.prop(s, "idle_damping", text="Spring Damping")
        # Новые настройки шума для idle
        box_noise = box_idle.box()
        box_noise.label(text="Procedural Noise", icon="MOD_NOISE")
        box_noise.prop(s, "use_noise_idle", text="Enable Noise")
        if s.use_noise_idle:
            col = box_noise.column(align=True)
            col.prop(s, "idle_noise_seed", text="Seed")
            col.prop(s, "idle_noise_frequency", text="Frequency")
            col.prop(s, "idle_noise_amp_vertical", text="Vertical Amp")
            col.prop(s, "idle_noise_amp_horizontal", text="Horizontal Amp")
            col.prop(s, "idle_noise_amp_rotation", text="Rotation Amp")
        # Slide / crawl (simple move)
        box_move = layout.box()
        box_move.label(text="Slide / Move Settings")
        box_move.prop(s, "simple_move_distance", text="Distance")
        box_move.prop(s, "move_bob_amp", text="Bob Amp")
        box_move.prop(s, "move_roll_deg", text="Roll Deg")
        box_move.prop(s, "move_stiffness", text="Move Stiffness")
        box_move.prop(s, "move_damping", text="Move Damping")
        box_move.prop(s, "simple_move_accel_frac", text="Accel Fraction")
        box_move.prop(s, "simple_move_decel_frac", text="Decel Fraction")

        # Turn
        box_turn = layout.box()
        box_turn.label(text="Simple Turn Settings")
        box_turn.prop(s, "turn_angle", text="Angle (deg)")
        box_turn.prop(s, "turn_speed", text="Speed")
        box_turn.prop(s, "turn_overshoot_deg", text="Overshoot Deg")
        box_turn.prop(s, "turn_ease", text="Ease Fraction (to overshoot)")

        # Jump
        box_jump = layout.box()
        row = box_jump.row()
        row.prop(s, "jump_physics_enabled", icon="PHYSICS", text="Physics Mode")

        # Common Target
        box_jump.prop(s, "jump_height", text="Target Height")

        if s.jump_physics_enabled:
            # Physics Settings
            col = box_jump.column(align=True)
            col.prop(s, "jump_gravity")
            col.prop(s, "jump_mass")
            col.prop(s, "jump_landing_height")

            row = col.row()
            row.prop(s, "jump_randomness")
            row.prop(s, "jump_tilt_amount")
        else:
            # Simple Settings (Legacy)
            col = box_jump.column(align=True)
            col.prop(s, "jump_takeoff_frac", text="Takeoff Duration")
            col.prop(s, "jump_hang_frac", text="Hang Duration")

        # Deformation (Always visible)
        box_def = box_jump.box()
        box_def.label(text="Deformation", icon="MOD_SIMPLEDEFORM")
        row = box_def.row(align=True)
        row.prop(s, "jump_squash", text="Squash")
        row.prop(s, "jump_stretch", text="Stretch")

        # Damage
        box_dmg = layout.box()
        box_dmg.label(text="Damage / Hit Settings")
        box_dmg.prop(s, "damage_shake_amp", text="Shake Amp")
        box_dmg.prop(s, "damage_recoil_z", text="Recoil Z")
        box_dmg.prop(s, "damage_shake_freq", text="Shake Freq")

        # Dodge
        box_dodge = layout.box()
        box_dodge.label(text="Dodge Settings")
        box_dodge.prop(s, "dodge_lateral_dist", text="Lateral Dist")
        box_dodge.prop(s, "dodge_quick_frac", text="Quick Fraction")
        box_dodge.prop(s, "dodge_side", text="Side (L/R)")

        # NEW ANIMATION SETTINGS
        box_surprise = layout.box()
        box_surprise.label(text="Surprise Settings")
        box_surprise.prop(s, "surprise_amp", text="Jump Height")

        box_panic = layout.box()
        box_panic.label(text="Panic Settings")
        box_panic.prop(s, "panic_speed", text="Vibration Speed")

        box_shoot = layout.box()
        box_shoot.label(text="Shoot Settings")
        box_shoot.prop(s, "shoot_recoil", text="Recoil Distance")

        box_roll = layout.box()
        box_roll.label(text="Roll Settings")
        box_roll.prop(s, "roll_rotations", text="Full Rotations")
        box_roll.prop(s, "dodge_lateral_dist", text="Roll Distance")  # Reuse dodge distance

        # Обновленный блок Fly Settings для новой функции
        box_fly = layout.box()
        box_fly.label(text="Fly Settings", icon="MOD_SOFT")

        col = box_fly.column(align=True)
        col.prop(s, "fly_amp", text="Vertical Amplitude")
        col.prop(s, "fly_lateral", text="Lateral Amplitude")
        col.prop(s, "fly_forward_dist", text="Forward Distance")
        col.prop(s, "fly_octaves", text="Octaves (Noise Detail)")
        col.prop(s, "fly_seed", text="Seed")

        box_fly.separator()

        col = box_fly.column(align=True)
        col.label(text="Rotation Response:", icon="ORIENTATION_LOCAL")
        col.prop(s, "fly_bank_strength", text="Bank Strength")
        col.prop(s, "fly_pitch_strength", text="Pitch Strength")
        col.prop(s, "fly_yaw_strength", text="Yaw Strength")

        box_fly.separator()
        if s.allow_scale_changes:
            col.prop(s, "fly_breath_amp", text="Breathing Amplitude")

        layout.separator()
        layout.label(text="Tip: enable 'Single Phase' for monopods/wheels/slimes.")
        layout.prop(s, "ui_language", text="UI Lang")


# Registration helpers
classes = (
    PW_PT_Panel,
    PW_PT_LivingPanel,
    PW_PT_MechPanel,
    PW_PT_VehiclePanel,
    PW_PT_SimplePanel,
)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)

def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
