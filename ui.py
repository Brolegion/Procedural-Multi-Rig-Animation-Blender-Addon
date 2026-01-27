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
    PW_OT_GenerateSimpleDeath,  # Добавьте эту строку
    PW_OT_GenerateSimpleStun  # И эту строку
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

            box.operator("pw.generate_animation", text=tr("Generate Animation", lang))

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

        # В методе draw класса PW_PT_LivingPanel добавьте блок для IDLE:
        elif settings.animation_type == 'IDLE':
            box = layout.box()
            box.label(text=tr("IDLE ANIMATION", lang))
            box.operator("pw.generate_animation", text=tr("Generate Idle Animation", lang))

            # Основные настройки
            box2 = layout.box()
            box2.label(text=tr("Idle Settings", lang))
            box2.prop(settings, "frame_start", text="Start Frame")
            box2.prop(settings, "frame_end", text="End Frame")
            box2.prop(settings, "idle_amp", text="Breathing Amplitude")
            box2.prop(settings, "idle_freq", text="Animation Frequency")
            box2.prop(settings, "idle_rot_deg", text="Spine Rotation (deg)")
            box2.prop(settings, "noise_amount", text="Noise Amount")

            # Настройки позвоночника
            box3 = layout.box()
            box3.label(text=tr("Spine Chain Settings", lang))
            box3.prop(settings, "spine_root_bone", text="Spine Root Bone")
            box3.prop(settings, "spine_end_bone", text="End Spine Bone")

            # Дополнительные опции
            box4 = layout.box()
            box4.label(text=tr("Additional Options", lang))
            box4.prop(settings, "use_arm_animation", text="Animate Arms")
            box4.prop(settings, "use_ik", text="Use IK (Pin Feet)")

        elif settings.animation_type == 'TURN':
            box = layout.box()
            box.label(text=tr("TURN", lang))
            box.operator("pw.generate_animation", text=tr("Generate Animation", lang))

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
            box.operator("pw.generate_animation", text=tr("Generate Jump Animation", lang))

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
            box.operator("pw.generate_animation", text=tr("Generate Animation", lang))

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

        # ui.py
        # ... (существующий код)

        # В классе PW_PT_LivingPanel, в методе draw, в блоке выбора animation_type нужно добавить 'PANIC':
        # ... (в начале метода draw, где есть row.prop(settings, "animation_type", ...)
        # Добавьте PANIC в список доступных animation_type (это должно быть уже в properties.py)

        # Затем в условных блоках (после блоков для WALK, IDLE, TURN, JUMP, FULL_BODY_SWING, DODGE, DAMAGE)
        # добавьте блок для PANIC:

        elif settings.animation_type == 'PANIC':
            box = layout.box()
            box.label(text=tr("PANIC ANIMATION", lang))
            box.operator("pw.generate_animation", text=tr("Generate Panic Animation", lang), icon='GHOST_ENABLED')

            # Основные настройки паники
            box2 = layout.box()
            box2.label(text=tr("Panic Settings", lang))
            col = box2.column(align=True)
            col.prop(settings, "panic_duration", text=tr("Panic Duration", lang))
            col.prop(settings, "panic_recovery_duration", text=tr("Recovery Duration", lang))
            col.prop(settings, "panic_intensity", text=tr("Intensity", lang))
            col.prop(settings, "panic_variation", text=tr("Panic Type", lang))

            # Фазовые настройки
            box3 = layout.box()
            box3.label(text=tr("Phase Settings", lang))
            box3.prop(settings, "panic_shock_response", text=tr("Shock Response Speed", lang))
            box3.prop(settings, "panic_tremble_frequency", text=tr("Tremble Frequency", lang))
            box3.prop(settings, "panic_breath_speed", text=tr("Breathing Speed", lang))

            # Выбор костей для паники
            box4 = layout.box()
            box4.label(text=tr("Bone Selection", lang), icon='BONE_DATA')

            # Кнопка автозаполнения
            row = box4.row()
            row.operator("pw.panic_auto_fill", icon='AUTO', text=tr("Auto-Fill Bones", lang))

            # Primary Bones (спина, голова)
            col = box4.column(align=True)
            row = col.row()
            row.prop(settings, "use_panic_primary", text=tr("Primary Bones (Spine/Head)", lang), toggle=True)

            if settings.use_panic_primary:
                # Кнопки управления для Primary
                row = col.row(align=True)
                op_add = row.operator("pw.bone_list_add", text="", icon='ADD')
                op_add.target_list = 'panic_primary'
                op_rem = row.operator("pw.bone_list_remove", text="", icon='REMOVE')
                op_rem.target_list = 'panic_primary'

                # Список Primary костей
                if settings.panic_bones_primary:
                    box_list = col.box()
                    for i, item in enumerate(settings.panic_bones_primary):
                        row = box_list.row(align=True)
                        # Маркер активного
                        if i == settings.panic_active_primary:
                            row.label(text="•", icon='DOT')
                        else:
                            row.label(text="", icon='BLANK1')

                        # Поле ввода кости
                        row.prop(item, "bone_name", text="")

                        # Вес
                        if item.bone_name:
                            sub = row.row(align=True)
                            sub.scale_x = 0.7
                            sub.prop(item, "weight", text="W", slider=True)

            # Secondary Bones (руки, ноги для дрожи)
            col.separator()
            row = col.row()
            row.prop(settings, "use_panic_secondary", text=tr("Secondary Bones (Arms/Legs)", lang), toggle=True)

            if settings.use_panic_secondary:
                # Кнопки управления для Secondary
                row = col.row(align=True)
                op_add = row.operator("pw.bone_list_add", text="", icon='ADD')
                op_add.target_list = 'panic_secondary'
                op_rem = row.operator("pw.bone_list_remove", text="", icon='REMOVE')
                op_rem.target_list = 'panic_secondary'

                # Список Secondary костей
                if settings.panic_bones_secondary:
                    box_list = col.box()
                    for i, item in enumerate(settings.panic_bones_secondary):
                        row = box_list.row(align=True)
                        if i == settings.panic_active_secondary:
                            row.label(text="•", icon='DOT')
                        else:
                            row.label(text="", icon='BLANK1')

                        row.prop(item, "bone_name", text="")

                        if item.bone_name:
                            sub = row.row(align=True)
                            sub.scale_x = 0.7
                            sub.prop(item, "weight", text="W", slider=True)

            # Расширенные настройки
            box5 = layout.box()
            box5.prop(settings, "show_advanced_panic",
                      text=tr("Advanced Settings", lang),
                      icon='TRIA_DOWN' if settings.show_advanced_panic else 'TRIA_RIGHT',
                      emboss=False)

            if settings.show_advanced_panic:
                # Настройки дрожи
                box_tremble = box5.box()
                box_tremble.label(text=tr("Tremble Details", lang))
                box_tremble.prop(settings, "panic_seed", text=tr("Noise Seed", lang))
                box_tremble.prop(settings, "panic_tremble_amplitude", text=tr("Tremble Amplitude", lang))
                box_tremble.prop(settings, "panic_head_shake", text=tr("Head Shake Intensity", lang))

                # Настройки дыхания
                box_breath = box5.box()
                box_breath.label(text=tr("Breathing Pattern", lang))
                box_breath.prop(settings, "panic_breath_amplitude", text=tr("Breath Amplitude", lang))
                box_breath.prop(settings, "panic_breath_variation", text=tr("Breath Variation", lang))

                # Физические параметры
                box_physics = box5.box()
                box_physics.label(text=tr("Physics Parameters", lang))
                box_physics.prop(settings, "panic_stiffness", text=tr("Body Stiffness", lang))
                box_physics.prop(settings, "panic_damping", text=tr("Body Damping", lang))

                # Дополнительные эффекты
                box_effects = box5.box()
                box_effects.label(text=tr("Additional Effects", lang))
                box_effects.prop(settings, "panic_eyes_wide", text=tr("Eyes Wide Open", lang))
                box_effects.prop(settings, "panic_micro_movements", text=tr("Micro Movements", lang))

            # Общие настройки (COM/центр)
            box6 = layout.box()
            box6.label(text=tr("Center of Mass", lang))
            box6.prop(settings, "center_enabled")
            if settings.center_enabled:
                box6.prop(settings, "center_apply_mode", text=tr("Center Apply", lang))
                box6.prop(settings, "center_target_bone")
                box6.prop(settings, "center_preserve_height")

            # Настройки IK (фиксация ног при дрожи)
            box7 = layout.box()
            box7.label(text=tr("Leg Stabilization", lang))
            box7.prop(settings, "use_ik", text=tr("Use IK (Sticky Feet)", lang))
            if settings.use_ik:
                row = box7.row(align=True)
                row.prop(settings, "left_leg_name", text=tr("Left Leg", lang))
                row.prop(settings, "right_leg_name", text=tr("Right Leg", lang))
                box7.prop(settings, "panic_leg_stiffness", text=tr("Leg Stiffness", lang))

        # ... (продолжение существующего кода)

        # В методе draw класса PW_PT_LivingPanel добавьте блок для DODGE:
        elif settings.animation_type == 'DODGE':
                    box = layout.box()
                    box.label(text="DODGE SETTINGS", icon='ACTION')
                    box.operator("pw.generate_animation", text="Generate Dodge")
                    
                    # --- Основные параметры ---
                    col = box.column(align=True)
                    col.prop(settings, "dodge_type")
                    col.prop(settings, "threat_direction")
                    row = col.row(align=True)
                    row.prop(settings, "dodge_distance", text="Distance")
                    row.prop(settings, "dodge_crouch", text="Crouch")
                    
                    # --- Тайминг и Физика ---
                    box_phys = box.box()
                    box_phys.label(text="Timing & Physics", icon='PHYSICS')
                    row = box_phys.row(align=True)
                    row.prop(settings, "dodge_duration", text="Action")
                    row.prop(settings, "dodge_recovery", text="Recovery") # Исправлено на dodge_recovery
                    
                    col = box_phys.column(align=True)
                    col.prop(settings, "dodge_anticipation", text="Anticipation (0-1)")
                    col.prop(settings, "dodge_spring_freq", text="Spring Speed")
                    col.prop(settings, "dodge_spring_damp", text="Spring Damping")
                    col.prop(settings, "dodge_overshoot", text="Overshoot/Elasticity")

                    # --- Центр Масс (COM) ---
                    box_com = box.box()
                    box_com.label(text="Center of Mass (Movement)", icon='CENTER_ONLY')
                    box_com.prop(settings, "center_enabled", text="Enable Body Displacement")
                    if settings.center_enabled:
                        box_com.prop(settings, "center_apply_mode", text="Apply To")
                        box_com.prop(settings, "center_target_bone", text="Root/Pelvis Bone")

                    # --- Ноги (Sticky Feet / IK) ---
                    box_ik = box.box()
                    box_ik.label(text="Sticky Feet (IK)", icon='CONSTRAINT_BONE')
                    box_ik.prop(settings, "dodge_sticky_feet", text="Use Sticky Feet")
                    if settings.dodge_sticky_feet:
                        row = box_ik.row(align=True)
                        row.prop(settings, "left_leg_name", text="L Leg")
                        row.prop(settings, "right_leg_name", text="R Leg")
                        box_ik.prop(settings, "use_ik", text="Enable IK Constraints")

                    # --- Вторичные движения и Шум ---
                    box_noise = box.box()
                    box_noise.label(text="Secondary & Noise", icon='COLORSET_03_VEC')
                    box_noise.prop(settings, "dodge_counter_rot", text="Enable Body Lean/Tilt")
                    box_noise.prop(settings, "dodge_tilt", text="Tilt Strength")
                    row = box_noise.row(align=True)
                    row.prop(settings, "dodge_noise_amount", text="Noise")
                    row.prop(settings, "dodge_noise_seed", text="Seed")

                    # === ВЫБОР КОСТЕЙ ===
                    box_bones = box.box()
                    row = box_bones.row()
                    row.label(text="Bone Selection", icon='BONE_DATA')
                    row.operator("pw.dodge_auto_fill", text="Auto-Fill", icon='MODIFIER')
                    
                    # (Далее ваши существующие блоки для Primary и Secondary костей...)
                    
# ... (внутри секции DODGE в draw_living_panel или аналогичной)

                    # === ВЫБОР КОСТЕЙ ===
                    box_bones = box.box()
                    row = box_bones.row()
                    row.label(text="Bone Selection", icon='BONE_DATA')
                    row.operator("pw.dodge_auto_fill", text="Auto-Fill", icon='MODIFIER')
                    
                    # --- Primary Bones ---
                    col = box_bones.column(align=True)
                    row = col.row()
                    row.prop(settings, "use_dodge_primary", text="Primary (Spine/Root)", toggle=True)
                    
                    if settings.use_dodge_primary:
                        # Кнопки
                        row = col.row(align=True)
                        op_add = row.operator("pw.bone_list_add", text="Add Selected", icon='ADD')
                        op_add.target_list = 'dodge_primary'
                        
                        op_rem = row.operator("pw.bone_list_remove", text="Remove", icon='REMOVE')
                        op_rem.target_list = 'dodge_primary'
                        
                        # Список
                        box_list = col.box()
                        for i, item in enumerate(settings.dodge_bones_primary):
                            row = box_list.row(align=True)
                            # Маркер активного
                            if i == settings.dodge_active_primary:
                                row.label(text="", icon='TRIA_RIGHT')
                            
                            row.prop(item, "bone_name", text="")
                            row.prop(item, "weight", text="W", slider=True)
                            
                            # Кнопки перемещения (можно вынести отдельно, но удобно тут)
                            sub = row.row(align=True)
                            op_up = sub.operator("pw.bone_list_move", text="", icon='TRIA_UP')
                            op_up.target_list = 'dodge_primary'
                            op_up.direction = 'UP'
                            
                            op_dn = sub.operator("pw.bone_list_move", text="", icon='TRIA_DOWN')
                            op_dn.target_list = 'dodge_primary'
                            op_dn.direction = 'DOWN'

                    # --- Secondary Bones ---
                    col.separator()
                    row = col.row()
                    row.prop(settings, "use_dodge_secondary", text="Secondary (Arms/Head)", toggle=True)
                    
                    if settings.use_dodge_secondary:
                        # Кнопки
                        row = col.row(align=True)
                        op_add = row.operator("pw.bone_list_add", text="Add Selected", icon='ADD')
                        op_add.target_list = 'dodge_secondary'
                        
                        op_rem = row.operator("pw.bone_list_remove", text="Remove", icon='REMOVE')
                        op_rem.target_list = 'dodge_secondary'
                        
                        # Список
                        box_list = col.box()
                        for i, item in enumerate(settings.dodge_bones_secondary):
                            row = box_list.row(align=True)
                            if i == settings.dodge_active_secondary:
                                row.label(text="", icon='TRIA_RIGHT')
                            
                            row.prop(item, "bone_name", text="")
                            row.prop(item, "weight", text="W", slider=True)
        # ui.py - в функции draw_damage_settings
        elif settings.animation_type == 'DAMAGE':
            box = layout.box()
            box.label(text=tr("DAMAGE ANIMATION", lang))
            box.operator("pw.generate_animation", text=tr("Generate Damage Animation", lang), icon='MOD_PHYSICS')

            # Быстрые пресеты урона
            box2 = layout.box()
            box2.label(text=tr("Quick Presets", lang))
            row = box2.row(align=True)

            # Кнопка Light Hit
            op1 = row.operator("pw.apply_preset", text=tr("Light Hit", lang))
            op1.preset_type = 'DAMAGE_LIGHT'

            # Кнопка Medium Hit
            op2 = row.operator("pw.apply_preset", text=tr("Medium Hit", lang))
            op2.preset_type = 'DAMAGE_MEDIUM'

            # Кнопка Heavy Hit
            op3 = row.operator("pw.apply_preset", text=tr("Heavy Hit", lang))
            op3.preset_type = 'DAMAGE_HEAVY'

            # Кнопка Knockdown
            row = box2.row(align=True)
            op4 = row.operator("pw.apply_preset", text=tr("Knockdown", lang))
            op4.preset_type = 'DAMAGE_KNOCKDOWN'

            # Основные настройки
            box3 = layout.box()
            box3.label(text=tr("Basic Settings", lang))
            box3.prop(settings, "damage_strength", text=tr("Impact Strength", lang))
            box3.prop(settings, "damage_direction", text=tr("Impact Direction", lang))

            row = box3.row()
            row.prop(settings, "damage_duration", text=tr("Duration (frames)", lang))
            row.prop(settings, "damage_frame_start", text=tr("Start Frame", lang))

            # Тип реакции
            box4 = layout.box()
            box4.label(text=tr("Reaction Type", lang))
            box4.prop(settings, "damage_reaction_type", text=tr("Reaction Type", lang), expand=False)

            # Форма атаки
            row = box4.row()
            row.prop(settings, "damage_attack_shape", text=tr("Attack Shape", lang))

            # ===== НОВЫЙ РАЗДЕЛ ВЫБОРА КОСТЕЙ =====
            box_bones = layout.box()
            box_bones.label(text=tr("Bone Selection", lang), icon='BONE_DATA')
            
            # Кнопка автозаполнения
            row = box_bones.row()
            row.operator("pw.dodge_auto_fill", icon='AUTO', text=tr("Auto-Fill from Selection", lang))
            
            # Две колонки для костей
            split = box_bones.split(factor=0.5)
            
            # Левая колонка - Primary Bones
            col = split.column()
            sub_box = col.box()
            row = sub_box.row()
            row.prop(settings, "use_primary_bones", text=tr("Primary Bones", lang), toggle=True)
            row.label(text=f"({len(settings.damage_bones_primary)})")
            
            if settings.use_primary_bones:
                # Кнопки управления для Primary
                row = sub_box.row(align=True)
                row.operator("pw.bone_list_add", text="", icon='ADD').target_list = 'damage_primary'
                row.operator("pw.bone_list_remove", text="", icon='REMOVE').target_list = 'damage_primary'
                
                if settings.damage_bones_primary:
                    row.separator()
                    row.operator("pw.bone_list_move", text="", icon='TRIA_UP').target_list = 'damage_primary'
                    row.operator("pw.bone_list_move", text="", icon='TRIA_DOWN').target_list = 'damage_primary'
                
                # Список Primary костей
                for i, item in enumerate(settings.damage_bones_primary):
                    row = sub_box.row(align=True)
                    
                    # Активный элемент
                    if i == settings.damage_active_primary:
                        row.label(text="•", icon='DOT')
                    else:
                        row.label(text="", icon='BLANK1')
                    
                    # Поле ввода кости
                    row.prop(item, "bone_name", text="")
                    
                    # Вес и жесткость
                    if item.bone_name:
                        sub = row.row(align=True)
                        sub.scale_x = 0.7
                        sub.prop(item, "weight", text="W", slider=True)
            
            # Правая колонка - Secondary Bones
            col = split.column()
            sub_box = col.box()
            row = sub_box.row()
            row.prop(settings, "use_secondary_bones", text=tr("Secondary Bones", lang), toggle=True)
            row.label(text=f"({len(settings.damage_bones_secondary)})")
            
            if settings.use_secondary_bones:
                # Кнопки управления для Secondary
                row = sub_box.row(align=True)
                row.operator("pw.bone_list_add", text="", icon='ADD').target_list = 'damage_secondary'
                row.operator("pw.bone_list_remove", text="", icon='REMOVE').target_list = 'damage_secondary'
                
                if settings.damage_bones_secondary:
                    row.separator()
                    row.operator("pw.bone_list_move", text="", icon='TRIA_UP').target_list = 'damage_secondary'
                    row.operator("pw.bone_list_move", text="", icon='TRIA_DOWN').target_list = 'damage_secondary'
                
                # Список Secondary костей
                for i, item in enumerate(settings.damage_bones_secondary):
                    row = sub_box.row(align=True)
                    
                    # Активный элемент
                    if i == settings.damage_active_secondary:
                        row.label(text="•", icon='DOT')
                    else:
                        row.label(text="", icon='BLANK1')
                    
                    # Поле ввода кости
                    row.prop(item, "bone_name", text="")
                    
                    # Вес
                    if item.bone_name:
                        sub = row.row(align=True)
                        sub.scale_x = 0.7
                        sub.prop(item, "weight", text="W", slider=True)
            
            # Настройки цепочки распространения (оставляем из старого UI)
            box_bones.prop(settings, "damage_propagation_chain", text=tr("Propagation Chain", lang))
            
            # Подсказка
            box_bones.label(text=tr("Tip: Select bones in pose mode, then click Auto-Fill", lang), icon='INFO')

            # Расширенные настройки (остальное без изменений)
            box5 = layout.box()
            box5.prop(settings, "show_advanced_damage",
                      text=tr("Advanced Settings", lang),
                      icon='TRIA_DOWN' if settings.show_advanced_damage else 'TRIA_RIGHT',
                      emboss=False)
                      
            if settings.show_advanced_damage:
                # Слои реакции
                box_layers = box5.box()
                box_layers.label(text=tr("Reaction Layers", lang), icon='NODETREE')

                # Impact Layer
                impact_col = box_layers.column(align=True)
                impact_col.prop(settings, "damage_impact_weight", text=tr("Impact Weight", lang))

                # Stagger Layer
                stagger_row = box_layers.row()
                stagger_row.prop(settings, "damage_enable_stagger", text=tr("Enable Stagger", lang), toggle=1)
                if settings.damage_enable_stagger:
                    stagger_col = box_layers.column(align=True)
                    stagger_col.prop(settings, "damage_stagger_weight", text=tr("Stagger Weight", lang))
                    stagger_col.prop(settings, "damage_stagger_pattern", text=tr("Stagger Pattern", lang))

                # Recovery Layer
                recovery_row = box_layers.row()
                recovery_row.prop(settings, "damage_enable_recovery", text=tr("Enable Recovery", lang), toggle=1)
                if settings.damage_enable_recovery:
                    recovery_col = box_layers.column(align=True)
                    recovery_col.prop(settings, "damage_recovery_weight", text=tr("Recovery Weight", lang))
                    recovery_col.prop(settings, "damage_recovery_type", text=tr("Recovery Type", lang))

                # Параметры физики
                box_physics = box5.box()
                box_physics.label(text=tr("Physics Parameters", lang), icon='PHYSICS')

                physics_col = box_physics.column(align=True)
                physics_col.prop(settings, "damage_stiffness", text=tr("Body Stiffness", lang))
                physics_col.prop(settings, "damage_damping", text=tr("Body Damping", lang))
                physics_col.prop(settings, "damage_pelvis_factor", text=tr("Pelvis Influence", lang))

                # Распределение массы
                mass_box = box_physics.box()
                mass_box.prop(settings, "show_mass_distribution",
                              text=tr("Mass Distribution", lang),
                              icon='TRIA_DOWN' if getattr(settings, "show_mass_distribution", False) else 'TRIA_RIGHT',
                              emboss=False)

                if getattr(settings, "show_mass_distribution", False):
                    mass_col = mass_box.column(align=True)
                    mass_col.prop(settings, "damage_spine_mass", text=tr("Spine Mass", lang))
                    mass_col.prop(settings, "damage_head_mass", text=tr("Head Mass", lang))
                    mass_col.prop(settings, "damage_arm_mass", text=tr("Arm Mass", lang))
                    mass_col.prop(settings, "damage_leg_mass", text=tr("Leg Mass", lang))

                # Затронутые зоны (оставляем для совместимости, но теперь они влияют на классификацию костей)
                box_zones = box5.box()
                box_zones.label(text=tr("Affected Zones", lang), icon='BONE_DATA')

                row = box_zones.row()
                row.prop(settings, "damage_affect_spine", text=tr("Spine", lang), toggle=1)
                row.prop(settings, "damage_affect_head", text=tr("Head", lang), toggle=1)

                row = box_zones.row()
                row.prop(settings, "damage_affect_arms", text=tr("Arms", lang), toggle=1)
                row.prop(settings, "damage_affect_legs", text=tr("Legs", lang), toggle=1)

                # Настройки восстановления
                box_recovery = box5.box()
                box_recovery.label(text=tr("Recovery Settings", lang), icon='ARMATURE_DATA')

                recovery_box_row = box_recovery.row()
                recovery_box_row.prop(settings, "damage_return_to_pose", text=tr("Return to Pose", lang), toggle=1)

                if settings.damage_return_to_pose:
                    recovery_col = box_recovery.column(align=True)
                    recovery_col.prop(settings, "damage_return_speed", text=tr("Return Speed", lang))

                # Дополнительные эффекты
                box_effects = box5.box()
                box_effects.label(text=tr("Additional Effects", lang), icon='WAVES')
                box_effects.prop(settings, "damage_shake", text=tr("Add Shake", lang), toggle=1)

                box_advanced = box5.box()
                box_advanced.label(text=tr("Advanced Physics", lang), icon='PHYSICS')
                box_advanced.prop(settings, "damage_wave_speed", text=tr("Wave Speed", lang))
                box_advanced.prop(settings, "damage_return_frames", text=tr("Return Frames", lang))
                box_advanced.prop(settings, "damage_micro_jitter", text=tr("Micro Jitter", lang))

            # Координатная система
            box6 = layout.box()
            box6.label(text=tr("Coordinate System", lang))
            box6.prop(settings, "forward_coordinate_system", text=tr("Coordinate System", lang))

            # Опционально: отладочная информация
            box7 = layout.box()
            box7.prop(settings, "debug_pw", text=tr("Debug Mode", lang))
            if settings.debug_pw:
                box7.prop(settings, "show_damage_debug", text=tr("Show Damage Debug", lang))

        # В классе PW_PT_LivingPanel, в методе draw, добавьте блок для 'SNEAK':
        elif settings.animation_type == 'SNEAK':
            box = layout.box()
            box.label(text=tr("SNEAK / CROUCH WALK", lang), icon='GHOST_ENABLED')

            # Preset & Action
            row = box.row(align=True)
            row.prop(settings, "creature_preset", text=tr("Preset", lang))
            row.operator("pw.apply_preset", text="", icon='CHECKMARK')
            box.operator("pw.generate_animation", text=tr("Generate Sneak Animation", lang), icon='ANIM_DATA')

            # 1. Основные параметры (Timing & Body)
            box2 = layout.box()
            box2.label(text=tr("Body & Timing", lang), icon='POSE_HLT')
            col = box2.column(align=True)
            col.prop(settings, "frame_start")
            col.prop(settings, "frame_end")
            col.prop(settings, "stride_angle")
            col.prop(settings, "frequency", text=tr("Sneak Speed", lang))
            col.prop(settings, "sneak_depth", text=tr("Crouch Depth", lang))
            col.prop(settings, "sneak_hesitation", text=tr("Cautious Step (Hesitation)", lang))

            # 2. Позвоночник и Голова
            box3 = layout.box()
            box3.label(text=tr("Upper Body & Stealth", lang), icon='HIDE_ON')
            box3.prop(settings, "sneak_spine_arch", text=tr("Spine Lean", lang))
            box3.prop(settings, "sneak_head_stabilize", text=tr("Head Focus", lang))
            box3.prop(settings, "sneak_noise_intensity", text=tr("Body Shiver/Tension", lang))

            col_sp = box3.column(align=True)
            col_sp.prop(settings, "spine_root_bone", text=tr("Root", lang))
            col_sp.prop(settings, "spine_end_bone", text=tr("End", lang))

            # 3. Ноги и Шаг
            box4 = layout.box()
            box4.label(text=tr("Legs & Footwork", lang), icon='CURVE_PATH')
            row_legs = box4.row(align=True)
            row_legs.prop(settings, "left_leg_name", text="")
            row_legs.prop(settings, "right_leg_name", text="")

            col_step = box4.column(align=True)
            col_step.prop(settings, "sneak_width_mult", text=tr("Stance Width", lang))
            col_step.prop(settings, "sneak_step_height_ratio", text=tr("Step Lift Mult", lang))
            col_step.prop(settings, "bone_name_mask")

            # 4. Центр Масс (COM)
            box5 = layout.box()
            box5.label(text=tr("Center of Mass", lang), icon='CENTER_ONLY')
            box5.prop(settings, "center_enabled")
            if settings.center_enabled:
                col_com = box5.column(align=True)
                col_com.prop(settings, "center_apply_mode", text="")
                col_com.prop(settings, "center_target_bone", text="Target")
                col_com.prop(settings, "center_bob_amount", text="Vertical Bob")
                col_com.prop(settings, "center_preserve_height")

            # 5. Стабилизация (IK)
            box6 = layout.box()
            box6.label(text=tr("Stabilization (IK)", lang), icon='CONSTRAINT_BONE')
            box6.prop(settings, "use_ik", text=tr("Use Sticky Feet", lang))
            if settings.use_ik:
                box6.prop(settings, "ik_chain_count")
                box6.prop(settings, "ik_interpolation_mode", text="Interpolation")

        # В методе draw класса PW_PT_LivingPanel, добавьте:

        elif settings.animation_type == 'STUN':
            box = layout.box()
            box.label(text=tr("STUN / DIZZY ANIMATION", lang))
            box.operator("pw.generate_animation", text=tr("Generate Stun Animation", lang), icon='MOD_PHYSICS')

            # Основные настройки - ТОЛЬКО те, что есть в функции
            box2 = layout.box()
            box2.label(text=tr("Stun Settings", lang))
            col = box2.column(align=True)
            col.prop(settings, "stun_duration", text=tr("Stun Duration (s)", lang))
            col.prop(settings, "stun_severity", text=tr("Wobble Amount", lang))

            # Информация о фиксированных фазах (из кода функции)
            box_info = box2.box()
            box_info.label(text="Fixed phases (from code):", icon='INFO')
            box_info.label(text="• Impact: 0.3s (hardcoded)")
            box_info.label(text="• Recovery: 0.8s (hardcoded)")

            # Общие настройки кадров
            box3 = layout.box()
            box3.label(text=tr("Frame Settings", lang))
            box3.prop(settings, "frame_start", text="Start Frame")
            # frame_end будет вычислен автоматически в функции

            # Настройки ног (используются в функции для IK)
            box4 = layout.box()
            box4.label(text=tr("Leg Settings", lang))
            box4.prop(settings, "use_ik", text=tr("Use IK (Keep Feet Planted)", lang))
            if settings.use_ik:
                row = box4.row(align=True)
                row.prop(settings, "left_leg_name", text=tr("Left Leg", lang))
                row.prop(settings, "right_leg_name", text=tr("Right Leg", lang))

            # Центр масс (COM) - используется в функции через apply_center_motion
            box5 = layout.box()
            box5.label(text=tr("Center of Mass", lang))
            box5.prop(settings, "center_enabled", text=tr("Enable Body Movement", lang))
            if settings.center_enabled:
                box5.prop(settings, "center_apply_mode", text=tr("Apply To", lang))
                box5.prop(settings, "center_target_bone")
                box5.prop(settings, "center_preserve_height")
                box5.prop(settings, "center_bob_amount", text=tr("Bob Amount", lang))

            # Подсказка
            layout.separator()
            layout.label(text="Note: Uses existing stun_animation.py function", icon='INFO')

        elif settings.animation_type == 'DEATH':
            box = layout.box()
            box.label(text=tr("DEATH ANIMATION", lang))
            box.operator("pw.generate_animation", text=tr("Generate Death Animation", lang), icon='MOD_PHYSICS')

            # Основные настройки смерти
            box2 = layout.box()
            box2.label(text=tr("Death Settings", lang))
            box2.prop(settings, "death_type", text=tr("Death Style", lang))
            box2.prop(settings, "death_speed", text=tr("Death Speed", lang))
            box2.prop(settings, "death_agony_duration", text=tr("Agony Frames", lang))

            # Настройки падения (используются в смерти)
            box3 = layout.box()
            box3.label(text=tr("Fall Settings (for Death)", lang))
            box3.prop(settings, "fall_height", text=tr("Drop Height", lang))

            # Настройки ног (для IK и позиционирования)
            box4 = layout.box()
            box4.label(text=tr("Leg Settings", lang))
            box4.prop(settings, "left_leg_name", text=tr("Left Bone", lang))
            box4.prop(settings, "right_leg_name", text=tr("Right Bone", lang))
            box4.prop(settings, "bone_name_mask")

            # Центр масс (важно для падения)
            box5 = layout.box()
            box5.label(text=tr("Center of Mass", lang))
            box5.prop(settings, "center_enabled")
            box5.prop(settings, "center_apply_mode", text=tr("Center Apply", lang))
            box5.prop(settings, "center_target_bone", text=tr("Pelvis/Root Bone", lang))

            # IK настройки (для прилипания ног)
            box6 = layout.box()
            box6.label(text=tr("IK Settings", lang))
            box6.prop(settings, "use_ik", text=tr("Use IK for Legs", lang))
            if settings.use_ik:
                box6.prop(settings, "ik_interpolation_mode", text=tr("IK Interpolation", lang))

        # В классе PW_PT_LivingPanel в методе draw добавьте блок для RAGE:
        elif settings.animation_type == 'RAGE':
            box = layout.box()
            box.label(text="RAGE ANIMATION", icon='GHOST_ENABLED')
            box.operator("pw.generate_animation", text="Generate Rage Animation", icon='FORCE_FORCE')

            # Быстрые пресеты
            box2 = layout.box()
            box2.label(text="Quick Presets")
            row = box2.row(align=True)

            op1 = row.operator("pw.apply_preset", text="Berserk Roar")
            op1.preset_type = 'RAGE_BERSERK'

            op2 = row.operator("pw.apply_preset", text="Stalker Intimidate")
            op2.preset_type = 'RAGE_STALKER'

            # Основные настройки
            box3 = layout.box()
            box3.label(text="Rage Settings")
            col = box3.column(align=True)
            col.prop(settings, "rage_profile", text="Rage Profile")
            col.prop(settings, "rage_intensity", text="Intensity")

            # Фазовые настройки
            box4 = layout.box()
            box4.label(text="Phase Settings")
            box4.prop(settings, "rage_use_phases", text="Use Phases")

            if settings.rage_use_phases:
                col = box4.column(align=True)
                col.prop(settings, "rage_intro_frames", text="Intro Frames")
                col.prop(settings, "rage_loop_frames", text="Loop Frames")
                col.prop(settings, "rage_end_frames", text="End Frames")

            # Выбор костей
            box5 = layout.box()
            box5.label(text="Bone Selection", icon='BONE_DATA')

            # Кнопка автозаполнения
            row = box5.row()
            row.operator("pw.rage_auto_fill", icon='AUTO', text="Auto-Fill Bones")

            # Primary Bones (спина)
            col = box5.column(align=True)
            row = col.row()
            row.prop(settings, "use_rage_primary", text="Primary Bones (Spine)", toggle=True)

            if settings.use_rage_primary:
                # Кнопки управления для Primary
                row = col.row(align=True)
                op_add = row.operator("pw.bone_list_add", text="", icon='ADD')
                op_add.target_list = 'rage_primary'
                op_rem = row.operator("pw.bone_list_remove", text="", icon='REMOVE')
                op_rem.target_list = 'rage_primary'

                # Список Primary костей
                if settings.rage_bones_primary:
                    box_list = col.box()
                    for i, item in enumerate(settings.rage_bones_primary):
                        row = box_list.row(align=True)
                        if i == settings.rage_active_primary:
                            row.label(text="•", icon='DOT')
                        else:
                            row.label(text="", icon='BLANK1')

                        row.prop(item, "bone_name", text="")

                        if item.bone_name:
                            sub = row.row(align=True)
                            sub.scale_x = 0.7
                            sub.prop(item, "weight", text="W", slider=True)

            # Secondary Bones (голова, руки)
            col.separator()
            row = col.row()
            row.prop(settings, "use_rage_secondary", text="Secondary Bones (Head/Arms)", toggle=True)

            if settings.use_rage_secondary:
                # Кнопки управления для Secondary
                row = col.row(align=True)
                op_add = row.operator("pw.bone_list_add", text="", icon='ADD')
                op_add.target_list = 'rage_secondary'
                op_rem = row.operator("pw.bone_list_remove", text="", icon='REMOVE')
                op_rem.target_list = 'rage_secondary'

                # Список Secondary костей
                if settings.rage_bones_secondary:
                    box_list = col.box()
                    for i, item in enumerate(settings.rage_bones_secondary):
                        row = box_list.row(align=True)
                        if i == settings.rage_active_secondary:
                            row.label(text="•", icon='DOT')
                        else:
                            row.label(text="", icon='BLANK1')

                        row.prop(item, "bone_name", text="")

                        if item.bone_name:
                            sub = row.row(align=True)
                            sub.scale_x = 0.7
                            sub.prop(item, "weight", text="W", slider=True)

            # Центр масс
            box6 = layout.box()
            box6.label(text="Center of Mass")
            box6.prop(settings, "center_enabled")
            if settings.center_enabled:
                box6.prop(settings, "center_apply_mode", text="Apply To")
                box6.prop(settings, "center_target_bone")
                box6.prop(settings, "center_preserve_height")

            # IK фиксация ног
            box7 = layout.box()
            box7.label(text="Leg Stabilization")
            box7.prop(settings, "use_ik", text="Use IK (Sticky Feet)")
            if settings.use_ik:
                row = box7.row(align=True)
                row.prop(settings, "left_leg_name", text="Left Leg")
                row.prop(settings, "right_leg_name", text="Right Leg")

            # Расширенные настройки
            box8 = layout.box()
            box8.prop(settings, "show_advanced_rage",
                      text="Advanced Settings",
                      icon='TRIA_DOWN' if settings.show_advanced_rage else 'TRIA_RIGHT',
                      emboss=False)

            if settings.show_advanced_rage:
                # Физические параметры
                box_physics = box8.box()
                box_physics.label(text="Physics Parameters")
                box_physics.prop(settings, "noise_amount", text="Adrenaline Noise")

                # Дыхание
                box_breath = box8.box()
                box_breath.label(text="Breathing Pattern")
                box_breath.prop(settings, "idle_amp", text="Breath Amplitude")
                box_breath.prop(settings, "idle_freq", text="Breath Frequency")

                # Эффекты
                box_effects = box8.box()
                box_effects.label(text="Additional Effects")
                box_effects.prop(settings, "damage_shake", text="Add Micro Shake")

        elif settings.animation_type == 'FALL':
            box = layout.box()
            box.label(text=tr("FALL ANIMATION", lang))
            box.operator("pw.generate_animation", text=tr("Generate Fall Animation", lang), icon='MOD_PHYSICS')

            # --- Настройки падения ---
            box2 = layout.box()
            box2.label(text=tr("Fall Parameters", lang))
            box2.prop(settings, "fall_height", text=tr("Drop Height", lang))
            box2.prop(settings, "fall_speed_fwd", text=tr("Forward Speed", lang))
            box2.prop(settings, "fall_type", text=tr("Fall Type", lang))

            # --- Подтипы и детали ---
            box3 = layout.box()
            box3.label(text=tr("Fall Details", lang))
            box3.prop(settings, "fall_initial_jump", text=tr("Initial Hop", lang))
            box3.prop(settings, "fall_air_resistance", text=tr("Air Drag", lang))
            box3.prop(settings, "fall_land_heavy", text=tr("Impact Heaviness", lang))

            # --- Выбор ног ---
            box4 = layout.box()
            box4.label(text=tr("Leg Selection", lang))
            box4.prop(settings, "left_leg_name", text=tr("Left Bone (override)", lang))
            box4.prop(settings, "right_leg_name", text=tr("Right Bone (override)", lang))
            box4.prop(settings, "bone_name_mask")

            # --- Центр масс ---
            box5 = layout.box()
            box5.label(text=tr("Center of Mass", lang))
            box5.prop(settings, "center_enabled", text=tr("Enable COM", lang))
            if settings.center_enabled:
                box5.prop(settings, "center_apply_mode", text=tr("Apply To", lang))
                box5.prop(settings, "center_target_bone", text=tr("COM Bone", lang))
                box5.prop(settings, "center_preserve_height", text=tr("Preserve Height", lang))
                box5.prop(settings, "center_bob_amount", text=tr("Impact Bob", lang))

            # --- Анимация рук ---
            box6 = layout.box()
            box6.label(text=tr("Arm Animation", lang))
            box6.prop(settings, "use_arm_animation", text=tr("Animate Arms", lang))
            if settings.use_arm_animation:
                box6.prop(settings, "arm_swing_amount", text=tr("Arm Flail Amount", lang))
                box6.prop(settings, "arm_stiffness", text=tr("Arm Stiffness", lang))

            # --- IK / Ноги ---
            box7 = layout.box()
            box7.label(text=tr("Leg Stabilization", lang))
            box7.prop(settings, "use_ik", text=tr("Use IK (Sticky Feet)", lang))
            if settings.use_ik:
                box7.prop(settings, "ik_chain_count", text=tr("IK Chain Count", lang))
                box7.prop(settings, "ik_interpolation_mode", text=tr("IK Interpolation", lang))
                if settings.ik_interpolation_mode == 'CONSTANT_SPEED':
                    box7.prop(settings, "ik_constant_speed", text=tr("IK Speed", lang))
                elif settings.ik_interpolation_mode == 'SPRING':
                    box7.prop(settings, "ik_spring_stiffness", text=tr("Spring Stiffness", lang))
                    box7.prop(settings, "ik_spring_damping", text=tr("Spring Damping", lang))

            # --- Squash & Stretch ---
            box8 = layout.box()
            box8.label(text=tr("Impact Deformation", lang))
            box8.prop(settings, "jump_squash", text=tr("Squash on Impact", lang))
            box8.prop(settings, "jump_stretch", text=tr("Stretch During Fall", lang))

            # --- Тайминг ---
            box9 = layout.box()
            box9.label(text=tr("Timing", lang))
            box9.prop(settings, "frame_start", text=tr("Start Frame", lang))
            box9.prop(settings, "frame_end", text=tr("End Frame", lang))
            box9.prop(settings, "frequency", text=tr("Fall Cycles", lang))
        else:
            # Fallback: show a compact set for other animation types
            box = layout.box()
            box.label(text=tr(settings.animation_type, settings.ui_language))
            box.operator("pw.generate_animation", text=tr("Generate Animation", lang))

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

        # НОВЫЕ КНОПКИ: DEATH и STUN
        row = layout.row(align=True)
        row.operator("pw.simple_generate_death", text="Death", icon='MOD_PHYSICS')
        row.operator("pw.simple_generate_stun", text="Stun", icon='FORCE_TURBULENCE')

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
        box_jump.prop(s, "move_distance", text=tr("Jump Distance"))

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
        # ===== ОБНОВЛЕННЫЙ БЛОК DODGE SETTINGS =====
        box_dodge = layout.box()
        box_dodge.label(text="Dodge Settings", icon='ARMATURE_DATA')

        col = box_dodge.column(align=True)
        col.prop(s, "dodge_style", text="Style")
        col.prop(s, "dodge_lateral_dist", text="Lateral Distance")
        col.prop(s, "dodge_side", text="Side (L/R)")

        if s.dodge_style == 'SIMPLE':
            col.prop(s, "dodge_quick_frac", text="Dodge Speed")
            box_info = box_dodge.box()
            box_info.label(text="Simple Style Phases:", icon='INFO')
            box_info.label(text="1. Dodge: Move + Z-axis turn")
            box_info.label(text="2. Hold: Short pause")
            box_info.label(text="3. Return: Full recovery to start")

        # Показываем параметры в зависимости от выбранного стиля
        elif s.dodge_style == 'HOP':
            col.prop(s, "dodge_hop_height", text="Hop Height")
            col.prop(s, "dodge_quick_frac", text="Quick Fraction")

            box_info = box_dodge.box()
            box_info.label(text="Hop Style Phases:", icon='INFO')
            box_info.label(text="1. Anticipation (15%): Crouch preparation")
            box_info.label(text="2. Air (55%): Jump arc with banking")
            box_info.label(text="3. Land (30%): Impact and recovery")

        elif s.dodge_style == 'SLIDE':
            col.prop(s, "dodge_slide_tilt", text="Slide Tilt (deg)")
            col.prop(s, "dodge_quick_frac", text="Quick Fraction")

            box_info = box_dodge.box()
            box_info.label(text="Slide Style Phases:", icon='INFO')
            box_info.label(text="1. Quick start: Fast lateral movement")
            box_info.label(text="2. Slide: Tilted body glide")
            box_info.label(text="3. Slow stop: Controlled deceleration")

        elif s.dodge_style == 'MATRIX':
            col.prop(s, "dodge_matrix_lean", text="Matrix Lean (deg)")
            col.prop(s, "dodge_quick_frac", text="Quick Fraction")

            box_info = box_dodge.box()
            box_info.label(text="Matrix Style Phases:", icon='INFO')
            box_info.label(text="1. Lean (30%): Body leans to side")
            box_info.label(text="2. Hold (30%): Maintain position")
            box_info.label(text="3. Return (40%): Smooth recovery")

        # Общая информация
        box_dodge.separator()
        col_info = box_dodge.column(align=True)
        col_info.label(text="Compatible with:", icon='CHECKBOX_HLT')
        col_info.label(text="- Simple objects (cubes, spheres)")
        col_info.label(text="- Single-bone armatures")
        col_info.label(text="- Target bone in complex armatures")

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

        # === DEATH ANIMATION SETTINGS ===
        box_death = layout.box()
        box_death.label(text="Death Animation Settings", icon='MOD_PHYSICS')

        col = box_death.column(align=True)
        col.prop(s, "death_style", text="Style")

        # Show settings based on selected style
        if s.death_style == 'COLLAPSE':
            col.prop(s, "death_collapse_power", text="Collapse Power")
            col.prop(s, "death_dissolve_speed", text="Dissolve Speed")
            col.prop(s, "death_collapse_shake", text="Shake Intensity")

            # Info box for COLLAPSE
            box_info = box_death.box()
            box_info.label(text="Collapse Phases:", icon='INFO')
            box_info.label(text="1. Tremble (20%): Intense shaking")
            box_info.label(text="2. Collapse (40%): Sinking with shakes")
            box_info.label(text="3. Dissolve (30%): Slow fading")
            box_info.label(text="4. Stillness (10%): Complete stop")

        elif s.death_style == 'EXPLOSION':
            col.prop(s, "death_explosion_power", text="Explosion Power")
            col.prop(s, "death_spin_speed", text="Spin Speed")
            col.prop(s, "death_explosion_shake", text="Pre-explosion Shake")

            # Info box for EXPLOSION
            box_info = box_death.box()
            box_info.label(text="Explosion Phases:", icon='INFO')
            box_info.label(text="1. Shake (15%): Pre-explosion vibration")
            box_info.label(text="2. Explosion (40%): Lift, spin & fall")
            box_info.label(text="3. Fall (30%): Debris descent")
            box_info.label(text="4. Smoke (15%): Final remains")

        elif s.death_style == 'FALL':
            col.prop(s, "death_fall_height", text="Fall Height")
            col.prop(s, "death_fall_bounce", text="Bounce Strength")
            col.prop(s, "death_fall_side", text="Side Fall")
            col.prop(s, "death_fall_rotation", text="Rotation (deg)")

            # Info box for FALL
            box_info = box_death.box()
            box_info.label(text="Fall Phases:", icon='INFO')
            box_info.label(text="1. Stagger (15%): Loss of balance")
            box_info.label(text="2. Fall (50%): Parabolic drop with spin")
            box_info.label(text="3. Impact (20%): Bounce and vibration")
            box_info.label(text="4. Settle (15%): Tremble then stillness")

        elif s.death_style == 'SIDE_TUMBLE':
            col.prop(s, "death_side_angle", text="Side Angle")
            col.prop(s, "death_side_wobble", text="Wobble Amount")
            col.prop(s, "death_side_slide", text="Slide Distance")

            # Info box for SIDE_TUMBLE
            box_info = box_death.box()
            box_info.label(text="Side Tumble Phases:", icon='INFO')
            box_info.label(text="1. Wobble (25%): Losing balance")
            box_info.label(text="2. Tumble (45%): Falling to side with bounce")
            box_info.label(text="3. Slide (20%): Sliding on ground")
            box_info.label(text="4. Settle (10%): Final position")

        elif s.death_style == 'KNOCKBACK':
            col.prop(s, "death_knockback_distance", text="Knockback Distance")
            col.prop(s, "death_knockback_height", text="Knockback Height")
            col.prop(s, "death_knockback_spin", text="Spin Amount")

            # Info box for KNOCKBACK
            box_info = box_death.box()
            box_info.label(text="Knockback Phases:", icon='INFO')
            box_info.label(text="1. Impact (10%): Sudden push back")
            box_info.label(text="2. Flight (40%): Arcing through air")
            box_info.label(text="3. Landing (30%): Impact and slide")
            box_info.label(text="4. Rest (20%): Coming to stop")


        # General death info
        box_death.separator()
        col_info = box_death.column(align=True)
        col_info.label(text="Compatible with:", icon='CHECKBOX_HLT')
        col_info.label(text="- Simple objects (cubes, spheres)")
        col_info.label(text="- Single-bone armatures (slimes, drones)")
        col_info.label(text="- Target bone in complex armatures")

        # === SIMPLE STUN SETTINGS ===
        box_stun = layout.box()
        box_stun.label(text="Simple Stun Settings", icon='FORCE_TURBULENCE')

        col = box_stun.column(align=True)
        col.prop(s, "simple_stun_stagger", text="Stagger Amount")
        col.prop(s, "simple_stun_shake", text="Shake Intensity")
        col.prop(s, "simple_stun_recovery", text="Recovery Speed")

        # Расширенные настройки Stun
        box_adv = box_stun.box()
        box_adv.prop(s, "show_advanced_simple_stun",
                     text="Advanced Settings",
                     icon='TRIA_DOWN' if getattr(s, "show_advanced_simple_stun", False) else 'TRIA_RIGHT',
                     emboss=False)

        if getattr(s, "show_advanced_simple_stun", False):
            adv_col = box_adv.column(align=True)
            adv_col.prop(s, "simple_stun_tilt_amount", text="Body Tilt (deg)")
            adv_col.prop(s, "simple_stun_lean_amount", text="Body Lean (deg)")
            adv_col.prop(s, "simple_stun_vibration_freq", text="Vibration Freq")
            adv_col.prop(s, "simple_stun_decay_rate", text="Decay Rate")

        # Информация о фазах
        box_info = box_stun.box()
        box_info.label(text="Animation Phases:", icon='INFO')
        box_info.label(text="1. Stagger (30%): Loss of balance, tilt")
        box_info.label(text="2. Shake (40%): Multi-frequency trembling")
        box_info.label(text="3. Recovery (30%): Smooth return to normal")

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
