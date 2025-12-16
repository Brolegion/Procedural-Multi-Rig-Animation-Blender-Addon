# crouch_animation.py
import bpy
import math
import time
from mathutils import Vector, Matrix, Quaternion
from . import utils
from .translations import tr


def generate_crouch_walk_for_pair(arm_obj, left_name_in, right_name_in, settings):
    """
    Генерация crouch walk и squat анимации
    Ключевое изменение: наклон позвоночника применяется ВНУТРИ apply_center_motion
    """
    frame_start = int(getattr(settings, "frame_start", 1))
    frame_end = int(getattr(settings, "frame_end", 120))

    is_squat = settings.animation_type == 'SQUAT'
    is_crouch_walk = settings.animation_type == 'CROUCH_WALK'

    # Сохраняем оригинальные настройки
    original_step_height = settings.step_height
    original_stride_angle = settings.stride_angle
    original_center_stride_scale = settings.center_stride_scale
    original_center_enabled = getattr(settings, "center_enabled", False)
    original_center_apply_mode = getattr(settings, "center_apply_mode", "NONE")
    original_center_bob_amount = getattr(settings, "center_bob_amount", 0.0)

    # Применяем crouch-модификаторы
    if is_squat:
        settings.step_height *= 0.3
        settings.stride_angle *= 0.2
        settings.center_stride_scale *= 0.1
        settings.center_bob_amount *= 0.3  # Меньше покачивания для squat
    elif is_crouch_walk:
        settings.step_height *= 0.6
        settings.stride_angle *= settings.crouch_stride_scale
        settings.center_stride_scale *= settings.crouch_stride_scale
        settings.center_bob_amount *= 0.7  # Умеренное покачивание для crouch walk

    # ВАЖНО: Включаем центр и устанавливаем режим POSE_BONE для crouch
    settings.center_enabled = True
    settings.center_apply_mode = 'POSE_BONE'

    try:
        # Используем существующую инициализацию
        state = utils.init_walk_state(arm_obj, left_name_in, right_name_in, settings, frame_start, frame_end)
    except Exception as e:
        print(f"[PW ERROR] Crouch walk init error: {e}")
        # Восстанавливаем настройки
        settings.step_height = original_step_height
        settings.stride_angle = original_stride_angle
        settings.center_stride_scale = original_center_stride_scale
        settings.center_enabled = original_center_enabled
        settings.center_apply_mode = original_center_apply_mode
        settings.center_bob_amount = original_center_bob_amount
        return

    # Сохраняем crouch параметры в state
    state['crouch_amount'] = settings.crouch_amount
    state['spine_lean_rad'] = math.radians(settings.crouch_spine_lean)
    state['stance_mult'] = settings.crouch_stance_mult
    state['is_squat'] = is_squat
    state['is_crouch_walk'] = is_crouch_walk

    # Сохраняем базовое вращение центральной кости
    if state.get('center_pose_bone'):
        pose_bone = state['center_pose_bone']
        state['center_base_rotation'] = pose_bone.rotation_quaternion.copy()
        if getattr(settings, "debug_pw", False):
            utils._dbg_write(f"[CROUCH] Saved base rotation: {state['center_base_rotation']}")

    # Основной цикл
    for f in range(frame_start, frame_end + 1):
        bpy.context.scene.frame_set(f)
        bpy.context.view_layer.update()

        state['current_frame'] = f

        # Вычисление фаз
        phase_left, phase_right = utils.compute_phases(f, frame_start, state['total_frames'], settings)

        # Для SQUAT добавляем дыхание
        breathing_offset = 0.0
        if is_squat:
            total_frames = frame_end - frame_start + 1
            breathing_rate = 2.0 * math.pi / max(1, total_frames) * 2
            breathing_phase = breathing_rate * (f - frame_start)
            breathing_offset = math.sin(breathing_phase) * 0.015 * settings.crouch_amount

        # Вычисление позиций стоп
        try:
            left_foot_world, left_contact, left_h_offset, left_lift = utils.precise_estimate_foot_world_pos(
                arm_obj, state['left_rest'], phase_left, settings, state['fw_world'], target_bone_type="foot"
            )
            right_foot_world, right_contact, right_h_offset, right_lift = utils.precise_estimate_foot_world_pos(
                arm_obj, state['right_rest'], phase_right, settings, state['fw_world'], target_bone_type="foot"
            )
        except Exception as e:
            utils._dbg_write(f"[PW ERROR] precise_estimate_foot_world_pos failed: {e}")
            left_foot_world, right_foot_world = Vector((0, 0, 0)), Vector((0, 0, 0))
            left_contact, right_contact = False, False
            left_lift, right_lift = 0.0, 0.0

        # Применяем расширение стойки
        if settings.crouch_stance_mult != 1.0:
            center_world = state['rest_pelvis_world']
            lateral_world = state['lateral_world_base']

            left_vec = left_foot_world - center_world
            right_vec = right_foot_world - center_world

            left_lateral = left_vec.dot(lateral_world)
            right_lateral = right_vec.dot(lateral_world)

            left_foot_world += lateral_world * (left_lateral * (settings.crouch_stance_mult - 1.0))
            right_foot_world += lateral_world * (right_lateral * (settings.crouch_stance_mult - 1.0))

        # Вычисление весов
        W_left, W_right, sp_left, sp_right = utils.compute_weights(
            phase_left, phase_right, state['p1'], state['p2'], settings
        )

        # Вертикальное движение центра масс
        utils.calculate_vertical_bob(state, settings, sp_left, sp_right, left_lift, right_lift)

        # Добавляем crouch опускание
        if settings.crouch_amount > 0:
            state['vertical_bob'] -= settings.crouch_amount + breathing_offset

        # Вычисление смещения центра масс
        delta_world, additional_delta_yaw = utils.compute_com_delta(
            state, settings, W_left, W_right, convert_forward_to_rotation=False
        )

        utils.compensate_lateral_if_cycle_start(state, settings, phase_left)

        # КРИТИЧЕСКОЕ ИЗМЕНЕНИЕ: Используем модифицированную функцию apply_center_motion
        apply_center_motion_with_crouch(
            state, settings, arm_obj, delta_world,
            delta_yaw=additional_delta_yaw, convert_forward_to_rotation=False
        )

        # Пропуск ног, если только центр
        if getattr(settings, "center_only_compute", False):
            state['prev_W_left'] = W_left
            state['prev_W_right'] = W_right
            continue

        # Применение позы ног
        try:
            _apply_leg_pose_with_crouch(
                state, settings, arm_obj,
                left_foot_world, right_foot_world,
                phase_left, phase_right
            )
        except Exception as e:
            utils._dbg_write(f"[PW ERROR] apply_leg_pose_with_crouch failed: {e}")

        # Анимация рук
        try:
            if getattr(settings, "use_arm_animation", False):
                utils.apply_arm_pose_and_keys(state, settings, arm_obj, phase_left, phase_right)
        except Exception as e:
            utils._dbg_write(f"[PW ERROR] Arm animation failed at frame {f}: {e}")

        # Сохранение весов
        state['prev_W_left'] = W_left
        state['prev_W_right'] = W_right

    # Восстанавливаем оригинальные настройки
    settings.step_height = original_step_height
    settings.stride_angle = original_stride_angle
    settings.center_stride_scale = original_center_stride_scale
    settings.center_enabled = original_center_enabled
    settings.center_apply_mode = original_center_apply_mode
    settings.center_bob_amount = original_center_bob_amount

    # Создание IK-трекеров (если включено)
    if getattr(settings, "use_ik", False):
        if state.get('ik_targets') is None:
            state['ik_targets'] = {
                state['left_name']: {
                    'target_bone_name': utils.find_target_bone(
                        arm_obj, state['left_name'], 'foot', settings
                    )
                },
                state['right_name']: {
                    'target_bone_name': utils.find_target_bone(
                        arm_obj, state['right_name'], 'foot', settings
                    )
                }
            }

        utils.create_foot_trackers(arm_obj, state, settings)
        utils.setup_ik_on_feet_with_trackers(arm_obj, state, settings)

    # Вывод результата
    anim_type = "Squat" if is_squat else "Crouch walk"
    print(f"[PW SUCCESS] {anim_type} generated for {state['left_name']}, {state['right_name']}")


def apply_center_motion_with_crouch(state, settings, arm_obj, delta_world, delta_yaw=0.0,
                                    convert_forward_to_rotation=False):
    """
    Модифицированная версия apply_center_motion для crouch-анимации.
    Ключевое изменение: применяет наклон позвоночника ВНУТРИ этой функции.
    """
    rest = state['rest_pelvis_world']
    state['pelvis_world_current'] += delta_world

    if not getattr(settings, "center_enabled", False) or getattr(settings, "center_apply_mode", "NONE") == 'NONE':
        return

    vertical_bob = state.get('vertical_bob', 0.0)
    WORLD_UP = Vector((0.0, 0.0, 1.0))

    mode = getattr(settings, "center_apply_mode", "NONE")

    # -----------------------
    # MODE: POSE_BONE
    # -----------------------
    if mode == 'POSE_BONE' and state['center_pose_bone']:
        pose_bone = state['center_pose_bone']
        bone = pose_bone.bone

        # Мировая дельта перемещения таза относительно rest
        delta_world_loc = state['pelvis_world_current'] - rest

        # Добавляем вертикальный bob строго по глобальной Z (WORLD_UP)
        if abs(vertical_bob) > 1e-9:
            delta_world_loc += WORLD_UP * vertical_bob

        # Преобразуем мировую дельту в локальные координаты кости
        arm_inv_rot = arm_obj.matrix_world.inverted().to_3x3()
        delta_local_arm = arm_inv_rot @ delta_world_loc

        bone_rest_inv_rot = bone.matrix_local.inverted().to_3x3()
        delta_local_bone = bone_rest_inv_rot @ delta_local_arm

        # Устанавливаем позицию кости
        pose_bone.location = delta_local_bone

        # КРИТИЧЕСКОЕ ИЗМЕНЕНИЕ: Применяем наклон позвоночника
        spine_lean_rad = state.get('spine_lean_rad', 0.0)
        if spine_lean_rad != 0:
            # Получаем боковую ось в мировых координатах
            lateral_world = state.get('lateral_world_base', Vector((1, 0, 0)))

            # Конвертируем мировую ось в локальное пространство кости
            inv_rest = bone.matrix_local.inverted().to_3x3()
            local_axis = inv_rest @ lateral_world

            # Нормализуем ось
            if local_axis.length < 0.001:
                local_axis = Vector((1, 0, 0))
            local_axis.normalize()

            # Создаем кватернион наклона
            lean_quat = Quaternion(local_axis, spine_lean_rad)

            # Устанавливаем режим вращения
            if pose_bone.rotation_mode != 'QUATERNION':
                pose_bone.rotation_mode = 'QUATERNION'

            # Применяем наклон к базовому вращению
            base_rotation = state.get('center_base_rotation', pose_bone.rotation_quaternion.copy())
            pose_bone.rotation_quaternion = (lean_quat @ base_rotation).normalized()

        # Вставляем ключевые кадры
        try:
            pose_bone.keyframe_insert(data_path="location", frame=state['current_frame'])
            if spine_lean_rad != 0:
                pose_bone.keyframe_insert(data_path="rotation_quaternion", frame=state['current_frame'])
        except Exception as e:
            if getattr(settings, "debug_pw", False):
                utils._dbg_write(f"[CROUCH] Keyframe insert failed: {e}")

        # Применяем delta_yaw как приращение к вращению кости
        if convert_forward_to_rotation and delta_yaw != 0.0:
            try:
                inv_rest = bone.matrix_local.inverted().to_3x3()
                axis_local = inv_rest @ WORLD_UP
                if axis_local.length < 1e-6:
                    axis_local = Vector((0.0, 0.0, 1.0))
                else:
                    axis_local.normalize()
                rot_quat_local = Quaternion(axis_local, delta_yaw)

                try:
                    existing_rot = pose_bone.rotation_quaternion
                    pose_bone.rotation_quaternion = (rot_quat_local @ existing_rot).normalized()
                    pose_bone.keyframe_insert(data_path="rotation_quaternion", frame=state['current_frame'])
                except Exception:
                    pass
            except Exception:
                pass

    # -----------------------
    # MODE: ARMATURE_OBJECT
    # -----------------------
    elif mode == 'ARMATURE_OBJECT':
        # Горизонтальные трансформации к объекту арматуры
        inv_world = arm_obj.matrix_world.inverted()
        delta_local = inv_world.to_3x3() @ delta_world
        arm_obj.location += delta_local
        try:
            arm_obj.keyframe_insert(data_path="location", frame=state['current_frame'])
        except Exception:
            pass

        # Применяем delta_yaw к объекту арматуры
        if convert_forward_to_rotation and delta_yaw != 0.0:
            try:
                arm_obj.rotation_euler.z += delta_yaw
                arm_obj.keyframe_insert(data_path="rotation_euler", frame=state['current_frame'])
            except Exception:
                pass

        # Вертикальный подъем к корневой кости
        if state['center_pose_bone']:
            pose_bone = state['center_pose_bone']
            bone = pose_bone.bone

            # REST head позиции кости в world
            rest_head_arm_local = bone.matrix_local.to_translation()
            rest_head_world = arm_obj.matrix_world @ rest_head_arm_local

            # Желаемая мировая позиция = rest_head_world + глобальный Z * vertical_bob
            desired_world = rest_head_world + WORLD_UP * vertical_bob

            # Переводим желаемую world позицию в armature-local
            desired_arm_local = arm_obj.matrix_world.inverted() @ desired_world

            # Переводим arm-local в bone-local
            new_pose_loc = bone.matrix_local.inverted() @ desired_arm_local

            # Устанавливаем локальную позицию pose_bone
            pose_bone.location = Vector(new_pose_loc)
            try:
                pose_bone.keyframe_insert(data_path="location", frame=state['current_frame'])
            except Exception:
                pass

    # -----------------------
    # Обновляем rest_pelvis_world
    # -----------------------
    state['rest_pelvis_world'] = state['pelvis_world_current'].copy()


def _apply_leg_pose_with_crouch(state, settings, arm_obj,
                                left_foot_world, right_foot_world,
                                phase_left, phase_right):
    """Применяет позу ног с crouch-компенсацией"""
    try:
        bpy.ops.object.mode_set(mode='POSE')

        left_rest = state['left_rest']
        right_rest = state['right_rest']
        pose = state['pose']
        left_pose = pose.bones[state['left_name']]
        right_pose = pose.bones[state['right_name']]

        # Локальные оси для вращения
        left_local_axis = left_rest.matrix_local.inverted().to_3x3() @ state['lateral_world_base']
        right_local_axis = right_rest.matrix_local.inverted().to_3x3() @ state['lateral_world_base']

        if left_local_axis.length_squared < 1e-6:
            left_local_axis = Vector((1.0, 0.0, 0.0))
        else:
            left_local_axis.normalize()

        if right_local_axis.length_squared < 1e-6:
            right_local_axis = Vector((1.0, 0.0, 0.0))
        else:
            right_local_axis.normalize()

        # Амплитуды и угол шага
        left_amp = float(getattr(settings, "left_amplitude", 1.0))
        right_amp = float(getattr(settings, "right_amplitude", 1.0))

        swing_left = math.sin(phase_left) * float(getattr(settings, "stride_angle", 0.35)) * left_amp
        swing_right = math.sin(phase_right) * float(getattr(settings, "stride_angle", 0.35)) * right_amp

        if getattr(settings, "use_invert", False):
            swing_left, swing_right = -swing_left, -swing_right

        # Crouch компенсация: сгибаем колени
        crouch_amount = settings.crouch_amount
        if crouch_amount > 0.01 and not getattr(settings, "use_ik", False):
            knee_bend_angle = crouch_amount * 1.2
            crouch_quat_left = Quaternion(left_local_axis, -knee_bend_angle * 0.6)
            crouch_quat_right = Quaternion(right_local_axis, -knee_bend_angle * 0.6)

            # Создаем матрицы вращения с компенсацией
            rot_mat_left_local = Matrix.Rotation(swing_left, 4, left_local_axis) @ crouch_quat_left.to_matrix().to_4x4()
            rot_mat_right_local = Matrix.Rotation(swing_right, 4,
                                                  right_local_axis) @ crouch_quat_right.to_matrix().to_4x4()
        else:
            rot_mat_left_local = Matrix.Rotation(swing_left, 4, left_local_axis)
            rot_mat_right_local = Matrix.Rotation(swing_right, 4, right_local_axis)

        left_pose.matrix_basis = rot_mat_left_local
        right_pose.matrix_basis = rot_mat_right_local

        # Подъём бедра
        left_local_up_axis = utils.choose_local_up_axis(left_rest)
        right_local_up_axis = utils.choose_local_up_axis(right_rest)

        step_height = float(getattr(settings, "step_height", 0.08))
        lift_left_amount = max(0.0, math.sin(phase_left)) * step_height * (
                1.0 + float(getattr(settings, "floatiness", 0.0)))
        lift_right_amount = max(0.0, math.sin(phase_right)) * step_height * (
                1.0 + float(getattr(settings, "floatiness", 0.0)))

        left_pose.location = left_local_up_axis * lift_left_amount
        right_pose.location = right_local_up_axis * lift_right_amount

        # Установка кватернионов
        try:
            left_pose.rotation_quaternion = left_pose.matrix_basis.to_quaternion()
            right_pose.rotation_quaternion = right_pose.matrix_basis.to_quaternion()
        except Exception:
            left_pose.rotation_quaternion = rot_mat_left_local.to_quaternion()
            right_pose.rotation_quaternion = rot_mat_right_local.to_quaternion()

        # Удаление IK-констрейнтов, если use_ik отключен
        if not getattr(settings, "use_ik", False):
            utils.remove_ik_constraints(left_pose)
            utils.remove_ik_constraints(right_pose)

        # Вставка ключевых кадров
        f = state['current_frame']
        left_pose.keyframe_insert(data_path="rotation_quaternion", frame=f)
        right_pose.keyframe_insert(data_path="rotation_quaternion", frame=f)
        left_pose.keyframe_insert(data_path="location", frame=f)
        right_pose.keyframe_insert(data_path="location", frame=f)

        arm_obj.pose.update()

    except Exception as e:
        raise e