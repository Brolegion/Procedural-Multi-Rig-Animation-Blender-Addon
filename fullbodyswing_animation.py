# fullbodyswing_animation.py
import bpy
import math
from mathutils import Vector, Quaternion
from .utils import (
    init_walk_state, compute_phases, precise_estimate_foot_world_pos,
    compute_weights, compute_com_delta, compensate_lateral_if_cycle_start,
    apply_center_motion, apply_leg_pose_and_keys,
    ease_in_out_cubic, clamp, find_spine_chain, save_bone_rotations,
    parse_turn_limits, apply_bone_rotation, retarget_rear_legs,
    calculate_effective_turn_angle, setup_spine_chain, calculate_desired_yaw,
    apply_rotation_to_armature, distribute_spine_rotation, update_center_bone_twist,
    calculate_vertical_bob, compute_turn_modulation
)
from .translations import tr


def generate_full_body_swing(arm_obj, left_name_in, right_name_in, settings):
    """
    Процедурный поворот на месте с модуляцией от шагов.
    """
    if arm_obj is None:
        print("generate_full_body_swing: arm_obj is None")
        return

    frame_start = int(getattr(settings, "frame_start", 1))
    frame_end = int(getattr(settings, "frame_end", 120))
    preserve_pose = getattr(settings, "jump_preserve_pose", False)

    # Получаем угол поворота
    angle_rad = calculate_effective_turn_angle(settings)

    # Инициализация состояния
    try:
        state = init_walk_state(arm_obj, left_name_in, right_name_in, settings, frame_start, frame_end, preserve_pose)
    except Exception as e:
        print("generate_full_body_swing init error:", e)
        return

    # Режим применения
    apply_mode = getattr(settings, "center_apply_mode", "ARMATURE_OBJECT")

    # Начальный угол поворота
    try:
        start_obj_rot_z = arm_obj.rotation_euler.z
    except Exception:
        start_obj_rot_z = 0.0

    prev_yaw = start_obj_rot_z

    # ПАРАМЕТРЫ ИНТЕРПОЛЯЦИИ (ВОССТАНАВЛИВАЕМ ВСЕ!)
    interpolation_mode = getattr(settings, "turn_interpolation_mode", "PHASE_MODULATED")
    inertia_in = getattr(settings, "turn_inertia_in", 0.3)
    inertia_out = getattr(settings, "turn_inertia_out", 0.3)
    step_influence = getattr(settings, "turn_step_influence", 0.5)
    modulation_strength = getattr(settings, "turn_modulation_strength", 0.5)
    sensitivity = getattr(settings, "turn_sensitivity", 1.0)

    # Отключаем forward движение для поворота на месте
    original_stride_length = getattr(settings, "stride_length", 0.3778)
    settings.stride_length = 0.0

    # Временно применяем параметры поворота
    original_step_height = getattr(settings, "step_height", 0.08)
    original_stride_angle = getattr(settings, "stride_angle", 0.35)
    original_floatiness = getattr(settings, "floatiness", 0.4)

    settings.step_height = getattr(settings, "turn_step_height", 0.08)
    settings.stride_angle = getattr(settings, "turn_stride_angle", 0.35)
    settings.floatiness = getattr(settings, "turn_floatiness", 0.4)

    accumulated_turn_progress = 0.0
    previous_modulation = 0.0

    # Основной цикл анимации
    for f in range(state['frame_start'], state['frame_end'] + 1):
        state['current_frame'] = f

        # Фазы
        phase_left, phase_right = compute_phases(f, state['frame_start'], state['total_frames'], settings)

        # Позиции стоп
        left_foot_world, left_contact, _, left_lift = precise_estimate_foot_world_pos(
            arm_obj, state['left_rest'], phase_left, settings, state['fw_world']
        )
        right_foot_world, right_contact, _, right_lift = precise_estimate_foot_world_pos(
            arm_obj, state['right_rest'], phase_right, settings, state['fw_world']
        )

        # Веса
        W_left, W_right, sp_left, sp_right = compute_weights(phase_left, phase_right, state['p1'], state['p2'],
                                                             settings)

        # Базовый прогресс
        base_progress = (f - state['frame_start']) / float(state['total_frames'])

        # Применяем интерполяцию с полной логикой модуляции
        if interpolation_mode == 'PHASE_MODULATED':
            turn_modulation = compute_turn_modulation(phase_left, phase_right, W_left, W_right, settings)
            modulation_alpha = 0.5
            smoothed_modulation = (1.0 - modulation_alpha) * previous_modulation + modulation_alpha * turn_modulation
            previous_modulation = smoothed_modulation

            modulation_effect = 1.0 + (smoothed_modulation * modulation_strength * step_influence * sensitivity)
            frame_increment = (1.0 / state['total_frames']) * modulation_effect
            accumulated_turn_progress += frame_increment

            progress = accumulated_turn_progress
            # Применяем инерцию входа
            if inertia_in > 0.0 and progress < inertia_in:
                progress = progress * (progress / inertia_in)
            # Применяем инерцию выхода
            if inertia_out > 0.0 and progress > (1.0 - inertia_out):
                remaining = 1.0 - progress
                progress = 1.0 - (remaining * (remaining / inertia_out))

            effective_progress = clamp(progress, 0.0, 1.0)
        else:
            effective_progress = base_progress

        # Желаемый угол поворота
        desired_yaw, eased = calculate_desired_yaw(start_obj_rot_z, angle_rad, effective_progress)
        delta_yaw = desired_yaw - prev_yaw

        # Для поворота на месте - НЕТ смещения вперёд
        delta_world = Vector((0.0, 0.0, 0.0))

        # Применяем движение центра (ВСЕГДА с convert_forward_to_rotation=True)
        apply_center_motion(state, settings, arm_obj, delta_world, delta_yaw=delta_yaw,
                            convert_forward_to_rotation=True)

        # Вертикальное движение
        calculate_vertical_bob(state, settings, sp_left, sp_right, left_lift, right_lift)

        # Применение позы ног
        if not getattr(settings, "center_only_compute", False):
            apply_leg_pose_and_keys(state, settings, arm_obj, left_foot_world, right_foot_world, phase_left,
                                    phase_right)
            retarget_rear_legs(arm_obj, settings, state, state['left_name'], state['right_name'])

        # Сохранение состояний
        state['prev_W_left'] = W_left
        state['prev_W_right'] = W_right
        prev_yaw = desired_yaw

    # Восстанавливаем оригинальные настройки
    settings.step_height = original_step_height
    settings.stride_angle = original_stride_angle
    settings.floatiness = original_floatiness
    settings.stride_length = original_stride_length

    # Финальная проверка
    final_angle = prev_yaw - start_obj_rot_z
    if getattr(settings, "debug_pw", False):
        print(f"[SWING FINAL] Target: {math.degrees(angle_rad):.1f}°, Actual: {math.degrees(final_angle):.1f}°")

    lang = settings.ui_language if hasattr(settings, "ui_language") else 'AUTO'
    print(tr("Generated full body swing animation for {left}, {right}", lang).format(left=state['left_name'],
                                                                                     right=state['right_name']))