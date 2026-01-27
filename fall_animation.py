import bpy
import math
import random
from mathutils import Vector, Quaternion, Matrix
from . import utils


def generate_fall_animation(arm_obj, settings):
    """
    Генерирует анимацию падения, сохраняя исходную позу (Additive Animation).
    """

    # -------------------------------------------------------------------------
    # 1. ИНИЦИАЛИЗАЦИЯ
    # -------------------------------------------------------------------------
    frame_start = int(settings.frame_start)
    fps = float(bpy.context.scene.render.fps)

    # Инициализируем состояние
    # preserve_pose=True критически важен, чтобы не сбрасывать скелет в T-pose
    state = utils.init_walk_state(
        arm_obj, settings.left_leg_name, settings.right_leg_name, settings,
        frame_start, frame_start + 100, preserve_pose=True
    )

    # Параметры физики
    gravity = float(getattr(settings, "jump_gravity", 9.81))
    drop_height = settings.fall_height
    fwd_speed = settings.fall_speed_fwd
    initial_jump = settings.fall_initial_jump
    fall_type = settings.fall_type

    # Рассчитываем время падения
    v0 = math.sqrt(2 * gravity * initial_jump) if initial_jump > 0 else 0.0
    discriminant = (v0 ** 2) + (2 * gravity * drop_height)
    t_fall_phys = (v0 + math.sqrt(discriminant)) / gravity

    # Конвертация в кадры
    frames_fall = int(t_fall_phys * fps)
    frames_impact = 6
    frames_recover = int(20 * settings.fall_land_heavy)

    total_frames = frames_fall + frames_impact + frames_recover
    settings.frame_end = frame_start + total_frames

    # Базовые векторы
    fwd_vec = state['fw_world'].normalized()
    up_vec = Vector((0, 0, 1))

    # Запоминаем стартовую позицию мира для корректного расчета дельты таза
    bpy.context.scene.frame_set(frame_start)
    bpy.context.view_layer.update()

    # Это "базовая" точка таза в мире, от которой мы будем считать offset падения
    pelvis_start_world = state['pelvis_world_current'].copy()

    # Вектор "вбок" для вращения рук
    lateral_vec = state['lateral_world_base']

    # Начальные позиции стоп (для IK)
    left_foot_start = utils.get_current_foot_position(arm_obj, state['left_name'], settings)
    right_foot_start = utils.get_current_foot_position(arm_obj, state['right_name'], settings)

    # SNAPSHOT ПОЗЫ: Сохраняем исходные вращения всех костей
    # Это позволяет накладывать анимацию поверх текущей позы (с оружием и т.д.)
    original_poses = {}
    for pb in arm_obj.pose.bones:
        original_poses[pb.name] = {
            'rot': pb.rotation_quaternion.copy(),
            'loc': pb.location.copy(),
            'scale': pb.scale.copy()
        }

    # Данные для IK трекеров
    tracker_data = {state['left_name']: [], state['right_name']: []}
    rng = random.Random(frame_start)

    print(f"\n[PW FALL] === НАЧАЛО ПАДЕНИЯ ===")
    print(f"  - Высота: {drop_height}m, Кадров: {total_frames}")

    # -------------------------------------------------------------------------
    # 2. ГЕНЕРАЦИЯ КАДРОВ
    # -------------------------------------------------------------------------
    for i in range(total_frames + 1):  # +1 чтобы захватить последний кадр
        frame = frame_start + i
        bpy.context.scene.frame_set(frame)
        state['current_frame'] = frame

        # Фазы
        if i < frames_fall:
            phase = 'AIR'
            t = i / max(1, frames_fall)
            t_phys = t * t_fall_phys
        elif i < frames_fall + frames_impact:
            phase = 'IMPACT'
            t = (i - frames_fall) / max(1, frames_impact)
        else:
            phase = 'RECOVER'
            t = (i - frames_fall - frames_impact) / max(1, frames_recover)
            # Clamp t to 1.0 to avoid overshoot
            if t > 1.0: t = 1.0

        # --- A. ДВИЖЕНИЕ ЦЕНТРА МАСС (PELVIS) ---
        pelvis_offset = Vector((0, 0, 0))

        # Целевая конечная позиция смещения (относительно старта)
        # Мы должны прийти ровно сюда, чтобы сохранить исходную позу ног
        final_offset_z = -drop_height
        final_offset_xy = fwd_speed * t_fall_phys

        if phase == 'AIR':
            z_curr = (v0 * t_phys) - (0.5 * gravity * (t_phys ** 2))
            xy_curr = fwd_speed * t_phys
            pelvis_offset = (fwd_vec * xy_curr) + (up_vec * z_curr)

            if fall_type == 'PANIC':
                wiggle = math.sin(t * 15.0) * 0.1
                pelvis_offset += (lateral_vec * wiggle)

        elif phase == 'IMPACT':
            xy_curr = final_offset_xy

            # Squashing
            squash_depth = settings.fall_land_heavy * 0.45
            squash = math.sin(t * math.pi) * squash_depth if t < 0.5 else squash_depth * (1.0 - ((t - 0.5) * 2)) * 0.5

            # Slide
            slide = fwd_speed * 0.15 * (1.0 - t)

            pelvis_offset = (fwd_vec * (xy_curr + slide)) + (up_vec * (final_offset_z - squash))

            # Shake
            shake = rng.uniform(-0.04, 0.04) * (1.0 - t) * settings.fall_land_heavy
            pelvis_offset += Vector((shake, shake, shake))

        elif phase == 'RECOVER':
            xy_end = final_offset_xy + (fwd_speed * 0.15)

            # Smoothstep
            ts = t * t * (3 - 2 * t)

            # Мы интерполируем от "позиции удара" к "нормальной высоте -drop_height".
            # Но для точности лучше интерполировать от текущей (слайд) к идеальной конечной.
            # Однако, чтобы ноги выпрямились полностью, Z должен стать ровно final_offset_z.

            current_xy = utils.interpolate_linear(final_offset_xy + (fwd_speed * 0.15), xy_end, ts)  # Slide ends
            current_z = final_offset_z  # Идеал

            # Добавляем остаточный bob, который затухает в 0
            bob = math.sin(t * math.pi * 2) * 0.015 * (1.0 - t)

            pelvis_offset = (fwd_vec * current_xy) + (up_vec * (current_z + bob))

        # Применяем дельту таза
        target_pelvis_world = pelvis_start_world + pelvis_offset
        delta_pelvis = target_pelvis_world - state['pelvis_world_current']

        # apply_center_motion двигает таз на delta_pelvis + (возможно) start_down
        # Но так как мы используем delta от start_world, относительное смещение сохраняется.
        utils.apply_center_motion(state, settings, arm_obj, delta_pelvis)

        # Обновляем трекер текущей позиции
        state['pelvis_world_current'] = target_pelvis_world.copy()

        # --- B. КОНЕЧНОСТИ (IK) ---

        # Цель стоп: Старт - высота падения.
        land_l = left_foot_start + (fwd_vec * final_offset_xy)
        land_l.z = left_foot_start.z - drop_height

        land_r = right_foot_start + (fwd_vec * final_offset_xy)
        land_r.z = right_foot_start.z - drop_height

        if fall_type == 'PANIC':
            land_l += (state['lateral_world_base'] * 0.25)
            land_r -= (state['lateral_world_base'] * 0.25)

        curr_l_pos = Vector((0, 0, 0))
        curr_r_pos = Vector((0, 0, 0))

        if phase == 'AIR':
            drag_factor = settings.fall_air_resistance
            current_fall_speed = gravity * t_fall_phys * t
            drag_offset = up_vec * (current_fall_speed * drag_factor * 0.08)

            base_l = left_foot_start.lerp(land_l, t)
            base_r = right_foot_start.lerp(land_r, t)

            flail = Vector((0, 0, 0))
            if fall_type == 'PANIC':
                flail.x = math.sin(t * 20.0) * 0.15
                flail.z = math.cos(t * 18.0) * 0.15
            elif fall_type == 'CONTROLLED':
                if t > 0.5:
                    drag_offset.z += 0.15

            curr_l_pos = base_l + drag_offset + flail
            curr_r_pos = base_r + drag_offset - flail

            if t > 0.85:
                reach = (t - 0.85) / 0.15
                curr_l_pos = curr_l_pos.lerp(land_l, reach)
                curr_r_pos = curr_r_pos.lerp(land_r, reach)

        elif phase == 'IMPACT':
            slide_amt = (1.0 - t) * 0.15 if fall_type == 'PANIC' else 0.0
            curr_l_pos = land_l + fwd_vec * slide_amt
            curr_r_pos = land_r + fwd_vec * slide_amt
        else:
            curr_l_pos = land_l
            curr_r_pos = land_r

        tracker_data[state['left_name']].append({'frame': frame, 'loc': curr_l_pos})
        tracker_data[state['right_name']].append({'frame': frame, 'loc': curr_r_pos})

        # --- C. РУКИ (ARMS) - ADDITIVE DELTAS ---
        if settings.use_arm_animation:
            curr_velocity_z = gravity * t_phys if phase == 'AIR' else 0.0

            for side in ['left', 'right']:
                arm_bone_name = state.get(f'{side}_arm_name')
                forearm_name = state.get(f'{side}_forearm_name')
                if not arm_bone_name: continue

                pb = arm_obj.pose.bones.get(arm_bone_name)
                if not pb: continue

                # 1. Берем ИСХОДНУЮ ротацию (из снапшота)
                base_rot = original_poses[arm_bone_name]['rot']

                # 2. Вычисляем анимационную добавку (Delta)
                anim_angle = 0.0
                bend_angle = 0.0

                if phase == 'AIR':
                    target_angle = min(2.5, curr_velocity_z * 0.15 * settings.fall_air_resistance)
                    if fall_type == 'CONTROLLED':
                        target_angle *= 0.4
                        if side == 'right': target_angle *= -0.5
                    elif fall_type == 'PANIC':
                        target_angle += math.sin(i * 0.8) * 0.5

                    anim_angle = -target_angle
                    bend_angle = 0.5

                elif phase == 'IMPACT':
                    impact_strength = (1.0 - t)
                    anim_angle = 1.0 * impact_strength * settings.fall_land_heavy
                    bend_angle = 0.2

                elif phase == 'RECOVER':
                    # Плавный возврат в 0 (к исходной позе)
                    anim_angle = utils.interpolate_linear(0.2, 0.0, t)
                    bend_angle = utils.interpolate_linear(0.2, 0.0, t)

                # 3. Ось вращения
                rest_bone = state.get(f'{side}_arm_rest') or arm_obj.data.bones[arm_bone_name]
                inv_matrix = rest_bone.matrix_local.inverted().to_3x3()
                local_lateral = (inv_matrix @ state['lateral_world_base']).normalized()

                if side == 'right': anim_angle *= -1

                q_delta = Quaternion(local_lateral, anim_angle)

                # 4. ПРИМЕНЯЕМ: BASE @ DELTA
                pb.rotation_quaternion = base_rot @ q_delta
                pb.keyframe_insert(data_path="rotation_quaternion", frame=frame)

                # Предплечья
                if forearm_name:
                    pf = arm_obj.pose.bones.get(forearm_name)
                    if pf:
                        base_forearm = original_poses[forearm_name]['rot']
                        # Упрощенно сгиб по X
                        q_bend = Quaternion(Vector((1, 0, 0)), bend_angle)
                        pf.rotation_quaternion = base_forearm @ q_bend
                        pf.keyframe_insert(data_path="rotation_quaternion", frame=frame)

    # -------------------------------------------------------------------------
    # 3. ФИНАЛИЗАЦИЯ
    # -------------------------------------------------------------------------
    if settings.use_ik:
        utils.create_jump_trackers_with_anchored_behavior(
            arm_obj, state, settings, tracker_data
        )

    print(f"[PW FALL] End.")
    return True