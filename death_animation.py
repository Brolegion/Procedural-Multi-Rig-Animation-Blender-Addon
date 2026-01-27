import bpy
import math
import random
from mathutils import Vector, Quaternion, Matrix
from . import utils


def generate_death_animation(arm_obj, settings):
    """
    Генерирует процедурную анимацию смерти, вдохновленную подходом Spore.
    Мы манипулируем Центром Масс (COM) и IK-таргетами, игнорируя конкретную топологию костей внутри цепи.
    """

    # 1. ИНИЦИАЛИЗАЦИЯ
    frame_start = int(settings.frame_start)
    fps = float(bpy.context.scene.render.fps)
    speed_mult = settings.death_speed

    # Инициализируем состояние (находим таз, ноги, векторы)
    state = utils.init_walk_state(
        arm_obj, settings.left_leg_name, settings.right_leg_name, settings, frame_start, frame_start + 100, preserve_pose=True
    )

    death_type = settings.death_type

    # Временные фазы (в кадрах)
    # Agony: фаза перед падением (шатание, замирание)
    # Fall: активное падение
    # Settle: затухание движения на земле

    frames_agony = int(settings.death_agony_duration / speed_mult)

    if death_type == 'COLLAPSE':
        frames_fall = int(20 / speed_mult)
        frames_settle = int(20 / speed_mult)
    elif death_type == 'DRAMATIC':
        frames_fall = int(35 / speed_mult)
        frames_settle = int(25 / speed_mult)
    elif death_type == 'SPIRAL':
        frames_fall = int(30 / speed_mult)
        frames_settle = int(10 / speed_mult)
    else:  # CRUMPLE
        frames_fall = int(25 / speed_mult)
        frames_settle = int(15 / speed_mult)

    total_frames = frames_agony + frames_fall + frames_settle
    settings.frame_end = frame_start + total_frames

    # Базовые векторы и точки
    pelvis_start_loc = state['rest_pelvis_world'].copy()
    fwd_vec = state['fw_world'].normalized()
    right_vec = state['lateral_world_base'].normalized()
    up_vec = Vector((0, 0, 1))

    # Высота таза над землей (предполагаем, что Z=0 это уровень стоп, если они на земле)
    # Или берем Z таза, если стопы не определены
    ground_z = pelvis_start_loc.z - settings.fall_height  # Используем fall_height как эвристику "высоты ног"
    if ground_z < 0: ground_z = 0  # Защита

    # Стартовые позиции ног
    l_foot_start = utils.get_current_foot_position(arm_obj, state['left_name'], settings)
    r_foot_start = utils.get_current_foot_position(arm_obj, state['right_name'], settings)

    tracker_data = {state['left_name']: [], state['right_name']: []}

    print(f"[PW DEATH] Generating '{death_type}' over {total_frames} frames.")
    print(f"[PW DEATH] Center bone name: {state['center_name']}")
    print(f"[PW DEATH] Pelvis start location: {pelvis_start_loc}")

    # Логирование начального состояния
    if state['center_name']:
        center_pose = arm_obj.pose.bones.get(state['center_name'])
        if center_pose:
            initial_rot = center_pose.rotation_quaternion.copy()
            axis, angle = initial_rot.to_axis_angle()
            print(f"[PW DEATH] Initial rotation - Axis: {axis}, Angle: {math.degrees(angle):.2f}°")

    # 2. ГЕНЕРАЦИЯ ПО КАДРАМ
    for i in range(total_frames):
        frame = frame_start + i
        bpy.context.scene.frame_set(frame)
        state['current_frame'] = frame

        # Нормализованное время t для каждой фазы
        pelvis_offset = Vector((0, 0, 0))
        pelvis_rot_delta = Quaternion((1, 0, 0, 0))

        # IK цели (по умолчанию стоят на месте)
        l_target = l_foot_start.copy()
        r_target = r_foot_start.copy()

        # =====================================================================
        # ВАРИАЦИЯ 1: COLLAPSE (RAGDOLL)
        # Простое падение вниз. Ноги разъезжаются (splay).
        # =====================================================================
        if death_type == 'COLLAPSE':
            if i < frames_agony:
                # Дрожь перед смертью
                shake = math.sin(i * 2.0) * 0.02
                pelvis_offset = Vector((shake, 0, -shake * 0.5))

            else:
                # Фаза падения
                local_i = i - frames_agony
                t = min(1.0, local_i / frames_fall)

                # Easing: BounceOut или QuadIn
                # Для мешка с костями лучше QuadIn (ускорение)
                drop_t = t * t

                # 1. COM падает на землю
                # Но не в 0, а на некую толщину тела (например, 20% от высоты)
                target_z = - (pelvis_start_loc.z - ground_z) * 0.85
                current_z = target_z * drop_t

                pelvis_offset.z = current_z

                # 2. Смещение вперед/назад случайно
                pelvis_offset += fwd_vec * (0.2 * drop_t)

                # 3. Вращение (заваливание на бок)
                # Рандомный наклон
                angle = math.radians(70) * drop_t
                rot_axis = (fwd_vec + right_vec * 0.5).normalized()  # По диагонали
                pelvis_rot_delta = Quaternion(rot_axis, angle)

                # 4. Ноги разъезжаются (Splay)
                # IK таргеты скользят наружу по мере падения таза
                spread = right_vec * (0.3 * drop_t)
                l_target += spread  # Влево (right_vec * +0.3, т.к. right это X+) - надо проверить направление
                # right_vec это "вправо". Left leg (обычно +X или -X).
                # Упростим: от центра наружу

                # Вектор от центра проекции таза к ноге
                center_proj = pelvis_start_loc.copy()
                center_proj.z = 0

                vec_l = (l_foot_start - center_proj).normalized()
                vec_r = (r_foot_start - center_proj).normalized()

                l_target += vec_l * (0.5 * drop_t)
                r_target += vec_r * (0.5 * drop_t)


        # =====================================================================
        # ВАРИАЦИЯ 2: DRAMATIC (HUMANOID)
        # Удар в грудь -> шаг назад -> падение на спину -> отскок
        # =====================================================================
        elif death_type == 'DRAMATIC':
            total_dyn_frames = frames_agony + frames_fall
            t_glob = i / total_dyn_frames

            if i < frames_agony:
                t = i / frames_agony
                # "Agony": Подъем груди, шаг назад
                # Поднимаемся на носки (pelvis up)
                pelvis_offset.z = math.sin(t * math.pi) * 0.1
                # Отходим назад
                pelvis_offset -= fwd_vec * (0.3 * t)

                # Вращение: Грудь вверх (pitch back)
                pitch = -math.radians(20) * math.sin(t * math.pi * 0.5)
                pelvis_rot_delta = Quaternion(right_vec, pitch)

            else:
                # Падение
                local_i = i - frames_agony
                t = min(1.0, local_i / frames_fall)

                # Парабола падения назад
                # Z падает
                target_z = - (pelvis_start_loc.z - ground_z) * 0.9
                # EaseInCubic
                t_sq = t * t
                pelvis_offset.z = target_z * t_sq

                # XY продолжает двигаться назад, но замедляясь
                back_dist = 0.3 + (0.5 * t)  # Итого 0.8м назад
                pelvis_offset -= fwd_vec * back_dist

                # Вращение: Полный завал назад (90 градусов)
                # Начинали с -20, идем до -90
                start_pitch = -20
                end_pitch = -90
                curr_deg = start_pitch + (end_pitch - start_pitch) * t_sq
                pelvis_rot_delta = Quaternion(right_vec, math.radians(curr_deg))

                # Ноги:
                # При падении назад, ноги часто подлетают вверх.
                # Мы можем симулировать это, поднимая IK таргеты
                if t > 0.3 and t < 0.8:
                    lift = math.sin((t - 0.3) / 0.5 * math.pi) * 0.2
                    l_target.z += lift
                    r_target.z += lift

        # =====================================================================
        # ВАРИАЦИЯ 3: SPIRAL (CARTOON)
        # Вращение вокруг оси + уменьшение масштаба (опционально) + падение
        # =====================================================================
        elif death_type == 'SPIRAL':
            t = i / (frames_agony + frames_fall)
            if t > 1.0: t = 1.0

            # Вращение: много оборотов (например, 720 градусов)
            spins = 2
            angle = t * math.pi * 2 * spins
            pelvis_rot_delta = Quaternion((0, 0, 1), angle)

            # Падение: задержка в воздухе (coyote time), потом резкий спад
            if t < 0.3:
                pelvis_offset.z = 0  # Висим
            else:
                t_fall = (t - 0.3) / 0.7
                pelvis_offset.z = - (pelvis_start_loc.z - ground_z) * 0.95 * (t_fall * t_fall)

            # Сжатие ног к центру (ноги заплетаются)
            center_proj = pelvis_start_loc.copy()
            center_proj.z = 0

            # Lerp от позиции ног к центру
            l_target = l_target.lerp(center_proj, t * 0.8)
            r_target = r_target.lerp(center_proj, t * 0.8)


        # =====================================================================
        # ВАРИАЦИЯ 4: CRUMPLE (INSECT / FETAL)
        # Гравитация тянет вниз, но ноги не разъезжаются, а подтягиваются.
        # =====================================================================
        elif death_type == 'CRUMPLE':
            t = min(1.0, i / (frames_agony + frames_fall))

            # Плавное опускание (EaseInOut)
            t_smooth = t * t * (3 - 2 * t)
            pelvis_offset.z = - (pelvis_start_loc.z - ground_z) * 0.9 * t_smooth

            # Наклон вперед (Curl)
            pitch = math.radians(45) * t_smooth
            pelvis_rot_delta = Quaternion(right_vec, pitch)

            # Ноги подтягиваются К ТЕЛУ
            # IK Targets движутся навстречу друг другу и чуть назад
            inward = (r_foot_start - l_foot_start).length * 0.2 * t_smooth

            l_target += (right_vec * inward) - (fwd_vec * 0.1 * t_smooth)
            r_target -= (right_vec * inward) - (fwd_vec * 0.1 * t_smooth)

        # --- ПРИМЕНЕНИЕ ТРАНСФОРМАЦИЙ ---

        # Логирование вращения ДО применения
        if state['center_name'] and (i % 5 == 0 or i == total_frames - 1):
            center_pose = arm_obj.pose.bones.get(state['center_name'])
            if center_pose:
                before_rot = center_pose.rotation_quaternion.copy()
                axis_before, angle_before = before_rot.to_axis_angle()
                axis_delta, angle_delta = pelvis_rot_delta.to_axis_angle()

                print(f"[PW DEATH] Frame {frame} ({phase if 'phase' in locals() else 'UNKNOWN'}):")
                print(f"  - Before rotation: Axis {axis_before}, Angle {math.degrees(angle_before):.2f}°")
                print(f"  - Delta rotation: Axis {axis_delta}, Angle {math.degrees(angle_delta):.2f}°")
                print(f"  - Pelvis offset: {pelvis_offset}")

        # 1. Центр масс
        target_pelvis_world = pelvis_start_loc + pelvis_offset
        delta_pelvis = target_pelvis_world - state['pelvis_world_current']

        # Для вращения таза (нужна доработка utils, но используем костыль через spine modulation или apply_center)
        # Мы просто повернем кость таза напрямую, если есть доступ
        # В текущей архитектуре utils.apply_center_motion двигает и поворачивает root/pelvis
        # Нам нужно добавить вращение к дельте?
        # utils.apply_center_motion в основном про location.
        # Добавим вращение вручную:
        pb_pelvis = arm_obj.pose.bones.get(state['center_name'])
        if pb_pelvis:
            # ИСПРАВЛЕНИЕ: Вместо умножения текущего вращения на дельту,
            # используем дельту как абсолютное вращение от начального состояния
            # Но сначала получим начальное вращение (rest pose)
            if i == 0:
                # Сохраним начальное вращение для логирования
                state['initial_pelvis_rot'] = pb_pelvis.rotation_quaternion.copy()

            # Применяем вращение КАК абсолютное, а не относительное
            # Для этого умножаем начальное вращение на дельту
            if 'initial_pelvis_rot' in state:
                new_rotation = state['initial_pelvis_rot'] @ pelvis_rot_delta
            else:
                new_rotation = pelvis_rot_delta

            pb_pelvis.rotation_quaternion = new_rotation.normalized()
            pb_pelvis.keyframe_insert("rotation_quaternion", frame=frame)

        utils.apply_center_motion(state, settings, arm_obj, delta_pelvis)

        # Логирование вращения ПОСЛЕ применения
        if state['center_name'] and (i % 5 == 0 or i == total_frames - 1) and pb_pelvis:
            after_rot = pb_pelvis.rotation_quaternion.copy()
            axis_after, angle_after = after_rot.to_axis_angle()
            print(f"  - After rotation: Axis {axis_after}, Angle {math.degrees(angle_after):.2f}°")
            print("-" * 50)

        # 2. IK Ноги
        tracker_data[state['left_name']].append({'frame': frame, 'loc': l_target})
        tracker_data[state['right_name']].append({'frame': frame, 'loc': r_target})

        # 3. Руки (Secondary Motion)
        # Если это Dramatic, рука тянется
        if death_type == 'DRAMATIC' and i < frames_agony:
            # Найти правую руку
            arm_name = state.get('right_arm_name')
            if arm_name:
                pb_arm = arm_obj.pose.bones.get(arm_name)
                if pb_arm:
                    # Поднять руку (локальный X или Z, зависит от рига, пробуем эвристику)
                    # Просто шум
                    pass

    # ФИНАЛИЗАЦИЯ
    if settings.use_ik:
        # Для смерти мы не хотим "Sticky feet" (прилипания) в том смысле, как при ходьбе.
        # Но мы хотим, чтобы ноги следовали за рассчитанными таргетами.
        # Используем стандартный генератор трекеров
        utils.create_jump_trackers_with_anchored_behavior(
            arm_obj, state, settings, tracker_data
        )

    print(f"[PW DEATH] Animation completed for death type: {death_type}")
    return True