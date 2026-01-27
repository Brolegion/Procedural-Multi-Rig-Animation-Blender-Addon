import bpy
import math
import random
from mathutils import Vector, Quaternion, Matrix, Euler
from . import utils
from .noise_provider import get_default_noise_provider


def generate_sneak_animation(arm_obj, settings):
    """
    Продвинутая генерация анимации крадущегося шага (Sneak).

    Особенности:
    1. Absolute Floor Locking: IK-цели рассчитываются в мировых координатах от уровня пола.
    2. Hesitation: Нелинейная скорость времени (замирания перед шагом).
    3. Predatory Posture: Наклон позвоночника вперед, голова поднята.
    """

    # -------------------------------------------------------------------------
    # 1. ИНИЦИАЛИЗАЦИЯ И СБОР ДАННЫХ
    # -------------------------------------------------------------------------
    frame_start = int(getattr(settings, "frame_start", 1))
    frame_end = int(getattr(settings, "frame_end", 60))

    # Параметры "Сочности"
    crouch_depth = getattr(settings, "step_height", 0.1) * 2.5  # Глубина приседа (зависит от высоты шага или хардкод)
    spine_tilt = 0.35  # Наклон корпуса вперед (радианы)
    hesitation_strength = 0.6  # Сила "замирания" (0 - робот, 1 - сильные паузы)
    step_lift_height = getattr(settings, "step_height", 0.15) * 0.8

    try:
        # Инициализируем состояние, но force_ground_z=True (если есть) или вычислим сами
        state = utils.init_walk_state(arm_obj, None, None, settings, frame_start, frame_end, preserve_pose=False)
    except Exception as e:
        print(f"[PW ERROR] Sneak init failed: {e}")
        return False

    # Определяем уровень "пола" на основе Rest Pose стоп
    # Это решает проблему "слишком большой коррекции". Мы просто ищем, где ноги были изначально.
    left_name = state['left_name']
    right_name = state['right_name']

    # Получаем исходные мировые позиции стоп в Rest Pose (или 1 кадре)
    l_foot_orig = arm_obj.matrix_world @ arm_obj.pose.bones[left_name].head
    r_foot_orig = arm_obj.matrix_world @ arm_obj.pose.bones[right_name].head

    # Уровень земли - это минимум из высот ног (на случай если одна чуть выше)
    ground_z = min(l_foot_orig.z, r_foot_orig.z)

    # Ширина шага (Lateral) и Длина (Stride)
    stride_len = getattr(settings, "stride_length", 0.4) * 0.7  # Красться - шаги короче

    # Шум для дрожания (напряжения)
    np_noise = get_default_noise_provider()

    # -------------------------------------------------------------------------
    # 2. ОСНОВНОЙ ЦИКЛ АНИМАЦИИ
    # -------------------------------------------------------------------------
    # Предварительно очищаем данные трекеров, будем писать свои
    if 'tracker_data' in state:
        state['tracker_data'][left_name] = []
        state['tracker_data'][right_name] = []

    # Находим цепочку позвоночника для наклона
    spine_chain = utils.find_spine_chain(arm_obj, settings)

    total_frames = frame_end - frame_start

    for i in range(total_frames + 1):
        frame = frame_start + i
        progress = i / total_frames  # 0.0 to 1.0

        # --- A. HESITATION (НЕЛИНЕЙНОЕ ВРЕМЯ) ---
        # Делаем время "тягучим". Синус модулирует скорость прохождения фазы.
        # Когда производная низкая - персонаж "замирает", когда высокая - делает быстрый перенос ноги.
        t_linear = progress * getattr(settings, "frequency", 1.0) * (total_frames / 60.0) * math.pi * 2

        # Модуляция фазы: добавляем sin(t) к t, чтобы создать неравномерность
        # Сдвигаем фазу так, чтобы "замирание" (медленная часть) приходилось на момент касания земли (stance)
        t_warped = t_linear + math.sin(t_linear - math.pi / 2) * hesitation_strength

        # Вычисляем фазы ног (0..2PI)
        phase_l = t_warped % (math.pi * 2)
        phase_r = (t_warped + math.pi) % (math.pi * 2)  # Противофаза

        bpy.context.scene.frame_set(frame)

        # --- B. ДВИЖЕНИЕ ТЕЛА (CENTER / PELVIS) ---
        # 1. Приседание (Crouch)
        # Опускаем таз вниз. ВАЖНО: Мы смещаем кость таза, но IK стопы будем считать ОТДЕЛЬНО.
        base_loc = state['pelvis_rest_loc'].copy()

        # Добавляем покачивание вверх-вниз (Bobbing) - в противофазе шагам (вниз при переносе веса)
        bob = math.sin(t_warped * 2) * 0.02

        # Добавляем покачивание влево-вправо (Sway) - вес переносится на опорную ногу
        sway_amp = 0.04
        sway = math.cos(t_warped) * sway_amp

        # Применяем к тазу (локально или через apply_center_motion)
        # Смещение вниз (Z) + Sway (X) + Forward (Y - если есть движение вперед)

        # В крадущемся режиме таз всегда опущен
        crouch_vec = Vector((sway, 0, -crouch_depth + bob))

        # Применяем поворот таза (Roll) - таз опускается в сторону маховой ноги
        roll_amp = 0.05
        pelvis_roll = math.sin(t_warped) * roll_amp

        pb_root = arm_obj.pose.bones.get(state.get('center_bone_name', ''))
        if pb_root:
            # Сброс и применение
            pb_root.location = base_loc + crouch_vec
            pb_root.rotation_euler = Euler((0, pelvis_roll, 0), 'XYZ')  # Упрощенно, зависит от ориентации костей
            pb_root.keyframe_insert("location", frame=frame)
            pb_root.keyframe_insert("rotation_euler", frame=frame)

        # --- C. НАКЛОН ПОЗВОНОЧНИКА (SPINE TILT) ---
        # Спина дугой, голова прямо.
        if spine_chain:
            # Делим наклон на количество костей (кроме последней - головы/шеи)
            bone_tilt = spine_tilt / max(1, len(spine_chain) - 1)

            for idx, b_name in enumerate(spine_chain):
                pb = arm_obj.pose.bones.get(b_name)
                if not pb: continue

                # Если это последняя кость (голова/шея) - компенсируем наклон обратно
                if idx == len(spine_chain) - 1:
                    # Look up/forward
                    pb.rotation_euler = Euler((-spine_tilt * 1.2, 0, 0), 'XYZ')
                else:
                    # Lean forward
                    # Добавляем немного шума (дыхание/напряжение)
                    noise_rot = np_noise.noise1d(frame * 0.1 + idx) * 0.05
                    pb.rotation_euler = Euler((bone_tilt + noise_rot, 0, 0), 'XYZ')

                pb.keyframe_insert("rotation_euler", frame=frame)

        # --- D. РАСЧЕТ ПОЗИЦИИ НОГ (IK TARGETS) ---
        # Самая важная часть. Мы считаем, где должна быть нога в МИРЕ.

        # Глобальный сдвиг персонажа (если он движется вперед)
        # Для анимации "на месте" (In Place) forward_offset = 0.
        # Если нужно движение - можно добавить (i * speed).
        world_fwd_offset = Vector((0, 0, 0))

        # Функция для расчета одной ноги
        def calc_foot_world_pos(phase, rest_x, side_sign):
            # phase: 0..2PI.
            # 0..PI = Stance (на земле, движется назад относительно тела)
            # PI..2PI = Swing (в воздухе, движется вперед)

            # Нормализуем фазу в 0..1 для цикла
            p_norm = (phase % (2 * math.pi)) / (2 * math.pi)

            # Расчет Y (вперед-назад)
            # В stance фазе нога едет назад (от +stride/2 до -stride/2)
            # В swing фазе нога летит вперед (от -stride/2 до +stride/2)

            y_pos = 0.0
            z_pos = ground_z  # База - пол

            if 0 <= p_norm < 0.5:  # Swing (Мах) - переносим ногу вперед
                t_swing = p_norm * 2.0  # 0..1
                # Easing для маха
                t_eased = utils.ease_in_out_cubic(t_swing)
                y_pos = -stride_len / 2 + stride_len * t_eased

                # Подъем ноги (Parabola)
                lift = math.sin(t_swing * math.pi) * step_lift_height
                z_pos += lift

            else:  # Stance (Опора) - нога стоит (или едет назад если перс идет)
                t_stance = (p_norm - 0.5) * 2.0  # 0..1
                # Linear для опоры (чтобы не проскальзывало при равномерном движении)
                y_pos = stride_len / 2 - stride_len * t_stance
                z_pos = ground_z  # Жестко пол

            # X позиция (ширина)
            x_pos = rest_x  # Можно добавить небольшое сужение при шаге

            return Vector((x_pos, y_pos, z_pos))

        # Вычисляем для левой и правой
        # ВАЖНО: Учитываем текущую позицию арматуры, если она повернута или смещена
        # Но здесь мы считаем в локальных координатах движения и переводим в мировые

        # Получаем X координату ног из Rest Pose (чтобы не схлопнулись)
        l_rest_x = (arm_obj.matrix_world.inverted() @ l_foot_orig).x
        r_rest_x = (arm_obj.matrix_world.inverted() @ r_foot_orig).x

        l_target_local = calc_foot_world_pos(phase_l, l_rest_x, 1)
        r_target_local = calc_foot_world_pos(phase_r, r_rest_x, -1)

        # Переводим в мировые координаты.
        # Трюк: Мы берем Y из расчета шага, но Z берем АБСОЛЮТНЫЙ (ground_z + lift).
        # Мы игнорируем то, что таз опустился. IK solver сам согнет колени.

        # Для корректности нам нужно знать, где "Центр" в мире X/Y, но Z игнорировать.
        # Допустим, персонаж стоит в (0,0,0).
        l_world_final = arm_obj.matrix_world @ Vector((l_target_local.x, l_target_local.y, 0))
        l_world_final.z = l_target_local.z  # Hard override Z

        r_world_final = arm_obj.matrix_world @ Vector((r_target_local.x, r_target_local.y, 0))
        r_world_final.z = r_target_local.z  # Hard override Z

        # Сохраняем в трекеры
        state['tracker_data'][left_name].append({'frame': frame, 'loc': l_world_final})
        state['tracker_data'][right_name].append({'frame': frame, 'loc': r_world_final})

        # --- E. РУКИ (ВТОРИЧНАЯ АНИМАЦИЯ) ---
        # Руки в режиме "Sneak" часто подняты ("T-Rex" или баланс) и движутся меньше
        for side, arm_n in [('left', state.get('left_arm_name')), ('right', state.get('right_arm_name'))]:
            if not arm_n: continue
            pb_arm = arm_obj.pose.bones.get(arm_n)
            if not pb_arm: continue

            # Базовый подъем рук (sneak posture)
            lift_angle = 0.4  # Радианы
            # Противофаза ногам
            arm_phase = phase_r if side == 'left' else phase_l
            swing = math.sin(arm_phase) * 0.15

            # Вращение (локально, упрощенно)
            # Предполагаем, что вращение вокруг X или Z зависит от рига. Часто Z - это ось маха.
            # Используем кватернионы для стабильности
            base_rot = pb_arm.rotation_quaternion if pb_arm.rotation_mode == 'QUATERNION' else pb_arm.rotation_euler.to_quaternion()

            # Поднимаем руки (X axis usually) и машем (Z axis)
            # Это псевдокод для вращения, зависит от ориентации костей
            # Лучше использовать смещение от Rest Pose, если оно сохранено в state, но здесь упростим

            # Просто добавляем шум дрожания
            shake = np_noise.noise1d(frame * 0.2 + (10 if side == 'left' else 20)) * 0.05

            # В реальном коде лучше использовать apply_bone_rotation из utils, но сделаем напрямую:
            # Считаем, что мы просто чуть вращаем относительно текущего (или базового)
            pass  # Оставим руки в покое или на совести Idle слоя, чтобы не сломать позу

    # -------------------------------------------------------------------------
    # 3. ФИНАЛИЗАЦИЯ (IK TRACKERS)
    # -------------------------------------------------------------------------
    if getattr(settings, "use_ik", False):
        # Генерируем пустышки на основе наших ИДЕАЛЬНЫХ мировых координат
        if state['ik_targets'] is None:
            state['ik_targets'] = {
                state['left_name']: {
                    'target_bone_name': utils.find_target_bone(arm_obj, state['left_name'], 'foot', settings)},
                state['right_name']: {
                    'target_bone_name': utils.find_target_bone(arm_obj, state['right_name'], 'foot', settings)}
            }

        # ВАЖНО: Мы передаем None в correction_vector, потому что мы УЖЕ учли всё в мировых координатах
        # Мы не хотим, чтобы utils смещал наши идеально рассчитанные координаты
        utils.create_foot_trackers(arm_obj, state, settings, correction_vector=Vector((0, 0, 0)))

        # Включаем IK
        utils.setup_ik_on_feet_with_trackers(arm_obj, state, settings)

    print(f"[PW SNEAK] Generated {total_frames} frames with JUICY logic.")
    return True