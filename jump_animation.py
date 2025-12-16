import bpy
import math
import random
from mathutils import Vector, Quaternion
from . import utils


def generate_armature_jump(arm_obj, settings, **kwargs):
    """
    Генерирует процедурную анимацию прыжка с физически корректной траекторией,
    адаптацией под скелет (Spore-like) и жесткой фиксацией IK-трекеров.
    """
    # ----------------------------------------------------------
    # 1. ИНИЦИАЛИЗАЦИЯ И ДАННЫЕ
    # ----------------------------------------------------------
    frame_start = int(getattr(settings, "frame_start", 1))
    frame_end = int(getattr(settings, "frame_end", 60))
    fps = float(bpy.context.scene.render.fps)
    # 1. Инициализация состояния
    left_leg_name = getattr(settings, "left_leg_name", "")
    right_leg_name = getattr(settings, "right_leg_name", "")
    preserve_pose = getattr(settings, "jump_preserve_pose", True)

    try:
        state = utils.init_walk_state(
            arm_obj,
            left_leg_name if left_leg_name else None,
            right_leg_name if right_leg_name else None,
            settings,
            frame_start,
            frame_end,
            preserve_pose
        )

        # --- ТОЧЕЧНАЯ ПРАВКА #1: Приоритет ввода пользователя для имен пары ---
        # Если пользователь явно ввел имена, они переопределяют автоматическое обнаружение,
        # чтобы гарантировать, что для IK используются именно эти кости.
        if left_leg_name:
            state['left_name'] = left_leg_name
        if right_leg_name:
            state['right_name'] = right_leg_name

        utils._dbg_write(f"[PW JUMP] Left: {state['left_name']}, Right: {state['right_name']}")
    except Exception as e:
        print(f"[PW ERROR] Jump Init Failed: {e}")
        return False

    # Получаем морфологию для адаптивности
    morph = utils.detect_jump_morphology(arm_obj, settings)
    leg_length = morph.get('leg_length', 0.5)

    # Параметры из UI
    target_height = float(getattr(settings, "jump_height", 1.0))
    jump_dist = float(getattr(settings, "move_distance", 2.0))
    landing_h = float(getattr(settings, "jump_landing_height", 0.0))

    gravity = float(getattr(settings, "jump_gravity", 9.81))
    mass = float(getattr(settings, "jump_mass", 50.0))

    # Рандомизация (Seed от позиции для детерминизма)
    orig_loc = state['rest_pelvis_world']
    rand_seed = int(orig_loc.x * 137.0 + orig_loc.y * 59.0)
    rng = random.Random(rand_seed)
    randomness = float(getattr(settings, "jump_randomness", 0.05))

    # Вариативность высоты
    height_factor = 1.0 + (rng.uniform(-1.0, 1.0) * 0.15 * randomness)
    actual_height = max(0.05, target_height * height_factor)

    # ----------------------------------------------------------
    # 2. ФИЗИКА И ТАЙМИНГ
    # ----------------------------------------------------------

    # Расчет физической скорости и времени
    v0_phys, t_up_phys, t_total_phys_base = utils.calculate_jump_physics(actual_height, gravity)

    # Коррекция времени падения (если приземляемся ниже/выше старта)
    # h(t) = v0*t - 0.5*g*t^2. Решаем квадратное уравнение для h = landing_h
    # 0.5*g*t^2 - v0*t + landing_h = 0
    # D = v0^2 - 2*g*landing_h
    discrim = (v0_phys ** 2) - (2 * gravity * landing_h)

    if discrim < 0:
        # Недолет (цель слишком высоко). Принудительно увеличиваем высоту прыжка.
        actual_height = landing_h + 0.1
        v0_phys, t_up_phys, _ = utils.calculate_jump_physics(actual_height, gravity)
        discrim = (v0_phys ** 2) - (2 * gravity * landing_h)

    t_flight_phys = (v0_phys + math.sqrt(discrim)) / gravity

    # Распределение кадров анимации
    total_frames = frame_end - frame_start + 1
    takeoff_frac = getattr(settings, "jump_takeoff_frac", 0.2)
    recover_frac = 0.15

    frames_takeoff = int(total_frames * takeoff_frac)
    frames_recover = int(total_frames * recover_frac)
    frames_flight = total_frames - frames_takeoff - frames_recover

    # Защита от слишком короткого прыжка
    if frames_flight < 3: frames_flight = 3

    # Time Dilation (Масштабирование времени)
    # Чтобы физическая парабола уложилась в выделенные аниматором кадры
    t_anim_flight = frames_flight / fps
    time_scale = t_flight_phys / max(0.001, t_anim_flight)

    # Скорость по горизонтали (равномерная)
    vx_anim = jump_dist / max(0.001, t_anim_flight)

    # ----------------------------------------------------------
    # 3. ПОДГОТОВКА ТРАЕКТОРИЙ
    # ----------------------------------------------------------

    # Начальная позиция таза
    pelvis_start_world = state['rest_pelvis_world'].copy()

    # Вычисляем позиции стоп (где они стоят СЕЙЧАС)
    # Получаем реальные позиции стоп из текущей позы (rest)
    # Находим кости стоп
    # ПРЯМОЕ ПОЛУЧЕНИЕ ТЕКУЩИХ ПОЗИЦИЙ СТОП ИЗ ПОЗЫ
    left_foot_start = utils.get_current_foot_position(arm_obj, state['left_name'], settings)
    right_foot_start = utils.get_current_foot_position(arm_obj, state['right_name'], settings)

    # Уровень земли для заякорения
    ground_level = min(left_foot_start.z, right_foot_start.z)
    utils._dbg_write(
        f"[PW JUMP] Ground level: {ground_level:.3f}, Left foot Z: {left_foot_start.z:.3f}, Right foot Z: {right_foot_start.z:.3f}")

    # Вектор движения
    fwd_vec = state['fw_world'].normalized()

    # Позиции стоп приземления
    left_foot_end = left_foot_start + (fwd_vec * jump_dist)
    left_foot_end.z += landing_h
    right_foot_end = right_foot_start + (fwd_vec * jump_dist)
    right_foot_end.z += landing_h

    # Хранилище для записи трекеров
    tracker_data = {state['left_name']: [], state['right_name']: []}

    # ----------------------------------------------------------
    # 4. ГЕНЕРАЦИЯ КАДРОВ
    # ----------------------------------------------------------

    for i in range(total_frames):
        frame = frame_start + i
        bpy.context.scene.frame_set(frame)

        # Обновляем state текущим кадром (критично для utils)
        state['current_frame'] = frame

        # Переменные смещения для таза (относительно pelvis_start_world)
        pelvis_offset = Vector((0, 0, 0))

        # Определение фазы и локального прогресса (0..1)
        if i < frames_takeoff:
            phase = 'TAKEOFF'
            p = i / max(1, frames_takeoff)
        elif i < frames_takeoff + frames_flight:
            phase = 'FLIGHT'
            p = (i - frames_takeoff) / max(1, frames_flight)
        else:
            phase = 'RECOVER'
            p = (i - frames_takeoff - frames_flight) / max(1, frames_recover)

        # --- ЛОГИКА ТАЗА (Center of Mass) ---

        if phase == 'TAKEOFF':
            # Подготовка: приседание (Squash)
            # Используем Easing для "пружинистости"
            eased_p = utils.ease_in_out_cubic(p)

            # Адаптивный присед: чем длиннее ноги, тем глубже можно сесть
            # Максимум в середине фазы или ближе к концу? Для прыжка - перед самым концом.
            # Формула синуса смещена, чтобы пик был в конце
            squash = math.sin(p * math.pi)

            # Глубина зависит от leg_length и настроек
            crouch_depth = leg_length * 0.4 * float(getattr(settings, "jump_crouch_factor", 1.0))
            pelvis_offset.z = -crouch_depth * squash

            # IK: Стопы МЕРТВО стоят на старте
            curr_left_foot = left_foot_start.copy()
            curr_right_foot = right_foot_start.copy()

        elif phase == 'FLIGHT':
            # Полет: Физическая баллистика
            t_anim_curr = p * t_anim_flight
            t_phys_curr = t_anim_curr * time_scale

            # Z = v0*t - 0.5*g*t^2
            z_pos = (v0_phys * t_phys_curr) - (0.5 * gravity * (t_phys_curr ** 2))

            # XY = vx * t
            xy_dist = vx_anim * t_anim_curr

            pelvis_offset = (fwd_vec * xy_dist)
            pelvis_offset.z = z_pos

            # IK: Стопы интерполируются + поджимаются (Tuck)
            # Линейная база
            curr_left_foot = left_foot_start.lerp(left_foot_end, p)
            curr_right_foot = right_foot_start.lerp(right_foot_end, p)

            # Поджатие (Tuck): ноги должны подняться, чтобы не волочиться
            # Сила поджатия зависит от высоты прыжка
            tuck_amount = actual_height * 0.3 + (leg_length * 0.1)
            tuck_curve = math.sin(p * math.pi)  # Пик в середине полета

            tuck_vec = Vector((0, 0, tuck_amount * tuck_curve))
            curr_left_foot += tuck_vec
            curr_right_foot += tuck_vec

            # Stretch (Растяжение): В начале полета ноги отстают (drag)
            if p < 0.3:
                drag_factor = (1.0 - p / 0.3) * 0.2
                drag_vec = -fwd_vec * drag_factor
                curr_left_foot += drag_vec
                curr_right_foot += drag_vec

        elif phase == 'RECOVER':
            # Приземление: Амортизация
            # Базовая позиция - конец пути
            base_xy = jump_dist
            base_z = landing_h

            pelvis_offset = (fwd_vec * base_xy)
            pelvis_offset.z = base_z

            # Эффект удара (Squash)
            # Резкий вход, медленный выход
            impact = (1.0 - p) * math.exp(-p * 4.0)
            squash_amount = leg_length * 0.3 * float(getattr(settings, "jump_squash", 0.2))

            # Тряска от удара (Noise)
            shake = rng.uniform(-0.02, 0.02) * impact

            pelvis_offset.z += (-squash_amount * impact) + shake

            # IK: Стопы МЕРТВО стоят на финише
            curr_left_foot = left_foot_end.copy()
            curr_right_foot = right_foot_end.copy()

            # Slide (Скольжение по инерции) - опционально
            if p < 0.15:
                slide_p = 1.0 - (p / 0.15)
                slide_vec = fwd_vec * (0.15 * slide_p)
                curr_left_foot += slide_vec
                curr_right_foot += slide_vec

        # --- ПРИМЕНЕНИЕ (АНТИ-ДРИФТ) ---

        # 1. Применяем движение таза
        # Считаем абсолютную целевую точку в мире
        target_pelvis_world = pelvis_start_world + pelvis_offset

        # Вычисляем дельту от ТЕКУЩЕГО (возможно смещенного) положения таза
        # Это предотвращает накопление ошибок float и "улет в космос"
        delta_pelvis = target_pelvis_world - state['pelvis_world_current']

        # Передаем дельту в apply_center_motion
        utils.apply_center_motion(state, settings, arm_obj, delta_pelvis)

        # 2. Сохраняем данные для IK трекеров
        # Мы НЕ применяем их прямо сейчас к пустышкам, мы копим данные,
        # чтобы создать чистые кривые в post-processing.
        tracker_data[state['left_name']].append({'frame': frame, 'loc': curr_left_foot})
        tracker_data[state['right_name']].append({'frame': frame, 'loc': curr_right_foot})

        # 3. Вторичная анимация (Руки/Позвоночник)
        # Если нужно, можно добавить вызов utils.apply_spine_rotation здесь

    # ----------------------------------------------------------
    # 5. ПОСТ-ПРОЦЕССИНГ (IK TRACKERS)
    # ----------------------------------------------------------

    if getattr(settings, "use_ik", False):
        utils.create_jump_trackers_with_anchored_behavior(
            arm_obj, state, settings, tracker_data
        )

    print(f"[PW JUMP] Finished. Height: {actual_height:.2f}m, Distance: {jump_dist:.2f}m")
    return True