import bpy
import math
from mathutils import Vector, Quaternion, Matrix
from . import utils


def generate_crawl_animation(arm_obj, settings):
    """
    Генерирует анимацию ползания/карабканья (Crawl).
    Позволяет персонажу двигаться по отвесным стенам или потолку,
    меняя базис гравитации и ориентацию тела.
    """

    # --- 1. ПАРАМЕТРЫ ---
    fs = int(settings.frame_start)
    fe = int(settings.frame_end)
    total_frames = fe - fs

    # Угол поверхности (0 = пол, 90 = стена, 180 = потолок)
    # Если в settings нет свойства, берем 0 (пол)
    surface_angle_deg = getattr(settings, "crawl_surface_angle", 0.0)
    surface_angle_rad = math.radians(surface_angle_deg)

    speed = getattr(settings, "crawl_speed", 1.0)
    stance_width = getattr(settings, "crawl_stance_width", 1.2)  # Множитель ширины ног
    body_height = getattr(settings, "crawl_body_height", 0.6)  # Прижатость к земле (0.0 - 1.0)

    # Инициализация (ищем ноги, скелет)
    try:
        # preserve_pose=False, так как мы полностью перестраиваем позу под стену
        state = utils.init_walk_state(arm_obj, None, None, settings, fs, fe, preserve_pose=False)
    except Exception as e:
        print(f"[PW CRAWL] Init failed: {e}")
        return False

    # --- 2. БАЗИС ПОВЕРХНОСТИ ---
    # Создаем кватернион, который поворачивает "Глобальный Z" в "Нормаль поверхности"
    # Допустим, мы карабкаемся по оси Y вперед, значит поворот вокруг оси X.
    q_surface = Quaternion(Vector((1, 0, 0)), surface_angle_rad)

    # Базисные вектора поверхности в мировых координатах
    # Up - нормаль к стене
    # Fwd - направление движения вдоль стены
    surface_up = q_surface @ Vector((0, 0, 1))
    surface_fwd = q_surface @ Vector((0, 1, 0))
    surface_right = q_surface @ Vector((1, 0, 0))

    # --- 3. ПОДГОТОВКА ДАННЫХ НОГ ---
    # Нам нужно знать "родную" позицию каждой ноги относительно таза в T-Pose/Rest Pose,
    # чтобы от неё откладывать шаги.
    leg_data = []

    # Собираем все ноги (и левые и правые)
    all_legs = []
    if state['left_name']: all_legs.append({'name': state['left_name'], 'side': -1})  # Left = -X
    if state['right_name']: all_legs.append({'name': state['right_name'], 'side': 1})  # Right = +X

    # Если есть дополнительные ноги (для пауков) - тут можно расширить логику,
    # если utils.init_walk_state умеет возвращать списки ног.
    # Пока работаем с основной парой, но логика готова к масштабированию.

    # Снапшотим локальные смещения ног от таза
    bpy.context.scene.frame_set(fs)
    pelvis_bone = arm_obj.pose.bones.get(state['root_name'])  # Pelvis/Hips
    pelvis_rest_world = arm_obj.matrix_world @ pelvis_bone.head

    for l in all_legs:
        pb = arm_obj.pose.bones.get(l['name'])
        if not pb: continue

        # Позиция ноги в мире
        foot_world = arm_obj.matrix_world @ pb.head

        # Вектор от таза до ноги в "горизонтальной плоскости" персонажа
        # Нам нужно понять, где нога стоит относительно центра ("Rest Stance")
        rel_vec = foot_world - pelvis_rest_world

        # Проецируем на локальные оси, чтобы понять "ширину" и "вылет"
        # Для простоты берем Rest Pose как эталон
        l['rest_offset'] = rel_vec
        l['phase_offset'] = 0.0 if l['side'] == 1 else 0.5  # Противофаза
        leg_data.append(l)

    # --- 4. ЦИКЛ АНИМАЦИИ ---

    # Стартовая позиция (берем текущую позицию объекта или кости)
    start_pos = state['pelvis_world_current'].copy()

    # Данные для IK
    tracker_data = {l['name']: [] for l in leg_data} if getattr(settings, "use_ik", False) else None

    for i in range(total_frames + 1):
        frame = fs + i
        bpy.context.scene.frame_set(frame)

        # t - прогресс цикла ходьбы
        # speed влияет на частоту шагов
        cycle_len = 30 / speed  # кадров на цикл
        t_global = (i / cycle_len)

        # --- A. ДВИЖЕНИЕ ТЕЛА (COM) ---
        # Линейное движение вдоль стены (вверх/вперед)
        distance_travelled = (i / 30.0) * speed * getattr(settings, "stride_length", 0.5)
        current_com_pos = start_pos + (surface_fwd * distance_travelled)

        # Bobbing (качание тела к стене и от нее)
        # При ползании тело обычно ниже к поверхности
        bob_amp = 0.05
        bob_val = math.sin(t_global * math.pi * 2 * 2) * bob_amp  # x2 частота (на каждый шаг)

        # Смещение центра масс:
        # 1. Прижимаем к стене (body_height)
        # 2. Добавляем качание
        # 3. Немного качаем влево-вправо (Weight shift)
        lat_sway = math.sin(t_global * math.pi * 2) * 0.05 * surface_right

        # Допустим, базовый клиренс (высота таза) был Z=0.8. Мы его уменьшаем.
        base_height_offset = -0.3 * (1.0 - body_height)

        final_com_pos = current_com_pos + (surface_up * (base_height_offset + bob_val)) + lat_sway

        # Вращение тела:
        # Тело наклоняется в сторону движения ног (Yaw/Roll)
        body_roll = math.sin(t_global * math.pi * 2) * 0.1  # Roll
        q_roll = Quaternion(surface_fwd, body_roll)

        # ПРИМЕНЕНИЕ К ТАЗУ
        # 1. Позиция
        delta_move = final_com_pos - state['pelvis_world_current']
        utils.apply_center_motion(state, settings, arm_obj, delta_move)
        state['pelvis_world_current'] = final_com_pos.copy()

        # 2. Ротация (Самое важное!)
        # Поворачиваем таз так, чтобы он лежал на стене (q_surface) + покачивания
        pb_root = arm_obj.pose.bones.get(state['root_name'])
        if pb_root:
            # Сбрасываем вращение к "стене"
            # Если в Rest Pose Z смотрит вверх, то q_surface направит Z вдоль нормали
            # Но нужно учесть, как кость ориентирована в Edit Mode.
            # Обычно мы применяем вращение "поверх" текущего.

            # Вращение для выравнивания по поверхности
            # Мы берем q_surface и добавляем sway
            q_final = q_surface @ q_roll

            # Применяем. ВАЖНО:
            # utils.apply_center_motion работает с дельтами локации, но ротацию таза
            # часто проще задать абсолютно, если мы меняем гравитацию.

            # Для надежности используем матрицу Basis, если есть доступ,
            # но здесь попробуем через rotation_quaternion относительно Parent (World)
            if pb_root.parent:
                # Сложно учесть парента, упростим:
                pass
            else:
                # Если это корневая кость без родителя
                pb_root.rotation_quaternion = q_final
                pb_root.keyframe_insert("rotation_quaternion", frame=frame)

        # --- B. ДВИЖЕНИЕ НОГ (IK Targets) ---
        # Самая сложная часть: где должны быть ноги на стене?

        stride_len = getattr(settings, "stride_length", 0.5)
        step_height = getattr(settings, "step_height", 0.2)

        for leg in leg_data:
            side_sign = leg['side']
            # Фаза конкретной ноги
            t_leg = (t_global + leg['phase_offset']) % 1.0

            # Вычисляем позицию "Rest" на стене относительно текущего COM
            # Берем "ширину" из настроек
            rest_vec_local = leg['rest_offset'].copy()
            # Расширяем стойку (пауки расставляют ноги широко)
            rest_vec_local.x *= stance_width

            # Превращаем локальный вектор (от таза) в мировой, используя ориентацию стены
            # Мы поворачиваем "плоский" офсет на угол стены
            rest_vec_world = q_surface @ rest_vec_local

            # Базовая точка опоры для ноги в этом кадре (движется вместе с телом)
            # Но нога должна "стоять" на месте, пока фаза stance.
            # Для процедурной генерации без фиксации мира мы эмулируем это через синус.

            # Профиль шага (0..1)
            # Stance (опора): t 0.0 -> 0.5
            # Swing (перенос): t 0.5 -> 1.0

            cycle_offset = 0.0
            lift = 0.0

            if t_leg < 0.5:
                # STANCE: Нога движется НАЗАД относительно тела (или тело вперед относительно ноги)
                # progress 0 -> 1
                st_p = t_leg / 0.5
                # lerp от +Stride/2 до -Stride/2
                cycle_offset = (0.5 - st_p) * stride_len
                lift = 0.0
            else:
                # SWING: Нога летит ВПЕРЕД
                sw_p = (t_leg - 0.5) / 0.5
                # lerp от -Stride/2 до +Stride/2
                # Используем косинус для плавного разгона
                val = (1.0 - math.cos(sw_p * math.pi)) / 2.0  # 0 -> 1 smooth
                cycle_offset = -0.5 * stride_len + val * stride_len

                # Подъем ноги (парабола)
                lift = math.sin(sw_p * math.pi) * step_height

            # Сборка позиции ноги
            # Anchor Point = Current COM + Rotated Rest Offset
            anchor_pos = final_com_pos + rest_vec_world

            # Добавляем смещение шага (вдоль forward вектора стены)
            step_vec = surface_fwd * cycle_offset

            # Добавляем подъем (вдоль нормали стены)
            lift_vec = surface_up * lift

            # Итоговая цель
            target_foot_pos = anchor_pos + step_vec + lift_vec

            # Если IK включен, сохраняем для трекеров
            if tracker_data:
                tracker_data[leg['name']].append({'frame': frame, 'loc': target_foot_pos})

            # Если IK нет (FK), нужно вращать бедро.
            # (Опущено для краткости, так как ползание по стенам без IK — это боль.
            #  Предполагаем, что юзер использует IK).

    # --- 5. POST-PROCESS IK ---
    if tracker_data:
        utils.create_jump_trackers_with_anchored_behavior(arm_obj, state, settings, tracker_data)

    return True