import bpy
import math
import random
from mathutils import Vector, Quaternion, Matrix
from . import utils
from .noise_provider import get_default_noise_provider


# ==============================================================================
# ЛОГИКА АНИМАЦИИ ПАНИКИ
# ==============================================================================

def generate_panic_animation(arm_obj, settings):
    """
    Генерирует процедурную анимацию паники/испуга.
    3 Фазы: Shock (Испуг), Tremble (Дрожь), Recovery (Одышка/Возврат).
    2 Вариации: COWER (Сжаться) и SPOOKED (Оцепенение/Активная паника).
    """

    # --- 1. ПАРАМЕТРЫ И ИНИЦИАЛИЗАЦИЯ ---
    fs = int(settings.frame_start)
    duration = int(getattr(settings, "panic_duration", 40))
    recovery_len = int(getattr(settings, "panic_recovery_duration", 20))
    total_frames = duration + recovery_len

    # Вариация: 'COWER' (сжаться) или 'SPOOKED' (оцепенение/оглядывание)
    # Если в settings нет свойства, берем случайное или дефолт
    variation = getattr(settings, "panic_variation", 'COWER')

    intensity = getattr(settings, "panic_intensity", 1.0)

    # Инициализация состояния (как в dodge/idle)
    try:
        # preserve_pose=True важно, чтобы старт был с текущей позы
        state = utils.init_walk_state(
            arm_obj,
            settings.left_leg_name,
            settings.right_leg_name,
            settings,
            fs,
            fs + total_frames,
            preserve_pose=True
        )
    except Exception as e:
        print(f"[PW PANIC] Init failed: {e}")
        return False

    # Запоминаем стартовую позицию мира для корректной дельты
    bpy.context.scene.frame_set(fs)
    bpy.context.view_layer.update()
    initial_com_pos = state['pelvis_world_current'].copy()

    # Убедимся, что current_frame установлен
    state['current_frame'] = fs


    # --- 2. ФИЗИКА (Second Order Dynamics) ---
    # Для испуга нужна "жесткая" пружина: быстрая частота, малое затухание в начале
    f = 4.0  # Скорость реакции
    z = 0.5  # Damping (0.5 = slight overshoot, тряска)
    r = 2.0  # Initial response (агрессивный старт)

    dt = 1.0 / bpy.context.scene.render.fps
    # Используем нулевое начальное состояние для плавного старта
    dyn_pos = utils.SecondOrderDynamics(f, z, r, Vector((0, 0, 0)))
    dyn_rot = utils.SecondOrderDynamics(3.0, 0.6, 1.0, Vector((0, 0, 0)))

    # "Прогреваем" системы нулевыми значениями для плавного старта
    for _ in range(5):
        dyn_pos.update(dt, Vector((0, 0, 0)))
        dyn_rot.update(dt, Vector((0, 0, 0)))

    # Шум для дрожи (Tremble)
    np_shake = get_default_noise_provider(seed=random.randint(0, 999))
    # Шум для головы (оглядывание)
    np_look = get_default_noise_provider(seed=random.randint(0, 999))

    # Снапшоты оригинальных ротаций для возврата
    original_poses = {}
    for pb in arm_obj.pose.bones:
        original_poses[pb.name] = {
            'loc': pb.location.copy(),
            'rot': pb.rotation_quaternion.copy(),
            'scale': pb.scale.copy()
        }

    # --- 3. ЦИКЛ ГЕНЕРАЦИИ ---

    # Настройки IK (если есть) - фиксируем ноги
    tracker_data = {state['left_name']: [], state['right_name']: []}
    if getattr(settings, "use_ik", False):
        lf_pos = utils.get_current_foot_position(arm_obj, state['left_name'], settings)
        rf_pos = utils.get_current_foot_position(arm_obj, state['right_name'], settings)

    for i in range(total_frames + 1):
        frame = fs + i
        bpy.context.scene.frame_set(frame)

        # Нормализованное время
        if i < duration:
            # Фаза 1 и 2: Shock + Tremble
            phase_t = i / duration
            is_recovery = False
        else:
            # Фаза 3: Recovery
            phase_t = (i - duration) / recovery_len
            is_recovery = True

        # ==========================
        # А. РАСЧЕТ ЦЕЛЕВОЙ ПОЗЫ (TARGET)
        # ==========================

        target_offset = Vector((0, 0, 0))
        target_spine_bend = Vector((0, 0, 0))  # Euler-like vector for spine
        head_look_target = Vector((0, 0, 0))

        if not is_recovery:
            # --- ВАРИАЦИЯ 1: COWER (Сжаться) ---
            if variation == 'COWER':
                # Резко вниз (-Z) и чуть назад (-Y)
                target_offset = Vector((0, -0.2, -0.3)) * intensity
                # Сгибаем позвоночник вперед (X)
                target_spine_bend = Vector((0.4, 0, 0)) * intensity

                # --- ВАРИАЦИЯ 2: SPOOKED (Оцепенение/Алерт) ---
            elif variation == 'SPOOKED':
                # Чуть вверх (+Z) или на месте, корпус назад (-Y) (отшатывание)
                target_offset = Vector((0, -0.15, 0.05)) * intensity
                # Позвоночник выпрямлен или чуть назад
                target_spine_bend = Vector((-0.1, 0, 0)) * intensity

                # Хаотичное оглядывание (только в SPOOKED активно)
                # Генерируем резкие скачки взгляда
                look_noise_x = np_look.noise1d(i * 0.15)
                look_noise_z = np_look.noise1d(i * 0.15 + 100)
                # Если шум превышает порог, делаем "дерг" головой
                if abs(look_noise_x) > 0.3:
                    head_look_target = Vector((look_noise_x, 0, look_noise_z)) * 0.8 * intensity

        else:
            # Recovery: цель - ноль (исходная поза)
            target_offset = Vector((0, 0, 0))
            target_spine_bend = Vector((0, 0, 0))
            head_look_target = Vector((0, 0, 0))

        # ==========================
        # Б. ФИЗИКА И ДРОЖЬ
        # ==========================

        # 1. Применяем пружину к основной позе
        phys_offset = dyn_pos.update(dt, target_offset)
        phys_spine = dyn_rot.update(dt, target_spine_bend)

        # 2. Добавляем ВЫСОКОЧАСТОТНУЮ ДРОЖЬ (Tremble)
        # Дрожь сильнее всего в середине фазы Shock и начале Sustain
        if not is_recovery:
            tremble_amp = 0.03 * intensity
            tremble_vec = Vector((
                np_shake.noise1d(i * 0.8),  # Очень быстро
                np_shake.noise1d(i * 0.8 + 33),
                np_shake.noise1d(i * 0.8 + 66)
            )) * tremble_amp
        else:
            # В фазе восстановления дрожь затухает
            tremble_amp = 0.03 * intensity * (1.0 - phase_t)
            tremble_vec = Vector((
                np_shake.noise1d(i * 0.8),
                np_shake.noise1d(i * 0.8 + 33),
                np_shake.noise1d(i * 0.8 + 66)
            )) * tremble_amp

        # 3. Дыхание (тяжелое, быстрое)
        # В recovery оно замедляется
        breath_freq = 0.6 if not is_recovery else 0.3
        breath_amp = 0.05 if not is_recovery else 0.08 * (1.0 - phase_t)
        breath_val = math.sin(i * breath_freq) * breath_amp

        # ==========================
        # В. ПРИМЕНЕНИЕ К АРМАТУРЕ
        # ==========================

        # --- 1. ТАЗ (COM) ---
        total_com_offset = phys_offset + tremble_vec
        # Вертикальное дыхание
        total_com_offset.z += breath_val * 0.2

        # Расчет позиции в мире (как в Dodge fix)
        target_pelvis_world = initial_com_pos + total_com_offset
        delta_move = target_pelvis_world - state['pelvis_world_current']

        # Устанавливаем current_frame перед применением
        state['current_frame'] = frame

        utils.apply_center_motion(state, settings, arm_obj, delta_move)
        state['pelvis_world_current'] = target_pelvis_world.copy()

        # --- 2. ПОЗВОНОЧНИК (SPINE) ---
        # Если spine_chain не инициализирован, создаем его
        if 'spine_chain' not in state:
            spine_bones = []
            # Автоматически ищем кости позвоночника по ключевым словам
            for bone in arm_obj.data.bones:
                name_lower = bone.name.lower()
                if any(keyword in name_lower for keyword in ['spine', 'chest', 'torso', 'back']):
                    spine_bones.append(bone.name)

            # Сортируем по иерархии (от таза к голове)
            def get_bone_depth(bone_name):
                bone = arm_obj.data.bones.get(bone_name)
                depth = 0
                while bone.parent:
                    depth += 1
                    bone = bone.parent
                return depth

            spine_bones.sort(key=get_bone_depth)
            state['spine_chain'] = spine_bones
            print(f"[PANIC] Found spine bones: {spine_bones}")

        spine_bones = state.get('spine_chain', [])

        if spine_bones:
            num_bones = len(spine_bones)
            # 1. Базовый наклон (от физики) распределяем равномерно по всей длине
            # phys_spine - это вектор Эйлера (x=bend, y=twist, z=lean)
            # Делим на кол-во костей, чтобы наклон тела сложился из суммы наклонов позвонков
            dist_bend_x = phys_spine.x / num_bones
            dist_lean_z = phys_spine.z / num_bones

            # Дыхание тоже распределяем
            dist_breath = breath_val / num_bones

            for idx, bname in enumerate(spine_bones):
                pb = arm_obj.pose.bones.get(bname)
                if not pb: continue

                # Берем базу (Rest Pose)
                base_rot = original_poses[bname]['rot']

                # 2. ШУМ (Дрожь) - рассчитываем для КАЖДОЙ кости отдельно!
                # Сдвигаем фазу шума по индексу кости (idx * 10), чтобы дрожь шла волной
                # Умножаем частоту на 2.0 для скорости
                noise_idx = i * 0.9 + (idx * 5)

                # Локальная дрожь позвонка
                bone_shake_x = np_shake.noise1d(noise_idx) * tremble_amp * 3.0  # Усиливаем!
                bone_shake_z = np_shake.noise1d(noise_idx + 100) * tremble_amp * 2.0

                # 3. Собираем итоговое вращение
                # X: Наклон вперед (physics) + Дыхание + Дрожь
                rot_x = dist_bend_x + dist_breath + bone_shake_x

                # Z: Боковой наклон (physics) + Дрожь
                rot_z = dist_lean_z + bone_shake_z

                # Создаем кватернион отклонения
                # Предполагаем X - сгиб, Z - наклон (стандарт Rigify/Blender)
                q_bend = Quaternion(Vector((1, 0, 0)), rot_x)
                q_lean = Quaternion(Vector((0, 0, 1)), rot_z)

                # Применяем: Base -> Bend -> Lean
                pb.rotation_quaternion = base_rot @ q_bend @ q_lean
                pb.keyframe_insert("rotation_quaternion", frame=frame)

        # --- 3. ГОЛОВА (HEAD) ---
        # Дрожь + Оглядывание (если Spooked) + Втягивание (если Cower)
        # Если head_name не инициализирован, ищем его
        if 'head_name' not in state:
            head_name = None
            for bone in arm_obj.data.bones:
                name_lower = bone.name.lower()
                if any(keyword in name_lower for keyword in ['head', 'neck', 'skull']):
                    head_name = bone.name
                    break
            if not head_name:
                # Если не нашли, берем последнюю кость в spine_chain или первую с child
                if spine_bones:
                    last_spine = arm_obj.data.bones[spine_bones[-1]]
                    if last_spine.children:
                        head_name = last_spine.children[0].name
            state['head_name'] = head_name
            print(f"[PANIC] Head bone: {head_name}")

        head_name = state.get('head_name')
        if head_name:
            pb_head = arm_obj.pose.bones.get(head_name)
            if pb_head:
                base_rot = original_poses[head_name]['rot']

                # Базовая дрожь головы
                head_shake = Quaternion(Vector((0, 0, 1)), tremble_vec.x * 3.0) @ \
                             Quaternion(Vector((1, 0, 0)), tremble_vec.z * 3.0)

                # Логика взгляда
                look_q = Quaternion((1, 0, 0), 0)
                if variation == 'SPOOKED' and not is_recovery:
                    # Резкие повороты
                    look_q = Quaternion(Vector((0, 0, 1)), head_look_target.x) @ \
                             Quaternion(Vector((1, 0, 0)), head_look_target.z)
                elif variation == 'COWER' and not is_recovery:
                    # Втянуть голову (взгляд вниз)
                    look_q = Quaternion(Vector((1, 0, 0)), 0.5 * intensity)  # Вниз

                pb_head.rotation_quaternion = base_rot @ head_shake @ look_q
                pb_head.keyframe_insert("rotation_quaternion", frame=frame)

        # --- 4. РУКИ (ARMS) ---
        # Добавляем хаотичную дрожь рукам
        if 'left_arm_name' not in state or 'right_arm_name' not in state:
            left_arm = None
            right_arm = None

            for bone in arm_obj.data.bones:
                name_lower = bone.name.lower()
                if any(keyword in name_lower for keyword in ['arm', 'shoulder', 'clavicle']):
                    # Определяем сторону по X координате или имени
                    if 'left' in name_lower or 'l_' in name_lower or '.l' in name_lower:
                        left_arm = bone.name
                    elif 'right' in name_lower or 'r_' in name_lower or '.r' in name_lower:
                        right_arm = bone.name
                    elif bone.head_local.x > 0:  # Положительный X обычно правая сторона
                        right_arm = bone.name
                    else:
                        left_arm = bone.name

            state['left_arm_name'] = left_arm
            state['right_arm_name'] = right_arm
            print(f"[PANIC] Left arm: {left_arm}, Right arm: {right_arm}")
        for side in ['left', 'right']:
            arm_name = state.get(f'{side}_arm_name')
            if arm_name:
                pb_arm = arm_obj.pose.bones.get(arm_name)
                if pb_arm:
                    base_rot = original_poses[arm_name]['rot']

                    # Разная логика рук
                    arm_offset_q = Quaternion((1, 0, 0), 0)
                    if variation == 'COWER' and not is_recovery:
                        # Руки к лицу/голове (поднимаем вверх и сгибаем)
                        # Это грубое приближение, зависит от осей костей
                        # Допустим, локальная Z - это сгиб
                        # Здесь нужен тюнинг под конкретный риг, сделаем просто дрожь + подъем
                        lift = 0.3 * intensity
                        arm_offset_q = Quaternion(Vector((1, 0, 0)), lift)
                    elif variation == 'SPOOKED':
                        # Руки чуть в стороны (защита)
                        out = 0.1 * intensity
                        side_dir = 1 if side == 'left' else -1
                        arm_offset_q = Quaternion(Vector((0, 1, 0)), out * side_dir)

                    # Сильная дрожь в руках
                    arm_shake = Quaternion(Vector((1, 0, 0)),
                                           np_shake.noise1d(i * 0.9 + (10 if side == 'left' else 20)) * 0.1)

                    pb_arm.rotation_quaternion = base_rot @ arm_offset_q @ arm_shake
                    pb_arm.keyframe_insert("rotation_quaternion", frame=frame)

        # --- 5. IK FIX ---
        if getattr(settings, "use_ik", False):
            tracker_data[state['left_name']].append({'frame': frame, 'loc': lf_pos})
            tracker_data[state['right_name']].append({'frame': frame, 'loc': rf_pos})

    # --- 4. POST-PROCESS IK ---
    if getattr(settings, "use_ik", False):
        utils.create_jump_trackers_with_anchored_behavior(arm_obj, state, settings, tracker_data)

    return True