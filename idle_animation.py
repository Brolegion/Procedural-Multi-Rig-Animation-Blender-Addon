import bpy
import math
import random
from mathutils import Vector, Quaternion, Euler
from . import utils
from . import noise_provider  # Используем, если есть, иначе fallback внутри

def generate_complex_idle(arm_obj, settings):
    """
    Генерирует "Универсальный Idle" на основе текущей позы.
    
    Философия:
    1. Base Pose Preservation: Мы не ломаем позу, которую выставил аниматор.
    2. Composite Oscillator: Дыхание + Баланс (Лиссажу) + Шум.
    3. Kinetic Chain Lag: Энергия идет от таза к голове с задержкой.
    """

    # -------------------------------------------------------------------------
    # 1. ИНИЦИАЛИЗАЦИЯ И ПАРАМЕТРЫ
    # -------------------------------------------------------------------------
    frame_start = int(getattr(settings, "frame_start", 1))
    frame_end = int(getattr(settings, "frame_end", 60))
    total_frames = frame_end - frame_start + 1
    
    # Инициализация состояния (находит ноги, направление и т.д.)
    try:
        state = utils.init_walk_state(arm_obj, settings.left_leg_name, settings.right_leg_name, settings, frame_start, frame_end, preserve_pose=True)
    except Exception as e:
        print(f"[PW IDLE] Init failed: {e}")
        return False

    # Параметры (с фоллбэками)
    amp_breath = getattr(settings, "idle_amp", 0.05)       # Вертикальное дыхание
    freq = getattr(settings, "idle_freq", 1.0)             # Общая скорость
    rot_amp = math.radians(getattr(settings, "idle_rot_deg", 2.0)) # Сила вращения позвоночника
    noise_amount = getattr(settings, "noise_amount", 0.0)  # Сила дрожания
    spine_lag = 0.15  # Задержка волны (чем больше, тем более "желейный" персонаж)

    # Определяем цепочки
    spine_root = getattr(settings, "spine_root_bone", "spine")
    spine_end = getattr(settings, "spine_end_bone", "head")
    spine_chain = utils.find_spine_chain(arm_obj, spine_root, spine_end)
    
    # --- СОХРАНЕНИЕ БАЗОВОЙ ПОЗЫ (SNAPSHOT) ---
    # Это критически важно для работы поверх любой позы (сидя, стоя, в боевой стойке)
    base_transforms = {}
    pose_bones = arm_obj.pose.bones
    for pb in pose_bones:
        base_transforms[pb.name] = {
            'loc': pb.location.copy(),
            'rot': pb.rotation_quaternion.copy() if pb.rotation_mode == 'QUATERNION' else pb.rotation_euler.copy(),
            # Scale можно добавить, если хотим эффект "раздувания" при дыхании
        }

    # Подготовка IK (пиннинг стоп)
    left_foot_pos = utils.get_current_foot_position(arm_obj, state['left_name'], settings)
    right_foot_pos = utils.get_current_foot_position(arm_obj, state['right_name'], settings)
    tracker_data = {state['left_name']: [], state['right_name']: []}

    # Инициализация шума (если модуль доступен)
    noise_gen = None
    try:
        noise_gen = noise_provider.NoiseProvider(seed=random.randint(0, 1000))
    except:
        pass # Fallback to math.sin/random

    print(f"[PW IDLE] Generated for {arm_obj.name}. Chains: Spine={len(spine_chain)}")

    # -------------------------------------------------------------------------
    # 2. ОСНОВНОЙ ЦИКЛ (PHYSICS LOOP)
    # -------------------------------------------------------------------------
    for i in range(total_frames):
        frame = frame_start + i
        
        # Нормализованное время (фаза цикла 0..2PI)
        # Умножаем на freq. 
        t_cycle = (i / total_frames) * (2.0 * math.pi) * max(0.1, freq)
        # Абсолютное время для шума (чтобы он не "заедал" в коротких циклах)
        t_abs = i * 0.1 * freq 

        # --- A. UNIVERSAL CORE MOTION (COM) ---
        # Биология: Живое существо не просто качается влево-вправо. 
        # Центр масс описывает "восьмерку" (кривая Лиссажу) для поддержания равновесия.
        
        # 1. Дыхание (Вертикаль - Z)
        # sin(t) - классический вдох-выдох
        breath_z = math.sin(t_cycle) * amp_breath

        # 2. Баланс (Горизонталь - X/Y)
        # Кривая Лиссажу: x = A sin(at + d), y = B sin(bt)
        # Делаем sway_x медленнее (0.5), чтобы тело "плавало"
        sway_x = math.sin(t_cycle * 0.5) * (amp_breath * 0.6) 
        sway_y = math.cos(t_cycle * 0.5) * (amp_breath * 0.2) # Небольшое движение вперед-назад

        # 3. Шум (Micro-Jitter)
        jitter = Vector((0,0,0))
        if noise_amount > 0:
            if noise_gen:
                # Гладкий 1D шум
                nx = noise_gen.noise1d(t_abs, scale=10)
                nz = noise_gen.noise1d(t_abs + 100, scale=10)
            else:
                # Простой рандом (менее красиво)
                nx = (random.random() - 0.5)
                nz = (random.random() - 0.5)
            
            jitter = Vector((nx, sway_y * 0.1, nz)) * (noise_amount * 0.1)

        # Формируем итоговый вектор смещения таза в МИРОВЫХ осях
        fw = state['fw_world']
        up = Vector((0,0,1))
        right = fw.cross(up).normalized()

        # Sway (X) + Breath (Z) + Surge (Y/Forward)
        world_delta = (right * (sway_x + jitter.x)) + \
                      (up * (breath_z + jitter.z)) + \
                      (fw * (sway_y + jitter.y))

        # Применяем к корневой кости/объекту
        utils.apply_center_motion(state, settings, arm_obj, world_delta)

        # --- B. SPINE KINETICS (LAG & COUNTER-ROTATION) ---
        # Биология: Когда таз идет влево, позвоночник изгибается вправо, 
        # чтобы голова осталась на месте (стабилизация взгляда).
        
        for idx, b_name in enumerate(spine_chain):
            pb = pose_bones.get(b_name)
            if not pb: continue
            
            # Рассчитываем фазовый сдвиг (Lag)
            # Нижние позвонки двигаются раньше, верхние (голова) - позже.
            phase = t_cycle - (idx * spine_lag)

            # Вращение
            # X - Наклон вперед/назад (следует за дыханием)
            # Y/Z - Боковой наклон (компенсирует Sway)
            
            rot_x = math.sin(phase) * rot_amp
            # Инвертируем sway для позвоночника (Counter-Balance)
            # Коэфф (idx / len) усиливает эффект к голове (хлыст)
            stiffness = (idx + 1) / len(spine_chain) 
            rot_sway = -math.sin(phase * 0.5) * rot_amp * stiffness

            # Накладываем на Base Pose
            base_rot = base_transforms[b_name]['rot']
            
            # Создаем локальную дельту вращения
            # Примечание: оси зависят от ориентации костей. 
            # Для универсальности лучше использовать axis-angle или предполагать Y-up/Z-fwd bone space,
            # но для Blender rigs чаще всего X - это pitch, Z/Y - roll/yaw.
            # Попробуем универсальный эвристический подход:
            
            if pb.rotation_mode == 'QUATERNION':
                # Предполагаем стандартную ориентацию кости (X - pitch, Y - roll/twist)
                q_breath = Quaternion((1, 0, 0), rot_x)
                q_sway = Quaternion((0, 0, 1), rot_sway) # Обычно Z или Y для бокового наклона
                
                # Добавляем немного шума в поворот головы
                if idx == len(spine_chain) - 1 and noise_amount > 0:
                     q_noise = Quaternion((0,1,0), (random.random()-0.5)*noise_amount*0.2)
                     q_sway = q_sway @ q_noise

                pb.rotation_quaternion = base_rot @ q_breath @ q_sway
                pb.keyframe_insert("rotation_quaternion", frame=frame)
            else:
                # Euler Fallback
                pb.rotation_euler.x = base_rot.x + rot_x
                pb.rotation_euler.z = base_rot.z + rot_sway
                pb.keyframe_insert("rotation_euler", frame=frame)

        # --- C. APPENDAGES (ARMS/TAILS) - SECONDARY MOTION ---
        # Если есть руки, они должны "запаздывать" (Drag)
        if getattr(settings, "use_arm_animation", True):
             # Эвристика: кости, которые не спина и не ноги - это руки/хвосты
            ignore_bones = set(spine_chain) | {state.get('left_leg_name'), state.get('right_leg_name')}
            
            # Упрощенная логика: ищем только ключевые кости рук, если они заданы в state
            # или просто ищем по именам (arm, hand)
            for side in ['left', 'right']:
                arm_name = state.get(f'{side}_arm_name') # Из utils.init_walk_state
                if arm_name and arm_name in pose_bones:
                    pb = pose_bones[arm_name]
                    # Руки дышат в противофазе (грудь вверх -> плечи вниз/назад)
                    arm_t = t_cycle - 0.5 
                    d_rot = math.sin(arm_t) * 0.03
                    
                    base_rot = base_transforms[arm_name]['rot']
                    if pb.rotation_mode == 'QUATERNION':
                        # Вращаем чуть-чуть по Z (обычно опускание рук)
                        pb.rotation_quaternion = base_rot @ Quaternion((0,0,1), d_rot)
                        pb.keyframe_insert("rotation_quaternion", frame=frame)

        # --- D. IK PINNING (FIX FEET) ---
        if getattr(settings, "use_ik", False):
            tracker_data[state['left_name']].append({'frame': frame, 'loc': left_foot_pos})
            tracker_data[state['right_name']].append({'frame': frame, 'loc': right_foot_pos})

    # -------------------------------------------------------------------------
    # 3. POST-PROCESSING (IK)
    # -------------------------------------------------------------------------
    if getattr(settings, "use_ik", False):
        utils.create_jump_trackers_with_anchored_behavior(arm_obj, state, settings, tracker_data)
        
    return True