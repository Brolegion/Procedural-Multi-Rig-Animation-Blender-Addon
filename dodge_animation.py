# dodge_animation.py
import bpy
import math
import random
from mathutils import Vector, Quaternion, Matrix
from . import utils
from .noise_provider import get_default_noise_provider

# ==============================================================================
# 1. ДИНАМИКА ВТОРОГО ПОРЯДКА (Для сочности)
# ==============================================================================


# ==============================================================================
# 2. ВСПОМОГАТЕЛЬНАЯ ЛОГИКА
# ==============================================================================
def get_dodge_bones(arm_obj, settings):
    """Сбор костей из списков UI или автоопределение"""
    primary = []
    secondary = []
    
    # 1. Пробуем взять из UI списков
    if settings.use_dodge_primary:
        for item in settings.dodge_bones_primary:
            if item.bone_name in arm_obj.pose.bones:
                primary.append((item.bone_name, item.weight))
                
    if settings.use_dodge_secondary:
        for item in settings.dodge_bones_secondary:
            if item.bone_name in arm_obj.pose.bones:
                secondary.append((item.bone_name, item.weight))
                
    # 2. Если списки пусты (первый запуск), делаем авто-фоллбэк
    if not primary:
        center = utils.choose_center_bone(arm_obj)
        if center: primary.append((center, 1.0))
        # Можно добавить автопоиск спины, если нужно
        
    return primary, secondary

def calculate_dodge_vector(state, settings):
    """Расчет вектора уклонения на основе угрозы"""
    threat = settings.threat_direction
    dtype = settings.dodge_type
    
    fw = state['fw_world']
    up = Vector((0,0,1))
    right = fw.cross(up).normalized()
    
    dodge_dir = Vector((0,0,0))
    
    # Логика: Куда прыгаем?
    if dtype == 'SIDESTEP':
        # Если угроза спереди -> прыгаем влево или вправо
        if threat == 'FRONT':
            side = 1 if random.random() > 0.5 else -1
            dodge_dir = right * side
        # Если угроза слева -> прыгаем вправо
        elif threat == 'LEFT':
            dodge_dir = right
        elif threat == 'RIGHT':
            dodge_dir = -right
        elif threat == 'BACK':
            # От угрозы сзади sidestep тоже работает
            side = 1 if random.random() > 0.5 else -1
            dodge_dir = right * side
            
    elif dtype == 'BACKSTEP':
        if threat == 'FRONT':
            dodge_dir = -fw # Назад
        elif threat == 'BACK':
            dodge_dir = fw # Вперед (убегаем)
        else:
            dodge_dir = -fw # Дефолт
            
    elif dtype == 'DUCK':
        dodge_dir = Vector((0,0,-0.8)) # Вниз
        if threat == 'FRONT': dodge_dir += -fw * 0.2
        
    elif dtype == 'ROLL':
        # Кувырок обычно вперед-вбок
        side = 1 if random.random() > 0.5 else -1
        dodge_dir = (fw + right * side).normalized()

    return dodge_dir

# ==============================================================================
# 3. ГЕНЕРАТОР
# ==============================================================================
def generate_dodge_animation(arm_obj, settings):
    # 1. Параметры времени
    fs = int(settings.frame_start)
    fe = int(settings.frame_end) # Добавил fe для init_walk_state
    duration = settings.dodge_duration
    recovery = settings.dodge_recovery
    total_frames = duration + recovery
    
    # 2. Инициализация (находим ноги, оси)
    # Заменяем старый вызов init_walk_state на этот (как в jump_animation):
    left_leg = getattr(settings, "left_leg_name", "")
    right_leg = getattr(settings, "right_leg_name", "")
    
    try:
        state = utils.init_walk_state(
            arm_obj,
            left_leg if left_leg else None,    # ПЕРЕДАЕМ None вместо "", чтобы сработал автопоиск
            right_leg if right_leg else None,
            settings,
            fs,
            fe,
            preserve_pose=True
        )
    except Exception as e:
        print(f"[PW DODGE] Init failed: {e}")
        return False

    pelvis_start_world = state['rest_pelvis_world'].copy()


    # 3. Подготовка физики (настройки из UI!)
    f_freq = getattr(settings, "dodge_spring_freq", 2.5)
    f_damp = getattr(settings, "dodge_spring_damp", 0.6)
    f_resp = getattr(settings, "dodge_overshoot", 0.2) * 2.0 # Overshoot влияет на response
    # Вставьте это сразу после создания dyn_com
    overshoot_frames = int(duration * getattr(settings, "dodge_overshoot", 0.2))
    
    dyn_com = utils.SecondOrderDynamics(f_freq, f_damp, f_resp, Vector((0,0,0)))
    
    # Шум
    np = get_default_noise_provider(seed=settings.dodge_noise_seed)
    
    # Вектор движения
    move_dir = calculate_dodge_vector(state, settings)
    dist = settings.dodge_distance
    
    # Кости
    prim_bones, sec_bones = get_dodge_bones(arm_obj, settings)
    
    # Snapshots (Relative Deltas)
    original_poses = {}
    for pb in arm_obj.pose.bones:
        original_poses[pb.name] = {
            'location': pb.location.copy(),
            'rotation_quaternion': pb.rotation_quaternion.copy(),
            'matrix_basis': pb.matrix_basis.copy()
        }

    # IK Data
    tracker_data = {state['left_name']: [], state['right_name']: []}
    if settings.dodge_sticky_feet:
        lf_pos = utils.get_current_foot_position(arm_obj, state['left_name'], settings)
        rf_pos = utils.get_current_foot_position(arm_obj, state['right_name'], settings)

    # 4. ЦИКЛ
    dt = 1.0 / bpy.context.scene.render.fps
    
    for i in range(total_frames + 1):
        frame = fs + i
        bpy.context.scene.frame_set(frame)
        
        # КРИТИЧЕСКИ ВАЖНО: устанавливаем current_frame в state
        state['current_frame'] = frame
        
        t = i / duration if duration > 0 else 0.0
        
        # Целевая точка (Target)
        # Anticipation: Сначала чуть назад
        anticip = settings.dodge_anticipation
        if i < duration:
            if t < anticip:
                # Отход назад (противоположно движению)
                target = move_dir * dist * -0.1 * (t/anticip)
            else:
                # Основное движение
                target = move_dir * dist
        else:
            # Recovery: возвращаемся в 0
            target = Vector((0,0,0))
            
        # Физический апдейт
        phys_offset = dyn_com.update(dt, target)
        
        # Приседание (Crouch)
        crouch_val = 0.0
        if i < duration:
            # Синус во время движения
            crouch_val = settings.dodge_crouch * math.sin(((t - anticip)/(1-anticip)) * math.pi) 
            crouch_val = max(0, crouch_val)
            
        # Вертикальный вектор вниз
        up = Vector((0,0,1))
        # Итоговое смещение COM
        total_offset = phys_offset - (up * crouch_val)
        
        # Шум
        noise = Vector((
            np.noise1d(i*0.2), 
            np.noise1d(i*0.2 + 100), 
            np.noise1d(i*0.2 + 200)
        )) * settings.dodge_noise_amount
        
    # Применение движения (КАК В JUMP_ANIMATION)
        target_pelvis_world = pelvis_start_world + total_offset + noise
        delta_pelvis = target_pelvis_world - state['pelvis_world_current']
        
        print(f"[DODGE] Frame {i}: target={target.length:.3f}, offset={total_offset.length:.3f}, delta={delta_pelvis.length:.3f}")
        
        # Используем apply_center_motion как в jump_animation
        utils.apply_center_motion(state, settings, arm_obj, delta_pelvis)
        
        # ОБНОВЛЯЕМ состояние после применения
        state['pelvis_world_current'] = target_pelvis_world
        
        # Вторичные кости (Наклон / Lag)
        # Primary (Spine) -> Наклон в сторону движения (Lean) или против (Counter-Balance)
        # Для реализма: ноги на месте -> таз ушел влево -> корпус наклоняется вправо (Counter)
        if settings.dodge_counter_rot:
            # Ось вращения перпендикулярна движению
            rot_axis = move_dir.cross(up).normalized()
            # Угол зависит от скорости (производная смещения) или просто смещения
            tilt_angle = -phys_offset.length * settings.dodge_tilt * 0.2
            # Знак зависит от направления
            
            for bname, w in prim_bones:
                pb = arm_obj.pose.bones.get(bname)
                if pb:
                    # Применяем наклон относительно базы
                    q = Quaternion(rot_axis, tilt_angle * w)
                    # Hacky apply via basis to be robust
                    base = original_poses[bname]
                    # Вращение в мировых осях (примерно)
                    # Для точности лучше переводить axis в local space, но для скорости:
                    pb.rotation_quaternion = pb.rotation_quaternion @ q
                    pb.keyframe_insert("rotation_quaternion", frame=frame)

        # Secondary (Arms/Head) -> Drag (отставание)
        for bname, w in sec_bones:
            pb = arm_obj.pose.bones.get(bname)
            if pb:
                # Отставание = поворот против направления движения
                drag_axis = phys_offset.cross(up).normalized()
                drag_angle = phys_offset.length * 0.1 * w
                if i > duration: drag_angle *= -0.5 # Overshoot back
                
                q = Quaternion(drag_axis, -drag_angle)
                pb.rotation_quaternion = pb.rotation_quaternion @ q
                pb.keyframe_insert("rotation_quaternion", frame=frame)

        # Sticky Feet
# ... (конец вашего цикла)
        if settings.dodge_sticky_feet and settings.use_ik:
            tracker_data[state['left_name']].append({'frame': frame, 'loc': lf_pos})
            tracker_data[state['right_name']].append({'frame': frame, 'loc': rf_pos})

    # === ВСТАВЛЯТЬ СЮДА (СРАЗУ ПОСЛЕ ВЫХОДА ИЗ ЦИКЛА FOR) ===
    
    recovery_duration = int(getattr(settings, "dodge_recovery_time", 10))
    if recovery_duration > 0:
        last_sim_frame = fs + duration + overshoot_frames
        
        for i in range(1, recovery_duration + 1):
            f = last_sim_frame + i
            # Фактор смешивания (от 0 до 1)
            t = i / float(recovery_duration)
            # Плавное замедление (Ease-Out)
            factor = 1.0 - pow(1.0 - t, 3) 
            
            bpy.context.scene.frame_set(f)
            
            for bname, data in original_poses.items():
                pb = arm_obj.pose.bones.get(bname)
                if not pb: continue
                
                # Плавно интерполируем к исходной позе
                pb.location = pb.location.lerp(data['location'], factor)
                pb.rotation_quaternion = pb.rotation_quaternion.slerp(data['rotation_quaternion'], factor)
                
                pb.keyframe_insert("location", frame=f)
                pb.keyframe_insert("rotation_quaternion", frame=f)

            # ОЧЕНЬ ВАЖНО: продолжаем держать ноги приклеенными во время возврата!
            if settings.dodge_sticky_feet and settings.use_ik:
                tracker_data[state['left_name']].append({'frame': f, 'loc': lf_pos})
                tracker_data[state['right_name']].append({'frame': f, 'loc': rf_pos})

    # === КОНЕЦ ВСТАВКИ ===

    # Post-process IK
    if settings.dodge_sticky_feet and settings.use_ik:
        utils.create_jump_trackers_with_anchored_behavior(arm_obj, state, settings, tracker_data)

    return True