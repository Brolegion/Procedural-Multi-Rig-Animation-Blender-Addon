# simple_unit_anim_integrated.py - Восстановлено покачивание с поддержкой фаз

import bpy
import math
from mathutils import Vector, Euler, Quaternion
from .noise_provider import NoiseProvider
import random  # Не забудь добавить импорт в начале файла simple_unit_anim.py
from .utils import (
    clamp, ease_in_out_cubic, interpolate_spring,
    forward_vector_world, compute_phases, compute_weights, bezier_y_for_x,
    global_delta_to_bone_local, get_animation_target
)



def debug_print(msg):
    """Простая отладка"""
    print(f"[ANIM] {msg}")

def _scene_set_frame(f):
    """Установка кадра с fallback"""
    try:
        bpy.context.scene.frame_set(int(f))
    except Exception:
        bpy.context.scene.frame_current = int(f)

def _get_frame_bounds(settings):
    """Получение границ кадров"""
    fs = int(getattr(settings, "frame_start", bpy.context.scene.frame_start))
    fe = int(getattr(settings, "frame_end", bpy.context.scene.frame_end))
    if fe < fs:
        fe = fs + 24
    return fs, fe

def _get_forward_vector(obj, settings):
    """Универсальное получение вектора вперед с fallback"""
    try:
        fw = forward_vector_world(obj, settings, None)
        if fw.length < 1e-6:
            return Vector((0.0, 1.0, 0.0))
        return fw.normalized()
    except Exception as e:
        debug_print(f"Using fallback forward vector: {e}")
        return Vector((0.0, 1.0, 0.0))


def _remove_scale_fcurves(obj):
    """Быстро удаляет f-curves масштаба если они не нужны"""
    if not obj.animation_data or not obj.animation_data.action:
        return

    action = obj.animation_data.action
    scale_fcurves = [fc for fc in action.fcurves if '.scale' in fc.data_path]

    for fc in scale_fcurves:
        try:
            action.fcurves.remove(fc)
        except:
            pass

    if not action.fcurves:
        obj.animation_data.action = None

def _keyframe_object(obj, frame):
    """Прямое ключевание с Euler"""
    try:
        obj.keyframe_insert(data_path="location", frame=frame)
    except Exception:
        pass
    try:
        obj.keyframe_insert(data_path="rotation_euler", frame=frame)
    except Exception:
        pass
    try:
        obj.keyframe_insert(data_path="scale", frame=frame)
    except Exception:
        pass


def generate_idle(obj, settings, **kwargs):
    """Idle с улучшенным процедурным шумом - работает с арматурой и целевой костью"""
    try:
        debug_print(f"Enhanced Idle with noise for {obj.name}")

        # Получаем цель анимации
        target = get_animation_target(obj, settings)

        if obj.animation_data:
            obj.animation_data_clear()

        prev_mode = bpy.context.mode
        if prev_mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')

        frame_start, frame_end = _get_frame_bounds(settings)
        total = max(1, frame_end - frame_start + 1)

        # Параметры базовой анимации
        amp = kwargs.get('idle_amp', getattr(settings, "idle_amp", 0.1))
        freq = kwargs.get('idle_freq', getattr(settings, "idle_freq", 0.5))
        rot_deg = kwargs.get('idle_rot_deg', getattr(settings, "idle_rot_deg", 5.0))

        # Параметры шума
        use_noise = getattr(settings, "use_noise_idle", True)
        noise_seed = getattr(settings, "idle_noise_seed", 12345)
        noise_amp_vertical = getattr(settings, "idle_noise_amp_vertical", 0.08)
        noise_amp_horizontal = getattr(settings, "idle_noise_amp_horizontal", 0.04)
        noise_amp_rotation = getattr(settings, "idle_noise_amp_rotation", 3.0)
        noise_freq = getattr(settings, "idle_noise_frequency", 0.8)

        # Глобальный переключатель масштаба
        scale_enabled = bool(kwargs.get('allow_scale_changes', getattr(settings, "allow_scale_changes", True)))

        # Сохраняем исходные трансформации
        if isinstance(target, bpy.types.PoseBone):
            original_location = target.location.copy()
            original_rotation = target.rotation_quaternion.copy()
            original_scale = target.scale.copy()
        else:
            original_location = obj.location.copy()
            original_rotation = obj.rotation_euler.copy()
            original_scale = obj.scale.copy()

        # Инициализация генератора шума
        if use_noise:
            np = NoiseProvider(seed=noise_seed)
            debug_print(f"Using noise with seed: {noise_seed}")

        # Счетчик времени для шума
        time_counter = 0.0
        time_step = 0.05 * noise_freq

        for frame in range(frame_start, frame_end + 1):
            _scene_set_frame(frame)

            # Вычисляем фазы
            phase_center, _ = compute_phases(frame, frame_start, total, settings)

            # Базовое вертикальное колебание
            wobble = math.sin(phase_center)
            base_z = amp * wobble

            # Вычисляем шум
            if use_noise:
                time_counter += time_step

                noise_z = np.fbm1d(time_counter * 0.3, octaves=3) * noise_amp_vertical
                noise_x = np.fbm1d(time_counter * 0.4, octaves=2) * noise_amp_horizontal
                noise_y = np.fbm1d(time_counter * 0.35 + 10.0, octaves=2) * noise_amp_horizontal

                noise_rot_x = np.fbm1d(time_counter * 0.5) * math.radians(noise_amp_rotation)
                noise_rot_y = np.fbm1d(time_counter * 0.45 + 20.0) * math.radians(noise_amp_rotation * 0.3)
                noise_rot_z = np.fbm1d(time_counter * 0.6 + 30.0) * math.radians(noise_amp_rotation * 0.7)

                tremor = np.noise1d(time_counter * 8.0) * 0.003
                noise_x += tremor
                noise_y += tremor
            else:
                noise_x = noise_y = noise_z = 0.0
                noise_rot_x = noise_rot_y = noise_rot_z = 0.0

            # Масштаб (дыхание)
            breath_phase = phase_center * 0.3
            breath = 1.0 + math.sin(breath_phase) * 0.015

            # === ПРИМЕНЕНИЕ ТРАНСФОРМАЦИЙ ===
            if isinstance(target, bpy.types.PoseBone):
                # Для кости: вычисляем глобальные изменения
                delta_global_loc = Vector((noise_x, noise_y, base_z + noise_z))

                # Для вращения: создаем кватернион из углов Эйлера
                if use_noise:
                    # Используем шум для вращения
                    delta_global_rot_euler = Euler((noise_rot_x, noise_rot_y, noise_rot_z), 'XYZ')
                else:
                    # Базовое вращение без шума
                    tilt_angle = math.radians(rot_deg) * math.sin(phase_center * 1.3)
                    delta_global_rot_euler = Euler((tilt_angle, 0, tilt_angle * 0.5), 'XYZ')

                # Преобразуем в ось и угол
                delta_rot_quat = delta_global_rot_euler.to_quaternion()
                if delta_rot_quat.angle > 0.0001:
                    rot_axis, rot_angle = delta_rot_quat.to_axis_angle()
                else:
                    rot_axis, rot_angle = Vector((0, 0, 1)), 0.0

                # Преобразуем глобальные изменения в локальные для кости
                delta_local_loc, local_rot = global_delta_to_bone_local(
                    arm_obj=obj,
                    pose_bone=target,
                    delta_global_loc=delta_global_loc,
                    delta_global_rot_axis=rot_axis,
                    delta_global_rot_angle=rot_angle
                )

                # Применяем изменения
                target.location = original_location + delta_local_loc

                if local_rot is not None and local_rot.angle > 0.0001:
                    old_mode = target.rotation_mode
                    if old_mode != 'QUATERNION':
                        target.rotation_mode = 'QUATERNION'
                    target.rotation_quaternion = (local_rot @ original_rotation).normalized()
                    target.rotation_mode = old_mode
                else:
                    target.rotation_quaternion = original_rotation.copy()

                # Масштаб
                if scale_enabled:
                    target.scale = Vector((
                        original_scale.x * breath,
                        original_scale.y * breath,
                        original_scale.z * breath * 1.02
                    ))
                else:
                    target.scale = original_scale

                # Ключевые кадры
                target.keyframe_insert(data_path="location", frame=frame)
                target.keyframe_insert(data_path="rotation_quaternion", frame=frame)
                if scale_enabled:
                    target.keyframe_insert(data_path="scale", frame=frame)

            else:
                # Для обычного объекта
                obj.location = Vector((
                    original_location.x + noise_x,
                    original_location.y + noise_y,
                    original_location.z + base_z + noise_z
                ))

                if use_noise:
                    obj.rotation_euler.x = original_rotation.x + noise_rot_x
                    obj.rotation_euler.y = original_rotation.y + noise_rot_y
                    obj.rotation_euler.z = original_rotation.z + noise_rot_z
                else:
                    tilt_angle = math.radians(rot_deg) * math.sin(phase_center * 1.3)
                    obj.rotation_euler.x = original_rotation.x + tilt_angle
                    obj.rotation_euler.z = original_rotation.z + tilt_angle * 0.5

                if scale_enabled:
                    obj.scale = Vector((
                        original_scale.x * breath,
                        original_scale.y * breath,
                        original_scale.z * breath * 1.02
                    ))
                else:
                    obj.scale = original_scale

                _keyframe_object(obj, frame)

            depsgraph = bpy.context.evaluated_depsgraph_get()
            depsgraph.update()

        if prev_mode != 'OBJECT':
            bpy.ops.object.mode_set(mode=prev_mode)

        debug_print(
            f"Enhanced Idle animation completed. Noise: {use_noise}, Seed: {noise_seed if use_noise else 'N/A'}")
        return True

    except Exception as e:
        debug_print(f"Idle error: {e}")
        import traceback
        traceback.print_exc()
        return False

def generate_move(obj, settings, **kwargs):
    """Движение с мировыми координатами и фазовым боббингом"""
    try:
        debug_print(f"Move for {obj.name}")
        
        if obj.animation_data:
            obj.animation_data_clear()
        prev_mode = bpy.context.mode
        if prev_mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')
        
        frame_start, frame_end = _get_frame_bounds(settings)
        total = max(1, frame_end - frame_start + 1)
        
        distance = kwargs.get('distance', getattr(settings, "move_distance", 2.0))
        bob_amp = kwargs.get('bob_amp', getattr(settings, "move_bob_amp", 0.06))
        roll_deg = kwargs.get('roll_deg', getattr(settings, "move_roll_deg", 6.0))
        freq = kwargs.get('idle_freq', getattr(settings, "idle_freq", 0.5))
        
        original_location = obj.location.copy()
        original_rotation = obj.rotation_euler.copy()
        
        forward = _get_forward_vector(obj, settings).normalized()
        
        # Bezier контрольные точки для весов
        p1 = Vector((0.0, 0.0))
        p2 = Vector((1.0, 1.0))
        
        for frame in range(frame_start, frame_end + 1):
            _scene_set_frame(frame)
            progress = (frame - frame_start) / max(1, frame_end - frame_start)
            
            move_progress = ease_in_out_cubic(progress)
            current_pos = original_location + forward * (distance * move_progress)
            
            # Фазовый боббинг
            phase_center, _ = compute_phases(frame, frame_start, total, settings)
            bob = bob_amp * math.sin(phase_center)
            current_pos.z += bob
            obj.location = current_pos
            
            roll_angle = math.radians(roll_deg) * math.sin(phase_center)
            obj.rotation_euler.z = original_rotation.z + roll_angle
            
            _keyframe_object(obj, frame)
            depsgraph = bpy.context.evaluated_depsgraph_get()
            depsgraph.update()
        
        if prev_mode != 'OBJECT':
            bpy.ops.object.mode_set(mode=prev_mode)
        
        if obj.animation_data and obj.animation_data.action:
            debug_print(f"Keys created: {len(obj.animation_data.action.fcurves)} fcurves")
        else:
            debug_print("No animation_data or action created!")
        
        debug_print("Move animation completed")
        return True
        
    except Exception as e:
        debug_print(f"Move error: {e}")
        return False

def generate_turn(obj, settings, **kwargs):
    """Поворот с overshoot"""
    try:
        debug_print(f"Turn for {obj.name}")
        
        if obj.animation_data:
            obj.animation_data_clear()
        prev_mode = bpy.context.mode
        if prev_mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')
        
        frame_start, frame_end = _get_frame_bounds(settings)
        total = max(1, frame_end - frame_start + 1)
        
        angle_deg = kwargs.get('angle_deg', getattr(settings, "turn_angle", 90.0))
        angle_rad = math.radians(float(angle_deg))
        overshoot_deg = getattr(settings, "turn_overshoot_deg", 6.0)
        ease_frac = getattr(settings, "turn_ease", 0.85)
        
        original_rotation = obj.rotation_euler.copy()
        
        for frame in range(frame_start, frame_end + 1):
            _scene_set_frame(frame)
            progress = (frame - frame_start) / max(1, frame_end - frame_start)
            
            if progress < ease_frac:
                phase_progress = progress / ease_frac
                current_angle = angle_rad * ease_in_out_cubic(phase_progress) + math.radians(overshoot_deg) * phase_progress
            else:
                phase_progress = (progress - ease_frac) / (1.0 - ease_frac)
                overshoot_factor = 1.0 - ease_in_out_cubic(phase_progress)
                current_angle = angle_rad + math.radians(overshoot_deg) * overshoot_factor
            
            obj.rotation_euler.z = original_rotation.z + current_angle
            
            _keyframe_object(obj, frame)
            depsgraph = bpy.context.evaluated_depsgraph_get()
            depsgraph.update()
        
        if prev_mode != 'OBJECT':
            bpy.ops.object.mode_set(mode=prev_mode)
        
        if obj.animation_data and obj.animation_data.action:
            debug_print(f"Keys created: {len(obj.animation_data.action.fcurves)} fcurves")
        else:
            debug_print("No animation_data or action created!")
        
        debug_print("Turn animation completed")
        return True
        
    except Exception as e:
        debug_print(f"Turn error: {e}")
        return False



# simple_unit_anim.py (Частичная замена generate_jump)

def generate_jump(obj, settings, **kwargs):
    """
    Физически обоснованный прыжок с поддержкой Squash/Stretch и Rigid Body стилей.
    Использует кинематику для расчета траектории.
    Сохраняет оригинальную логику, но использует utils.py для вспомогательных вычислений.
    """
    try:
        debug_print(f"Physical Jump for {obj.name}")

        if obj.animation_data:
            obj.animation_data_clear()
        prev_mode = bpy.context.mode
        if prev_mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')

        # --- 1. Сбор параметров ---
        frame_start, frame_end = _get_frame_bounds(settings)
        total_frames = max(1, frame_end - frame_start + 1)

        # Рассчитываем фазы как в оригинальной функции
        takeoff_frac = getattr(settings, "jump_takeoff_frac", 0.25)
        recover_frac = 0.15

        takeoff_frames = int(total_frames * takeoff_frac)
        recover_frames = int(total_frames * recover_frac)
        flight_frames_available = total_frames - takeoff_frames - recover_frames

        if flight_frames_available < 1:
            flight_frames_available = 1

        fps = float(bpy.context.scene.render.fps)

        # Целевые параметры
        target_height = kwargs.get('height', getattr(settings, "jump_height", 1.0))
        landing_h = getattr(settings, "jump_landing_height", 0.0)

        # Физические параметры (сохраняем оригинальную физику)
        use_physics = getattr(settings, "jump_physics_enabled", True)
        g = getattr(settings, "jump_gravity", 9.81)
        mass = getattr(settings, "jump_mass", 50.0)
        randomness = getattr(settings, "jump_randomness", 0.05)

        # Визуальные параметры
        squash_factor = getattr(settings, "jump_squash", 0.12)
        stretch_factor = getattr(settings, "jump_stretch", 0.08)
        tilt_deg = getattr(settings, "jump_tilt_amount", 15.0)
        scale_enabled = bool(kwargs.get('allow_scale_changes',
                                        getattr(settings, "allow_scale_changes", True)))

        # Исходные данные
        orig_loc = obj.location.copy()
        orig_rot = obj.rotation_euler.copy()
        orig_scale = obj.scale.copy()

        # Используем forward_vector_world из utils
        forward = _get_forward_vector(obj, settings)

        # Вычисляем вектор "вправо" для правильной оси вращения (Pitch)
        world_up = Vector((0, 0, 1))
        right_vec = forward.cross(world_up).normalized()

        # Если forward параллелен UP (редкий кейс), берем X
        if right_vec.length < 0.001:
            right_vec = Vector((1, 0, 0))

        # Случайная вариация (симулируем несовершенство толчка)
        rand_seed = int(orig_loc.x * 100 + orig_loc.y * 10)
        random.seed(rand_seed)

        height_var = 1.0 + (random.uniform(-1, 1) * 0.1 * randomness)
        actual_h_max = target_height * height_var

        # --- 2. Физический расчет (сохраняем оригинальную физику) ---
        # Вертикальная скорость для достижения высоты H: v0 = sqrt(2 * g * H)
        if actual_h_max < 0.01:
            actual_h_max = 0.01

        v0_z = math.sqrt(2.0 * g * actual_h_max)

        # Время подъема до пика
        t_up = v0_z / g

        # Время спуска от пика до высоты приземления
        dist_down = actual_h_max - landing_h

        if dist_down < 0:
            # Если цель выше пика прыжка - мы не долетим. Увеличиваем силу прыжка автоматически.
            debug_print("Warning: Target too high for jump height, boosting force.")
            actual_h_max = landing_h + 0.1
            v0_z = math.sqrt(2.0 * g * actual_h_max)
            t_up = v0_z / g
            dist_down = 0.1

        t_down = math.sqrt(2.0 * dist_down / g)

        # Полное время в воздухе (физическое)
        flight_time_phys = t_up + t_down

        # Расчет кадров
        t_anim_flight = flight_frames_available / fps

        if t_anim_flight <= 0.001:
            t_anim_flight = 0.001
        time_scale = flight_time_phys / t_anim_flight

        debug_print(
            f"Physics: H={actual_h_max:.2f}m, T_phys={flight_time_phys:.2f}s, "
            f"T_anim={t_anim_flight:.2f}s, Scale={time_scale:.2f}")

        # Горизонтальное движение (линейное)
        jump_dist = kwargs.get('distance', getattr(settings, "move_distance", 2.0))
        v_x_anim = jump_dist / t_anim_flight

        # --- 3. Генерация ключевых кадров ---
        available_frames = total_frames

        for i in range(available_frames + 1):
            frame = frame_start + i
            _scene_set_frame(frame)

            # Определяем текущую фазу
            current_pos = orig_loc.copy()
            current_rot = orig_rot.copy()
            current_scale = orig_scale.copy()

            # --- PHASE 1: PREPARE (Anticipation) ---
            if i <= takeoff_frames:
                # Прогресс 0..1
                p = i / max(1, takeoff_frames)
                # Используем ease_in_out_cubic из utils
                eased_p = ease_in_out_cubic(p)

                # Приседание перед прыжком - ОРИГИНАЛЬНЫЙ РАСЧЕТ
                if scale_enabled:
                    anticipation_squash = math.sin(p * math.pi) * squash_factor * 0.5
                    sq_z = 1.0 - anticipation_squash
                    # Сохранение объема для Squash
                    sq_xy = 1.0 / math.sqrt(sq_z) if sq_z > 0.1 else 1.0

                    current_scale.z *= sq_z
                    current_scale.x *= sq_xy
                    current_scale.y *= sq_xy
                else:
                    # Rigid body: небольшой наклон назад перед рывком
                    tilt = math.radians(-tilt_deg * 0.3 * math.sin(p * math.pi))
                    current_rot.x += tilt

            # --- PHASE 2: FLIGHT (Physics) ---
            elif i <= takeoff_frames + flight_frames_available:
                # Время внутри полета (0..t_anim_flight)
                frame_in_flight = i - takeoff_frames
                t_curr = frame_in_flight / fps

                # Нормализуем t_curr к физическому времени
                t_phys_curr = t_curr * time_scale

                # Вертикальная позиция: z = v0*t - 0.5*g*t^2
                z_displacement = (v0_z * t_phys_curr) - (0.5 * g * (t_phys_curr ** 2))

                current_pos.z += z_displacement
                current_pos += forward * (v_x_anim * t_curr)

                # Расчет текущей вертикальной скорости
                v_z_current = v0_z - g * t_phys_curr

                # SQUASH / STRETCH (Cartoon Physics) - ОРИГИНАЛЬНЫЙ РАСЧЕТ
                if scale_enabled:
                    # Stretch зависит от скорости
                    speed_ratio = abs(v_z_current) / (v0_z + 0.001)
                    str_amount = stretch_factor * speed_ratio

                    s_z = 1.0 + str_amount
                    s_xy = 1.0 / math.sqrt(s_z)  # Volume conservation

                    current_scale.z *= s_z
                    current_scale.x *= s_xy
                    current_scale.y *= s_xy

                # RIGID BODY (Rotation Physics)
                else:
                    # Наклон корпуса по вектору скорости
                    pitch_ratio = v_z_current / (v0_z + 0.001)
                    tilt_angle = math.radians(tilt_deg) * pitch_ratio

                    # Создаем кватернион вращения вокруг оси "вправо"
                    base_quat = orig_rot.to_quaternion()
                    tilt_quat = Quaternion(right_vec, tilt_angle)
                    current_rot = (tilt_quat @ base_quat).to_euler(obj.rotation_mode)

            # --- PHASE 3: RECOVERY (Landing Impact) ---
            else:
                # Мы приземлились
                current_pos.z += landing_h

                # Инерция движения вперед (Slide)
                frame_in_recover = i - (takeoff_frames + flight_frames_available)
                p = frame_in_recover / max(1, recover_frames)

                # Используем ease_in_out_cubic из utils
                eased_p = ease_in_out_cubic(p)

                # Позиция приземления + небольшое скольжение по инерции
                slide_dist = 0.0
                if not scale_enabled:
                    slide_dist = 0.2

                # Ease out скольжения
                slide_amount = slide_dist * (1.0 - (1.0 - eased_p) ** 2)
                current_pos += forward * (jump_dist + slide_amount)

                # Затухающие колебания (Damped oscillation) для удара
                impact_force = mass / 50.0
                decay = math.exp(-p * 5.0)
                oscillate = math.sin(p * math.pi * 3.0)

                if scale_enabled:
                    # ОРИГИНАЛЬНЫЙ РАСЧЕТ для Squash при ударе
                    impact_squash = squash_factor * impact_force * (1.0 - p) * decay
                    if p < 0.2:  # Первый удар самый сильный
                        impact_squash += squash_factor * 0.5 * (1.0 - p / 0.2)

                    sq_z = 1.0 - impact_squash
                    if sq_z < 0.1:
                        sq_z = 0.1  # Safety limit
                    sq_xy = 1.0 / math.sqrt(sq_z)

                    current_scale.z *= sq_z
                    current_scale.x *= sq_xy
                    current_scale.y *= sq_xy

                else:
                    # Rigid Body: Тряска и наклон вперед - с использованием randomness
                    brake_tilt = math.radians(tilt_deg * 0.5) * decay * impact_force
                    current_rot.x += brake_tilt

                    # Микро-смещение Z (тряска без скейла) - с использованием randomness
                    z_shake = -0.05 * decay * oscillate * impact_force * randomness
                    current_pos.z += z_shake

            # Применяем трансформации
            obj.location = current_pos
            obj.rotation_euler = current_rot

            if scale_enabled:
                obj.scale = current_scale
            else:
                obj.scale = orig_scale

            # Keyframing
            _keyframe_object(obj, frame)

            # Принудительное обновление графа
            depsgraph = bpy.context.evaluated_depsgraph_get()
            depsgraph.update()

        # Очистка scale кривых, если скейл был отключен
        if not scale_enabled:
            _remove_scale_fcurves(obj)

        if prev_mode != 'OBJECT':
            bpy.ops.object.mode_set(mode=prev_mode)

        debug_print(f"Jump physics completed. Landing at Z+{landing_h}")
        debug_print(f"Random variation: height_var={height_var:.4f} (seed={rand_seed})")
        return True

    except Exception as e:
        debug_print(f"Jump error: {e}")
        import traceback
        traceback.print_exc()
        return False


def generate_damage(obj, settings, **kwargs):
    """Урон с отдачей и тряской - работает с арматурой и целевой костью"""
    try:
        debug_print(f"Damage for {obj.name}")

        # Получаем цель анимации
        target = get_animation_target(obj, settings)

        if obj.animation_data:
            obj.animation_data_clear()

        prev_mode = bpy.context.mode
        if prev_mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')

        frame_start, frame_end = _get_frame_bounds(settings)
        total = max(1, frame_end - frame_start + 1)

        intensity = kwargs.get('intensity', 1.0)
        shake_amp = getattr(settings, "damage_shake_amp", 0.06) * intensity
        recoil_z = getattr(settings, "damage_recoil_z", 0.12) * intensity
        shake_freq = getattr(settings, "damage_shake_freq", 18.0)

        # Сохраняем исходные трансформации
        if isinstance(target, bpy.types.PoseBone):
            original_location = target.location.copy()
            original_rotation = target.rotation_quaternion.copy()
        else:
            original_location = obj.location.copy()
            original_rotation = obj.rotation_euler.copy()

        forward = _get_forward_vector(obj, settings)
        recoil_dir = -forward * 0.3 * intensity

        for frame in range(frame_start, frame_end + 1):
            _scene_set_frame(frame)
            progress = (frame - frame_start) / max(1, frame_end - frame_start)

            decay = math.exp(-6.0 * progress)
            recoil_progress = clamp(1.0 - progress * 3.0, 0.0, 1.0)
            current_recoil = recoil_dir * recoil_progress

            shake_x = shake_amp * decay * math.sin(progress * shake_freq)
            shake_y = shake_amp * decay * math.cos(progress * shake_freq * 0.7)

            # Вычисляем глобальные изменения
            delta_global_loc = current_recoil + Vector((shake_x, shake_y, -recoil_z * recoil_progress))
            rot_shake = decay * 0.1 * math.sin(progress * shake_freq * 0.5)

            if isinstance(target, bpy.types.PoseBone):
                # Для кости: преобразуем глобальные изменения в локальные
                delta_local_loc, local_rot = global_delta_to_bone_local(
                    arm_obj=obj,
                    pose_bone=target,
                    delta_global_loc=delta_global_loc,
                    delta_global_rot_axis=Vector((0, 0, 1)),
                    delta_global_rot_angle=rot_shake
                )

                # Применяем изменения
                target.location = original_location + delta_local_loc

                if local_rot is not None and local_rot.angle > 0.0001:
                    old_mode = target.rotation_mode
                    if old_mode != 'QUATERNION':
                        target.rotation_mode = 'QUATERNION'
                    target.rotation_quaternion = (local_rot @ original_rotation).normalized()
                    target.rotation_mode = old_mode
                else:
                    target.rotation_quaternion = original_rotation.copy()

                # Ключевые кадры для кости
                target.keyframe_insert(data_path="location", frame=frame)
                target.keyframe_insert(data_path="rotation_quaternion", frame=frame)

            else:
                # Для обычного объекта
                obj.location = original_location + delta_global_loc
                obj.rotation_euler.z = original_rotation.z + rot_shake

                _keyframe_object(obj, frame)

            depsgraph = bpy.context.evaluated_depsgraph_get()
            depsgraph.update()

        if prev_mode != 'OBJECT':
            bpy.ops.object.mode_set(mode=prev_mode)

        if obj.animation_data and obj.animation_data.action:
            debug_print(f"Keys created: {len(obj.animation_data.action.fcurves)} fcurves")
        else:
            debug_print("No animation_data or action created!")

        debug_print("Damage animation completed")
        return True

    except Exception as e:
        debug_print(f"Damage error: {e}")
        import traceback
        traceback.print_exc()
        return False


# simple_unit_anim_integrated.py - IMPROVED DODGE SYSTEM

def generate_dodge(obj, settings, **kwargs):
    """
    Улучшенная система уклонения с 3 пресетами.
    Использует абсолютные смещения (без накопления ошибок).
    """
    try:
        debug_print(f"Dodge animation for {obj.name} (V2)")

        # Получаем параметры из kwargs или settings
        style = kwargs.get('dodge_style', getattr(settings, "dodge_style", "HOP")).upper()
        lateral_dist = kwargs.get('lateral_dist', getattr(settings, "dodge_lateral_dist", 1.0))
        side_char = kwargs.get('dodge_side', getattr(settings, "dodge_side", "R")).upper()

        # Выводим информацию о параметрах
        debug_print(f"Dodge params: style={style}, dist={lateral_dist}, side={side_char}")

        target = get_animation_target(obj, settings)
        if obj.animation_data:
            obj.animation_data_clear()

        prev_mode = bpy.context.mode
        if prev_mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')

        frame_start, frame_end = _get_frame_bounds(settings)
        total_frames = max(1, frame_end - frame_start + 1)

        # Параметры
        style = kwargs.get('dodge_style', getattr(settings, "dodge_style", "HOP")).upper()

        # Сохраняем исходные трансформации
        if isinstance(target, bpy.types.PoseBone):
            original_location = target.location.copy()
            original_rotation = target.rotation_quaternion.copy()
        else:
            original_location = obj.location.copy()
            original_rotation = obj.rotation_euler.copy()

        # Вектора
        forward = _get_forward_vector(obj, settings)
        up = Vector((0, 0, 1))
        lateral = forward.cross(up).normalized()

        # Определяем сторону (Left/Right)
        # Если в settings есть параметр side, используем его, иначе R
        side_char = kwargs.get('dodge_side', getattr(settings, "dodge_side", "R")).upper()
        # side_mult: 1.0 = Right, -1.0 = Left
        side_mult = -1.0 if side_char == "L" else 1.0

        # Словарь стратегий
        strategies = {
            "SIMPLE": _dodge_simple_physics,  # <-- Добавлено
            "HOP": _dodge_hop_physics,
            "SLIDE": _dodge_slide_physics,
            "MATRIX": _dodge_matrix_physics
        }

        # Если стиль не найден, используем SIMPLE как самый безопасный
        func = strategies.get(style, _dodge_simple_physics)

        # Запускаем физику
        success = func(
            obj, target, original_location, original_rotation,
            forward, lateral, up, side_mult,
            frame_start, frame_end, total_frames,
            settings, kwargs
        )

        if prev_mode != bpy.context.mode:
            bpy.ops.object.mode_set(mode=prev_mode)

        return success

    except Exception as e:
        debug_print(f"Dodge error: {e}")
        import traceback
        traceback.print_exc()
        return False


# --- 1. ДОБАВЛЯЕМ НОВЫЙ "SIMPLE" ПРЕСЕТ ---

def _dodge_simple_physics(obj, target, original_location, original_rotation,
                          forward, lateral, up, side_mult,
                          frame_start, frame_end, total_frames,
                          settings, kwargs):
    """
    STYLE: SIMPLE (Классика + Поворот)
    Смещение в сторону с небольшим доворотом корпуса по Z.
    """
    dist = kwargs.get('lateral_dist', getattr(settings, "dodge_distance", 1.0))
    q_f = kwargs.get('quick_frac', getattr(settings, "dodge_quick_frac", 0.3))
    # Тот самый угол из твоего оригинала
    max_lean = math.radians(15.0)

    f_out = int(total_frames * q_f)
    f_hold = int(total_frames * 0.2)
    f_ret = total_frames - f_out - f_hold

    for i in range(total_frames):
        frame = frame_start + i
        _scene_set_frame(frame)

        curve_val = 0.0
        if i < f_out:
            curve_val = ease_in_out_cubic(i / f_out)
        elif i < f_out + f_hold:
            curve_val = 1.0
        else:
            t = (i - f_out - f_hold) / max(1, f_ret)
            curve_val = 1.0 - ease_in_out_cubic(t)

        if i == total_frames - 1: curve_val = 0.0

        offset_loc = lateral * (dist * curve_val * side_mult)

        # Возвращаем поворот по Z (Yaw)
        # В оригинале было lean_angle = 15 deg * progress * -sign
        offset_rot = Vector((0, 0, 0))
        offset_rot.z = max_lean * curve_val * -side_mult

        _apply_dodge_transformation(
            obj, target, original_location, original_rotation,
            offset_loc, offset_rot, frame, forward
        )
    return True

# --- 2. ОБНОВЛЯЕМ HOP (ПРЫЖОК ТУДА И ОБРАТНО) ---

def _dodge_hop_physics(obj, target, original_location, original_rotation,
                       forward, lateral, up, side_mult,
                       frame_start, frame_end, total_frames,
                       settings, kwargs):
    """
    STYLE: HOP (Отскок с возвратом)
    Прыжок в сторону -> Приземление -> Прыжок обратно.
    """
    dist = kwargs.get('lateral_dist', getattr(settings, "dodge_distance", 1.5))
    height = kwargs.get('hop_height', getattr(settings, "dodge_height", 0.3))
    q_f = kwargs.get('quick_frac', getattr(settings, "dodge_quick_frac", 0.4))

    # Делим на два прыжка: Туда (45%), Пауза (10%), Обратно (45%)
    p_pause = 0.1

    f_jump_1 = int(total_frames * q_f)
    f_pause = int(total_frames * p_pause)
    f_jump_2 = total_frames - f_jump_1 - f_pause

    for i in range(total_frames):
        frame = frame_start + i
        _scene_set_frame(frame)

        offset_loc = Vector((0, 0, 0))
        offset_rot = Vector((0, 0, 0))

        current_lateral_pos = 0.0
        current_height = 0.0
        bank_angle = 0.0

        if i < f_jump_1:
            # === ПРЫЖОК ТУДА ===
            t = i / f_jump_1
            # Easing
            lat_t = ease_in_out_cubic(t)
            current_lateral_pos = dist * lat_t
            # Parabola height
            current_height = math.sin(t * math.pi) * height
            # Bank
            bank_angle = math.radians(15) * math.sin(t * math.pi)

        elif i < f_jump_1 + f_pause:
            # === ПАУЗА (Приземление) ===
            t = (i - f_jump_1) / f_pause
            current_lateral_pos = dist
            # Амортизация (Squash)
            current_height = -0.1 * math.sin(t * math.pi)
            bank_angle = 0.0

        else:
            # === ПРЫЖОК ОБРАТНО ===
            t = (i - f_jump_1 - f_pause) / f_jump_2
            # Easing (обратный)
            lat_t = 1.0 - ease_in_out_cubic(t)
            current_lateral_pos = dist * lat_t
            # Parabola height
            current_height = math.sin(t * math.pi) * height
            # Bank (в другую сторону, так как летим обратно)
            bank_angle = -math.radians(15) * math.sin(t * math.pi)

            # Фикс последнего кадра
            if i == total_frames - 1:
                current_lateral_pos = 0.0
                current_height = 0.0
                bank_angle = 0.0

        # Сборка векторов
        offset_loc += lateral * (current_lateral_pos * side_mult)
        offset_loc.z = current_height
        offset_rot.x = bank_angle * side_mult  # Roll

        _apply_dodge_transformation(
            obj, target, original_location, original_rotation,
            offset_loc, offset_rot, frame, forward
        )

    return True


# --- 3. ОБНОВЛЯЕМ SLIDE (СКОЛЬЖЕНИЕ ТУДА И ОБРАТНО) ---

def _dodge_slide_physics(obj, target, original_location, original_rotation,
                         forward, lateral, up, side_mult,
                         frame_start, frame_end, total_frames,
                         settings, kwargs):
    """
    STYLE: SLIDE (Скольжение с возвратом)
    Уезд в сторону -> Торможение -> Возврат.
    """
    dist = kwargs.get('lateral_dist', getattr(settings, "dodge_distance", 2.0))
    tilt_amount = kwargs.get('slide_tilt', 20.0)

    # 40% Туда, 20% Стоим, 40% Обратно
    f_out = int(total_frames * 0.4)
    f_hold = int(total_frames * 0.2)

    for i in range(total_frames):
        frame = frame_start + i
        _scene_set_frame(frame)

        offset_loc = Vector((0, 0, 0))
        offset_rot = Vector((0, 0, 0))

        progress = 0.0  # 0..1 (позиция)
        tilt_intensity = 0.0  # -1..1 (наклон)

        if i < f_out:
            # EASE OUT (Fast start, slow stop)
            t = i / f_out
            progress = 1.0 - (1.0 - t) ** 3  # Cubic Ease Out
            # Наклон сильный в начале, затухает к остановке
            tilt_intensity = (1.0 - t)

        elif i < f_out + f_hold:
            # HOLD
            progress = 1.0
            # Легкая вибрация на месте
            tilt_intensity = math.sin((i - f_out) * 0.5) * 0.1

        else:
            # RETURN (Ease In Out)
            t = (i - f_out - f_hold) / (total_frames - f_out - f_hold)
            progress = 1.0 - ease_in_out_cubic(t)
            # Наклон в обратную сторону при начале движения назад
            tilt_intensity = -math.sin(t * math.pi)

            # Фикс конца
        if i == total_frames - 1:
            progress = 0.0
            tilt_intensity = 0.0

        # Позиция
        offset_loc += lateral * (dist * progress * side_mult)

        # Вибрация пола (только когда движется быстро)
        if abs(tilt_intensity) > 0.3:
            offset_loc += lateral * math.sin(i * 2.0) * 0.02

        # Вращение
        roll_angle = math.radians(tilt_amount) * tilt_intensity * side_mult
        offset_rot.x = roll_angle

        _apply_dodge_transformation(
            obj, target, original_location, original_rotation,
            offset_loc, offset_rot, frame, forward
        )

    return True


def _dodge_matrix_physics(obj, target, original_location, original_rotation,
                          forward, lateral, up, side_mult,
                          frame_start, frame_end, total_frames,
                          settings, kwargs):
    """
    STYLE: MATRIX (Уклонение корпусом с возвратом)
    """
    lean_dist = kwargs.get('lateral_dist', getattr(settings, "dodge_distance", 1.0)) * 0.7
    max_lean = math.radians(kwargs.get('matrix_lean', 35.0))
    q_f = kwargs.get('quick_frac', getattr(settings, "dodge_quick_frac", 0.3))

    f_lean = int(total_frames * q_f)
    f_hold = int(total_frames * 0.3)  # "Матрица" требует долгой паузы
    f_ret = total_frames - f_lean - f_hold

    for i in range(total_frames):
        frame = frame_start + i
        _scene_set_frame(frame)

        curve = 0.0
        if i < f_lean:
            curve = ease_in_out_cubic(i / f_lean)
        elif i < f_lean + f_hold:
            curve = 1.0
        else:
            curve = 1.0 - ease_in_out_cubic((i - f_lean - f_hold) / max(1, f_ret))

        if i == total_frames - 1: curve = 0.0

        offset_loc = lateral * (lean_dist * curve * side_mult)
        offset_loc.z = -0.15 * curve  # Снижение центра тяжести
        offset_rot = Vector((max_lean * curve * side_mult, 0, 0))

        _apply_dodge_transformation(obj, target, original_location, original_rotation,
                                    offset_loc, offset_rot, frame, forward)
    return True


def _apply_dodge_transformation(obj, target, original_location, original_rotation,
                                offset_loc, offset_rot, frame, roll_axis=None):
    """
    Применение трансформации для уклонения.
    offset_rot: Vector(Roll, Pitch, Yaw) - условно.
    roll_axis: Ось для компонента X (Roll). Обычно Forward вектор.
    """

    # 1. Собираем вращение
    # Если передан roll_axis (Forward), то offset_rot.x - это вращение вокруг него

    final_quat_offset = Quaternion((1, 0, 0, 0))

    if roll_axis:
        # Roll (вокруг forward)
        q_roll = Quaternion(roll_axis, offset_rot.x)
        # Yaw (вокруг Z/Up)
        q_yaw = Quaternion(Vector((0, 0, 1)), offset_rot.z)
        # Pitch (вокруг Lateral - сложнее получить здесь, но допустим lateral = roll cross up)
        # Пока ограничимся Roll/Yaw
        final_quat_offset = q_yaw @ q_roll
    else:
        # Fallback Euler XYZ
        final_quat_offset = Euler(offset_rot, 'XYZ').to_quaternion()

    rot_axis, rot_angle = final_quat_offset.to_axis_angle()

    # 2. Применяем к цели
    if isinstance(target, bpy.types.PoseBone):
        delta_local_loc, local_rot = global_delta_to_bone_local(
            arm_obj=obj,
            pose_bone=target,
            delta_global_loc=offset_loc,
            delta_global_rot_axis=rot_axis,
            delta_global_rot_angle=rot_angle
        )

        target.location = original_location + delta_local_loc

        if local_rot is not None:
            if target.rotation_mode != 'QUATERNION': target.rotation_mode = 'QUATERNION'
            target.rotation_quaternion = (local_rot @ original_rotation).normalized()

        target.keyframe_insert("location", frame=frame)
        target.keyframe_insert("rotation_quaternion", frame=frame)

    else:
        obj.location = original_location + offset_loc

        if hasattr(obj, 'rotation_euler'):
            # Примитивное наложение для объектов
            # Для точности лучше переводить оригинал в кват, умножать и обратно
            base_q = original_rotation.to_quaternion()
            new_q = final_quat_offset @ base_q
            obj.rotation_euler = new_q.to_euler(obj.rotation_mode)

        _keyframe_object(obj, frame)
        
        
        
# simple_unit_anim_integrated.py - ИСПРАВЛЕННЫЕ новые функции

# simple_unit_anim_integrated.py - ПОЛНОСТЬЮ ПЕРЕПИСАННЫЕ новые функции

def generate_surprise(obj, settings, **kwargs):
    """Внезапный прыжок от удивления - работает с арматурой и целевой костью"""
    try:
        debug_print(f"Surprise for {obj.name}")

        # Получаем цель анимации
        target = get_animation_target(obj, settings)

        if obj.animation_data:
            obj.animation_data_clear()

        prev_mode = bpy.context.mode
        if prev_mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')

        frame_start, frame_end = _get_frame_bounds(settings)
        total_frames = max(1, frame_end - frame_start + 1)

        # Используем ТОЛЬКО параметры surprise
        jump_height = kwargs.get('surprise_amp', getattr(settings, "surprise_amp", 0.3))
        scale_enabled = bool(kwargs.get('allow_scale_changes', getattr(settings, "allow_scale_changes", True)))

        # Сохраняем исходные трансформации
        if isinstance(target, bpy.types.PoseBone):
            original_location = target.location.copy()
            original_rotation = target.rotation_quaternion.copy()
            original_scale = target.scale.copy()
        else:
            original_location = obj.location.copy()
            original_rotation = obj.rotation_euler.copy()
            original_scale = obj.scale.copy()

        # Анимация в 3 фазы: подготовка, прыжок, приземление
        prep_frames = int(total_frames * 0.15)  # Быстрая подготовка
        jump_frames = int(total_frames * 0.5)  # Основной прыжок
        land_frames = total_frames - prep_frames - jump_frames

        for i in range(total_frames):
            frame = frame_start + i
            _scene_set_frame(frame)

            if i < prep_frames:
                # Фаза 1: Быстрое приседание перед прыжком
                phase = i / prep_frames
                squash = math.sin(phase * math.pi * 0.5) * 0.4  # Быстрое сжатие

                if isinstance(target, bpy.types.PoseBone):
                    # Для кости: применяем масштаб
                    if scale_enabled:
                        target.scale = Vector((
                            original_scale.x * (1.0 + squash * 0.3),
                            original_scale.y * (1.0 + squash * 0.3),
                            original_scale.z * (1.0 - squash)
                        ))
                    else:
                        target.scale = original_scale

                    target.location = original_location
                    target.rotation_quaternion = original_rotation.copy()

                    # Ключевые кадры для кости
                    target.keyframe_insert(data_path="scale", frame=frame)
                    target.keyframe_insert(data_path="location", frame=frame)
                    target.keyframe_insert(data_path="rotation_quaternion", frame=frame)

                else:
                    # Для обычного объекта
                    if scale_enabled:
                        obj.scale = Vector((
                            original_scale.x * (1.0 + squash * 0.3),
                            original_scale.y * (1.0 + squash * 0.3),
                            original_scale.z * (1.0 - squash)
                        ))
                    else:
                        obj.scale = original_scale

                    obj.location = original_location
                    obj.rotation_euler = original_rotation

                    _keyframe_object(obj, frame)

            elif i < prep_frames + jump_frames:
                # Фаза 2: Прыжок с растяжением
                phase = (i - prep_frames) / jump_frames

                # Высокий прыжок с медленным подъемом и быстрым падением
                if phase < 0.7:
                    height_factor = ease_in_out_cubic(phase / 0.7)
                else:
                    height_factor = 1.0 - ease_in_out_cubic((phase - 0.7) / 0.3)

                height = height_factor * jump_height

                # Растяжение в прыжке
                stretch = math.sin(phase * math.pi) * 0.4

                # Вращение от удивления (вокруг глобальной оси Z)
                spin = math.sin(phase * math.pi * 3) * 0.5

                if isinstance(target, bpy.types.PoseBone):
                    # Для кости: вычисляем глобальные изменения
                    delta_global_loc = Vector((0, 0, height))

                    # Преобразуем глобальные изменения в локальные для кости
                    delta_local_loc, local_rot = global_delta_to_bone_local(
                        arm_obj=obj,
                        pose_bone=target,
                        delta_global_loc=delta_global_loc,
                        delta_global_rot_axis=Vector((0, 0, 1)),
                        delta_global_rot_angle=spin
                    )

                    # Применяем изменения
                    target.location = original_location + delta_local_loc

                    if local_rot is not None and local_rot.angle > 0.0001:
                        old_mode = target.rotation_mode
                        if old_mode != 'QUATERNION':
                            target.rotation_mode = 'QUATERNION'
                        target.rotation_quaternion = (local_rot @ original_rotation).normalized()
                        target.rotation_mode = old_mode
                    else:
                        target.rotation_quaternion = original_rotation.copy()

                    # Масштаб
                    if scale_enabled:
                        target.scale = Vector((
                            original_scale.x * (1.0 - stretch * 0.2),
                            original_scale.y * (1.0 - stretch * 0.2),
                            original_scale.z * (1.0 + stretch)
                        ))
                    else:
                        target.scale = original_scale

                    # Ключевые кадры для кости
                    target.keyframe_insert(data_path="location", frame=frame)
                    target.keyframe_insert(data_path="rotation_quaternion", frame=frame)
                    target.keyframe_insert(data_path="scale", frame=frame)

                else:
                    # Для обычного объекта
                    if scale_enabled:
                        obj.scale = Vector((
                            original_scale.x * (1.0 - stretch * 0.2),
                            original_scale.y * (1.0 - stretch * 0.2),
                            original_scale.z * (1.0 + stretch)
                        ))
                    else:
                        obj.scale = original_scale

                    obj.rotation_euler.z = original_rotation.z + spin
                    obj.location.z = original_location.z + height

                    _keyframe_object(obj, frame)

            else:
                # Фаза 3: Приземление с отскоком
                phase = (i - prep_frames - jump_frames) / land_frames

                # Несколько маленьких отскоков
                bounce1 = math.sin(phase * math.pi * 4) * 0.1 * math.exp(-phase * 3)
                bounce2 = math.sin(phase * math.pi * 8 + 1.5) * 0.05 * math.exp(-phase * 6)
                bounce_height = (bounce1 + bounce2) * jump_height * 0.3

                if isinstance(target, bpy.types.PoseBone):
                    # Для кости: вычисляем глобальные изменения
                    delta_global_loc = Vector((0, 0, bounce_height))

                    # Преобразуем глобальные изменения в локальные для кости
                    delta_local_loc, _ = global_delta_to_bone_local(
                        arm_obj=obj,
                        pose_bone=target,
                        delta_global_loc=delta_global_loc
                    )

                    # Применяем изменения
                    target.location = original_location + delta_local_loc
                    target.rotation_quaternion = original_rotation.copy()
                    target.scale = original_scale.copy()

                    # Ключевые кадры для кости
                    target.keyframe_insert(data_path="location", frame=frame)
                    target.keyframe_insert(data_path="rotation_quaternion", frame=frame)
                    target.keyframe_insert(data_path="scale", frame=frame)

                else:
                    # Для обычного объекта
                    obj.location.z = original_location.z + bounce_height
                    obj.scale = original_scale
                    obj.rotation_euler = original_rotation

                    _keyframe_object(obj, frame)

            depsgraph = bpy.context.evaluated_depsgraph_get()
            depsgraph.update()

        if prev_mode != 'OBJECT':
            bpy.ops.object.mode_set(mode=prev_mode)

        debug_print("Surprise animation completed")
        return True

    except Exception as e:
        debug_print(f"Surprise error: {e}")
        import traceback
        traceback.print_exc()
        return False



def generate_panic(obj, settings, **kwargs):
    """Паника - быстрая беспорядочная тряска - работает с арматурой и целевой костью"""
    try:
        debug_print(f"Panic for {obj.name}")

        # Получаем цель анимации
        target = get_animation_target(obj, settings)

        if obj.animation_data:
            obj.animation_data_clear()

        prev_mode = bpy.context.mode
        if prev_mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')

        frame_start, frame_end = _get_frame_bounds(settings)
        total_frames = max(1, frame_end - frame_start + 1)

        # Используем ТОЛЬКО параметры panic
        speed = kwargs.get('panic_speed', getattr(settings, "panic_speed", 15.0))
        scale_enabled = bool(kwargs.get('allow_scale_changes', getattr(settings, "allow_scale_changes", True)))

        # Сохраняем исходные трансформации
        if isinstance(target, bpy.types.PoseBone):
            original_location = target.location.copy()
            original_rotation = target.rotation_quaternion.copy()
            original_scale = target.scale.copy()
        else:
            original_location = obj.location.copy()
            original_rotation = obj.rotation_euler.copy()
            original_scale = obj.scale.copy()

        for frame in range(frame_start, frame_end + 1):
            _scene_set_frame(frame)

            progress = (frame - frame_start) / total_frames
            time = progress * speed

            # Многоканальная тряска с разными частотами
            shake_x = (math.sin(time * 13.7) * 0.7 + math.cos(time * 8.3) * 0.3) * 0.05
            shake_y = (math.sin(time * 11.2) * 0.6 + math.cos(time * 9.1) * 0.4) * 0.05
            shake_z = (math.sin(time * 15.3) * 0.8 + math.cos(time * 7.5) * 0.2) * 0.03

            # Вращательная тряска
            rot_x = math.sin(time * 17.1) * 0.2
            rot_y = math.cos(time * 12.4) * 0.2
            rot_z = (math.sin(time * 14.3) + math.cos(time * 10.7)) * 0.4

            # Пульсация масштаба
            pulse_fast = math.sin(time * 25.0) * 0.08
            pulse_slow = math.sin(time * 8.0) * 0.04
            pulse = 1.0 + pulse_fast + pulse_slow

            # Дрожание
            tremor = math.sin(time * 50.0) * 0.02 * math.exp(-progress * 2)

            if isinstance(target, bpy.types.PoseBone):
                # Для кости: вычисляем глобальные изменения
                delta_global_loc = Vector((shake_x + tremor, shake_y + tremor, shake_z + tremor))

                # Создаем кватернион вращения из Эйлеровых углов
                delta_global_rot_euler = Euler((rot_x, rot_y, rot_z), 'XYZ')
                delta_rot_quat = delta_global_rot_euler.to_quaternion()

                # Преобразуем в ось и угол
                if delta_rot_quat.angle > 0.0001:
                    rot_axis, rot_angle = delta_rot_quat.to_axis_angle()
                else:
                    rot_axis, rot_angle = Vector((0, 0, 1)), 0.0

                # Преобразуем глобальные изменения в локальные для кости
                delta_local_loc, local_rot = global_delta_to_bone_local(
                    arm_obj=obj,
                    pose_bone=target,
                    delta_global_loc=delta_global_loc,
                    delta_global_rot_axis=rot_axis,
                    delta_global_rot_angle=rot_angle
                )

                # Применяем изменения
                target.location = original_location + delta_local_loc

                if local_rot is not None and local_rot.angle > 0.0001:
                    old_mode = target.rotation_mode
                    if old_mode != 'QUATERNION':
                        target.rotation_mode = 'QUATERNION'
                    target.rotation_quaternion = (local_rot @ original_rotation).normalized()
                    target.rotation_mode = old_mode
                else:
                    target.rotation_quaternion = original_rotation.copy()

                # Масштаб
                if scale_enabled:
                    target.scale = original_scale * pulse
                else:
                    target.scale = original_scale

                # Ключевые кадры
                target.keyframe_insert(data_path="location", frame=frame)
                target.keyframe_insert(data_path="rotation_quaternion", frame=frame)
                target.keyframe_insert(data_path="scale", frame=frame)

            else:
                # Для обычного объекта
                obj.location = Vector((
                    original_location.x + shake_x + tremor,
                    original_location.y + shake_y + tremor,
                    original_location.z + shake_z + tremor
                ))

                obj.rotation_euler = Vector((
                    original_rotation.x + rot_x,
                    original_rotation.y + rot_y,
                    original_rotation.z + rot_z
                ))

                if scale_enabled:
                    obj.scale = original_scale * pulse
                else:
                    obj.scale = original_scale

                _keyframe_object(obj, frame)

            depsgraph = bpy.context.evaluated_depsgraph_get()
            depsgraph.update()

        if prev_mode != 'OBJECT':
            bpy.ops.object.mode_set(mode=prev_mode)

        debug_print("Panic animation completed")
        return True

    except Exception as e:
        debug_print(f"Panic error: {e}")
        import traceback
        traceback.print_exc()
        return False


def generate_shoot(obj, settings, **kwargs):
    """Стрельба с отдачей - работает с арматурой и целевой костью"""
    try:
        debug_print(f"Shoot for {obj.name}")

        # Получаем цель анимации
        target = get_animation_target(obj, settings)

        if obj.animation_data:
            obj.animation_data_clear()

        prev_mode = bpy.context.mode
        if prev_mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')

        frame_start, frame_end = _get_frame_bounds(settings)
        total_frames = max(1, frame_end - frame_start + 1)

        # Используем ТОЛЬКО параметры shoot
        recoil = kwargs.get('shoot_recoil', getattr(settings, "shoot_recoil", 0.1))
        scale_enabled = bool(kwargs.get('allow_scale_changes', getattr(settings, "allow_scale_changes", True)))

        # Сохраняем исходные трансформации
        if isinstance(target, bpy.types.PoseBone):
            original_location = target.location.copy()
            original_rotation = target.rotation_quaternion.copy()
            original_scale = target.scale.copy()
        else:
            original_location = obj.location.copy()
            original_rotation = obj.rotation_euler.copy()
            original_scale = obj.scale.copy()

        forward = _get_forward_vector(obj, settings)

        # Быстрый выстрел в начале анимации
        shot_frame = int(total_frames * 0.1)

        for i in range(total_frames):
            frame = frame_start + i
            _scene_set_frame(frame)

            progress = i / total_frames

            if i <= shot_frame:
                # Фаза выстрела: резкая отдача
                phase = i / shot_frame
                recoil_amount = math.sin(phase * math.pi * 0.5) * recoil

                # Вспышка - быстрое увеличение
                flash = math.sin(phase * math.pi * 3) * 0.3 + 1.0

                # Резкий толчок назад (вокруг глобальной оси X)
                kick = -recoil_amount * 0.5

                if isinstance(target, bpy.types.PoseBone):
                    # Для кости: вычисляем глобальные изменения
                    delta_global_loc = -forward * recoil_amount

                    # Преобразуем глобальные изменения в локальные для кости
                    delta_local_loc, local_rot = global_delta_to_bone_local(
                        arm_obj=obj,
                        pose_bone=target,
                        delta_global_loc=delta_global_loc,
                        delta_global_rot_axis=Vector((1, 0, 0)),  # Вращение вокруг глобальной оси X
                        delta_global_rot_angle=kick
                    )

                    # Применяем изменения
                    target.location = original_location + delta_local_loc

                    if local_rot is not None and local_rot.angle > 0.0001:
                        old_mode = target.rotation_mode
                        if old_mode != 'QUATERNION':
                            target.rotation_mode = 'QUATERNION'
                        target.rotation_quaternion = (local_rot @ original_rotation).normalized()
                        target.rotation_mode = old_mode
                    else:
                        target.rotation_quaternion = original_rotation.copy()

                    # Масштаб (вспышка)
                    if scale_enabled:
                        target.scale = original_scale * flash
                    else:
                        target.scale = original_scale

                    # Ключевые кадры для кости
                    target.keyframe_insert(data_path="location", frame=frame)
                    target.keyframe_insert(data_path="rotation_quaternion", frame=frame)
                    target.keyframe_insert(data_path="scale", frame=frame)

                else:
                    # Для обычного объекта
                    obj.location = original_location - forward * recoil_amount

                    if scale_enabled:
                        obj.scale = original_scale * flash
                    else:
                        obj.scale = original_scale

                    obj.rotation_euler.x = original_rotation.x + kick

                    _keyframe_object(obj, frame)

            else:
                # Фаза восстановления: плавное возвращение
                recovery_phase = (i - shot_frame) / (total_frames - shot_frame)
                recovery = (1.0 - ease_in_out_cubic(recovery_phase)) * recoil * 0.3

                # Небольшая вибрация после выстрела
                vibration = math.sin(recovery_phase * 20.0) * 0.02 * math.exp(-recovery_phase * 5)

                if isinstance(target, bpy.types.PoseBone):
                    # Для кости: вычисляем глобальные изменения
                    delta_global_loc = -forward * recovery

                    # Преобразуем глобальные изменения в локальные для кости
                    delta_local_loc, _ = global_delta_to_bone_local(
                        arm_obj=obj,
                        pose_bone=target,
                        delta_global_loc=delta_global_loc
                    )

                    # Применяем изменения
                    target.location = original_location + delta_local_loc

                    # Масштаб с вибрацией
                    if scale_enabled:
                        target.scale = original_scale * (1.0 + vibration * 0.1)
                    else:
                        target.scale = original_scale

                    # Ключевые кадры для кости
                    target.keyframe_insert(data_path="location", frame=frame)
                    target.keyframe_insert(data_path="scale", frame=frame)

                else:
                    # Для обычного объекта
                    obj.location = original_location - forward * recovery

                    if scale_enabled:
                        obj.scale = original_scale * (1.0 + vibration * 0.1)
                    else:
                        obj.scale = original_scale

                    obj.rotation_euler.x = original_rotation.x + vibration

                    _keyframe_object(obj, frame)

            depsgraph = bpy.context.evaluated_depsgraph_get()
            depsgraph.update()

        if prev_mode != 'OBJECT':
            bpy.ops.object.mode_set(mode=prev_mode)

        debug_print("Shoot animation completed")
        return True

    except Exception as e:
        debug_print(f"Shoot error: {e}")
        import traceback
        traceback.print_exc()
        return False

def generate_roll(obj, settings, **kwargs):
    """Перекатывание/кувырок - ПОЛНОСТЬЮ ПЕРЕПИСАНО"""
    try:
        debug_print(f"Roll for {obj.name}")
        
        if obj.animation_data:
            obj.animation_data_clear()
        prev_mode = bpy.context.mode
        if prev_mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')
        
        frame_start, frame_end = _get_frame_bounds(settings)
        total_frames = max(1, frame_end - frame_start + 1)
        
        # Используем ТОЛЬКО параметры roll
        rotations = kwargs.get('rotations', getattr(settings, "roll_rotations", 2))
        scale_enabled = bool(kwargs.get('allow_scale_changes', getattr(settings, "allow_scale_changes", True)))
        
        original_location = obj.location.copy()
        original_rotation = obj.rotation_euler.copy()
        original_scale = obj.scale.copy()
        
        forward = _get_forward_vector(obj, settings)
        
        # Длина переката зависит от количества вращений
        roll_distance = rotations * 2.0
        
        for i in range(total_frames):
            frame = frame_start + i
            _scene_set_frame(frame)
            
            progress = i / total_frames
            eased = ease_in_out_cubic(progress)
            
            # Вращение вокруг оси Y (как настоящее перекатывание)
            roll_angle = eased * rotations * 2.0 * math.pi
            
            # Настоящее перекатывание: вращение + движение вперед
            obj.rotation_euler.y = original_rotation.y + roll_angle
            
            # Движение вперед с дугой
            distance_progress = eased
            arc_height = math.sin(progress * math.pi) * 0.4  # Дуга вверх
            
            obj.location = original_location + forward * (distance_progress * roll_distance)
            obj.location.z = original_location.z + arc_height
            
            # Динамическое изменение масштаба при перекатывании
            # Сжатие при контакте с землей, растяжение в воздухе
            squash = math.sin(progress * math.pi * rotations * 2) * 0.15
            
            # Больше сжатия по Z, меньше по X/Y
            if scale_enabled:
                obj.scale = Vector((
                    original_scale.x * (1.0 + squash * 0.3),
                    original_scale.y * (1.0 - squash * 0.2),
                    original_scale.z * (1.0 - squash * 0.5)
                ))
            else:
                obj.scale = original_scale

            _keyframe_object(obj, frame)
            depsgraph = bpy.context.evaluated_depsgraph_get()
            depsgraph.update()
        
        if prev_mode != 'OBJECT':
            bpy.ops.object.mode_set(mode=prev_mode)
        
        debug_print("Roll animation completed")
        return True
        
    except Exception as e:
        debug_print(f"Roll error: {e}")
        return False


def generate_fly(obj, settings, **kwargs):
    """
    Полет по процедурной кривой (Smooth Trajectory Flight).
    Генерирует плавный путь через пространство с наслоением турбулентности
    и автоматическим расчетом вращения (Banking) по вектору движения.
    """
    try:
        debug_print(f"[Fly] Start procedural trajectory for '{obj.name}'")

        if obj.animation_data:
            obj.animation_data_clear()

        prev_mode = bpy.context.mode
        if prev_mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')

        # --- 1. Настройка кадров ---
        fs = int(getattr(settings, "frame_start", bpy.context.scene.frame_start))
        fe = int(getattr(settings, "frame_end", bpy.context.scene.frame_end))
        if fe < fs:
            fe = fs + 24
        total_frames = max(1, fe - fs + 1)

        # --- 2. Параметры (с возможностью тонкой настройки через kwargs) ---
        seed = int(kwargs.get('seed', getattr(settings, "fly_seed", 12345)))

        # Амплитуды (размах движения)
        amp_v = float(kwargs.get('fly_amp', getattr(settings, "fly_amp", 0.5)))  # Вертикальный размах
        amp_l = float(
            kwargs.get('fly_lateral', getattr(settings, "fly_lateral", 1.0)))  # Боковой размах (шире для полета)
        dist_fwd = float(kwargs.get('fly_forward_dist', getattr(settings, "fly_forward_dist", 5.0)))

        # Скорость и детализация
        # Frequency теперь работает как "Global Speed Multiplier"
        speed_mult = float(kwargs.get('frequency', getattr(settings, "frequency", 1.0)))
        octaves = int(kwargs.get('fly_octaves', getattr(settings, "fly_octaves", 3)))

        # Сила вращения
        bank_str = float(
            kwargs.get('bank_strength', getattr(settings, "fly_bank_strength", 1.5)))  # Сила наклона в повороте
        pitch_str = float(
            kwargs.get('pitch_strength', getattr(settings, "fly_pitch_strength", 1.0)))  # Реакция на высоту

        # Дыхание
        scale_enabled = bool(kwargs.get('allow_scale_changes', getattr(settings, "allow_scale_changes", True)))
        breath_amp = float(kwargs.get('breath_amp', getattr(settings, "fly_breath_amp", 0.03)))

        # --- 3. Исходные данные ---
        orig_loc = obj.location.copy()
        orig_rot = obj.rotation_euler.copy()
        orig_scale = obj.scale.copy()

        # Определяем базис (куда направлен объект)
        try:
            fw = forward_vector_world(obj, settings, None)
            if fw.length < 1e-6: fw = Vector((0, 1, 0))
        except:
            fw = Vector((0, 1, 0))

        fw = fw.normalized()
        up = Vector((0, 0, 1))
        right = fw.cross(up).normalized()

        # Инициализация шума
        np = NoiseProvider(seed=seed)

        # --- 4. Вспомогательная функция расчета позиции в момент времени T ---
        def get_procedural_pos(t):
            """
            Возвращает смещение (Vector) для времени t.
            Комбинирует "Macro Path" (плавный маршрут) и "Micro Turbulence" (тряску).
            """
            # A. MACRO PATH (Основная траектория - низкая частота, большая амплитуда)
            # Частота 0.3 - медленные плавные изгибы
            macro_y = np.noise1d(t * 0.3 + 100.0) * amp_l
            macro_z = np.noise1d(t * 0.25 + 200.0) * amp_v

            # B. MICRO TURBULENCE (Детали - высокая частота, малая амплитуда)
            # Частота 1.5 - мелкая дрожь, амплитуда зависит от octaves
            micro_amp_factor = 0.15 * (octaves / 4.0)  # Чем больше октав, тем заметнее турбулентность

            # Используем fbm для "шершавости"
            turb_y = np.fbm1d(t * 1.5, octaves=max(1, octaves - 1)) * amp_l * micro_amp_factor
            turb_z = np.fbm1d(t * 1.7 + 50.0, octaves=max(1, octaves - 1)) * amp_v * micro_amp_factor

            # C. Forward movement (прогресс вперед)
            # Просто линейное движение, шум добавляется к позиции
            # Но можно добавить джиттер и сюда
            turb_fwd = np.noise1d(t * 2.0 + 300.0) * (dist_fwd * 0.02)

            return Vector((turb_fwd, macro_y + turb_y, macro_z + turb_z))

        # --- 5. Генерация кадров ---
        # Look-ahead delta: насколько далеко вперед смотреть для расчета поворота (в секундах шума)
        # Больше значение = плавнее повороты, меньше = резче реакция
        look_ahead = 0.15

        for i in range(total_frames):
            frame = fs + i
            bpy.context.scene.frame_set(frame)

            # Нормализованный прогресс (0.0 -> 1.0)
            progress = i / max(1, total_frames - 1)

            # Время для шума (масштабируем, чтобы анимация была не слишком быстрой/медленной)
            # База: за 60 кадров проходит ~2.5 единицы времени шума при частоте 1.0
            noise_time = progress * 2.5 * speed_mult * (total_frames / 60.0)

            # 1. Расчет позиции
            # Берем базовое линейное движение вперед
            linear_fwd_pos = fw * (dist_fwd * ease_in_out_cubic(progress))

            # Получаем смещение от шума (в локальных координатах: X=fwd, Y=right, Z=up)
            offset_vec_local = get_procedural_pos(noise_time)

            # Переводим локальное смещение шума в мировые координаты
            # offset_vec_local.x -> добавка к forward
            # offset_vec_local.y -> right
            # offset_vec_local.z -> up (global Z)

            final_pos = (orig_loc +
                         linear_fwd_pos +
                         fw * offset_vec_local.x +
                         right * offset_vec_local.y +
                         Vector((0, 0, offset_vec_local.z)))

            obj.location = final_pos

            # 2. Расчет вращения (Banking & Pitching)
            # Смотрим немного вперед по времени шума, чтобы понять, куда загибается кривая
            offset_future = get_procedural_pos(noise_time + look_ahead)

            # Вектор "скорости" изменения шума (производная)
            delta_vec = offset_future - offset_vec_local

            # -- Pitch (Тангаж) --
            # Если delta.z положительный -> мы летим вверх -> нос вверх (отрицательный X вращение или как настроено)
            # pitch_str регулирует силу
            target_pitch = delta_vec.z * pitch_str * 2.0

            # -- Roll/Bank (Крен) --
            # Если мы смещаемся вправо (delta.y > 0), самолет должен накрениться вправо (roll)
            # bank_str регулирует силу
            target_bank = delta_vec.y * bank_str * 2.0

            # -- Yaw (Рыскание) --
            # Небольшой поворот носа в сторону движения
            target_yaw = delta_vec.y * 0.5

            # Накладываем вращения поверх оригинальных
            # (Предполагаем порядок Euler XYZ, где X-pitch, Y-roll, Z-yaw - зависит от объекта,
            #  здесь делаем универсально для Blender Z-up)

            # Плавная интерполяция к углам (эмуляция инерции массы)
            # Но т.к. мы идем по look-ahead, это уже выглядит плавно.

            obj.rotation_euler = Vector((
                orig_rot.x + target_pitch,
                orig_rot.y + target_yaw,  # Рыскание добавляем к оси Z (в Blender Z - это Yaw для вертикальных объектов)
                orig_rot.z - target_bank
            # Крен (Roll) - часто это Y или локальный X, здесь пробуем Z-Roll (зависит от ориентации)
            ))

            # Если объект ориентирован по Y-forward, Z-up:
            # Rot X = Roll/Bank? Нет, Rot Y = Roll.
            # Давайте сделаем стандартную самолетную схему:
            # Roll (крен) это вращение вокруг оси движения (Y).
            # Pitch (тангаж) вокруг оси крыльев (X).
            # Yaw (рыскание) вокруг вертикали (Z).

            # Применяем к глобальным осям (упрощенно):
            # Крен (Bank) зависит от бокового смещения.
            # Мы используем простые добавки к Euler, предполагая, что объект смотрит примерно вдоль Y или X.
            # Для улучшения используем `right` вектор для крена.

            # Простая модель для большинства Blender ригов:
            obj.rotation_euler.x = orig_rot.x + target_pitch
            obj.rotation_euler.y = orig_rot.y + target_bank  # Обычно Y - это ось крена для Y-forward объекта
            obj.rotation_euler.z = orig_rot.z - target_yaw * 0.5  # Инверсия рыскания часто нужна

            # 3. Дыхание (Scale)
            if scale_enabled:
                # Используем отдельную частоту для дыхания
                br = math.sin(noise_time * 3.0) * breath_amp
                obj.scale = Vector((
                    orig_scale.x * (1.0 + br * 0.5),
                    orig_scale.y * (1.0 - br * 0.2),  # Stretch/Squash conservation
                    orig_scale.z * (1.0 + br)
                ))

            # Keyframes
            obj.keyframe_insert(data_path="location", frame=frame)
            obj.keyframe_insert(data_path="rotation_euler", frame=frame)
            if scale_enabled:
                obj.keyframe_insert(data_path="scale", frame=frame)

            depsgraph = bpy.context.evaluated_depsgraph_get()
            depsgraph.update()

        if not scale_enabled:
            _remove_scale_fcurves(obj)

        if prev_mode != 'OBJECT':
            bpy.ops.object.mode_set(mode=prev_mode)

        debug_print(f"[Fly] Procedural flight completed. Seed: {seed}")
        return True

    except Exception as e:
        debug_print(f"[Fly] ERROR: {e}")
        import traceback
        traceback.print_exc()
        return False


# simple_unit_anim_integrated.py - FIX FOR DEATH ANIMATIONS

def generate_death(obj, settings, **kwargs):
    """
    Исправленная анимация смерти.
    Убрана ошибка накопления трансформаций (double integration bug).
    Теперь рассчитывается абсолютное смещение от start_pose для каждого кадра.
    """
    try:
        debug_print(f"Death animation for {obj.name} (FIXED & JUICED)")

        target = get_animation_target(obj, settings)
        if obj.animation_data:
            obj.animation_data_clear()

        prev_mode = bpy.context.mode
        if prev_mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')

        frame_start, frame_end = _get_frame_bounds(settings)
        total_frames = max(1, frame_end - frame_start + 1)

        death_style = kwargs.get('death_style', getattr(settings, "death_style", "COLLAPSE")).upper()

        # Сохраняем исходные трансформации (Rest/Start Pose)
        if isinstance(target, bpy.types.PoseBone):
            original_location = target.location.copy()
            original_rotation = target.rotation_quaternion.copy()
        else:
            original_location = obj.location.copy()
            original_rotation = obj.rotation_euler.copy()

        # Вектора направления
        forward = _get_forward_vector(obj, settings)
        up = Vector((0, 0, 1))
        # Lateral (вправо)
        lateral = forward.cross(up).normalized()
        # Корректируем forward, чтобы он был перпендикулярен UP (плоскость земли)
        forward = up.cross(lateral).normalized()

        # Словарь функций стратегий
        strategies = {
            "COLLAPSE": _death_collapse_physics,
            "EXPLOSION": _death_explosion_physics,
            "FALL": _death_fall_physics,
            "SIDE_TUMBLE": _death_side_tumble_physics,
            "KNOCKBACK": _death_knockback_physics
        }

        func = strategies.get(death_style, _death_collapse_physics)

        success = func(
            obj, target, original_location, original_rotation,
            forward, lateral, up,
            frame_start, frame_end, total_frames,
            settings, kwargs
        )

        if prev_mode != bpy.context.mode:
            bpy.ops.object.mode_set(mode=prev_mode)

        return success

    except Exception as e:
        debug_print(f"Death animation error: {e}")
        import traceback
        traceback.print_exc()
        return False


def _death_side_tumble_physics(obj, target, original_location, original_rotation,
                               forward, lateral, up,
                               frame_start, frame_end, total_frames,
                               settings, kwargs):
    """
    SIDE_TUMBLE: Исправленная версия.
    Заваливание на бок с легким подскоком основания и скольжением.
    """
    # Параметры
    side_angle = kwargs.get('side_angle', getattr(settings, "death_side_angle", 60.0))
    # Wobble теперь влияет на начальную "дрожь" перед падением
    wobble_amount = kwargs.get('side_wobble', getattr(settings, "death_side_wobble", 0.15))
    slide_distance = kwargs.get('side_slide', getattr(settings, "death_side_slide", 0.3))

    import random
    # Детерминированный рандом для направления
    r_seed = int(original_location.x * 100 + original_location.y * 10)
    random.seed(r_seed)
    fall_side = -1 if random.random() < 0.5 else 1

    # Фазы анимации (в процентах)
    p_anticipate = 0.2  # Подготовка (покачивание)
    p_fall = 0.4  # Сама фаза падения
    p_bounce = 0.4  # Отскок и успокоение

    frames_ant = int(total_frames * p_anticipate)
    frames_fall = int(total_frames * p_fall)
    frames_bounce = total_frames - frames_ant - frames_fall

    start_fall = frames_ant
    start_bounce = frames_ant + frames_fall

    for i in range(total_frames):
        frame = frame_start + i
        _scene_set_frame(frame)

        # Рассчитываем смещения (Offset) от ОРИГИНАЛА для текущего кадра
        offset_loc = Vector((0, 0, 0))
        offset_rot_euler = Vector((0, 0, 0))  # Локальное вращение XYZ

        if i < start_fall:
            # === ФАЗА 1: Anticipation (Потеря равновесия) ===
            t = i / max(1, frames_ant)
            # Легкое покачивание в противоположную сторону перед падением
            lean = math.sin(t * math.pi) * math.radians(5.0) * -fall_side

            offset_rot_euler.y = lean
            # Немного приседаем или приподнимаемся
            offset_loc.z = math.sin(t * math.pi * 2) * 0.02 * wobble_amount

        elif i < start_bounce:
            # === ФАЗА 2: Fall (Падение) ===
            t = (i - start_fall) / max(1, frames_fall)
            eased_t = ease_in_out_cubic(t)  # Используем кубик для разгона

            # Основной угол падения
            target_angle = math.radians(side_angle) * fall_side
            current_angle = target_angle * eased_t

            offset_rot_euler.y = current_angle

            # Смещение центра масс (чтобы не вращался вокруг пяток, а падал)
            # При падении на бок центр немного смещается в сторону падения и вниз
            arc_side = math.sin(t * math.pi * 0.5) * slide_distance * 0.5 * fall_side
            offset_loc += lateral * arc_side

            # Удар об землю в конце (эмуляция)
            if t > 0.8:
                shake = math.sin(t * 50.0) * 0.05 * (1.0 - t)
                offset_loc += Vector((shake, shake, 0))

        else:
            # === ФАЗА 3: Impact & Settle (Удар и затухание) ===
            t = (i - start_bounce) / max(1, frames_bounce)

            target_angle = math.radians(side_angle) * fall_side

            # Overshoot (пружинистость при ударе)
            # Затухающая синусоида вокруг целевого угла
            elasticity = 0.2 * wobble_amount
            bounce = math.sin(t * math.pi * 3) * elasticity * math.exp(-t * 3)

            offset_rot_euler.y = target_angle + bounce

            # Скольжение по инерции
            final_slide = slide_distance * fall_side
            # Линейная интерполяция скольжения с замедлением
            slide_curr = slide_distance * 0.5 * fall_side + (final_slide - slide_distance * 0.5 * fall_side) * (
                        1.0 - (1.0 - t) ** 2)

            offset_loc += lateral * slide_curr

            # Легкий "стук" об пол (вертикальная дрожь)
            if t < 0.3:
                v_bump = abs(math.sin(t * math.pi * 4)) * 0.05 * math.exp(-t * 5)
                offset_loc.z = v_bump

        # === ПРИМЕНЕНИЕ (БЕЗ НАКОПЛЕНИЯ) ===
        # Передаем рассчитанные offset_loc и offset_rot_euler
        _apply_death_transformation(
            obj, target, original_location, original_rotation,
            offset_loc, offset_rot_euler, frame, lateral  # Передаем lateral как ось Y для вращения
        )

        depsgraph = bpy.context.evaluated_depsgraph_get()
        depsgraph.update()

    return True


def _death_collapse_physics(obj, target, original_location, original_rotation,
                            forward, lateral, up,
                            frame_start, frame_end, total_frames,
                            settings, kwargs):
    """
    COLLAPSE: Схлопывание вниз. Исправлено.
    """
    power = kwargs.get('collapse_power', getattr(settings, "collapse_power", 0.7))
    shake_intensity = kwargs.get('collapse_shake', getattr(settings, "collapse_shake", 0.15))

    frames_tremble = int(total_frames * 0.25)
    frames_drop = int(total_frames * 0.5)

    for i in range(total_frames):
        frame = frame_start + i
        _scene_set_frame(frame)

        offset_loc = Vector((0, 0, 0))
        offset_rot = Vector((0, 0, 0))

        if i < frames_tremble:
            # Агония
            t = i / frames_tremble
            freq = 15.0 + t * 10.0
            amp = shake_intensity * t  # Нарастание

            offset_loc.x = math.sin(t * freq) * amp * 0.1
            offset_loc.y = math.cos(t * freq * 0.8) * amp * 0.1
            offset_rot.z = math.sin(t * freq * 1.2) * amp * 0.5

        else:
            # Падение
            t_total = (i - frames_tremble) / (total_frames - frames_tremble)
            # Ускоренное падение (t^2)
            drop_t = min(1.0, (i - frames_tremble) / frames_drop)
            z_drop = -power * (drop_t ** 2)

            # Случайный поворот при падении
            rot_spin = math.radians(90) * power * drop_t

            # Отскок в конце
            if drop_t >= 1.0:
                bounce_t = (i - frames_tremble - frames_drop) / max(1, total_frames - frames_tremble - frames_drop)
                z_drop = -power + abs(math.sin(bounce_t * math.pi * 2)) * 0.1 * math.exp(-bounce_t * 4)
                # Оседание на бок
                rot_spin += math.sin(bounce_t * math.pi) * 0.1

            offset_loc.z = z_drop
            offset_rot.x = rot_spin * 0.3  # Наклон вперед
            offset_rot.z = rot_spin * 0.7  # Закручивание

        _apply_death_transformation(
            obj, target, original_location, original_rotation,
            offset_loc, offset_rot, frame
        )

    return True


def _death_knockback_physics(obj, target, original_location, original_rotation,
                             forward, lateral, up,
                             frame_start, frame_end, total_frames,
                             settings, kwargs):
    """
    KNOCKBACK: Отлет назад по параболе. Исправлено.
    """
    dist = kwargs.get('knockback_distance', getattr(settings, "death_knockback_distance", 1.2))
    height = kwargs.get('knockback_height', getattr(settings, "death_knockback_height", 0.2))

    for i in range(total_frames):
        frame = frame_start + i
        _scene_set_frame(frame)

        t = i / total_frames

        offset_loc = Vector((0, 0, 0))
        offset_rot = Vector((0, 0, 0))

        # Полет назад (Easing Out - быстро в начале)
        move_t = 1.0 - (1.0 - t) ** 3
        current_dist = -dist * move_t

        # Высота (Parabola)
        # Подпрыгивает в первой половине, падает во второй
        if t < 0.6:
            h_t = t / 0.6
            current_height = math.sin(h_t * math.pi) * height
        else:
            # Отскок
            bounce_t = (t - 0.6) / 0.4
            current_height = abs(math.sin(bounce_t * math.pi * 2)) * height * 0.2 * (1.0 - bounce_t)

        # Вращение (сальто назад или просто наклон)
        spin = math.radians(45) * move_t

        offset_loc = forward * current_dist
        offset_loc.z = current_height

        # Вращение вокруг Lateral оси (сальто назад)
        # Мы передадим это в apply как Euler X (или используем axis logic)
        # Для простоты сформируем вектор вращения, где Y - это ось вращения
        # Но apply ожидает Euler XYZ. Lateral вращение зависит от ориентации.
        # Чтобы сделать правильно через global_delta_to_bone_local, лучше передать ось.
        # В этой упрощенной версии передадим как X наклон (pitch back)

        offset_rot.x = -spin  # Pitch back

        _apply_death_transformation(
            obj, target, original_location, original_rotation,
            offset_loc, offset_rot, frame, axis_override=lateral
        )

    return True


def _death_fall_physics(obj, target, original_location, original_rotation,
                        forward, lateral, up,
                        frame_start, frame_end, total_frames,
                        settings, kwargs):
    """
    FALL: Простое падение вперед/назад лицом.
    """
    import random
    random.seed(int(original_location.x * 100))
    direction = 1 if random.random() > 0.5 else -1  # 1 = forward, -1 = backward

    angle = 80.0

    for i in range(total_frames):
        frame = frame_start + i
        _scene_set_frame(frame)
        t = i / total_frames

        # Easing для падения (медленно в начале, быстро в конце)
        fall_t = t * t

        current_angle = math.radians(angle) * fall_t * direction

        offset_loc = Vector((0, 0, 0))
        offset_rot = Vector((0, 0, 0))

        # Смещение центра при падении (Pivot offset illusion)
        # Если падаем вперед, тело уходит немного вперед
        pivot_off = math.sin(fall_t * math.pi / 2) * 0.5 * direction
        offset_loc += forward * pivot_off

        # Удар в конце (bounce)
        if t > 0.8:
            b_t = (t - 0.8) / 0.2
            bounce_ang = math.sin(b_t * math.pi * 3) * math.radians(10) * (1.0 - b_t)
            current_angle += bounce_ang
            # Тряска позиции
            offset_loc.z += math.sin(b_t * 20) * 0.05 * (1.0 - b_t)

        # Падаем "вокруг" оси Lateral (Y в локальных, если Y-forward?? Нет, X pitch)
        # Передаем lateral как ось вращения

        _apply_death_transformation(
            obj, target, original_location, original_rotation,
            offset_loc, Vector((current_angle, 0, 0)), frame, axis_override=lateral
        )

    return True


def _death_explosion_physics(obj, target, original_location, original_rotation,
                             forward, lateral, up,
                             frame_start, frame_end, total_frames,
                             settings, kwargs):
    """
    EXPLOSION: Подлет вверх с вращением и падение.
    """
    power = kwargs.get('explosion_power', getattr(settings, "explosion_power", 0.8))

    for i in range(total_frames):
        frame = frame_start + i
        _scene_set_frame(frame)
        t = i / total_frames

        offset_loc = Vector((0, 0, 0))
        offset_rot = Vector((0, 0, 0))

        # Взлет и падение
        if t < 0.3:
            # Взлет
            sub_t = t / 0.3
            z = math.sin(sub_t * math.pi / 2) * power
        else:
            # Падение
            sub_t = (t - 0.3) / 0.7
            # Падение ниже нуля (если возможно) или до пола
            z = power * (1.0 - sub_t ** 2)
            if z < -0.2: z = -0.2  # Пол

            # Дрожь на полу
            if z <= -0.19:
                z += math.sin(t * 50) * 0.02 * (1.0 - t)

        offset_loc.z = z

        # Хаотичное вращение
        rot_mult = 5.0
        offset_rot.x = t * rot_mult
        offset_rot.y = t * rot_mult * 0.7
        offset_rot.z = t * rot_mult * 1.2

        _apply_death_transformation(
            obj, target, original_location, original_rotation,
            offset_loc, offset_rot, frame
        )

    return True


def _apply_death_transformation(obj, target, original_location, original_rotation,
                                offset_loc, offset_rot_euler, frame, axis_override=None):
    """
    Универсальное применение смещения.
    Args:
        offset_loc: Vector (смещение в мировых координатах относительно original_location)
        offset_rot_euler: Vector (Euler углы смещения).
        axis_override: Vector (Опционально. Если задан, то offset_rot_euler.x/y считаются углом вокруг этой оси).
    """

    # 1. Формируем глобальную дельту вращения (ось-угол)
    if axis_override:
        # Спец режим: используем переданную ось (например Lateral)
        # Берем длину вектора вращения или Y компоненту как угол
        angle = offset_rot_euler.length
        if offset_rot_euler.y != 0:
            angle = offset_rot_euler.y
        elif offset_rot_euler.x != 0:
            angle = offset_rot_euler.x

        rot_axis = axis_override
        rot_angle = angle
    else:
        # Стандарт: Euler -> Axis/Angle
        delta_quat = Euler(offset_rot_euler, 'XYZ').to_quaternion()
        if delta_quat.angle > 0.0001:
            rot_axis, rot_angle = delta_quat.to_axis_angle()
        else:
            rot_axis, rot_angle = Vector((0, 0, 1)), 0.0

    # 2. Применяем в зависимости от типа цели
    if isinstance(target, bpy.types.PoseBone):
        # === ВАЖНО: Используем global_delta_to_bone_local ===
        # Она преобразует ГЛОБАЛЬНЫЕ смещения (offset_loc) в ЛОКАЛЬНЫЕ координаты кости
        # с учетом её текущих родителей и ориентации.

        delta_local_loc, local_rot = global_delta_to_bone_local(
            arm_obj=obj,
            pose_bone=target,
            delta_global_loc=offset_loc,
            delta_global_rot_axis=rot_axis,
            delta_global_rot_angle=rot_angle
        )

        # Абсолютная позиция = Оригинал + Локальная Дельта
        target.location = original_location + delta_local_loc

        if local_rot is not None:
            # Для вращения: применяем дельту к оригиналу
            # (local_rot @ original) или (original @ local_rot) зависит от порядка,
            # обычно local_rot @ original работает для перемножения вращений

            # Смена режима, если надо
            if target.rotation_mode != 'QUATERNION':
                target.rotation_mode = 'QUATERNION'

            target.rotation_quaternion = (local_rot @ original_rotation).normalized()

        target.keyframe_insert(data_path="location", frame=frame)
        target.keyframe_insert(data_path="rotation_quaternion", frame=frame)

    else:
        # === ОБЪЕКТ ===
        # Тут просто прибавляем глобальные смещения
        obj.location = original_location + offset_loc

        # Вращение
        if hasattr(obj, 'rotation_euler'):
            # Примитивное сложение эйлеров (для простоты), лучше через кватернионы
            # Но для death animations сойдет
            obj.rotation_euler.x = original_rotation.x + offset_rot_euler.x
            obj.rotation_euler.y = original_rotation.y + offset_rot_euler.y
            obj.rotation_euler.z = original_rotation.z + offset_rot_euler.z

            # Если был axis override, надо считать сложнее, но пока оставим так
            # Для SIDE_TUMBLE (axis_override=Lateral) это может быть неточно для объекта,
            # если он повернут.
            if axis_override:
                # Более точный метод для объекта с произвольной осью
                base_q = original_rotation.to_quaternion()
                rot_q = Quaternion(axis_override, rot_angle)
                final_q = rot_q @ base_q
                obj.rotation_euler = final_q.to_euler(obj.rotation_mode)

        _keyframe_object(obj, frame)


def _apply_keyframe(target, obj, frame):
    """Вставка ключевых кадров"""
    if isinstance(target, bpy.types.PoseBone):
        target.keyframe_insert(data_path="location", frame=frame)
        target.keyframe_insert(data_path="rotation_quaternion", frame=frame)
    else:
        _keyframe_object(obj, frame)


def generate_stun(obj, settings, **kwargs):
    """Анимация оглушения - потеря равновесия и дрожь"""
    try:
        debug_print(f"Stun animation for {obj.name}")

        # Получаем цель анимации
        target = get_animation_target(obj, settings)

        if obj.animation_data:
            obj.animation_data_clear()

        prev_mode = bpy.context.mode
        if prev_mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')

        frame_start, frame_end = _get_frame_bounds(settings)
        total_frames = max(1, frame_end - frame_start + 1)

        # Параметры оглушения
        stagger_amount = kwargs.get('stun_stagger', getattr(settings, "stun_stagger", 0.8))
        shake_intensity = kwargs.get('stun_shake', getattr(settings, "stun_shake", 0.15))
        recovery_speed = kwargs.get('stun_recovery', getattr(settings, "stun_recovery", 0.3))

        # Сохраняем исходные трансформации
        if isinstance(target, bpy.types.PoseBone):
            original_location = target.location.copy()
            original_rotation = target.rotation_quaternion.copy()
            original_scale = target.scale.copy()
        else:
            original_location = obj.location.copy()
            original_rotation = obj.rotation_euler.copy()
            original_scale = obj.scale.copy()

        # Вектора направления
        forward = _get_forward_vector(obj, settings)
        up = Vector((0, 0, 1))
        lateral = forward.cross(up).normalized()

        # Разбиваем на фазы
        stagger_frames = int(total_frames * 0.3)  # Потеря равновесия
        shake_frames = int(total_frames * 0.4)  # Дрожь и покачивание
        recover_frames = total_frames - stagger_frames - shake_frames  # Восстановление

        # Случайное направление для реалистичности
        import random
        random.seed(hash(obj.name) % 1000)
        stagger_dir = random.uniform(-1, 1)

        for i in range(total_frames):
            frame = frame_start + i
            _scene_set_frame(frame)

            if i < stagger_frames:
                # Фаза 1: Потеря равновесия
                phase = i / stagger_frames

                # Движение назад с покачиванием
                stagger_back = math.sin(phase * math.pi) * stagger_amount * 0.5

                # Боковое покачивание
                stagger_side = math.sin(phase * math.pi * 1.5) * stagger_amount * stagger_dir * 0.3

                # Вращение при потере равновесия
                rotation_tilt = math.sin(phase * math.pi) * math.radians(15) * stagger_dir
                rotation_lean = math.sin(phase * math.pi * 0.7) * math.radians(10)

                # Общее смещение
                delta_global_loc = (-forward * stagger_back +
                                    lateral * stagger_side)

                if isinstance(target, bpy.types.PoseBone):
                    # Для кости
                    delta_local_loc, local_rot = global_delta_to_bone_local(
                        arm_obj=obj,
                        pose_bone=target,
                        delta_global_loc=delta_global_loc,
                        delta_global_rot_axis=lateral,
                        delta_global_rot_angle=rotation_tilt + rotation_lean
                    )

                    target.location = original_location + delta_local_loc

                    if local_rot is not None:
                        old_mode = target.rotation_mode
                        if old_mode != 'QUATERNION':
                            target.rotation_mode = 'QUATERNION'
                        target.rotation_quaternion = (local_rot @ original_rotation).normalized()
                        target.rotation_mode = old_mode

                    target.keyframe_insert(data_path="location", frame=frame)
                    target.keyframe_insert(data_path="rotation_quaternion", frame=frame)

                else:
                    # Для объекта
                    obj.location = original_location + delta_global_loc
                    obj.rotation_euler.x = original_rotation.x + rotation_lean
                    obj.rotation_euler.z = original_rotation.z + rotation_tilt

                    _keyframe_object(obj, frame)

            elif i < stagger_frames + shake_frames:
                # Фаза 2: Дрожь и покачивание
                phase = (i - stagger_frames) / shake_frames

                # Многоканальная дрожь
                time = phase * 20.0

                shake_x = (math.sin(time * 7.3) * 0.5 + math.cos(time * 5.7) * 0.5) * shake_intensity
                shake_y = (math.sin(time * 6.1) * 0.7 + math.cos(time * 4.3) * 0.3) * shake_intensity
                shake_z = (math.sin(time * 8.2) * 0.4 + math.cos(time * 6.9) * 0.6) * shake_intensity * 0.5

                # Вращательная дрожь
                rot_x = math.sin(time * 9.1) * shake_intensity * 0.3
                rot_y = math.cos(time * 7.5) * shake_intensity * 0.2
                rot_z = math.sin(time * 8.7) * shake_intensity * 0.4

                # Затухание дрожи со временем
                decay = 1.0 - phase * 0.7

                delta_global_loc = Vector((shake_x, shake_y, shake_z)) * decay

                if isinstance(target, bpy.types.PoseBone):
                    # Для кости
                    delta_rot_euler = Euler((rot_x, rot_y, rot_z), 'XYZ') * decay
                    delta_rot_quat = delta_rot_euler.to_quaternion()

                    if delta_rot_quat.angle > 0.0001:
                        rot_axis, rot_angle = delta_rot_quat.to_axis_angle()
                    else:
                        rot_axis, rot_angle = Vector((0, 0, 1)), 0.0

                    delta_local_loc, local_rot = global_delta_to_bone_local(
                        arm_obj=obj,
                        pose_bone=target,
                        delta_global_loc=delta_global_loc,
                        delta_global_rot_axis=rot_axis,
                        delta_global_rot_angle=rot_angle
                    )

                    # Получаем положение после потери равновесия
                    if i == stagger_frames:  # Первый кадр дрожи
                        stun_location = target.location.copy()
                        stun_rotation = target.rotation_quaternion.copy()

                    target.location = stun_location + delta_local_loc

                    if local_rot is not None:
                        old_mode = target.rotation_mode
                        if old_mode != 'QUATERNION':
                            target.rotation_mode = 'QUATERNION'
                        target.rotation_quaternion = (local_rot @ stun_rotation).normalized()
                        target.rotation_mode = old_mode

                    target.keyframe_insert(data_path="location", frame=frame)
                    target.keyframe_insert(data_path="rotation_quaternion", frame=frame)

                else:
                    # Для объекта
                    obj.location.x = original_location.x + delta_global_loc.x
                    obj.location.y = original_location.y + delta_global_loc.y
                    obj.location.z = original_location.z + delta_global_loc.z

                    obj.rotation_euler.x = original_rotation.x + rot_x * decay
                    obj.rotation_euler.y = original_rotation.y + rot_y * decay
                    obj.rotation_euler.z = original_rotation.z + rot_z * decay

                    _keyframe_object(obj, frame)

            else:
                # Фаза 3: Восстановление
                phase = (i - stagger_frames - shake_frames) / recover_frames

                # Плавное возвращение к исходному положению
                recovery_t = ease_in_out_cubic(phase)

                if isinstance(target, bpy.types.PoseBone):
                    # Плавная интерполяция для кости
                    current_loc = target.location.copy()
                    current_rot = target.rotation_quaternion.copy()

                    target.location = current_loc.lerp(original_location, recovery_t)
                    target.rotation_quaternion = current_rot.slerp(original_rotation, recovery_t)

                    target.keyframe_insert(data_path="location", frame=frame)
                    target.keyframe_insert(data_path="rotation_quaternion", frame=frame)

                else:
                    # Плавная интерполяция для объекта
                    obj.location = obj.location.lerp(original_location, recovery_t)

                    current_euler = obj.rotation_euler.copy()
                    target_euler = original_rotation.copy()

                    obj.rotation_euler.x = current_euler.x * (1 - recovery_t) + target_euler.x * recovery_t
                    obj.rotation_euler.y = current_euler.y * (1 - recovery_t) + target_euler.y * recovery_t
                    obj.rotation_euler.z = current_euler.z * (1 - recovery_t) + target_euler.z * recovery_t

                    _keyframe_object(obj, frame)

            depsgraph = bpy.context.evaluated_depsgraph_get()
            depsgraph.update()

        if prev_mode != 'OBJECT':
            bpy.ops.object.mode_set(mode=prev_mode)

        debug_print("Stun animation completed")
        return True

    except Exception as e:
        debug_print(f"Stun error: {e}")
        import traceback
        traceback.print_exc()
        return False

