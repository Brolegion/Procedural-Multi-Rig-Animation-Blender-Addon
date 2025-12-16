# simple_unit_anim_integrated.py - Восстановлено покачивание с поддержкой фаз

import bpy
import math
from mathutils import Vector, Euler, Quaternion
from .noise_provider import NoiseProvider
import random  # Не забудь добавить импорт в начале файла simple_unit_anim.py
from .utils import (
    clamp, ease_in_out_cubic, interpolate_spring,
    forward_vector_world, compute_phases, compute_weights, bezier_y_for_x
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
    """Idle с улучшенным процедурным шумом"""
    try:
        debug_print(f"Enhanced Idle with noise for {obj.name}")

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
        stiffness = kwargs.get('idle_stiffness', getattr(settings, "idle_stiffness", 8.0))
        damping = kwargs.get('idle_damping', getattr(settings, "idle_damping", 0.7))

        # Параметры шума
        use_noise = getattr(settings, "use_noise_idle", True)
        noise_seed = getattr(settings, "idle_noise_seed", 12345)
        noise_amp_vertical = getattr(settings, "idle_noise_amp_vertical", 0.08)
        noise_amp_horizontal = getattr(settings, "idle_noise_amp_horizontal", 0.04)
        noise_amp_rotation = getattr(settings, "idle_noise_amp_rotation", 3.0)
        noise_freq = getattr(settings, "idle_noise_frequency", 0.8)

        # Глобальный переключатель масштаба
        scale_enabled = bool(kwargs.get('allow_scale_changes', getattr(settings, "allow_scale_changes", True)))

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

            # Базовое вертикальное колебание (имитация дыхания)
            wobble = math.sin(phase_center)
            current_z = original_location.z + amp * wobble

            # Начальная позиция
            current_pos = Vector((original_location.x, original_location.y, current_z))
            current_rot = original_rotation.copy()

            # Добавляем процедурный шум если включен
            if use_noise:
                time_counter += time_step

                # 1. Вертикальный шум (медленные покачивания) с использованием FBM для гладкости
                noise_z = np.fbm1d(time_counter * 0.3, octaves=3) * noise_amp_vertical

                # 2. Горизонтальный шум (боковые покачивания) - разные частоты для X и Y
                noise_x = np.fbm1d(time_counter * 0.4, octaves=2) * noise_amp_horizontal
                noise_y = np.fbm1d(time_counter * 0.35 + 10.0, octaves=2) * noise_amp_horizontal

                # 3. Вращательный шум (наклоны) - разные оси с разной частотой
                noise_rot_x = np.fbm1d(time_counter * 0.5) * math.radians(noise_amp_rotation)
                noise_rot_y = np.fbm1d(time_counter * 0.45 + 20.0) * math.radians(noise_amp_rotation * 0.3)
                noise_rot_z = np.fbm1d(time_counter * 0.6 + 30.0) * math.radians(noise_amp_rotation * 0.7)

                # Применяем шум к позиции (комбинируем с базовой анимацией)
                current_pos.x = original_location.x + noise_x
                current_pos.y = original_location.y + noise_y
                current_pos.z = current_z + noise_z  # Добавляем к базовому колебанию

                # Применяем шум к вращению
                current_rot.x = original_rotation.x + noise_rot_x
                current_rot.y = original_rotation.y + noise_rot_y
                current_rot.z = original_rotation.z + noise_rot_z

                # 4. Микро-дрожь (высокочастотный шум) - добавляет живости
                tremor = np.noise1d(time_counter * 8.0) * 0.003
                current_pos.x += tremor
                current_pos.y += tremor

            else:
                # Классическое покачивание без шума
                tilt_angle = math.radians(rot_deg) * math.sin(phase_center * 1.3)
                current_rot.x = original_rotation.x + tilt_angle
                current_rot.z = original_rotation.z + tilt_angle * 0.5

            # Применяем позицию и вращение
            obj.location = current_pos
            obj.rotation_euler = current_rot

            # Обрабатываем масштаб (легкое дыхание)
            if scale_enabled:
                # Очень легкое дыхание - синхронизировано с основной фазой
                breath_phase = phase_center * 0.3
                breath = 1.0 + math.sin(breath_phase) * 0.015  # Очень небольшое изменение
                obj.scale = Vector((
                    original_scale.x * breath,
                    original_scale.y * breath,
                    original_scale.z * breath * 1.02  # Слегка больше по Z для естественности
                ))
            else:
                obj.scale = original_scale

            # Ключевые кадры
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
    """Урон с отдачей и тряской"""
    try:
        debug_print(f"Damage for {obj.name}")
        
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
            
            obj.location = original_location + current_recoil + Vector((shake_x, shake_y, -recoil_z * recoil_progress))
            
            rot_shake = decay * 0.1 * math.sin(progress * shake_freq * 0.5)
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
        return False

def generate_dodge(obj, settings, **kwargs):
    """Уклонение с улучшенной интерполяцией"""
    try:
        debug_print(f"Dodge for {obj.name}")
        
        if obj.animation_data:
            obj.animation_data_clear()
        prev_mode = bpy.context.mode
        if prev_mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')
        
        frame_start, frame_end = _get_frame_bounds(settings)
        total = max(1, frame_end - frame_start + 1)
        
        lateral_dist = kwargs.get('lateral_dist', getattr(settings, "dodge_distance", 1.0))
        quick_frac = kwargs.get('quick_frac', getattr(settings, "dodge_quick_frac", 0.25))
        side = kwargs.get('dodge_side', getattr(settings, "dodge_side", "R")).upper()
        sign = -1.0 if side == "L" else 1.0
        
        forward = _get_forward_vector(obj, settings)
        up = Vector((0, 0, 1))
        lateral = forward.cross(up).normalized() * sign
        
        original_location = obj.location.copy()
        original_rotation = obj.rotation_euler.copy()
        
        dodge_frames = int(total * quick_frac)
        
        for i in range(total + 1):
            frame = frame_start + i
            _scene_set_frame(frame)
            progress = i / max(1, total)
            
            if i <= dodge_frames:
                phase_t = i / dodge_frames
                lateral_progress = ease_in_out_cubic(phase_t)
                lean_angle = math.radians(15.0) * lateral_progress * -sign
            else:
                phase_t = (i - dodge_frames) / (total - dodge_frames)
                lateral_progress = 1.0 - ease_in_out_cubic(phase_t)
                lean_angle = math.radians(15.0) * lateral_progress * -sign
            
            obj.location = original_location + lateral * (lateral_dist * lateral_progress)
            obj.rotation_euler.z = original_rotation.z + lean_angle
            
            _keyframe_object(obj, frame)
            depsgraph = bpy.context.evaluated_depsgraph_get()
            depsgraph.update()
        
        if prev_mode != 'OBJECT':
            bpy.ops.object.mode_set(mode=prev_mode)
        
        if obj.animation_data and obj.animation_data.action:
            debug_print(f"Keys created: {len(obj.animation_data.action.fcurves)} fcurves")
        else:
            debug_print("No animation_data or action created!")
        
        debug_print("Dodge animation completed")
        return True
        
    except Exception as e:
        debug_print(f"Dodge error: {e}")
        return False
        
        
        
# simple_unit_anim_integrated.py - ИСПРАВЛЕННЫЕ новые функции

# simple_unit_anim_integrated.py - ПОЛНОСТЬЮ ПЕРЕПИСАННЫЕ новые функции

def generate_surprise(obj, settings, **kwargs):
    """Внезапный прыжок от удивления - ПОЛНОСТЬЮ ПЕРЕПИСАНО"""
    try:
        debug_print(f"Surprise for {obj.name}")
        
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

        original_location = obj.location.copy()
        original_rotation = obj.rotation_euler.copy()
        original_scale = obj.scale.copy()
        
        # Анимация в 3 фазы: подготовка, прыжок, приземление
        prep_frames = int(total_frames * 0.15)  # Быстрая подготовка
        jump_frames = int(total_frames * 0.5)   # Основной прыжок
        land_frames = total_frames - prep_frames - jump_frames
        
        for i in range(total_frames):
            frame = frame_start + i
            _scene_set_frame(frame)
            
            if i < prep_frames:
                # Фаза 1: Быстрое приседание перед прыжком
                phase = i / prep_frames
                squash = math.sin(phase * math.pi * 0.5) * 0.4  # Быстрое сжатие
                if scale_enabled:
                    obj.scale = Vector((
                        original_scale.x * (1.0 + squash * 0.3),
                        original_scale.y * (1.0 + squash * 0.3),
                        original_scale.z * (1.0 - squash)
                    ))
                else:
                    obj.scale = original_scale
                obj.location = original_location
                
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
                if scale_enabled:
                    obj.scale = Vector((
                        original_scale.x * (1.0 - stretch * 0.2),
                        original_scale.y * (1.0 - stretch * 0.2),
                        original_scale.z * (1.0 + stretch)
                    ))
                else:
                    obj.scale = original_scale
                # Вращение от удивления
                spin = math.sin(phase * math.pi * 3) * 0.5
                obj.rotation_euler.z = original_rotation.z + spin
                
                obj.location.z = original_location.z + height
                
            else:
                # Фаза 3: Приземление с отскоком
                phase = (i - prep_frames - jump_frames) / land_frames
                
                # Несколько маленьких отскоков
                bounce1 = math.sin(phase * math.pi * 4) * 0.1 * math.exp(-phase * 3)
                bounce2 = math.sin(phase * math.pi * 8 + 1.5) * 0.05 * math.exp(-phase * 6)
                
                obj.location.z = original_location.z + (bounce1 + bounce2) * jump_height * 0.3
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
        return False

def generate_panic(obj, settings, **kwargs):
    """Паника - быстрая беспорядочная тряска - ПОЛНОСТЬЮ ПЕРЕПИСАНО"""
    try:
        debug_print(f"Panic for {obj.name}")
        
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
            
            # Пульсация масштаба (учащенное "дыхание")
            pulse_fast = math.sin(time * 25.0) * 0.08
            pulse_slow = math.sin(time * 8.0) * 0.04
            pulse = 1.0 + pulse_fast + pulse_slow
            
            # Дрожание (высокочастотное)
            tremor = math.sin(time * 50.0) * 0.02 * math.exp(-progress * 2)
            
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
        return False

def generate_shoot(obj, settings, **kwargs):
    """Стрельба с отдачей - ПОЛНОСТЬЮ ПЕРЕПИСАНО"""
    try:
        debug_print(f"Shoot for {obj.name}")
        
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
                
                obj.location = original_location - forward * recoil_amount
                if scale_enabled:
                    obj.scale = original_scale * flash
                else:
                    obj.scale = original_scale
                
                # Резкий толчок назад
                kick = -recoil_amount * 0.5
                obj.rotation_euler.x = original_rotation.x + kick
                
            else:
                # Фаза восстановления: плавное возвращение
                recovery_phase = (i - shot_frame) / (total_frames - shot_frame)
                recovery = (1.0 - ease_in_out_cubic(recovery_phase)) * recoil * 0.3
                
                # Небольшая вибрация после выстрела
                vibration = math.sin(recovery_phase * 20.0) * 0.02 * math.exp(-recovery_phase * 5)
                
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

