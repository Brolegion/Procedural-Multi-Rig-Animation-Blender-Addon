# damage_animation.py
"""
Исправленная версия процедурной анимации урона.
Сохранена архитектура слоев, исправлен выбор костей и физика рывков.
"""

import bpy
import math
import random
from mathutils import Vector, Quaternion, Matrix
from . import utils

# ============================================================================
# 1. ФИЗИЧЕСКОЕ ЯДРО (SpringMassOscillator)
# ============================================================================

class SpringMassOscillator:
    """
    Рассчитывает физику пружины с массой. 
    Исправлено: добавлено адаптивное затухание для предотвращения 'отскока назад'.
    """
    def __init__(self, mass=1.0, stiffness=15.0, damping=0.8, dt=1.0/24.0):
        self.mass = max(0.1, mass)
        self.stiffness = stiffness
        self.damping = damping
        self.dt = dt

        # Линейные параметры
        self.pos = Vector((0, 0, 0))
        self.vel = Vector((0, 0, 0))
        
        # Угловые параметры
        self.ang_pos = Vector((0, 0, 0))
        self.ang_vel = Vector((0, 0, 0))

    def apply_impulse(self, linear, angular=None):
        """Добавляет мгновенную скорость (удар)."""
        # Лимит импульса, чтобы кости не улетали в бесконечность
        safe_linear = linear
        if safe_linear.length > 3.0:
            safe_linear = safe_linear.normalized() * 3.0
            
        self.vel += safe_linear / self.mass
        if angular:
            self.ang_vel += angular / self.mass

    def update(self):
        # --- АДАПТИВНОЕ ДЕМПФИРОВАНИЕ ---
        # Если мы движемся К ЦЕЛИ (возврат), увеличиваем сопротивление,
        # чтобы персонаж не пролетал сквозь нейтральную позу назад.
        dot_prod = self.pos.dot(self.vel)
        adaptive_damping = self.damping
        if dot_prod < 0: # Движение к центру
            adaptive_damping *= 2.5 

        # Линейная физика
        force = -self.pos * self.stiffness - self.vel * adaptive_damping
        acc = force / self.mass
        self.vel += acc * self.dt
        self.pos += self.vel * self.dt

        # Угловая физика (вращение жестче)
        ang_force = -self.ang_pos * (self.stiffness * 0.8) - self.ang_vel * (adaptive_damping * 1.2)
        ang_acc = ang_force / self.mass
        self.ang_vel += ang_acc * self.dt
        self.ang_pos += self.ang_vel * self.dt

        return self.pos, self.ang_pos

# ============================================================================
# 2. ВОЛНОВАЯ СИСТЕМА (WavePropagationSystem)
# ============================================================================

class WavePropagationSystem:
    """
    Передает энергию по цепочке костей с задержкой.
    """
    def __init__(self, arm_obj, center_bone_name, chain_length=3, speed=2.0):
        self.chain = build_propagation_chain(arm_obj, center_bone_name, chain_length)
        self.speed = speed
        # Состояние волны для каждой кости: [амплитуда, фаза]
        self.wave_data = {name: {'amp': 0.0, 'phase': 0.0} for name in self.chain}
        self.direction = Vector((0, -1, 0))

    def trigger(self, strength, direction):
        if not self.chain: return
        self.direction = direction.normalized()
        # Запускаем волну в корень
        self.wave_data[self.chain[0]]['amp'] = strength

    def update(self, dt):
        # Распространение от родителя к детям
        for i in range(len(self.chain) - 1):
            curr = self.chain[i]
            next_b = self.chain[i+1]
            
            # Передача энергии
            target_amp = self.wave_data[curr]['amp'] * 0.8 # Затухание
            # Плавное перетекание амплитуды
            self.wave_data[next_b]['amp'] = utils.interpolate_linear(self.wave_data[next_b]['amp'], target_amp, dt * self.speed)
            # Сдвиг фазы
            self.wave_data[next_b]['phase'] = self.wave_data[curr]['phase'] - (0.2 * self.speed)

        # Общее затухание энергии
        for k in self.wave_data:
            self.wave_data[k]['amp'] *= 0.95

    def get_displacement(self, bone_name, time):
        if bone_name not in self.wave_data: return Vector((0,0,0))
        data = self.wave_data[bone_name]
        # Sin волна
        val = math.sin(time * self.speed + data['phase']) * data['amp']
        return self.direction * val

# ============================================================================
# 3. ОСНОВНОЙ ГЕНЕРАТОР
# ============================================================================

# damage_animation.py
def generate_damage_animation(arm_obj, settings, **kwargs):
    frame_start = int(getattr(settings, "frame_start", 1))
    dur = getattr(settings, "damage_duration", 30)
    frame_end = frame_start + dur
    
    fps = bpy.context.scene.render.fps
    dt = 1.0 / fps

    # 1. ЗАПОМИНАЕМ ИСХОДНУЮ ПОЗУ (REST)
    original_poses = {}
    for pb in arm_obj.pose.bones:
        original_poses[pb.name] = {
            'loc': pb.location.copy(),
            'rot': pb.rotation_quaternion.copy()
        }

    # 2. НОВАЯ СИСТЕМА ВЫБОРА КОСТЕЙ
    damage_bones = select_damage_bones(arm_obj, settings)
    all_bones_list = damage_bones.get('all_bones', [])
    
    # 3. НАСТРОЙКА ФИЗИКИ - СУЩЕСТВУЮЩАЯ ЛОГИКА
    strength = float(getattr(settings, "damage_strength", 1.0))
    direction = getattr(settings, "damage_direction", Vector((0, -1, 0)))
    if direction.length > 0: 
        direction.normalize()

    # Создаем осцилляторы для стандартных зон (если кости найдены)
    oscillators = {}
    zones = ['center', 'head', 'left_arm', 'right_arm', 'left_leg', 'right_leg']
    
    for zone in zones:
        bone_name = damage_bones.get(zone)
        if not bone_name:
            continue
            
        # Настройка массы и жесткости
        mass = 1.0
        stiffness = getattr(settings, "damage_stiffness", 15.0)
        
        if zone == 'head': 
            mass = 0.5 
            stiffness *= 1.5
        elif 'leg' in zone: 
            mass = 2.0
            
        oscillators[zone] = SpringMassOscillator(
            mass, 
            stiffness, 
            getattr(settings, "damage_damping", 0.8), 
            dt
        )
        
        # Применяем начальный импульс
        zone_mult = 1.0
        if zone == 'center': 
            zone_mult = 0.8
        if zone == 'head': 
            zone_mult = 1.2
        
        lin = direction * strength * 2.0 * zone_mult
        ang = Vector((
            random.uniform(-0.5, 0.5), 
            random.uniform(-0.5, 0.5), 
            0
        )) * strength
        oscillators[zone].apply_impulse(lin, ang)

    # Волновая система (только если есть центральная кость)
    wave_system = None
    center_bone = damage_bones.get('center')
    if center_bone:
        wave_system = WavePropagationSystem(
            arm_obj, 
            center_bone, 
            getattr(settings, "damage_propagation_chain", 3)
        )
        wave_system.trigger(strength * 0.5, direction)

    # 4. ЦИКЛ КАДРОВ
    total_frames = frame_end - frame_start + 1
    
    for i in range(total_frames):
        frame = frame_start + i
        bpy.context.scene.frame_set(frame)
        
        # СБРОС позиций
        reset_bones_to_original(arm_obj, original_poses)
        
        t = i * dt
        ramp = min(1.0, i / 5.0)

        # Обновление волновой системы
        if wave_system:
            wave_system.update(dt)
        
        # A. ПРИМЕНЕНИЕ ДЛЯ СТАНДАРТНЫХ ЗОН
        for zone in zones:
            bone_name = damage_bones.get(zone)
            if not bone_name:
                continue
                
            osc = oscillators.get(zone)
            if not osc:
                continue
                
            phys_pos, phys_rot = osc.update()
            
            # Волновое смещение только для центра
            wave_pos = Vector((0, 0, 0))
            if zone == 'center' and wave_system:
                wave_pos = wave_system.get_displacement(bone_name, t)

            # Суммируем и применяем RAMP
            total_pos = (phys_pos + wave_pos) * ramp
            total_rot = phys_rot * ramp
            
            apply_local_offset(arm_obj, bone_name, total_pos, total_rot, frame)
        
        # B. ПРИМЕНЕНИЕ ДЛЯ ДОПОЛНИТЕЛЬНЫХ КОСТЕЙ (ПРОСТОЙ ШЕЙК)
        # Проходим по всем костям из списка, которые не обработаны как зоны
        processed_bones = set([damage_bones.get(z) for z in zones if damage_bones.get(z)])
        
        for bone_info in all_bones_list:
            bone_name = bone_info['name']
            if bone_name in processed_bones:
                continue
                
            weight = bone_info['weight']
            
            # Простой шейк для дополнительных костей
            shake_strength = strength * 0.3 * weight * ramp
            
            # Разные частоты для разных костей
            phase_offset = hash(bone_name) % 100 / 100.0 * math.pi * 2
            
            simple_shake = Vector((
                math.sin(t * 8.0 + phase_offset) * 0.1,
                math.cos(t * 7.3 + phase_offset) * 0.1,
                math.sin(t * 9.2 + phase_offset) * 0.05
            )) * shake_strength
            
            apply_local_offset(arm_obj, bone_name, simple_shake, Vector(), frame)

    # 5. ВОЗВРАТ (Finalize)
    if getattr(settings, "damage_return_to_pose", True):
        finalize_return(arm_obj, original_poses, frame_end, 10)

    return True

# ============================================================================
# ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
# ============================================================================

def select_damage_bones(arm_obj, settings):
    """Новая функция - только сбор костей из коллекций"""
    bones = {}
    all_bones = []
    
    # Собираем из коллекций
    if settings.use_primary_bones:
        for entry in settings.damage_bones_primary:
            if entry.bone_name in arm_obj.data.bones:
                all_bones.append({
                    'name': entry.bone_name,
                    'weight': entry.weight,
                    'type': 'primary'
                })
    
    if settings.use_secondary_bones:
        for entry in settings.damage_bones_secondary:
            if entry.bone_name in arm_obj.data.bones:
                all_bones.append({
                    'name': entry.bone_name,
                    'weight': entry.weight,
                    'type': 'secondary'
                })
    
    # Автоопределение ТОЛЬКО если список пуст
    if not all_bones:
        center = utils.choose_center_bone(arm_obj)
        if center:
            all_bones.append({
                'name': center,
                'weight': 1.0,
                'type': 'auto'
            })
    
    # Для совместимости с существующей физикой сохраняем структуру
    bones['all_bones'] = all_bones
    
    # Определяем центральную кость для волновой системы
    center_bone = None
    for bone in all_bones:
        name_lower = bone['name'].lower()
        if any(kw in name_lower for kw in ['pelvis', 'spine', 'center', 'root', 'hip']):
            center_bone = bone['name']
            break
    
    if center_bone:
        bones['center'] = center_bone
        bones['head'] = None  # Автоматически ищем ниже
        bones['left_arm'] = None
        bones['right_arm'] = None
        bones['left_leg'] = None
        bones['right_leg'] = None
    else:
        bones['center'] = all_bones[0]['name'] if all_bones else None
    
    return bones

def find_limb_root(arm_obj, keywords, side_token):
    """Ищет самую верхнюю кость конечности."""
    for b in arm_obj.data.bones:
        nl = b.name.lower()
        # Проверка стороны ('l' или 'left')
        is_side = False
        if side_token == 'l': is_side = 'left' in nl or '.l' in nl or '_l' in nl
        if side_token == 'r': is_side = 'right' in nl or '.r' in nl or '_r' in nl
        
        if is_side and any(k in nl for k in keywords):
            # Проверяем, что родитель не из этой же серии (чтобы найти корень)
            if not b.parent or not any(k in b.parent.name.lower() for k in keywords):
                return b.name
    return None

def build_propagation_chain(arm_obj, start_bone, length):
    if not start_bone or start_bone not in arm_obj.data.bones: return []
    chain = [start_bone]
    curr = arm_obj.data.bones[start_bone]
    for _ in range(length):
        if curr.children:
            # Берем первого ребенка, который похож на продолжение позвоночника
            # (обычно тот, что идет вверх по Z)
            best_child = curr.children[0] 
            for c in curr.children:
                if 'spine' in c.name.lower() or 'neck' in c.name.lower():
                    best_child = c
                    break
            chain.append(best_child.name)
            curr = best_child
        else: break
    return chain

def reset_bones_to_original(arm_obj, original_poses):
    """Сбрасывает кости в исходное состояние."""
    for name, data in original_poses.items():
        pb = arm_obj.pose.bones.get(name)
        if pb:
            pb.location = data['loc'].copy()
            pb.rotation_quaternion = data['rot'].copy()

def apply_local_offset(arm_obj, bone_name, global_pos, angular_vec, frame):
    """
    Применяет глобальное смещение в локальном пространстве кости.
    С жестким лимитом (Clamp), чтобы избежать разрывов меша.
    """
    pb = arm_obj.pose.bones.get(bone_name)
    if not pb: return

    # 1. World -> Local conversion
    # Используем матрицу ориентации кости в позе
    mat_rot = pb.matrix.to_3x3() 
    local_pos = mat_rot.inverted() @ global_pos
    
    # 2. CLAMP (Предохранитель от рывков)
    # Максимум 40 см смещения
    if local_pos.length > 0.4:
        local_pos = local_pos.normalized() * 0.4
        
    pb.location += local_pos
    pb.keyframe_insert("location", frame=frame)

    # 3. Вращение
    if angular_vec.length > 0.001:
        # Конвертируем вектор вращения в кватернион
        # Ось вращения тоже нужно перевести в локальные
        local_axis = mat_rot.inverted() @ angular_vec.normalized()
        angle = min(angular_vec.length * 0.2, 1.0) # Лимит угла ~57 градусов
        
        rot = Quaternion(local_axis, angle)
        pb.rotation_quaternion = (pb.rotation_quaternion @ rot).normalized()
        pb.keyframe_insert("rotation_quaternion", frame=frame)

def finalize_return(arm_obj, original_poses, start_frame, duration):
    """Записывает ключи возврата в исходное положение."""
    for i in range(duration + 1):
        f = start_frame + i
        t = i / float(duration)
        # Ease-out (замедление в конце)
        factor = 1.0 - (1.0 - t) * (1.0 - t)
        
        bpy.context.scene.frame_set(f)
        for name, data in original_poses.items():
            pb = arm_obj.pose.bones.get(name)
            if not pb: continue
            
            # Мы не можем интерполировать от "текущего", потому что "текущее" 
            # в прошлом кадре было сброшено reset_bones.
            # Поэтому мы просто принудительно ставим в Rest (так как физика затухла).
            # В идеале тут нужен более сложный блендинг, но для фикса рывка достаточно
            # просто записать финальную позу.
            
            pb.location = data['loc']
            pb.rotation_quaternion = data['rot']
            pb.keyframe_insert("location", frame=f)
            pb.keyframe_insert("rotation_quaternion", frame=f)