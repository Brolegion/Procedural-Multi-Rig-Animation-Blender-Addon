import bpy
import math
import random
from mathutils import Vector, Quaternion, Matrix
from . import utils
from .noise_provider import get_default_noise_provider


# ==============================================================================
# ВСПОМОГАТЕЛЬНЫЙ КЛАСС ФИЗИКИ (копия из dodge, чтобы модуль был автономен)
# ==============================================================================
class RageDynamics:
    """Динамика второго порядка для резких входов в позу ярости"""

    def __init__(self, f, z, r, x0):
        self.k1 = z / (math.pi * f)
        self.k2 = 1 / ((2 * math.pi * f) ** 2)
        self.k3 = r * z / (2 * math.pi * f)
        self.xp = x0
        self.y = x0
        self.yd = Vector((0, 0, 0))

    def update(self, dt, x):
        if dt <= 0: return self.y
        xd = (x - self.xp) / dt
        self.xp = x
        k2_stable = max(self.k2, 1.1 * (dt * dt / 4 + dt * self.k1 / 2))
        self.y = self.y + dt * self.yd
        self.yd = self.yd + dt * (x + self.k3 * xd - self.y - self.k1 * self.yd) / k2_stable
        return self.y


# ==============================================================================
# ГЕНЕРАТОР ЯРОСТИ
# ==============================================================================

def generate_rage_animation(arm_obj, settings):
    """
    Генерирует анимацию агрессии/ярости.
    Поддерживает фазы (Intro, Loop, End) или бесконечный цикл.
    """

    # --- 1. ПАРАМЕТРЫ ---
    profile = getattr(settings, "rage_profile", 'ROAR')  # 'ROAR' или 'INTIMIDATE'
    use_phases = getattr(settings, "rage_use_phases", True)

    # Тайминги
    intro_len = int(getattr(settings, "rage_intro_frames", 15)) if use_phases else 0
    loop_len = int(getattr(settings, "rage_loop_frames", 40))
    end_len = int(getattr(settings, "rage_end_frames", 20)) if use_phases else 0

    total_frames = intro_len + loop_len + end_len

    fs = int(settings.frame_start)

    # Инициализация (сохраняем позу)
    try:
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
        print(f"[PW RAGE] Init failed: {e}")
        return False

    state['current_frame'] = fs  # КРИТИЧЕСКИ ВАЖНО!

    # После state['current_frame'] = fs добавьте:

    # Инициализация spine_chain
    if 'spine_chain' not in state:
        spine_bones = []
        for bone in arm_obj.data.bones:
            name_lower = bone.name.lower()
            if any(keyword in name_lower for keyword in ['spine', 'chest', 'torso', 'back', 'pelvis']):
                spine_bones.append(bone.name)

        # Сортируем по иерархии
        def get_bone_depth(bone_name):
            bone = arm_obj.data.bones.get(bone_name)
            depth = 0
            while bone.parent:
                depth += 1
                bone = bone.parent
            return depth

        spine_bones.sort(key=get_bone_depth)
        state['spine_chain'] = spine_bones

    # Инициализация head_name
    if 'head_name' not in state:
        head_name = None
        for bone in arm_obj.data.bones:
            name_lower = bone.name.lower()
            if any(keyword in name_lower for keyword in ['head', 'neck', 'skull']):
                head_name = bone.name
                break
        state['head_name'] = head_name

    # Инициализация arm_names
    if 'left_arm_name' not in state or 'right_arm_name' not in state:
        left_arm = None
        right_arm = None
        for bone in arm_obj.data.bones:
            name_lower = bone.name.lower()
            if any(keyword in name_lower for keyword in ['arm', 'shoulder', 'clavicle']):
                if 'left' in name_lower or 'l_' in name_lower or '.l' in name_lower:
                    left_arm = bone.name
                elif 'right' in name_lower or 'r_' in name_lower or '.r' in name_lower:
                    right_arm = bone.name
                elif bone.head_local.x > 0:
                    right_arm = bone.name
                else:
                    left_arm = bone.name
        state['left_arm_name'] = left_arm
        state['right_arm_name'] = right_arm

    # Фиксируем стартовую точку мира
    bpy.context.scene.frame_set(fs)
    bpy.context.view_layer.update()
    initial_com_pos = state['pelvis_world_current'].copy()

    # Снапшоты оригинальных ротаций
    original_poses = {}
    for pb in arm_obj.pose.bones:
        original_poses[pb.name] = {
            'rot': pb.rotation_quaternion.copy(),
            'loc': pb.location.copy()
        }

    # --- 2. ФИЗИКА И ШУМ ---
    # Для ROAR нужна резкая, жесткая физика (взрыв).
    # Для INTIMIDATE нужна тяжелая, вязкая физика.

    if profile == 'ROAR':
        # Быстрая частота, малое затухание (вибрация)
        dyn_spine = RageDynamics(f=3.5, z=0.4, r=1.5, x0=Vector((0, 0, 0)))
        dyn_arms = RageDynamics(f=3.0, z=0.5, r=1.0, x0=Vector((0, 0, 0)))
    else:  # INTIMIDATE
        # Медленная, тяжелая
        dyn_spine = RageDynamics(f=1.5, z=0.8, r=0.0, x0=Vector((0, 0, 0)))
        dyn_arms = RageDynamics(f=1.2, z=0.7, r=0.0, x0=Vector((0, 0, 0)))

    np = get_default_noise_provider(seed=random.randint(0, 1000))

    # --- 3. ЦИКЛ ---
    dt = 1.0 / bpy.context.scene.render.fps

    # IK трекеры (ноги стоят на месте)
    tracker_data = {state['left_name']: [], state['right_name']: []}
    if getattr(settings, "use_ik", False):
        lf_pos = utils.get_current_foot_position(arm_obj, state['left_name'], settings)
        rf_pos = utils.get_current_foot_position(arm_obj, state['right_name'], settings)

    for i in range(total_frames + 1):
        frame = fs + i
        bpy.context.scene.frame_set(frame)

        # --- ЛОГИКА ФАЗ И ВЕСОВ ---
        # weight 0.0 -> покоя, 1.0 -> полная ярость
        weight = 1.0
        phase_progress = 0.0  # 0..1 внутри текущей фазы

        if use_phases:
            if i < intro_len:
                # INTRO: Резкий вход
                phase_progress = i / intro_len
                # Используем EaseOutBack для эффекта "удара"
                weight = 1.0 - pow(1.0 - phase_progress, 3)
            elif i < (intro_len + loop_len):
                # LOOP: Удержание
                weight = 1.0
                phase_progress = (i - intro_len) / loop_len
            else:
                # END: Затухание
                p = (i - (intro_len + loop_len)) / end_len
                weight = 1.0 - (p * p)  # Ease In Quad
        else:
            # Если фаз нет, просто крутим цикл
            phase_progress = i / total_frames

        # --- ЦЕЛЕВЫЕ ВЕКТОРА (TARGETS) ---
        target_spine = Vector((0, 0, 0))  # Euler: X (bend), Y (twist), Z (lean)
        target_arm_spread = 0.0  # 0 to 1
        target_head = Vector((0, 0, 0))
        target_com = Vector((0, 0, 0))

        intensity = getattr(settings, "rage_intensity", 1.0) * weight

        # ДЫХАНИЕ (Тяжелое)
        # Частота зависит от профиля
        breath_speed = 0.8 if profile == 'ROAR' else 0.4
        breath = math.sin(i * breath_speed) * intensity

        # >>> ПРОФИЛЬ 1: ROAR (БЕРСЕРК / КРИК) <<<
        if profile == 'ROAR':
            # Спина: Сильно выгибается назад (-X) и распрямляется
            target_spine = Vector((-0.4, 0, 0)) * intensity
            # Добавляем вибрацию крика в спину
            target_spine.x += np.noise1d(i * 2.0) * 0.1 * intensity

            # Руки: Широко в стороны и назад (поза Халка)
            target_arm_spread = 1.0 * intensity

            # Голова: Вверх (-X) и трясется
            target_head = Vector((-0.5, 0, 0)) * intensity

            # COM: Немного вверх (на цыпочки от напряжения)
            target_com = Vector((0, 0, 0.05)) * intensity

        # >>> ПРОФИЛЬ 2: INTIMIDATE (ХИЩНИК / УГРОЗА) <<<
        elif profile == 'INTIMIDATE':
            # Спина: Сгорблена вперед (+X)
            target_spine = Vector((0.3, 0, 0)) * intensity
            # Медленное покачивание (Sway)
            target_spine.y = math.sin(i * 0.1) * 0.2 * intensity

            # Руки: Свисают, но напряжены (Gorilla style)
            target_arm_spread = 0.2 * intensity

            # Голова: Вперед и вниз, взгляд исподлобья
            target_head = Vector((0.2, 0, 0)) * intensity

            # COM: Вниз (присед перед атакой)
            target_com = Vector((0, 0.1, -0.15)) * intensity

        # --- ФИЗИЧЕСКИЙ АПДЕЙТ ---
        phys_spine = dyn_spine.update(dt, target_spine)

        # Добавляем слой адреналинового шума поверх физики
        adrenaline = np.noise1d(i * 0.9) * 0.05 * intensity

        # --- ПРИМЕНЕНИЕ К КОСТЯМ ---

        # 1. SPINE (Цепочка)
        spine_chain = state.get('spine_chain', [])
        if spine_chain:
            # В дыхании участвует грудь (расширение)
            breath_expansion = breath * 0.1

            # Распределяем изгиб по позвонкам
            step = 1.0 / len(spine_chain)
            for idx, bname in enumerate(spine_chain):
                pb = arm_obj.pose.bones.get(bname)
                if not pb: continue

                base_rot = original_poses[bname]['rot']

                # Основной изгиб + шум
                x_rot = (phys_spine.x / len(spine_chain)) + adrenaline
                # Sway
                y_rot = (phys_spine.y * step)

                # Добавляем "волну" дыхания, идущую снизу вверх
                wave_breath = math.sin(i * breath_speed - idx * 0.5) * 0.02 * intensity

                q_rage = Quaternion(Vector((1, 0, 0)), x_rot + wave_breath) @ \
                         Quaternion(Vector((0, 1, 0)), y_rot)

                pb.rotation_quaternion = base_rot @ q_rage
                pb.keyframe_insert("rotation_quaternion", frame=frame)

        # 2. ARMS (Руки)
        for side in ['left', 'right']:
            arm_name = state.get(f'{side}_arm_name')
            if arm_name:
                pb = arm_obj.pose.bones.get(arm_name)
                if pb:
                    base_rot = original_poses[arm_name]['rot']
                    side_mul = 1 if side == 'left' else -1

                    # Логика разведения рук
                    # Z - обычно подъем/опускание, Y - вращение
                    # Это зависит от рига, берем усредненно для Rigify/Mixamo

                    if profile == 'ROAR':
                        # Поднимаем и разводим
                        # Вращаем "наружу"
                        q_spread = Quaternion(Vector((0, 1, 0)), target_arm_spread * -0.5 * side_mul) @ \
                                   Quaternion(Vector((1, 0, 0)), target_arm_spread * 0.3)
                    else:
                        # Локти в стороны (Gorilla)
                        q_spread = Quaternion(Vector((0, 0, 1)), target_arm_spread * 0.5 * side_mul)

                    # Тряска рук (сильная в ярости)
                    shake_amp = 0.08 * intensity
                    q_shake = Quaternion(Vector((1, 0, 0)), np.noise1d(i + (10 if side == 'left' else 0)) * shake_amp)

                    pb.rotation_quaternion = base_rot @ q_spread @ q_shake
                    pb.keyframe_insert("rotation_quaternion", frame=frame)

        # 3. HEAD (Голова)
        head_name = state.get('head_name')
        if head_name:
            pb = arm_obj.pose.bones.get(head_name)
            if pb:
                base_rot = original_poses[head_name]['rot']

                # Head Look + Shake
                q_look = Quaternion(Vector((1, 0, 0)), phys_spine.x * 0.5 + target_head.x)  # Наследуем часть спины

                # Тряска головы очень быстрая
                head_shake_val = np.noise1d(i * 2.5) * 0.05 * intensity
                q_shake = Quaternion(Vector((0, 0, 1)), head_shake_val)

                pb.rotation_quaternion = base_rot @ q_look @ q_shake
                pb.keyframe_insert("rotation_quaternion", frame=frame)

        # 4. COM (Таз)
        # Применяем вертикальное дыхание и позу
        total_com_offset = target_com.copy()
        total_com_offset.z += breath * 0.02  # Дыхание в таз
        total_com_offset.x += adrenaline * 0.1  # Тряска таза

        target_pelvis_world = initial_com_pos + total_com_offset
        delta_move = target_pelvis_world - state['pelvis_world_current']

        utils.apply_center_motion(state, settings, arm_obj, delta_move)
        state['current_frame'] = frame  # ОБНОВЛЯЕМ текущий кадр
        state['pelvis_world_current'] = target_pelvis_world.copy()

        # 5. IK LOCK
        if getattr(settings, "use_ik", False):
            tracker_data[state['left_name']].append({'frame': frame, 'loc': lf_pos})
            tracker_data[state['right_name']].append({'frame': frame, 'loc': rf_pos})

    # --- 4. POST-PROCESS ---
    if getattr(settings, "use_ik", False):
        utils.create_jump_trackers_with_anchored_behavior(arm_obj, state, settings, tracker_data)

    return True