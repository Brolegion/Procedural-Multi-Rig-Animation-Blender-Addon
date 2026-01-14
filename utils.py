# utils.py
# Utility module for Procedural Walk addon.
# Contains init_walk_state, IK helpers, bezier helpers, etc.
# NOTE: do NOT import from .utils inside this file (no self-import).

import bpy
import math
import numpy as np
import tempfile
import os
from mathutils import Vector, Quaternion, Matrix
from .translations import tr

def _dbg_write(s, tbname="PW_IK_DEBUG"):
    try:
        if tbname not in bpy.data.texts:
            bpy.data.texts.new(tbname)
        bpy.data.texts[tbname].write(str(s) + "\n")
    except Exception:
        pass
    try:
        fn = os.path.join(tempfile.gettempdir(), "pw_ik_debug.txt")
        with open(fn, "a", encoding="utf-8") as f:
            f.write(str(s) + "\n")
    except Exception:
        pass

def clamp(x, a, b):
    try:
        return max(a, min(b, x))
    except Exception:
        return a if x is None else x


def find_leg_pairs_by_name_or_space(arm_obj, mask_str):
    arm = arm_obj.data
    bones = list(arm.bones)
    mask_tokens = [t.strip().lower() for t in mask_str.split(';') if t.strip()]
    cand = []
    for b in bones:
        name_lower = b.name.lower()
        if any(tok in name_lower for tok in mask_tokens):
            cand.append(b)
    pairs = []
    if cand:
        by_side = {}
        for b in cand:
            name = b.name
            nl = name.lower()
            if name.endswith(('.L','_L','.l','_l')) or '.left' in nl or nl.split('.')[-1] in ('l','left'):
                by_side.setdefault('L', []).append(b)
            elif name.endswith(('.R','_R','.r','_r')) or '.right' in nl or nl.split('.')[-1] in ('r','right'):
                by_side.setdefault('R', []).append(b)
        if 'L' in by_side and 'R' in by_side:
            for bl in by_side['L']:
                best = None
                best_d = 1e9
                for br in by_side['R']:
                    d = (bl.head_local - Vector((-br.head_local.x, br.head_local.y, br.head_local.z))).length
                    if d < best_d:
                        best_d = d
                        best = br
                if best:
                    pairs.append((bl.name, best.name))
        if pairs:
            return pairs
    clusters = {}
    for b in bones:
        h = b.head_local
        key = (round(h.y, 3), round(h.z, 3))
        clusters.setdefault(key, []).append(b)
    for key, group in clusters.items():
        for i in range(len(group)):
            for j in range(i+1, len(group)):
                bi = group[i]
                bj = group[j]
                if bi.head_local.x * bj.head_local.x < 0:
                    if abs(abs(bi.head_local.x) - abs(bj.head_local.x)) < 0.5:
                        if bi.head_local.x > 0:
                            pairs.append((bi.name, bj.name))
                        else:
                            pairs.append((bj.name, bi.name))
    return pairs

def determine_left_right_by_world_x(arm_obj, bone_a, bone_b, center_name=None):
    arm_wm = arm_obj.matrix_world
    ref_world = (arm_wm @ Vector(arm_obj.data.bones[center_name].head_local)) if (center_name and center_name in arm_obj.data.bones) else arm_wm.translation
    arm_right = (arm_obj.matrix_world.to_3x3() @ Vector((1.0, 0.0, 0.0))).normalized()
    a_world = arm_wm @ Vector(bone_a.head_local)
    b_world = arm_wm @ Vector(bone_b.head_local)
    da = (a_world - ref_world).dot(arm_right)
    db = (b_world - ref_world).dot(arm_right)
    if da >= db:
        return bone_a, bone_b
    return bone_b, bone_a

def init_walk_state(arm_obj, left_name_in, right_name_in, settings, frame_start, frame_end, preserve_pose=False):
    state = {}
    state['frame_start'] = int(frame_start)
    state['frame_end'] = int(frame_end)
    state['total_frames'] = max(1, state['frame_end'] - state['frame_start'] + 1)
    state['hip_movement_history'] = {}
    state['foot_movement_history'] = {}
    state['frame_history'] = {}

    try:
        # ИЗМЕНИТЬ ЭТОТ БЛОК:
        if not preserve_pose:
            state['initial_loc'] = reset_armature_transforms(arm_obj, state['frame_start'])
        else:
            state['initial_loc'] = arm_obj.location.copy()
    except Exception:
        state['initial_loc'] = arm_obj.location.copy()

    if not arm_obj or not getattr(arm_obj, "data", None) or not arm_obj.data.bones:
        raise ValueError("Armature has no bones")

    use_override = False
    if left_name_in and right_name_in:
        if left_name_in in arm_obj.data.bones and right_name_in in arm_obj.data.bones:
            use_override = True
            b_left = arm_obj.data.bones[left_name_in]
            b_right = arm_obj.data.bones[right_name_in]
        else:
            use_override = False

    if not use_override:
        if left_name_in and left_name_in in arm_obj.data.bones and (not right_name_in or right_name_in not in arm_obj.data.bones):
            b_left = arm_obj.data.bones[left_name_in]
            candidates = [b for b in arm_obj.data.bones]
            best = None; bestd = 1e9
            for b in candidates:
                if b.name == b_left.name: continue
                if b.head_local.x * b_left.head_local.x < 0:
                    d = abs(abs(b.head_local.x) - abs(b_left.head_local.x))
                    if d < bestd:
                        bestd = d; best = b
            b_right = best or candidates[0]
        elif right_name_in and right_name_in in arm_obj.data.bones and (not left_name_in or left_name_in not in arm_obj.data.bones):
            b_right = arm_obj.data.bones[right_name_in]
            candidates = [b for b in arm_obj.data.bones]
            best = None; bestd = 1e9
            for b in candidates:
                if b.name == b_right.name: continue
                if b.head_local.x * b_right.head_local.x < 0:
                    d = abs(abs(b.head_local.x) - abs(b_right.head_local.x))
                    if d < bestd:
                        bestd = d; best = b
            b_left = best or candidates[0]
        else:
            bones = list(arm_obj.data.bones)
            if len(bones) < 2:
                raise ValueError("Not enough bones to form a leg pair")
            best_left = max(bones, key=lambda b: b.head_local.x)
            best_right = min(bones, key=lambda b: b.head_local.x)
            b_left, b_right = best_left, best_right

    center_name = getattr(settings, "center_target_bone", None) or choose_center_bone(arm_obj)
    state['center_name'] = center_name

    if use_override:
        left_rest, right_rest = b_left, b_right
    else:
        left_rest, right_rest = determine_left_right_by_world_x(arm_obj, b_left, b_right, center_name)

    state['left_rest'] = left_rest
    state['right_rest'] = right_rest
    state['left_name'] = left_rest.name
    state['right_name'] = right_rest.name
    # В функцию init_walk_state, после обнаружения ног, добавить:

    # --- Arm Detection and Initialization ---
    if getattr(settings, "use_arm_animation", False):
        arm_mask = getattr(settings, "arm_name_mask", "shoulder;upperarm;arm;clavicle")
        arm_pairs = find_arm_pairs_by_name_or_space(arm_obj, arm_mask)

        if arm_pairs:
            # Берем первую найденную пару
            left_arm_name, right_arm_name = arm_pairs[0]

            # Определяем левую/правую сторону (уточняем)
            left_arm_bone = arm_obj.data.bones[left_arm_name]
            right_arm_bone = arm_obj.data.bones[right_arm_name]

            # Уточняем сторону по геометрии
            if left_arm_bone.head_local.x > right_arm_bone.head_local.x:
                left_arm_name, right_arm_name = right_arm_name, left_arm_name
                left_arm_bone, right_arm_bone = right_arm_bone, left_arm_bone

            # Сохраняем в state
            state['left_arm_name'] = left_arm_name
            state['right_arm_name'] = right_arm_name
            state['left_arm_rest'] = left_arm_bone
            state['right_arm_rest'] = right_arm_bone

            # Находим цепочки для каждой руки
            left_chain = find_arm_chain(arm_obj, left_arm_name)
            right_chain = find_arm_chain(arm_obj, right_arm_name)

            # Сохраняем предплечья и кисти
            state['left_forearm_name'] = left_chain[1] if len(left_chain) > 1 else None
            state['right_forearm_name'] = right_chain[1] if len(right_chain) > 1 else None
            state['left_hand_name'] = left_chain[2] if len(left_chain) > 2 else None
            state['right_hand_name'] = right_chain[2] if len(right_chain) > 2 else None

            if getattr(settings, "debug_pw", False):
                _dbg_write(f"[ARM INIT] Left chain: {left_chain}")
                _dbg_write(f"[ARM INIT] Right chain: {right_chain}")
        else:
            # Если руки не найдены, отключаем анимацию рук
            state['left_arm_name'] = None
            state['right_arm_name'] = None
            if getattr(settings, "debug_pw", False):
                _dbg_write("[ARM INIT] No arm bones found")
    else:
        state['left_arm_name'] = None
        state['right_arm_name'] = None

    fw = forward_vector_world(arm_obj, settings, center_name)
    if fw.length < 1e-6:
        fw = Vector((0.0, 1.0, 0.0))
    state['fw_world'] = fw.normalized()
    state['lateral_world_base'] = compute_lateral_orientation(arm_obj, settings, center_name, state['fw_world'], left_rest, right_rest)

    state['pose'] = arm_obj.pose
    state['center_pose_bone'] = state['pose'].bones.get(center_name) if center_name else None
    state['rest_pelvis_world'] = (arm_obj.matrix_world @ Vector(state['center_pose_bone'].bone.head_local)) if state['center_pose_bone'] else arm_obj.matrix_world.translation
    state['pelvis_world_current'] = state['rest_pelvis_world'].copy()

    # ДОБАВЛЕНО: Инициализация vertical_bob
    state['vertical_bob'] = 0.0

    state['p1'], state['p2'] = profile_control_points(getattr(settings, "push_profile", "SMOOTH"), getattr(settings, "push_aggressiveness", 0.6))
    state['left_stride_len_const'] = geometric_stride_length(left_rest, settings)
    state['right_stride_len_const'] = geometric_stride_length(right_rest, settings)
    state['avg_stride_len'] = (state['left_stride_len_const'] + state['right_stride_len_const']) * 0.5
    settings.center_stride_scale = clamp(getattr(settings, "center_stride_scale", 1.0), 0.1, 3.0)
    settings.push_strength = clamp(getattr(settings, "push_strength", 0.9), 0.0, 2.0)

    # Вычисляем фазы для первого кадра
    phase_left, phase_right = compute_phases(frame_start, frame_start, state['total_frames'], settings)

    # Вычисляем начальные позиции стоп
    try:
        left_foot_world, left_contact, _, left_lift = precise_estimate_foot_world_pos(
            arm_obj, left_rest, phase_left, settings, state['fw_world'], target_bone_type="foot"
        )
        right_foot_world, right_contact, _, right_lift = precise_estimate_foot_world_pos(
            arm_obj, right_rest, phase_right, settings, state['fw_world'], target_bone_type="foot"
        )
    except Exception as e:
        _dbg_write(f"[PW DEBUG] ERROR: Failed to compute initial foot positions: {e}")
        left_foot_world = arm_obj.matrix_world @ Vector(arm_obj.data.bones[find_target_bone(arm_obj, left_rest.name, 'foot', settings)].tail_local)
        right_foot_world = arm_obj.matrix_world @ Vector(arm_obj.data.bones[find_target_bone(arm_obj, right_rest.name, 'foot', settings)].tail_local)

    # Инициализируем историю движения стоп
    state['foot_movement_history'] = {
        state['left_name']: [left_foot_world.copy()],
        state['right_name']: [right_foot_world.copy()]
    }
    state['frame_history'] = {
        state['left_name']: [frame_start],
        state['right_name']: [frame_start]
    }

    # Удаляем IK constraints, если use_ik отключен
    if not settings.use_ik:
        left_pose = arm_obj.pose.bones.get(state['left_name'])
        right_pose = arm_obj.pose.bones.get(state['right_name'])
        if left_pose:
            remove_ik_constraints(left_pose)
        if right_pose:
            remove_ik_constraints(right_pose)
        if settings.rear_copy_mode != 'NONE':
            mask_str = settings.bone_name_mask
            pairs = find_leg_pairs_by_name_or_space(arm_obj, mask_str)
            for l, r in pairs:
                if l == state['left_name'] and r == state['right_name']: continue
                lp = arm_obj.pose.bones.get(l)
                rp = arm_obj.pose.bones.get(r)
                if lp: remove_ik_constraints(lp)
                if rp: remove_ik_constraints(rp)

    # Всегда устанавливаем ik_targets в None, так как IK функциональность удалена
    state['ik_targets'] = None

    calibration_scale = 1.0
    if getattr(settings, "calibration_enabled", False):
        try:
            calibration_scale = calibrate_push_for_pair(arm_obj, left_rest, right_rest, settings, state['fw_world'], samples=getattr(settings, "calibration_samples", 128), debug=False)
        except Exception:
            calibration_scale = 1.0
    state['calibration_scale'] = clamp(calibration_scale, getattr(settings, "calibration_clamp_min", 0.25), getattr(settings, "calibration_clamp_max", 4.0))

    state['prev_W_left'] = 0.0
    state['prev_W_right'] = 0.0
    state['smoothed_delta'] = 0.0
    state['initialized'] = False
    state['lateral_smoothed'] = 0.0
    state['lateral_integral_scalar'] = 0.0
    state['lateral_smooth_alpha'] = 0.25
    state['arm_world_mat3'] = arm_obj.matrix_world.to_3x3()
    state['arm_world'] = arm_obj.matrix_world

    return state


def compute_com_delta(state, settings, W_left, W_right, convert_forward_to_rotation=False):
    fw_world = state['fw_world']
    left_stride_len_const = state['left_stride_len_const']
    right_stride_len_const = state['right_stride_len_const']
    avg_stride_len = state['avg_stride_len']
    left_stride_for_push = avg_stride_len * float(getattr(settings, "left_amplitude", 1.0))
    right_stride_for_push = avg_stride_len * float(getattr(settings, "right_amplitude", 1.0))
    raw_inc_left = max(0.0, (W_left - state['prev_W_left'])) * left_stride_for_push * float(
        getattr(settings, "push_strength", 0.9))
    raw_inc_right = max(0.0, (W_right - state['prev_W_right'])) * right_stride_for_push * float(
        getattr(settings, "push_strength", 0.9))
    raw_delta_total = raw_inc_left + raw_inc_right
    if not state['initialized']:
        state['smoothed_delta'] = 0.0
        state['initialized'] = True
    inertia = clamp(getattr(settings, "com_inertia", 0.6), 0.0, 0.98)
    alpha = 1.0 - inertia
    state['smoothed_delta'] = (1.0 - alpha) * state['smoothed_delta'] + alpha * raw_delta_total
    max_per_frame = (left_stride_len_const + right_stride_len_const) * 0.9
    state['smoothed_delta'] = clamp(state['smoothed_delta'], -max_per_frame, max_per_frame)
    state['smoothed_delta'] *= float(state.get('calibration_scale', 1.0))

    delta_world = fw_world * state['smoothed_delta']
    raw_com_lateral = (W_left - W_right) * float(getattr(settings, "com_lateral_amount", 0.0))
    alpha_lat = float(state.get('lateral_smooth_alpha', 0.25))
    state['lateral_smoothed'] = (1.0 - alpha_lat) * state['lateral_smoothed'] + alpha_lat * raw_com_lateral
    state['lateral_integral_scalar'] += state['lateral_smoothed']
    max_lat = float(getattr(settings, "com_lateral_amount", 0.0)) * 1.5
    state['lateral_smoothed'] = clamp(state['lateral_smoothed'], -max_lat, max_lat)
    delta_world += state['lateral_world_base'] * state['lateral_smoothed']

    # УБИРАЕМ ВСЮ ЛОГИКУ ПРЕОБРАЗОВАНИЯ В ROTATION
    # Вращение теперь управляется через модуляцию интерполяции в fullbodyswing
    delta_yaw = 0.0

    return delta_world, delta_yaw


def compute_turn_modulation(phase_left, phase_right, W_left, W_right, settings):
    """
    Вычисляет модуляцию интерполяции вращения на основе фаз шагов.
    """
    turn_modulation_strength = float(getattr(settings, "turn_modulation_strength", 0.5))
    turn_sensitivity = float(getattr(settings, "turn_sensitivity", 1.0))

    def get_phase_modulation(phase, weight):
        """Вычисляет модуляцию для одной фазы"""
        phase_norm = phase % (2.0 * math.pi)

        # Улучшенная модуляция с плавными переходами
        if phase_norm < math.pi:  # Фаза переноса
            # Синусоида с пиком в середине переноса
            efficiency = math.sin(phase_norm)
        else:  # Фаза опоры
            # Плавный рост от начала к концу опоры
            support_progress = (phase_norm - math.pi) / math.pi
            efficiency = support_progress * 0.7  # Меньше влияния в фазе опоры

        return efficiency * weight

    # Вычисляем модуляцию для каждой ноги
    left_mod = get_phase_modulation(phase_left, W_left)
    right_mod = get_phase_modulation(phase_right, W_right)

    # Комбинируем модуляции (балансированное среднее)
    combined_modulation = (left_mod + right_mod) * 0.5

    # Применяем настройки
    modulated_t = combined_modulation * turn_modulation_strength * turn_sensitivity

    return clamp(modulated_t, -0.8, 0.8)  # Ограничиваем экстремальные значения

def compensate_lateral_if_cycle_start(state, settings, phase_left):
    if abs((phase_left % (2.0 * math.pi)) - 0.0) < 1e-3:
        avg_drift = state['lateral_integral_scalar'] / float(state['total_frames'])
        state['lateral_smoothed'] -= avg_drift
        state['lateral_integral_scalar'] = 0.0


def apply_center_motion(state, settings, arm_obj, delta_world, delta_yaw=0.0, convert_forward_to_rotation=False):
    """
    Улучшенная версия с разделением трансформаций:

    В режиме ARMATURE_OBJECT:
      - Вращение (delta_yaw) → arm_obj.rotation_euler.z (только если convert_forward_to_rotation=True)
      - Вертикальный подъём (vertical_bob) → корневая кость
      - Стартовый оффсет → корневая кость

    В режиме POSE_BONE:
      - Вращение (delta_yaw) → корневая кость (pose_bone.rotation_quaternion)
      - Вертикальный подъём → корневая кость
      - Стартовый оффсет → корневая кость
    """

    rest = state.get('rest_pelvis_world', None)
    if rest is None:
        return

    # Обновляем мировую текущую позицию таза
    state['pelvis_world_current'] += delta_world

    if not getattr(settings, "center_enabled", False) or getattr(settings, "center_apply_mode", "NONE") == 'NONE':
        return

    WORLD_UP = Vector((0.0, 0.0, 1.0))
    vertical_bob = state.get('vertical_bob', 0.0)

    # Стартовый оффсет
    start_down = float(getattr(settings, "center_start_down", getattr(settings, "crouch_amount", 0.0) * 0.2 or 0.02))
    start_tilt_deg = float(getattr(settings, "center_start_tilt", getattr(settings, "crouch_spine_lean", 5.0)))
    apply_start = not state.get('center_start_applied', False)
    start_tilt_rad = math.radians(start_tilt_deg) if apply_start and abs(start_tilt_deg) > 1e-6 else 0.0

    # Режим применения
    mode = getattr(settings, "center_apply_mode", "NONE")

    # Помощники для констрейнтов
    def _mute_constraints(pb, mute=True):
        saved = []
        if not pb:
            return saved
        for c in pb.constraints:
            try:
                saved.append((c, c.mute))
                c.mute = mute
            except Exception:
                pass
        return saved

    def _restore_constraints(saved):
        for c, prev in saved:
            try:
                c.mute = prev
            except Exception:
                pass

    # -----------------------
    # MODE: POSE_BONE
    # -----------------------
    if mode == 'POSE_BONE' and state.get('center_pose_bone'):
        pose_bone = state['center_pose_bone']
        bone = pose_bone.bone

        # Мировая дельта перемещения таза относительно rest
        delta_world_loc = state['pelvis_world_current'] - rest

        # Добавляем vertical bob
        if abs(vertical_bob) > 1e-9:
            delta_world_loc += WORLD_UP * vertical_bob

        # Стартовый оффсет: вниз по WORLD_UP
        if apply_start and start_down != 0.0:
            delta_world_loc += -WORLD_UP * abs(start_down)

        # Переводим в локальные координаты кости
        try:
            arm_inv_rot = arm_obj.matrix_world.inverted().to_3x3()
        except Exception:
            arm_inv_rot = arm_obj.matrix_world.inverted_safe().to_3x3() if hasattr(arm_obj.matrix_world,
                                                                                   'inverted_safe') else arm_obj.matrix_world.inverted().to_3x3()

        delta_local_arm = arm_inv_rot @ delta_world_loc
        bone_rest_inv_rot = bone.matrix_local.inverted().to_3x3()
        delta_local_bone = bone_rest_inv_rot @ delta_local_arm

        # Текущая локальная позиция
        try:
            current_loc = Vector(pose_bone.location)
        except Exception:
            current_loc = Vector((0.0, 0.0, 0.0))

        new_local_pos = current_loc + delta_local_bone

        # Стартовый наклон (только при первом вызове)
        tilt_quat_local = None
        if apply_start and start_tilt_rad != 0.0:
            lateral_world = state.get('lateral_world_base', Vector((1.0, 0.0, 0.0)))
            inv_rest = bone.matrix_local.inverted().to_3x3()
            axis_local = inv_rest @ lateral_world
            if axis_local.length < 1e-6:
                axis_local = Vector((1.0, 0.0, 0.0))
            else:
                axis_local.normalize()
            tilt_quat_local = Quaternion(axis_local, start_tilt_rad)

        # Переключаемся в quaternion mode
        old_mode = pose_bone.rotation_mode
        if pose_bone.rotation_mode != 'QUATERNION':
            pose_bone.rotation_mode = 'QUATERNION'

        # Временно отключаем констрейнты
        saved_cons = _mute_constraints(pose_bone, mute=True)

        try:
            # Устанавливаем позицию
            pose_bone.location = new_local_pos
            try:
                pose_bone.keyframe_insert(data_path="location", frame=state.get('current_frame'))
            except Exception:
                pass

            # Вращение (delta_yaw) ПРИМЕНЯЕМ ТОЛЬКО В РЕЖИМЕ POSE_BONE
            if convert_forward_to_rotation and abs(delta_yaw) > 1e-9:
                try:
                    inv_rest = bone.matrix_local.inverted().to_3x3()
                    axis_local = inv_rest @ WORLD_UP
                    if axis_local.length < 1e-6:
                        axis_local = Vector((0.0, 0.0, 1.0))
                    else:
                        axis_local.normalize()
                    rot_quat_local = Quaternion(axis_local, delta_yaw)
                    existing_rot = pose_bone.rotation_quaternion
                    pose_bone.rotation_quaternion = (rot_quat_local @ existing_rot).normalized()
                except Exception:
                    try:
                        pose_bone.rotation_euler.z += delta_yaw
                    except Exception:
                        pass

            # Стартовый наклон (однократно)
            if tilt_quat_local is not None:
                try:
                    pose_bone.rotation_quaternion = (tilt_quat_local @ pose_bone.rotation_quaternion).normalized()
                except Exception:
                    pass

            # Ключ вращения
            try:
                pose_bone.keyframe_insert(data_path="rotation_quaternion", frame=state.get('current_frame'))
            except Exception:
                pass

        finally:
            _restore_constraints(saved_cons)
            try:
                pose_bone.rotation_mode = old_mode
            except Exception:
                pass

        # Отмечаем, что стартовый оффсет применён
        if apply_start:
            state['center_start_applied'] = True

    # -----------------------
    # MODE: ARMATURE_OBJECT
    # -----------------------
    elif mode == 'ARMATURE_OBJECT':
        # 1) Горизонтальное смещение объекта арматуры
        try:
            inv_world = arm_obj.matrix_world.inverted()
        except Exception:
            inv_world = arm_obj.matrix_world.inverted_safe() if hasattr(arm_obj.matrix_world,
                                                                        'inverted_safe') else arm_obj.matrix_world.inverted()

        delta_local = inv_world.to_3x3() @ delta_world
        arm_obj.location += delta_local
        try:
            arm_obj.keyframe_insert(data_path="location", frame=state.get('current_frame'))
        except Exception:
            pass

        # 2) ВРАЩЕНИЕ: применяем ТОЛЬКО К ОБЪЕКТУ АРМАТУРЫ
        if convert_forward_to_rotation and abs(delta_yaw) > 1e-9:
            try:
                arm_obj.rotation_euler.z += delta_yaw
                arm_obj.keyframe_insert(data_path="rotation_euler", frame=state.get('current_frame'))
            except Exception:
                pass

        # 3) Вертикальный подъём и стартовый оффсет → КОРНЕВАЯ КОСТЬ (БЕЗ ВРАЩЕНИЯ)
        if state.get('center_pose_bone'):
            pose_bone = state['center_pose_bone']
            bone = pose_bone.bone

            # REST head позиции в arm-local
            rest_head_arm_local = bone.matrix_local.to_translation()
            rest_head_world = arm_obj.matrix_world @ rest_head_arm_local

            # Желаемая мировая позиция = rest_head_world + vertical_bob + start_down
            desired_world = rest_head_world + WORLD_UP * vertical_bob
            if apply_start and start_down != 0.0:
                desired_world += -WORLD_UP * abs(start_down)

            # Переводим desired world → arm-local → bone-local (pose translation)
            desired_arm_local = arm_obj.matrix_world.inverted() @ desired_world
            new_pose_loc = bone.matrix_local.inverted() @ desired_arm_local

            # Временно отключаем констрейнты
            saved_cons = _mute_constraints(pose_bone, True)

            try:
                # Устанавливаем позицию (без накопления)
                pose_bone.location = Vector(new_pose_loc)
                try:
                    pose_bone.keyframe_insert(data_path="location", frame=state.get('current_frame'))
                except Exception:
                    pass

                # ВНИМАНИЕ: В режиме ARMATURE_OBJECT НЕ применяем delta_yaw к кости!
                # Применяем только стартовый tilt (однократно)
                if apply_start and start_tilt_rad != 0.0:
                    try:
                        inv_rest = bone.matrix_local.inverted().to_3x3()
                        lateral_world = state.get('lateral_world_base', Vector((1.0, 0.0, 0.0)))
                        axis_local_tilt = inv_rest @ lateral_world
                        if axis_local_tilt.length < 1e-6:
                            axis_local_tilt = Vector((1.0, 0.0, 0.0))
                        else:
                            axis_local_tilt.normalize()
                        tilt_q = Quaternion(axis_local_tilt, start_tilt_rad)

                        old_mode = pose_bone.rotation_mode
                        if pose_bone.rotation_mode != 'QUATERNION':
                            pose_bone.rotation_mode = 'QUATERNION'

                        existing_q = pose_bone.rotation_quaternion
                        new_q = (tilt_q @ existing_q).normalized()
                        pose_bone.rotation_quaternion = new_q

                        try:
                            pose_bone.keyframe_insert(data_path="rotation_quaternion", frame=state.get('current_frame'))
                        except Exception:
                            pass

                        if old_mode != 'QUATERNION':
                            pose_bone.rotation_mode = old_mode
                    except Exception:
                        pass

            finally:
                _restore_constraints(saved_cons)

            # Отмечаем, что стартовый оффсет применён
            if apply_start:
                state['center_start_applied'] = True

    # Обновляем rest_pelvis_world
    try:
        state['rest_pelvis_world'] = state['pelvis_world_current'].copy()
    except Exception:
        try:
            state['rest_pelvis_world'] = Vector(state['pelvis_world_current'])
        except Exception:
            pass




def _sample_path_at(samples, t):
    if not samples:
        return None
    n = len(samples)
    if n == 1:
        return samples[0].copy()
    t = clamp(float(t), 0.0, 1.0)
    posf = t * (n - 1)
    i0 = int(math.floor(posf))
    i1 = min(n - 1, i0 + 1)
    if i0 == i1:
        return samples[i0].copy()
    frac = posf - i0
    return samples[i0].lerp(samples[i1], frac)



# utils.py - добавим новые функции интерполяции

def interpolate_constant_speed(current, target, speed):
    direction = target - current
    distance = direction.length
    if distance == 0:
        return current
    direction.normalize()
    step_length = min(distance, speed)
    return current + direction * step_length

def interpolate_linear(current, target, t):
    """
    Линейная интерполяция между двумя значениями (скалярными или векторными).
    Возвращает: current + (target - current) * t
    """
    return current + (target - current) * t

def interpolate_exponential(current, target, alpha):
    return current + (target - current) * alpha

def interpolate_spring(current, target, velocity, stiffness, damping, dt=1.0/24.0):
    acceleration = (target - current) * stiffness - velocity * damping
    new_velocity = velocity + acceleration * dt
    new_position = current + new_velocity * dt
    return new_position, new_velocity

def interpolate_step(current, target, factor):
    return current + (target - current) * factor

def interpolate_quadratic(current, target, t):
    t = min(1.0, max(0.0, t))
    t = t * t * (3 - 2 * t)  # Smoothstep function
    return current.lerp(target, t)

def interpolate_bezier(p0, p1, p2, p3, t):
    u = 1 - t
    return u*u*u*p0 + 3*u*u*t*p1 + 3*u*t*t*p2 + t*t*t*p3

def find_target_bone(arm_obj, rest_name, target_bone_type=None, settings=None, return_bone=False):
    """
    Находит целевую листовую кость (например, стопа) в цепочке от rest_name.
    Поддерживает override, foot_name_mask и приоритет токенов.
    Возвращает имя кости или объект Bone (если return_bone=True), или rest_name при неудаче.
    """
    # Проверяем валидность арматуры
    if not arm_obj or not arm_obj.data or not arm_obj.data.bones:
        if settings and getattr(settings, "debug_pw", False):
            _dbg_write(f"ERROR: Недействительная арматура для rest_name={rest_name}")
        return arm_obj.data.bones.get(rest_name) if return_bone else rest_name

    # Проверяем пустую строку
    if not rest_name:
        if settings and getattr(settings, "debug_pw", False):
            _dbg_write(f"ERROR: Пустое rest_name")
        return None if return_bone else ""

    name_lower = rest_name.lower()
    # Определяем сторону (left/right) с поддержкой разных форматов
    side = 'left' if any(s in name_lower for s in ['left', 'l_', 'l.', 'l-']) else \
           'right' if any(s in name_lower for s in ['right', 'r_', 'r.', 'r-']) else None

    # Проверяем override, если переданы settings
    if settings:
        candidate_name = None
        if side == 'left' and getattr(settings, "ik_foot_override_left", ""):
            candidate_name = getattr(settings, "ik_foot_override_left")
        elif side == 'right' and getattr(settings, "ik_foot_override_right", ""):
            candidate_name = getattr(settings, "ik_foot_override_right")
        if not candidate_name and getattr(settings, "ik_foot_override", ""):
            candidate_name = getattr(settings, "ik_foot_override")
        if candidate_name and candidate_name in arm_obj.data.bones:
            bone = arm_obj.data.bones[candidate_name]
            # Проверяем, что override-кость листовая
            if not bone.children:
                if settings and getattr(settings, "debug_pw", False):
                    _dbg_write(f"[PW DEBUG] Используется override '{candidate_name}' для rest '{rest_name}' (листовая)")
                return bone if return_bone else candidate_name
            elif settings and getattr(settings, "debug_pw", False):
                _dbg_write(f"[PW DEBUG] Override '{candidate_name}' не листовая, игнорируется")
        elif candidate_name and settings and getattr(settings, "debug_pw", False):
            _dbg_write(f"[PW DEBUG] Override '{candidate_name}' не найден, игнорируется")

    # Собираем листовые кости в цепочке от rest_name
    def collect_leaf_descendants(bone_name):
        if bone_name not in arm_obj.data.bones:
            return []
        out = []
        stack = [arm_obj.data.bones[bone_name]]
        visited = {bone_name}
        while stack:
            cur = stack.pop()
            if not cur.children:
                out.append(cur)
                continue
            for c in cur.children:
                if c.name not in visited:
                    visited.add(c.name)
                    stack.append(c)
        return out

    leaves = collect_leaf_descendants(rest_name)
    if not leaves:
        if settings and getattr(settings, "debug_pw", False):
            _dbg_write(f"[PW DEBUG] Не найдены листовые кости для rest '{rest_name}', возвращается {rest_name}")
        return arm_obj.data.bones.get(rest_name) if return_bone else rest_name

    # Определяем токены для поиска
    tokens = [target_bone_type.lower()] if target_bone_type else \
             ['toe', 'foot', 'ankle', 'toebase'] if 'hip' not in name_lower and 'thigh' not in name_lower else \
             ['hip', 'thigh', 'pelvis']

    # Добавляем токены из foot_name_mask, если переданы settings
    if settings:
        mask = (getattr(settings, "foot_name_mask", "") or "").lower()
        mask_tokens = [t.strip() for t in mask.split(';') if t.strip()]
        if mask_tokens:
            tokens.extend(mask_tokens)

    # Фильтруем листовые кости по токенам и стороне
    candidates = []
    for bone in leaves:
        bone_name_lower = bone.name.lower()
        if any(token in bone_name_lower for token in tokens):
            if side is None or \
               (side == 'left' and any(s in bone_name_lower for s in ['left', 'l_', 'l.', 'l-'])) or \
               (side == 'right' and any(s in bone_name_lower for s in ['right', 'r_', 'r.', 'r-'])):
                candidates.append(bone)

    # Если есть кости с токенами, выбираем первую
    if candidates:
        best_bone = candidates[0]
        if settings and getattr(settings, "debug_pw", False):
            _dbg_write(f"[PW DEBUG] Выбрана листовая кость '{best_bone.name}' с токеном для rest '{rest_name}'")
        return best_bone if return_bone else best_bone.name

    # Если нет костей с токенами, берём первую листовую кость
    if leaves:
        best_bone = leaves[0]
        if settings and getattr(settings, "debug_pw", False):
            _dbg_write(f"[PW DEBUG] Выбрана листовая кость '{best_bone.name}' (без токенов) для rest '{rest_name}'")
        return best_bone if return_bone else best_bone.name

    # Fallback, если ничего не найдено
    if settings and getattr(settings, "debug_pw", False):
        _dbg_write(f"ERROR: Не найдены листовые кости для {target_bone_type or 'target'}, возвращается {rest_name}")
    return arm_obj.data.bones.get(rest_name) if return_bone else rest_name


def choose_primary_pair(arm_obj, settings):
    left_override = getattr(settings, "left_leg_name", "") or ""
    right_override = getattr(settings, "right_leg_name", "") or ""
    mask = getattr(settings, "bone_name_mask", "") or ""
    bones = arm_obj.data.bones
    if left_override and right_override and left_override in bones and right_override in bones:
        return left_override, right_override
    if left_override and left_override in bones:
        left_b = bones[left_override]
        best = None; best_d = 1e9
        for b in bones:
            if b.name == left_b.name: continue
            d = abs(abs(b.head_local.x) - abs(left_b.head_local.x)) + abs(b.head_local.y - left_b.head_local.y)
            if d < best_d:
                best_d = d; best = b
        if best:
            left_rest, right_rest = determine_left_right_by_world_x(arm_obj, left_b, best, getattr(settings, "center_target_bone", None))
            return left_rest.name, right_rest.name
    if right_override and right_override in bones:
        right_b = bones[right_override]
        best = None; best_d = 1e9
        for b in bones:
            if b.name == right_b.name: continue
            d = abs(abs(b.head_local.x) - abs(right_b.head_local.x)) + abs(b.head_local.y - right_b.head_local.y)
            if d < best_d:
                best_d = d; best = b
        if best:
            left_rest, right_rest = determine_left_right_by_world_x(arm_obj, best, right_b, getattr(settings, "center_target_bone", None))
            return left_rest.name, right_rest.name
    pairs = find_leg_pairs_by_name_or_space(arm_obj, mask)
    if pairs:
        left_name, right_name = pairs[0]
        left_b, right_b = bones[left_name], bones[right_name]
        left_rest, right_rest = determine_left_right_by_world_x(arm_obj, left_b, right_b, getattr(settings, "center_target_bone", None))
        return left_rest.name, right_rest.name
    best_left = None; best_right = None
    max_x = -1e9; min_x = 1e9
    for b in bones:
        x = b.head_local.x
        if x > max_x:
            max_x = x; best_left = b
        if x < min_x:
            min_x = x; best_right = b
    if best_left and best_right:
        left_rest, right_rest = determine_left_right_by_world_x(arm_obj, best_left, best_right, getattr(settings, "center_target_bone", None))
        return left_rest.name, right_rest.name
    return None, None

def forward_vector_world(arm_obj, settings, bone_name=None):
    axis = getattr(settings, "forward_axis", "Y")
    if axis == 'X': local = Vector((1.0, 0.0, 0.0))
    elif axis == '-X': local = Vector((-1.0, 0.0, 0.0))
    elif axis == 'Y': local = Vector((0.0, 1.0, 0.0))
    elif axis == '-Y': local = Vector((0.0, -1.0, 0.0))
    elif axis == 'Z': local = Vector((0.0, 0.0, 1.0))
    else: local = Vector((0.0, 0.0, -1.0))
    coord_sys = getattr(settings, "forward_coordinate_system", "GLOBAL")
    try:
        if coord_sys == 'GLOBAL':
            fw_world = local.copy()
        else:
            if bone_name and arm_obj and bone_name in getattr(arm_obj, "pose", {}).bones:
                bone = arm_obj.pose.bones[bone_name]
                fw_world = (arm_obj.matrix_world.to_3x3() @ bone.matrix.to_3x3()) @ local
            else:
                fw_world = arm_obj.matrix_world.to_3x3() @ local
    except Exception:
        fw_world = Vector((0.0, 1.0, 0.0))
    if fw_world.length == 0.0:
        return Vector((0.0, 1.0, 0.0))
    return fw_world.normalized()

def bezier_point(p0, p1, p2, p3, t):
    u = 1.0 - t
    return (u**3) * p0 + 3.0 * (u**2) * t * p1 + 3.0 * u * (t**2) * p2 + (t**3) * p3

#ИСПОЛЬЗУЕТСЯ в FULLBODYSWING
def ease_in_out_cubic(t):
    if t is None:
        return 0.0
    t = clamp(t, 0.0, 1.0)
    if t < 0.5:
        return 4.0 * t * t * t
    u = (2.0 * t) - 2.0
    return 0.5 * u * u * u + 1.0

def bezier_y_for_x(p1, p2, x_target, iters=24):
    if x_target is None: return 0.0
    x_target = clamp(float(x_target), 0.0, 1.0)
    p1x = clamp(float(p1.x), 0.0, 1.0); p2x = clamp(float(p2.x), 0.0, 1.0)
    p1y = clamp(float(p1.y), 0.0, 1.0); p2y = clamp(float(p2.y), 0.0, 1.0)
    lo, hi = 0.0, 1.0
    t_final = x_target
    for _ in range(iters):
        t = 0.5 * (lo + hi)
        x = bezier_point(0.0, p1x, p2x, 1.0, t)
        if abs(x - x_target) <= 1e-9:
            t_final = t; break
        if x < x_target: lo = t
        else: hi = t
        t_final = 0.5 * (lo + hi)
    y = bezier_point(0.0, p1y, p2y, 1.0, t_final)
    return clamp(y, 0.0, 1.0)

def profile_control_points(profile, aggressiveness):
    a = clamp(float(aggressiveness), 0.0, 1.5)
    if profile == 'SNAPPY':
        p1 = Vector((0.12 + 0.05*a, 0.0 + 0.15*a))
        p2 = Vector((0.55 - 0.05*a, 0.95 - 0.2*a))
    elif profile == 'SUSTAIN':
        p1 = Vector((0.25 - 0.05*a, 0.05 + 0.05*a))
        p2 = Vector((0.80 + 0.05*a, 0.90 - 0.10*a))
    else:
        p1 = Vector((0.18 + 0.02*a, 0.03 + 0.08*a))
        p2 = Vector((0.62 - 0.03*a, 0.90 - 0.12*a))
    p1.x = clamp(p1.x, 0.0, 1.0); p1.y = clamp(p1.y, 0.0, 1.0)
    p2.x = clamp(p2.x, 0.0, 1.0); p2.y = clamp(p2.y, 0.0, 1.0)
    return p1, p2

def choose_center_bone(arm_obj, prefer_name=None):
    if prefer_name:
        for b in arm_obj.data.bones:
            if b.name.lower() == prefer_name.lower():
                return b.name
    name_candidates = ['pelvis', 'hip', 'root', 'spine', 'torso']
    for b in arm_obj.data.bones:
        nl = b.name.lower()
        if any(cand in nl for cand in name_candidates):
            return b.name
    for b in arm_obj.data.bones:
        if b.parent is None:
            return b.name
    return arm_obj.data.bones[0].name if arm_obj.data.bones else None

def find_spine_chain(arm_obj, root_bone_name, end_bone_name):
    spine_chain = []
    root_bone = arm_obj.data.bones.get(root_bone_name)
    end_bone = arm_obj.data.bones.get(end_bone_name)
    if not root_bone or not end_bone:
        return spine_chain
    current_bone = root_bone
    spine_chain.append(current_bone.name)
    while current_bone and current_bone != end_bone:
        children = current_bone.children
        if not children:
            break
        next_bone = None
        min_distance = float('inf')
        for child in children:
            if child.name in spine_chain:
                continue
            distance = (Vector(child.head_local) - Vector(end_bone.head_local)).length
            if distance < min_distance:
                min_distance = distance
                next_bone = child
        if not next_bone:
            break
        current_bone = next_bone
        spine_chain.append(current_bone.name)
        if current_bone == end_bone:
            break
    return spine_chain if spine_chain and spine_chain[-1] == end_bone.name else []

def parse_turn_limits(s):
    out = {}
    if not s:
        return out
    for token in s.split(';'):
        token = token.strip()
        if not token:
            continue
        if ':' in token:
            name, val = token.split(':', 1)
            try:
                out[name.strip()] = float(val.strip())
            except Exception:
                pass
    return out


def save_bone_rotations(pose, spine_chain):
    current_rotations = {}
    rest_rotations = {}
    for name in spine_chain:
        pb = pose.bones[name]
        current_rotations[name] = pb.rotation_quaternion.copy()
        try:
            rest_rotations[name] = pb.bone.matrix_local.to_quaternion().copy()
        except Exception:
            rest_rotations[name] = pb.rotation_quaternion.copy()
    return current_rotations, rest_rotations

def parse_turn_configs(settings):
    turn_configs = []
    try:
        for token in settings.turn_angles.split(';'):
            token = token.strip()
            if not token:
                continue
            parts = token.split(',')
            if len(parts) >= 2:
                ang = float(parts[0].strip())
                spd = float(parts[1].strip()) if float(parts[1].strip()) > 0 else settings.turn_speed
            else:
                ang = float(parts[0].strip())
                spd = settings.turn_speed
            turn_configs.append({'angle': ang, 'speed': spd})
    except Exception as e:
        print("Invalid turn config format, fallback:", e)
        turn_configs = [{'angle': 35.0, 'speed': settings.turn_speed}]
    return turn_configs

def apply_bone_rotation(pose_bone, angle, axis_world, base_quat, bone_name, per_bone_limits, frame):
    pose_bone.rotation_mode = 'QUATERNION'
    if bone_name in per_bone_limits:
        lim = math.radians(abs(per_bone_limits[bone_name]))
        angle = clamp(angle, -lim, lim)
    try:
        inv_rest = pose_bone.bone.matrix_local.inverted().to_3x3()
        axis_local = inv_rest @ axis_world
        if axis_local.length < 1e-6:
            axis_local = Vector((0.0, 0.0, 1.0))
        else:
            axis_local.normalize()
    except Exception:
        axis_local = Vector((0.0, 0.0, 1.0))
    rot_quat_local = Quaternion(axis_local, angle)
    pose_bone.rotation_quaternion = (rot_quat_local @ base_quat).normalized()
    try:
        pose_bone.keyframe_insert(data_path="rotation_quaternion", frame=frame)
    except Exception:
        pass

def reset_armature_transforms(arm_obj, frame_start):
    initial_loc = arm_obj.location.copy()
    arm_obj.rotation_euler = (0, 0, 0)
    arm_obj.scale = (1, 1, 1)
    try:
        arm_obj.keyframe_insert(data_path="rotation_euler", frame=frame_start)
        arm_obj.keyframe_insert(data_path="scale", frame=frame_start)
    except Exception:
        pass
    return initial_loc

def _select_best_foot_name_for_chain(rest_bone, arm_obj, settings):
    try:
        override = getattr(settings, 'ik_foot_override', None)
    except Exception:
        override = None
    try:
        override_l = getattr(settings, 'ik_foot_override_left', None)
    except Exception:
        override_l = None
    try:
        override_r = getattr(settings, 'ik_foot_override_right', None)
    except Exception:
        override_r = None
    name_low = rest_bone.name.lower()
    bones = arm_obj.data.bones
    if name_low.endswith(('.l', '_l', '.left')) and override_l and override_l in bones:
        return override_l
    if name_low.endswith(('.r', '_r', '.right')) and override_r and override_r in bones:
        return override_r
    if override and override in bones:
        return override
    def collect_leaves(b):
        leaves = []
        stack = [b]
        visited = set()
        while stack:
            cur = stack.pop()
            visited.add(cur.name)
            if not cur.children:
                leaves.append(cur)
            else:
                for c in cur.children:
                    if c.name not in visited:
                        stack.append(c)
        return leaves
    leaves = collect_leaves(rest_bone)
    if not leaves:
        return rest_bone.name
    mask = (getattr(settings, 'foot_name_mask', '') or '').lower()
    mask_tokens = [t.strip() for t in mask.split(';') if t.strip()]
    candidates = []
    if mask_tokens:
        for l in leaves:
            nl = l.name.lower()
            if any(tok in nl for tok in mask_tokens):
                candidates.append(l)
    if not candidates:
        candidates = leaves
    best = None; best_len = -1.0
    for c in candidates:
        cur = c
        total = 0.0
        while cur is not None and cur.name != rest_bone.name:
            parent = cur.parent
            if parent is None:
                break
            total += (Vector(cur.head_local) - Vector(parent.head_local)).length
            cur = parent
        if total > best_len:
            best_len = total; best = c
    return best.name if best else rest_bone.name




#ИЗМЕНАНА ДЛЯ ДОБАВЛЕНИЯ ОДНОФАЗНЫХ ЮНИТОВ!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
def compute_phases(f, frame_start, total_frames, settings):
    t = (f - frame_start) / float(total_frames)
    base_phase = 2.0 * math.pi * getattr(settings, "frequency", 0.5) * t

    # Проверяем, является ли юнит однофазным
    is_single_phase = getattr(settings, "single_phase", False)

    if is_single_phase:
        # Для однофазных юнитов используем только одну фазу
        phase_center = base_phase

        # Добавляем шум, если нужно
        noise = getattr(settings, "phase_noise", 0.0) * math.sin(base_phase * 3.37 + 0.1)
        phase_center += noise

        # Добавляем запаздывание, если нужно
        if getattr(settings, "use_lag", False):
            phase_center += 0.08 * math.sin(2 * math.pi * 0.5 * t)

        # Учитываем направление оси
        axis = getattr(settings, "forward_axis", 'Y')
        fw_sign = -1 if axis.startswith('-') else 1
        if fw_sign < 0:
            phase_center = -phase_center
            if getattr(settings, "debug_pw", False):
                _dbg_write(f"[PW DEBUG] Inverted phase for negative forward_axis={axis}")

        return phase_center, phase_center  # Возвращаем одинаковые фазы для совместимости
    else:
        # Для двуфазных юнитов используем оригинальную логику
        half_offset = 0.5 * getattr(settings, "phase_offset", math.pi)
        half_offset += getattr(settings, "leg_phase_drift", 0.0) * t
        noise_l = getattr(settings, "phase_noise", 0.0) * math.sin(base_phase * 3.37 + 0.1)
        noise_r = getattr(settings, "phase_noise", 0.0) * math.sin(base_phase * 3.37 + 0.1 + math.pi)
        phase_left = base_phase - half_offset + noise_l
        phase_right = base_phase + half_offset + noise_r
        if getattr(settings, "use_lag", False):
            phase_left += 0.08 * math.sin(2 * math.pi * 0.5 * t)
            phase_right += 0.08 * math.cos(2 * math.pi * 0.5 * t)
        # NEW: Учитываем знак forward_axis для синхронизации фаз
        axis = getattr(settings, "forward_axis", 'Y')
        fw_sign = -1 if axis.startswith('-') else 1
        if fw_sign < 0:
            phase_left, phase_right = phase_right, phase_left
            if getattr(settings, "debug_pw", False):
                _dbg_write(f"[PW DEBUG] Inverted phases for negative forward_axis={axis}")
        return phase_left, phase_right


#ИЗМЕНАНА ДЛЯ ДОБАВЛЕНИЯ ОДНОФАЗНЫХ ЮНИТОВ!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
def compute_weights(phase_left, phase_right, p1, p2, settings=None):
    def stance_progress(phase):
        p = phase % (2.0 * math.pi)
        return (p - math.pi) / math.pi if p >= math.pi else None

    # Проверяем, является ли юнит однофазным
    is_single_phase = settings is not None and getattr(settings, "single_phase", False)

    if is_single_phase:
        # Для однофазных юнитов используем только одну фазу
        sp_center = stance_progress(phase_left)
        W_center = 0.0 if sp_center is None else bezier_y_for_x(p1, p2, sp_center)
        return W_center, W_center, sp_center, sp_center  # Возвращаем одинаковые значения
    else:
        # Для двуфазных юнитов используем оригинальную логику
        sp_left = stance_progress(phase_left)
        sp_right = stance_progress(phase_right)
        W_left = 0.0 if sp_left is None else bezier_y_for_x(p1, p2, sp_left)
        W_right = 0.0 if sp_right is None else bezier_y_for_x(p1, p2, sp_right)
        return W_left, W_right, sp_left, sp_right


def compute_lateral_orientation(arm_obj, settings, center_name, fw_world, left_rest, right_rest):
    """
    Возвращает единичный вектор lateral_world_base (величина 'влево' в мировых координатах).
    - Проецирует forward на горизонталь и берёт cross(WORLD_UP, fw_horiz)
    - При коллинеарности использует fallback
    - Корректирует знак, чтобы вектор указывал в сторону левой ноги
    """
    WORLD_UP = Vector((0.0, 0.0, 1.0))
    if fw_world is None:
        fw_world = Vector((0.0, 1.0, 0.0))
    fw_h = Vector((fw_world.x, fw_world.y, 0.0))
    if fw_h.length < 1e-6:
        axis = getattr(settings, "forward_axis", "Y")
        fw_h = {
            'X': Vector((1.0, 0.0, 0.0)), '-X': Vector((-1.0, 0.0, 0.0)),
            'Y': Vector((0.0, 1.0, 0.0)), '-Y': Vector((0.0, -1.0, 0.0)),
            'Z': Vector((0.0, 0.0, 1.0)), '-Z': Vector((0.0, 0.0, -1.0))
        }.get(axis, Vector((0.0, 1.0, 0.0)))
    try:
        fw_h.normalize()
    except Exception:
        fw_h = Vector((0.0, 1.0, 0.0))
    lateral = WORLD_UP.cross(fw_h)
    if lateral.length < 1e-6:
        lateral = Vector((1.0, 0.0, 0.0))
    try:
        lateral.normalize()
    except Exception:
        lateral = Vector((1.0, 0.0, 0.0))
    try:
        ref_world = None
        if center_name and arm_obj is not None and center_name in arm_obj.data.bones:
            ref_world = arm_obj.matrix_world @ Vector(arm_obj.data.bones[center_name].head_local)
        else:
            ref_world = arm_obj.matrix_world.translation if arm_obj is not None else Vector((0.0, 0.0, 0.0))
        left_world = arm_obj.matrix_world @ Vector(left_rest.head_local)
        vec_to_left = left_world - ref_world
        vec_to_left.z = 0.0
        if vec_to_left.length > 1e-6:
            if lateral.dot(vec_to_left) < 0.0:
                lateral = -lateral
    except Exception:
        pass
    try:
        lateral.normalize()
    except Exception:
        lateral = Vector((1.0, 0.0, 0.0))
    return lateral


def choose_local_up_axis(rest_bone):
    """Выбирает локальную ось подъема для кости"""
    m = rest_bone.matrix_local.to_3x3()
    cols = [Vector((m[0][0], m[1][0], m[2][0])),
            Vector((m[0][1], m[1][1], m[2][1])),
            Vector((m[0][2], m[1][2], m[2][2]))]
    best_idx, best_dot = 0, -1.0
    for i, c in enumerate(cols):
        if c.length < 1e-6:
            continue
        d = abs(c.normalized().dot(Vector((0.0, 0.0, 1.0))))
        if d > best_dot:
            best_dot = d
            best_idx = i
    axis_local = Vector((0.0, 0.0, 0.0))
    axis_local[best_idx] = 1.0
    return axis_local

def precise_estimate_foot_world_pos(arm_obj, rest_bone, phase, settings, fw_world, target_bone_type="foot"):
    """Вычисляет позицию стопы для FK-анимации"""
    _dbg_write(f"[PW DEBUG] precise_estimate_foot_world_pos for {rest_bone.name}, phase={phase}")

    foot_bone_name = find_target_bone(arm_obj, rest_bone.name, target_bone_type, settings)
    if not foot_bone_name:
        _dbg_write(f"[PW ERROR] No foot bone found for {rest_bone.name}")
        return Vector((0, 0, 0)), False, 0.0, 0.0

    foot_bone = arm_obj.data.bones.get(foot_bone_name)
    if not foot_bone:
        _dbg_write(f"[PW ERROR] Foot bone {foot_bone_name} not found")
        return Vector((0, 0, 0)), False, 0.0, 0.0

    try:
        # База — tail стопы в rest (чтобы lift был от rest Z стопы, а не hip)
        rest_foot_tail = arm_obj.matrix_world @ Vector(foot_bone.tail_local)
        rest_foot_z = rest_foot_tail.z  # Rest Z стопы для ограничения

        # stride_len
        stride_len = geometric_stride_length(rest_bone, settings)
        step_height = float(getattr(settings, "step_height", 0.08))

        # horizontal offset с backward_bias
        base_horiz = math.cos(phase)
        if base_horiz < 0.0:  # stance phase — сокращаем backward
            base_horiz *= float(getattr(settings, "backward_bias", 0.6))
        horiz_off = base_horiz * stride_len

        # vertical lift (только в swing; добавляется к rest стопы)
        lift = max(0.0, math.sin(phase)) * step_height * (1.0 + float(getattr(settings, "floatiness", 0.0)))
        contact = lift < 0.01

        # forward projection на горизонталь
        WORLD_UP = Vector((0.0, 0.0, 1.0))
        fw_h = fw_world.copy()
        fw_h = fw_h - fw_h.dot(WORLD_UP) * WORLD_UP
        if fw_h.length < 1e-6:
            axis = getattr(settings, "forward_axis", "Y")
            fw_h = {
                'X': Vector((1.0, 0.0, 0.0)), '-X': Vector((-1.0, 0.0, 0.0)),
                'Y': Vector((0.0, 1.0, 0.0)), '-Y': Vector((0.0, -1.0, 0.0)),
                'Z': Vector((0.0, 0.0, 0.0)), '-Z': Vector((0.0, 0.0, 0.0))
            }.get(axis, Vector((0.0, 1.0, 0.0)))
        fw_h.normalize()

        # ИНВЕРСИЯ по forward_axis
        axis = getattr(settings, "forward_axis", "Y")
        adjusted_horiz_off = -horiz_off if axis.startswith('-') else horiz_off

        # Применяем offset от rest стопы (гориз + lift)
        foot_world = rest_foot_tail + (fw_h * adjusted_horiz_off) + (WORLD_UP * lift)

        # Ограничение: цель не ниже rest Z стопы (чтобы подъём, не опускание)
        foot_world.z = max(foot_world.z, rest_foot_z)

        _dbg_write(f"[PW DEBUG] foot_world={foot_world}, contact={contact}, h_offset={adjusted_horiz_off}, v_lift={lift}, axis={axis}, rest_foot_z={rest_foot_z}")
        return foot_world, contact, adjusted_horiz_off, lift
    except Exception as e:
        _dbg_write(f"[PW ERROR] precise_estimate_foot_world_pos failed: {e}")
        return Vector((0, 0, 0)), False, 0.0, 0.0


def apply_leg_pose_and_keys(state, settings, arm_obj, left_foot_world, right_foot_world, phase_left, phase_right):
    """Применяет позу бедра, чтобы стопа достигала расчётной позиции"""
    _dbg_write(f"[PW DEBUG] apply_leg_pose_and_keys frame={state['current_frame']}")
    try:
        bpy.ops.object.mode_set(mode='POSE')
        left_rest = state['left_rest']
        right_rest = state['right_rest']
        pose = state['pose']
        left_pose = pose.bones[state['left_name']]  # LeftUpLeg
        right_pose = pose.bones[state['right_name']]  # RightUpLeg

        # Находим кости стоп
        left_foot_bone_name = find_target_bone(arm_obj, state['left_name'], "foot", settings)
        right_foot_bone_name = find_target_bone(arm_obj, state['right_name'], "foot", settings)
        left_foot_pose = arm_obj.pose.bones.get(left_foot_bone_name) if left_foot_bone_name else None
        right_foot_pose = arm_obj.pose.bones.get(right_foot_bone_name) if right_foot_bone_name else None

        # Локальные оси для вращения
        left_local_axis = left_rest.matrix_local.inverted().to_3x3() @ state['lateral_world_base']
        right_local_axis = right_rest.matrix_local.inverted().to_3x3() @ state['lateral_world_base']
        if left_local_axis.length_squared == 0.0:
            left_local_axis = Vector((1.0, 0.0, 0.0))
        else:
            left_local_axis.normalize()
        if right_local_axis.length_squared == 0.0:
            right_local_axis = Vector((1.0, 0.0, 0.0))
        else:
            right_local_axis.normalize()

        # Амплитуды и угол шага
        left_amp = float(getattr(settings, "left_amplitude", 1.0))
        right_amp = float(getattr(settings, "right_amplitude", 1.0))
        swing_left = math.sin(phase_left) * float(getattr(settings, "stride_angle", 0.35)) * left_amp
        swing_right = math.sin(phase_right) * float(getattr(settings, "stride_angle", 0.35)) * right_amp
        if getattr(settings, "use_invert", False):
            swing_left, swing_right = -swing_left, -swing_right
        if float(getattr(settings, "noise_amount", 0.0)) > 0.0:
            swing_left += (math.sin(phase_left * 7.1 + 0.3) * float(settings.noise_amount) * 0.05)
            swing_right += (math.sin(phase_right * 6.3 + 1.2) * float(settings.noise_amount) * 0.05)

        # Матрицы вращения для бедра
        rot_mat_left_local = Matrix.Rotation(swing_left, 4, left_local_axis)
        rot_mat_right_local = Matrix.Rotation(swing_right, 4, right_local_axis)
        left_pose.matrix_basis = rot_mat_left_local
        right_pose.matrix_basis = rot_mat_right_local

        # Подъём бедра (усиленный коэффициент для большего подъёма)
        left_local_up_axis = choose_local_up_axis(left_rest)
        right_local_up_axis = choose_local_up_axis(right_rest)
        lift_left_amount = max(0.0, math.sin(phase_left)) * float(getattr(settings, "step_height", 0.08)) * (1.0 + float(getattr(settings, "floatiness", 0.0))) * 1.5  # Усиление
        lift_right_amount = max(0.0, math.sin(phase_right)) * float(getattr(settings, "step_height", 0.08)) * (1.0 + float(getattr(settings, "floatiness", 0.0))) * 1.5
        left_pose.location = left_local_up_axis * lift_left_amount
        right_pose.location = right_local_up_axis * lift_right_amount

        # Корректировка: Только горизонталь (z>=0), относительно rest hip
        if left_foot_pose and left_foot_world is not None:
            world_hip_head = arm_obj.matrix_world @ Vector(left_rest.head_local)
            rel_foot = (arm_obj.matrix_world.inverted() @ left_foot_world) - (arm_obj.matrix_world.inverted() @ world_hip_head)
            rel_foot.z = max(0.0, rel_foot.z)  # Не тянем вниз
            left_pose.location += rel_foot / 2.0
            _dbg_write(f"[PW DEBUG] Adjusted {state['left_name']} horiz with rel_foot={rel_foot}, lift={lift_left_amount}")
        if right_foot_pose and right_foot_world is not None:
            world_hip_head = arm_obj.matrix_world @ Vector(right_rest.head_local)
            rel_foot = (arm_obj.matrix_world.inverted() @ right_foot_world) - (arm_obj.matrix_world.inverted() @ world_hip_head)
            rel_foot.z = max(0.0, rel_foot.z)
            right_pose.location += rel_foot / 2.0
            _dbg_write(f"[PW DEBUG] Adjusted {state['right_name']} horiz with rel_foot={rel_foot}, lift={lift_right_amount}")

        # Установка кватернионов вращения
        try:
            left_pose.rotation_quaternion = left_pose.matrix_basis.to_quaternion()
            right_pose.rotation_quaternion = right_pose.matrix_basis.to_quaternion()
        except Exception:
            left_pose.rotation_quaternion = rot_mat_left_local.to_quaternion()
            right_pose.rotation_quaternion = rot_mat_right_local.to_quaternion()

        # Удаление IK-констрейнтов, если use_ik отключен
        if not getattr(settings, "use_ik", False):
            if left_pose:
                remove_ik_constraints(left_pose)
            if right_pose:
                remove_ik_constraints(right_pose)
            if getattr(settings, "rear_copy_mode", "NONE") != "NONE":
                mask_str = getattr(settings, "bone_name_mask", "foot;toe;toebase;ankle")
                pairs = find_leg_pairs_by_name_or_space(arm_obj, mask_str)
                for l, r in pairs:
                    if l == state['left_name'] and r == state['right_name']:
                        continue
                    lp = arm_obj.pose.bones.get(l)
                    rp = arm_obj.pose.bones.get(r)
                    if lp:
                        remove_ik_constraints(lp)
                    if rp:
                        remove_ik_constraints(rp)

        # Установка ключевых кадров
        f = state['current_frame']
        left_pose.keyframe_insert(data_path="rotation_quaternion", frame=f)
        right_pose.keyframe_insert(data_path="rotation_quaternion", frame=f)
        left_pose.keyframe_insert(data_path="location", frame=f)
        right_pose.keyframe_insert(data_path="location", frame=f)

        arm_obj.pose.update()
    except Exception as e:
        _dbg_write(f"[PW ERROR] apply_leg_pose_and_keys failed: {e}")


def geometric_stride_length(rest_bone, settings):
    chain_len = _compute_chain_length(rest_bone)
    angle = float(getattr(settings, "stride_angle", 0.35))
    return 2.0 * chain_len * math.sin(angle * 0.5) * float(getattr(settings, "center_stride_scale", 1.0))

def _compute_chain_length(rest_bone):
    chain_len = 0.0
    cur = rest_bone
    prev_head = Vector(rest_bone.head_local)
    visited = set([cur.name])
    while cur.children:
        next_child = next((c for c in cur.children if c.name not in visited), None)
        if not next_child:
            break
        visited.add(next_child.name)
        next_head = Vector(next_child.head_local)
        chain_len += (next_head - prev_head).length
        prev_head = next_head
        cur = next_child
    return chain_len if chain_len > 1e-4 else max(0.5, getattr(rest_bone, "length", 0.5))

def calibrate_push_for_pair(arm_obj, left_rest, right_rest, settings, fw_world, samples=128, debug=False):
    N = max(16, int(getattr(settings, "calibration_samples", samples)))
    p1, p2 = profile_control_points(getattr(settings, "push_profile", "SMOOTH"), getattr(settings, "push_aggressiveness", 0.6))
    left_amp = float(getattr(settings, "left_amplitude", 1.0))
    right_amp = float(getattr(settings, "right_amplitude", 1.0))
    push_strength = float(getattr(settings, "push_strength", 0.9))
    half_offset = 0.5 * float(getattr(settings, "phase_offset", math.pi))
    prev_Wl = prev_Wr = 0.0
    total_raw = 0.0
    left_stride_len_const = geometric_stride_length(left_rest, settings)
    right_stride_len_const = geometric_stride_length(right_rest, settings)
    for i in range(N + 1):
        base_phase = 2.0 * math.pi * (i / float(N))
        phase_l = base_phase - half_offset
        phase_r = base_phase + half_offset
        p = phase_l % (2.0 * math.pi)
        sp_l = (p - math.pi) / math.pi if p >= math.pi else None
        p = phase_r % (2.0 * math.pi)
        sp_r = (p - math.pi) / math.pi if p >= math.pi else None
        Wl = 0.0 if sp_l is None else bezier_y_for_x(p1, p2, sp_l)
        Wr = 0.0 if sp_r is None else bezier_y_for_x(p1, p2, sp_r)
        inc_l = max(0.0, (Wl - prev_Wl)) * (left_stride_len_const * left_amp) * push_strength
        inc_r = max(0.0, (Wr - prev_Wr)) * (right_stride_len_const * right_amp) * push_strength
        total_raw += inc_l + inc_r
        prev_Wl, prev_Wr = Wl, Wr
    desired_total = (left_stride_len_const * left_amp) + (right_stride_len_const * right_amp)
    scale = 1.0 if total_raw <= 1e-12 else desired_total / total_raw
    cmin = float(getattr(settings, "calibration_clamp_min", 0.25)); cmax = float(getattr(settings, "calibration_clamp_max", 4.0))
    scale = max(cmin, min(cmax, scale))
    if debug:
        return scale, total_raw, desired_total
    return scale

def _ensure_pose_mode():
    prev = bpy.context.mode
    if prev != 'POSE':
        try:
            bpy.ops.object.mode_set(mode='POSE')
        except Exception:
            prev = None
    return prev

def _restore_mode(prev):
    if prev:
        try:
            bpy.ops.object.mode_set(mode=prev)
        except Exception:
            pass

def copy_leg_pose_with_retarget(source_pose, source_rest, target_pose, target_rest, invert=False):
    if not (source_pose and source_rest and target_pose and target_rest):
        return False
    prev_mode = _ensure_pose_mode()
    success = False
    try:
        P_s = source_pose.matrix.copy()
        R_s = source_rest.matrix_local.copy()
        R_t = target_rest.matrix_local.copy()
        try:
            delta = R_s.inverted() @ P_s
        except Exception:
            delta = Matrix.Identity(4)
        if invert:
            try:
                delta = delta.inverted()
            except Exception:
                m = delta.copy()
                m[0][0] *= -1.0; m[0][1] *= -1.0; m[0][2] *= -1.0
                delta = m
        P_t = R_t @ delta
        try:
            target_pose.matrix = P_t
            success = True
        except Exception:
            try:
                loc, rot, sc = P_t.decompose()
                target_pose.location = loc
                if hasattr(rot, "to_quaternion"):
                    target_pose.rotation_quaternion = rot.to_quaternion()
                else:
                    target_pose.rotation_quaternion = rot
                success = True
            except Exception:
                success = False
    finally:
        _restore_mode(prev_mode)
    return success

def retarget_rear_legs(arm_obj, settings, state, front_left_name, front_right_name):
    mode = getattr(settings, "rear_copy_mode", "NONE")
    if mode not in ('SAME_SIDE', 'OPPOSITE_SIDE'):
        return
    mask = getattr(settings, "bone_name_mask", "") or ""
    pairs = find_leg_pairs_by_name_or_space(arm_obj, mask)
    if not pairs:
        return
    front_idx = 0
    for i, (l, r) in enumerate(pairs):
        if l == front_left_name and r == front_right_name:
            front_idx = i; break
    src_l, src_r = pairs[front_idx]
    src_fl_pose = arm_obj.pose.bones.get(src_l) or arm_obj.pose.bones.get(front_left_name)
    src_fr_pose = arm_obj.pose.bones.get(src_r) or arm_obj.pose.bones.get(front_right_name)
    src_fl_rest = arm_obj.data.bones.get(src_l) or arm_obj.data.bones.get(front_left_name)
    src_fr_rest = arm_obj.data.bones.get(src_r) or arm_obj.data.bones.get(front_right_name)
    if not (src_fl_pose and src_fr_pose and src_fl_rest and src_fr_rest):
        return
    for i, (l_name, r_name) in enumerate(pairs):
        if i == front_idx: continue
        rear_l_pose = arm_obj.pose.bones.get(l_name)
        rear_r_pose = arm_obj.pose.bones.get(r_name)
        rear_l_rest = arm_obj.data.bones.get(l_name)
        rear_r_rest = arm_obj.data.bones.get(r_name)
        if not (rear_l_pose and rear_r_pose and rear_l_rest and rear_r_rest):
            continue
        # Удаляем IK constraints для задних ног, если use_ik отключен
        if not getattr(settings, "use_ik", False):
            remove_ik_constraints(rear_l_pose)
            remove_ik_constraints(rear_r_pose)
        try:
            # Копируем позу с учетом инверсии для OPPOSITE_SIDE
            if mode == 'SAME_SIDE':
                copy_leg_pose_with_retarget(src_fl_pose, src_fl_rest, rear_l_pose, rear_l_rest, invert=False)
                copy_leg_pose_with_retarget(src_fr_pose, src_fr_rest, rear_r_pose, rear_r_rest, invert=False)
            else:
                copy_leg_pose_with_retarget(src_fr_pose, src_fr_rest, rear_l_pose, rear_l_rest, invert=False)
                copy_leg_pose_with_retarget(src_fl_pose, src_fl_rest, rear_r_pose, rear_r_rest, invert=False)
            prev = _ensure_pose_mode()
            try:
                f = int(state.get('current_frame', bpy.context.scene.frame_current))
                rear_l_pose.keyframe_insert(data_path="rotation_quaternion", frame=f)
                rear_r_pose.keyframe_insert(data_path="rotation_quaternion", frame=f)
                rear_l_pose.keyframe_insert(data_path="location", frame=f)
                rear_r_pose.keyframe_insert(data_path="location", frame=f)
            finally:
                _restore_mode(prev)
        except Exception:
            if getattr(settings, "debug_pw", False):
                print(f"[PW DEBUG] retarget_rear_legs failed for {l_name},{r_name}")
            continue


def remove_ik_constraints(pose_bone, ik_name="PW_IK"):
    if pose_bone is None:
        return
    for c in list(pose_bone.constraints):
        if c.type == 'IK' and c.name.startswith(ik_name):
            pose_bone.constraints.remove(c)

# автоматическое определение категории
def detect_unit_category(obj, min_bones_for_living=5):
    """
    Эвристика для определения категории юнита: 'LIVING' / 'MECHANICAL' / 'SIMPLE'.

    Правила:
      - Armature:
          * >= min_bones_for_living костей -> LIVING
          * 1..(min_bones_for_living-1) костей:
              - если в именах костей встречаются органические токены -> LIVING
              - иначе -> SIMPLE
      - Mesh/Empty:
          * колёсные дети или wheel-like имя -> MECHANICAL
          * имя с prop/rotor/blade/car/truck/tank/track/tyre/tire -> MECHANICAL
          * если parent Armature ≤ 3 костей -> SIMPLE
      - иначе -> SIMPLE
    """
    if obj is None:
        return 'SIMPLE'

    if obj.type == 'ARMATURE' and getattr(obj, 'data', None):
        bcount = len(obj.data.bones)
        if bcount >= int(max(1, min_bones_for_living)):
            return 'LIVING'

        # проверка имён костей для малых арматур
        names = " ".join(b.name.lower() for b in obj.data.bones)
        living_tokens = (
            'spine', 'pelvis', 'thigh', 'calf', 'hip', 'foot', 'toe',
            'neck', 'shoulder', 'clav', 'rib', 'tentacle', 'head', 'skull',
            'jaw', 'fang', 'horn', 'antenna', 'wing', 'fin', 'tail'
        )
        if any(tok in names for tok in living_tokens):
            return 'LIVING'

        return 'SIMPLE'

    if obj.type in ('MESH', 'EMPTY', 'LIGHT', 'CURVE', 'SURFACE', 'FONT', 'META'):
        name_l = (obj.name or "").lower()

        for ch in getattr(obj, 'children', []):
            cn = (ch.name or "").lower()
            if any(w in cn for w in (
                'wheel', 'tyre', 'tire', 'track',
                '_wl', '_wr', '_fl', '_fr', '_rl', '_rr'
            )):
                return 'MECHANICAL'

        mech_tokens = (
            'wheel', 'car', 'truck', 'tank', 'tracked', 'track',
            'tyre', 'tire', 'prop', 'rotor', 'blade'
        )
        if any(k in name_l for k in mech_tokens):
            return 'MECHANICAL'

        if getattr(obj, 'parent', None) and obj.parent.type == 'ARMATURE':
            try:
                pbones = len(obj.parent.data.bones)
                if pbones <= 3:
                    return 'SIMPLE'
            except Exception:
                pass

    return 'SIMPLE'

# ПОВОРОТ

# utils.py - добавленные функции

def calculate_bone_weight(j, chain_len):
    """Вычисляет вес кости на основе её позиции в цепочке."""
    idx_norm = float(j) / float(chain_len - 1) if chain_len > 1 else 1.0
    weight = 0.2 + 0.8 * (idx_norm ** 1.1)
    if j >= chain_len - 2:
        weight *= 0.85
    return weight

def apply_spine_chain_rotation(pose, spine_chain, current_rotations, angle_total,
                              rotation_axis_world, per_bone_limits, frame):
    """Применяет поворот ко всей цепочке костей позвоночника."""
    chain_len = len(spine_chain)
    for j, bone_name in enumerate(spine_chain):
        bone = pose.bones[bone_name]
        base_quat = current_rotations[bone_name].copy()

        weight = calculate_bone_weight(j, chain_len)
        angle_for_bone = angle_total * weight

        apply_bone_rotation(bone, angle_for_bone, rotation_axis_world,
                          base_quat, bone_name, per_bone_limits, frame)

def calculate_turn_segments(frame_start, frame_end, turn_configs, settings):
    """Вычисляет сегменты анимации для каждого поворота."""
    total_frames = max(1, frame_end - frame_start + 1)
    configs_count = max(1, len(turn_configs))
    frames_per_config = max(4, total_frames // configs_count)

    segments = []
    current_frame = frame_start

    for cfg in turn_configs:
        if current_frame > frame_end:
            break

        target_angle = math.radians(cfg['angle'])
        max_ang = math.radians(180.0)
        if abs(target_angle) > max_ang:
            target_angle = math.copysign(max_ang, target_angle)

        speed_factor = cfg['speed'] / max(1e-6, getattr(settings, 'turn_speed', 1.0))
        total_seg_frames = max(4, int(frames_per_config * speed_factor))
        remaining = frame_end - current_frame + 1
        if total_seg_frames > remaining:
            total_seg_frames = remaining

        seg_frames_forward = total_seg_frames // 2
        seg_frames_back = total_seg_frames - seg_frames_forward

        segments.append({
            'target_angle': target_angle,
            'start_frame': current_frame,
            'end_forward': min(current_frame + seg_frames_forward - 1, frame_end),
            'start_back': min(current_frame + seg_frames_forward, frame_end),
            'end_back': min(current_frame + total_seg_frames - 1, frame_end)
        })

        current_frame += total_seg_frames

    return segments

def determine_spine_axis(arm_obj, spine_chain, forward_dir_world, turn_axis=None):
    WORLD_UP = Vector((0.0, 0.0, 1.0))
    WORLD_X = Vector((1.0, 0.0, 0.0))
    WORLD_Y = Vector((0.0, 1.0, 0.0))
    try:
        fwd = Vector(forward_dir_world)
        if fwd.length > 1e-6:
            fwd.normalize()
        else:
            fwd = (arm_obj.matrix_world.to_3x3() @ Vector((0.0, 1.0, 0.0))).normalized()
    except Exception:
        fwd = (arm_obj.matrix_world.to_3x3() @ Vector((0.0, 1.0, 0.0))).normalized()
    spine_vec_world = Vector((0.0, 0.0, 1.0))
    try:
        pose = arm_obj.pose
        b_head = pose.bones[spine_chain[0]].head
        b_tail = pose.bones[spine_chain[-1]].tail
        start_world = arm_obj.matrix_world @ Vector(b_head)
        end_world = arm_obj.matrix_world @ Vector(b_tail)
        spine_vec_world = (end_world - start_world).normalized()
    except Exception:
        pass
    if turn_axis:
        ta = str(turn_axis).upper()
        if ta == "X":
            rot_axis_world = (arm_obj.matrix_world.to_3x3() @ WORLD_X).normalized()
        elif ta == "Y":
            rot_axis_world = (arm_obj.matrix_world.to_3x3() @ WORLD_Y).normalized()
        else:
            rot_axis_world = (arm_obj.matrix_world.to_3x3() @ WORLD_UP).normalized()
        return WORLD_UP.copy(), spine_vec_world, rot_axis_world
    if abs(fwd.z) >= 0.9:
        rot_axis_world = (arm_obj.matrix_world.to_3x3() @ WORLD_Y).normalized()
    else:
        rot_axis_world = (arm_obj.matrix_world.to_3x3() @ WORLD_UP).normalized()
    return WORLD_UP.copy(), spine_vec_world, rot_axis_world

def setup_turn_animation(arm_obj, settings, spine_root_bone, spine_end_bone):
    """Настраивает параметры анимации поворота."""
    spine_chain = find_spine_chain(arm_obj, spine_root_bone, spine_end_bone)
    if not spine_chain:
        return None, None, None, None

    turn_configs = parse_turn_configs(settings)
    current_rotations, rest_rotations = save_bone_rotations(arm_obj.pose, spine_chain)

    forward_dir = forward_vector_world(arm_obj, settings, spine_root_bone)
    if forward_dir.length < 1e-6:
        forward_dir = Vector((0.0, -1.0, 0.0))

    forward_dir_world = (arm_obj.matrix_world.to_3x3() @ forward_dir).normalized()
    up_vector_world, spine_vector_world, rotation_axis_world = determine_spine_axis(
        arm_obj, spine_chain, forward_dir_world, getattr(settings, 'turn_axis', None))

    per_bone_limits = parse_turn_limits(getattr(settings, "turn_bone_limits", ""))

    return spine_chain, turn_configs, current_rotations, rotation_axis_world, per_bone_limits

#FullBodySwing:

# utils.py - добавленные функции

def calculate_effective_turn_angle(settings):
    """Вычисляет эффективный угол поворота на основе настроек."""
    angle_deg = float(getattr(settings, "turn_angle", 360.0))
    turn_speed = float(getattr(settings, "turn_speed", 5.0))
    effective_angle = angle_deg * (turn_speed / 5.0)
    return math.radians(effective_angle)

def setup_spine_chain(arm_obj, settings):
    """Настраивает цепочку позвоночника для поворота."""
    spine_root = getattr(settings, "spine_root_bone", "") or None
    spine_end = getattr(settings, "spine_end_bone", "") or None
    spine_chain = find_spine_chain(arm_obj, spine_root, spine_end) if spine_root and spine_end else []

    pose = arm_obj.pose
    _, rest_rotations = (save_bone_rotations(pose, spine_chain) if spine_chain else ({}, {}))
    per_bone_limits = parse_turn_limits(getattr(settings, "turn_bone_limits", "") or "")

    return spine_chain, rest_rotations, per_bone_limits

def calculate_desired_yaw(start_obj_rot_z, angle_rad, t):
    """Вычисляет желаемый угол поворота."""
    eased = ease_in_out_cubic(t)
    return start_obj_rot_z + angle_rad * eased, eased

def apply_rotation_to_armature(arm_obj, desired_yaw, frame):
    """Применяет поворот к объекту арматуры."""
    try:
        arm_obj.rotation_euler.z = desired_yaw
        try:
            arm_obj.keyframe_insert(data_path="rotation_euler", frame=frame)
        except Exception:
            pass
    except Exception:
        pass


def distribute_spine_rotation(pose, spine_chain, rest_rotations, angle_rad, t, per_bone_limits, frame):
    """Распределяет поворот по костям позвоночника."""
    if not spine_chain:
        return

    N = max(1, len(spine_chain))
    denom = float(sum(range(1, N + 1)))
    WORLD_UP = Vector((0.0, 0.0, 1.0))
    eased = ease_in_out_cubic(t)

    for idx, bname in enumerate(spine_chain):
        frac = float(idx + 1) / denom
        angle_for_bone = angle_rad * eased * frac
        base_quat = rest_rotations.get(bname, pose.bones[bname].rotation_quaternion.copy())
        apply_bone_rotation(pose.bones[bname], angle_for_bone, WORLD_UP, base_quat, bname, per_bone_limits, frame)

def update_center_bone_twist(center_pose_bone, delta_yaw, frame):
    """Обновляет поворот центральной кости."""
    if not center_pose_bone:
        return

    WORLD_UP = Vector((0.0, 0.0, 1.0))
    try:
        inv_rest = center_pose_bone.bone.matrix_local.inverted().to_3x3()
        axis_local = inv_rest @ WORLD_UP
        if axis_local.length < 1e-6:
            axis_local = Vector((0.0, 0.0, 1.0))
        else:
            axis_local.normalize()

        rotq = Quaternion(axis_local, delta_yaw)
        center_pose_bone.rotation_mode = 'QUATERNION'
        center_pose_bone.rotation_quaternion = (rotq @ center_pose_bone.rotation_quaternion).normalized()

        try:
            center_pose_bone.keyframe_insert(data_path="rotation_quaternion", frame=frame)
        except Exception:
            pass
    except Exception:
        pass

def calculate_vertical_bob(state, settings, sp_left, sp_right, left_lift, right_lift):
    """Вычисляет вертикальное движение центра масс."""
    mode = getattr(settings, "com_vertical_mode", "SUM")
    bob_amount = float(getattr(settings, "center_bob_amount", 0.0))

    if mode == "SUM":
        total_lift = (left_lift + right_lift) * 0.5  # average для smoothness
    elif mode == "STANCE":
        pl = 0.0 if sp_left is None else ease_in_out_cubic(sp_left) * left_lift
        pr = 0.0 if sp_right is None else ease_in_out_cubic(sp_right) * right_lift
        total_lift = (pl + pr) * 0.5  # average
    else:
        total_lift = 0.0

    # ИЗМЕНЕНО: Сохраняем значение vertical_bob в state, а НЕ изменяем pelvis_world_current.z напрямую
    state['vertical_bob'] = total_lift * bob_amount


# В utils.py или основном файле аддона добавьте следующую функцию для создания трекеров (empties).
# Она использует улучшенный capture_foot_path для захвата траектории (с depsgraph для точности),
# затем bake в keyframes на empty. Безопасно: нет edit mode, только evaluated_get.
# Вызывается после генерации анимации, если use_ik=True.

def create_foot_trackers(arm_obj, state, settings):
    """
    Создаёт empties-трекеры для траектории стоп (left/right), если use_ik=True.
    Применяет интерполяцию только к траектории (не к f-curves, кроме LINEAR/BEZIER).
    """
    if not getattr(settings, "use_ik", False):
        return

    # Гарантируем наличие ik_targets
    ik_targets = state.get('ik_targets', {})
    if not ik_targets:
        ik_targets = {
            state['left_name']: {'target_bone_name': find_target_bone(arm_obj, state['left_name'], 'foot', settings)},
            state['right_name']: {'target_bone_name': find_target_bone(arm_obj, state['right_name'], 'foot', settings)}
        }

    # Захват траектории
    bone_map = capture_foot_path(arm_obj, state, settings, target_bone_type="foot")

    # Применяем интерполяцию к траекториям (только если не LINEAR/BEZIER)
    mode = getattr(settings, "ik_interpolation_mode", "LINEAR")
    if mode not in ['LINEAR', 'BEZIER']:
        for rest_name, trajectory in bone_map.items():
            which_leg = 'left' if 'left' in rest_name.lower() else 'right'
            bone_map[rest_name] = apply_ik_interpolation(trajectory, settings, state['frame_start'], state['frame_end'], state, which_leg)

    # Коллекция для трекеров
    coll_name = f"PW_Trackers_{arm_obj.name}"
    if coll_name not in bpy.data.collections:
        trackers_coll = bpy.data.collections.new(coll_name)
        bpy.context.scene.collection.children.link(trackers_coll)
    else:
        trackers_coll = bpy.data.collections[coll_name]

    original_frame = bpy.context.scene.frame_current

    try:
        for rest_name, info in ik_targets.items():
            trajectory = bone_map.get(rest_name, [])
            if not trajectory:
                continue

            empty_name = f"PW_Tracker_{rest_name}"
            if empty_name in bpy.data.objects:
                bpy.data.objects.remove(bpy.data.objects[empty_name], do_unlink=True)

            empty = bpy.data.objects.new(empty_name, None)
            empty.empty_display_type = 'PLAIN_AXES'
            empty.empty_display_size = 0.05
            trackers_coll.objects.link(empty)

            # Bake траектории в keyframes
            for i, data in enumerate(trajectory):
                frame = state['frame_start'] + i
                bpy.context.scene.frame_set(frame)
                bpy.context.view_layer.update()

                empty.location = data.get('loc', Vector((0,0,0)))
                empty.rotation_quaternion = data.get('rot', Quaternion((1,0,0,0)))
                empty.keyframe_insert(data_path="location", frame=frame)
                empty.keyframe_insert(data_path="rotation_quaternion", frame=frame)

            # Настройка f-curves интерполяции (только для LINEAR/BEZIER)
            if mode in ['LINEAR', 'BEZIER']:
                apply_default_interpolation(empty, mode)

    finally:
        bpy.context.scene.frame_set(original_frame)
        bpy.context.view_layer.update()



# Улучшенный capture_foot_path: добавляем rot, используем decompose для точности.
# Режим DUPLICATE для безопасности (temp dup), ANALYTIC как fallback.
def capture_foot_path(arm_obj, state, settings, target_bone_type=None):
    mode = getattr(settings, 'ik_capture_mode', 'DUPLICATE')
    frame_start = state.get('frame_start', 1)
    frame_end = state.get('frame_end', frame_start)  # Исправление: frame_end из state
    ik_targets = state.get('ik_targets', {})
    if not ik_targets:
        _dbg_write("ERROR: No ik_targets in state, returning empty bone_map")
        return {k: [] for k in []}

    bone_map = {k: [] for k in ik_targets.keys()}
    original_frame = bpy.context.scene.frame_current

    if mode == 'ANALYTIC':
        if getattr(settings, "debug_pw", False):
            _dbg_write("[PW DEBUG] Using ANALYTIC mode")
        for f in range(frame_start, frame_end + 1):
            bpy.context.scene.frame_set(f)
            bpy.context.view_layer.update()
            for rest_name, info in ik_targets.items():
                rest_bone = arm_obj.data.bones.get(rest_name)
                if rest_bone is None:
                    bone_map[rest_name].append({'loc': arm_obj.matrix_world.translation.copy(), 'rot': Quaternion((1,0,0,0))})
                    continue
                target_bone_name = info.get('target_bone_name') or find_target_bone(arm_obj, rest_name, target_bone_type, settings)
                try:
                    pos, _, _, _ = precise_estimate_foot_world_pos(arm_obj, rest_bone, 0.0, settings, state.get('fw_world'))
                    bone_map[rest_name].append({'loc': Vector(pos), 'rot': Quaternion((1,0,0,0))})  # Rot fallback
                except Exception as e:
                    if getattr(settings, "debug_pw", False):
                        _dbg_write(f"[PW ERROR] ANALYTIC failed for {rest_name}: {e}")
                    bone_map[rest_name].append({'loc': arm_obj.matrix_world.translation.copy(), 'rot': Quaternion((1,0,0,0))})
        bpy.context.scene.frame_set(original_frame)
        return bone_map

    # DUPLICATE режим (безопасный)
    try:
        if getattr(settings, "debug_pw", False):
            _dbg_write("[PW DEBUG] Using DUPLICATE mode")
        dup = arm_obj.copy()
        dup.data = arm_obj.data.copy()
        dup.name = arm_obj.name + "_pw_capture"
        bpy.context.collection.objects.link(dup)

        # Удаляем constraints в dup для чистоты (не трогаем оригинал) — исправление: в pose mode
        try:
            bpy.context.view_layer.objects.active = dup
            bpy.ops.object.mode_set(mode='POSE')
            for pb in dup.pose.bones:
                for c in list(pb.constraints):
                    pb.constraints.remove(c)
            bpy.ops.object.mode_set(mode='OBJECT')
        except Exception as e:
            if getattr(settings, "debug_pw", False):
                _dbg_write(f"[PW DEBUG] Constraints cleanup in dup failed: {e}")

        for f in range(frame_start, frame_end + 1):
            bpy.context.scene.frame_set(f)
            bpy.context.view_layer.update()
            deps = bpy.context.evaluated_depsgraph_get()
            dup_eval = dup.evaluated_get(deps)
            for rest_name, info in ik_targets.items():
                target_bone_name = info.get('target_bone_name') or find_target_bone(arm_obj, rest_name, target_bone_type, settings)
                world_mat = dup_eval.matrix_world  # Matrix, не translation
                try:
                    pb = dup_eval.pose.bones.get(target_bone_name)
                    if pb:
                        world_mat = dup_eval.matrix_world @ pb.matrix  # Полная matrix
                except Exception as e:
                    if getattr(settings, "debug_pw", False):
                        _dbg_write(f"[PW DEBUG] Pose bone access failed for {target_bone_name}: {e}")
                loc, rot, _ = world_mat.decompose()  # Точность: loc + rot на Matrix
                bone_map[rest_name].append({'loc': loc.copy(), 'rot': rot.copy()})

        bpy.data.objects.remove(dup, do_unlink=True)
    except Exception as e:
        _dbg_write(f"capture_foot_path failed: {e}")
        # Fallback на ANALYTIC (без рекурсии, чтобы избежать loop)
        return capture_foot_path(arm_obj, state, settings, target_bone_type) if mode != 'ANALYTIC' else {k: [] for k in ik_targets.keys()}

    bpy.context.scene.frame_set(original_frame)
    return bone_map

def setup_ik_on_feet_with_trackers(arm_obj, state, settings):
    """
    Накладывает IK-констрейнты на стопы с целью в виде пустышек-трекеров.
    Работает ТОЛЬКО если use_ik=True.
    """
    if not getattr(settings, "use_ik", False):
        return

    chain_count = max(1, int(getattr(settings, "ik_chain_count", 2)))

    # Гарантируем наличие ik_targets
    ik_targets = state.get('ik_targets')
    if not ik_targets:
        ik_targets = {
            state['left_name']: {'target_bone_name': find_target_bone(arm_obj, state['left_name'], 'foot', settings)},
            state['right_name']: {'target_bone_name': find_target_bone(arm_obj, state['right_name'], 'foot', settings)}
        }

    original_frame = bpy.context.scene.frame_current
    try:
        for rest_name, info in ik_targets.items():
            foot_bone_name = info.get('target_bone_name')
            if not foot_bone_name:
                continue

            foot_pose = arm_obj.pose.bones.get(foot_bone_name)
            if not foot_pose:
                continue

            # Удаляем старые PW_IK констрейнты
            for c in list(foot_pose.constraints):
                if c.type == 'IK' and c.name.startswith("PW_IK"):
                    foot_pose.constraints.remove(c)

            # Находим пустышку-цель
            empty_target = bpy.data.objects.get(f"PW_Tracker_{rest_name}")
            if not empty_target:
                continue

            # Создаём новый IK констрейнт
            ik_con = foot_pose.constraints.new('IK')
            ik_con.name = "PW_IK_Tracker"
            ik_con.target = empty_target
            ik_con.chain_count = chain_count
            ik_con.use_rotation = False
            ik_con.use_stretch = False
            ik_con.influence = 1.0

            # Опционально: ключ на influence
            bpy.context.scene.frame_set(state['frame_start'])
            ik_con.keyframe_insert(data_path="influence")

    finally:
        bpy.context.scene.frame_set(original_frame)
        bpy.context.view_layer.update()





def _find_swing_frame_range_for_leg(current_frame, state, settings, which='left'):
    frame_start = state.get('frame_start', current_frame)
    frame_end = state.get('frame_end', current_frame)
    start = current_frame
    for f in range(current_frame, frame_start - 1, -1):
        try:
            ph_left, ph_right = compute_phases(f, state['frame_start'], state['total_frames'], settings)
        except Exception:
            base = 2.0 * math.pi * ((f - state['frame_start']) / float(max(1, state['total_frames'])))
            ph_left = base; ph_right = base + 0.5 * math.pi
        ph = ph_left if which == 'left' else ph_right
        if (ph % (2.0 * math.pi)) >= math.pi:
            start = f + 1
            break
        start = f
    end = current_frame
    for f in range(current_frame, frame_end + 1):
        try:
            ph_left, ph_right = compute_phases(f, state['frame_start'], state['total_frames'], settings)
        except Exception:
            base = 2.0 * math.pi * ((f - state['frame_start']) / float(max(1, state['total_frames'])))
            ph_left = base; ph_right = base + 0.5 * math.pi
        ph = ph_left if which == 'left' else ph_right
        if (ph % (2.0 * math.pi)) >= math.pi:
            end = f - 1
            break
        end = f
    start = max(frame_start, min(start, frame_end))
    end = max(frame_start, min(end, frame_end))
    if end < start:
        end = start
    return start, end

def _resample_positions_from_frames(pos_list, start_idx, end_idx, samples_cnt):
    if not pos_list:
        return [Vector((0,0,0))] * max(1, samples_cnt)
    n = len(pos_list)
    start_idx = max(0, min(start_idx, n - 1))
    end_idx = max(0, min(end_idx, n - 1))
    if end_idx < start_idx:
        end_idx = start_idx
    if start_idx == end_idx:
        return [pos_list[start_idx].copy() for _ in range(max(1, samples_cnt))]
    samples = []
    for i in range(samples_cnt):
        t = float(i) / float(max(1, samples_cnt - 1))
        idxf = start_idx + t * (end_idx - start_idx)
        i0 = int(math.floor(idxf)); i1 = min(len(pos_list) - 1, i0 + 1)
        frac = idxf - i0
        p = pos_list[i0].lerp(pos_list[i1], frac) if i0 != i1 else pos_list[i0].copy()
        samples.append(Vector(p))
    return samples

def apply_ik_interpolation(trajectory, settings, frame_start, frame_end, state, which_leg='left'):
    """
    Применяет выбранный режим интерполяции к траектории IK-трекера, по циклам движения ног.
    Только для SPRING/EXPONENTIAL/QUADRATIC/CONSTANT_SPEED/STEP — меняет траекторию.
    LINEAR/BEZIER — возвращает исходную (настройка на f-curves).
    """
    if not trajectory or len(trajectory) < 2:
        return trajectory

    mode = getattr(settings, "ik_interpolation_mode", "LINEAR")

    # LINEAR/BEZIER — без изменений траектории
    if mode in ['LINEAR', 'BEZIER']:
        return trajectory

    # Разбиваем на циклы
    cycles = []
    current_start = 0
    while current_start < len(trajectory):
        cycle_start, cycle_end = _find_swing_frame_range_for_leg(current_start + frame_start, state, settings, which=which_leg)
        cycle_start -= frame_start
        cycle_end -= frame_start
        if cycle_start >= len(trajectory):
            break
        cycle_end = min(cycle_end, len(trajectory) - 1)
        cycles.append((cycle_start, cycle_end))
        current_start = cycle_end + 1

    # Применяем к каждому циклу
    new_trajectory = trajectory.copy()
    for cycle_start, cycle_end in cycles:
        cycle_trajectory = new_trajectory[cycle_start:cycle_end+1]
        if len(cycle_trajectory) < 2:
            continue

        positions = [data['loc'] for data in cycle_trajectory]
        total_frames = len(positions)
        # Ресэмплинг для большей плавности (опционально, можно настроить через settings)
        samples_cnt = max(total_frames, int(getattr(settings, "ik_interpolation_samples", 10)))
        resampled_positions = _resample_positions_from_frames(positions, 0, total_frames-1, samples_cnt)
        new_positions = [resampled_positions[0].copy()]

        if mode == 'CONSTANT_SPEED':
            speed = getattr(settings, "ik_constant_speed", 0.1)
            current_pos = resampled_positions[0].copy()
            for i in range(1, samples_cnt):
                target_pos = resampled_positions[i]
                current_pos = interpolate_constant_speed(current_pos, target_pos, speed)
                new_positions.append(current_pos.copy())

        elif mode == 'EXPONENTIAL':
            alpha = getattr(settings, "ik_exponential_alpha", 0.3)
            current_pos = resampled_positions[0].copy()
            for i in range(1, samples_cnt):
                target_pos = resampled_positions[i]
                current_pos = interpolate_exponential(current_pos, target_pos, alpha)
                new_positions.append(current_pos.copy())

        elif mode == 'SPRING':
            stiffness = getattr(settings, "ik_spring_stiffness", 10.0)
            damping = getattr(settings, "ik_spring_damping", 0.5)
            dt = 1.0 / 24.0
            current_pos = resampled_positions[0].copy()
            velocity = Vector((0, 0, 0))
            for i in range(1, samples_cnt):
                target_pos = resampled_positions[i]
                current_pos, velocity = interpolate_spring(current_pos, target_pos, velocity, stiffness, damping, dt)
                new_positions.append(current_pos.copy())

        elif mode == 'STEP':
            factor = getattr(settings, "ik_step_factor", 0.5)
            current_pos = resampled_positions[0].copy()
            for i in range(1, samples_cnt):
                target_pos = resampled_positions[i]
                if i % max(1, int(1.0 / factor)) == 0:
                    current_pos = interpolate_step(current_pos, target_pos, 1.0)
                new_positions.append(current_pos.copy())

        elif mode == 'QUADRATIC':
            current_pos = resampled_positions[0].copy()
            for i in range(1, samples_cnt):
                target_pos = resampled_positions[i]
                t = i / (samples_cnt - 1) if samples_cnt > 1 else 0
                current_pos = interpolate_quadratic(current_pos, target_pos, t)
                new_positions.append(current_pos.copy())

        # Обратно ресэмплируем к исходному количеству кадров
        final_positions = _resample_positions_from_frames(new_positions, 0, len(new_positions)-1, total_frames)
        for i in range(len(final_positions)):
            idx = cycle_start + i
            new_trajectory[idx]['loc'] = final_positions[i]

    return new_trajectory



def apply_default_interpolation(obj, mode='LINEAR'):
    """
    Настраивает интерполяцию f-curves для empties.
    """
    if not obj.animation_data or not obj.animation_data.action:
        return
    action = obj.animation_data.action
    for fc in action.fcurves:
        if "location" in fc.data_path or "rotation_quaternion" in fc.data_path:
            for kp in fc.keyframe_points:
                kp.interpolation = mode
                if mode == 'BEZIER':
                    kp.handle_left_type = 'AUTO_CLAMPED'
                    kp.handle_right_type = 'AUTO_CLAMPED'
                elif mode == 'LINEAR':
                    kp.handle_left_type = 'VECTOR'
                    kp.handle_right_type = 'VECTOR'


#ДЛЯ РУК!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

# Добавить в utils.py после функции find_leg_pairs_by_name_or_space

def find_arm_chain(arm_obj, bone_name, max_depth=3):
    """
    Находит кинематическую цепь руки: плечо → предплечье → кисть
    Возвращает список имен костей в цепи
    """
    if bone_name not in arm_obj.data.bones:
        return []

    chain = [bone_name]
    current = arm_obj.data.bones[bone_name]
    depth = 0

    while current.children and depth < max_depth:
        # Ищем наиболее вероятную следующую кость в цепи
        best_child = None
        best_score = -1

        for child in current.children:
            score = 0

            # Приоритет: имя содержит признаки следующего звена
            name_lower = child.name.lower()
            if 'forearm' in name_lower or 'lowerarm' in name_lower:
                score += 3
            elif 'arm' in name_lower and 'upper' not in name_lower:
                score += 2
            elif 'hand' in name_lower or 'wrist' in name_lower:
                score += 1

            # Приоритет: самая длинная кость (обычно основное звено)
            score += child.length * 0.5

            if score > best_score:
                best_score = score
                best_child = child

        if best_child:
            chain.append(best_child.name)
            current = best_child
            depth += 1
        else:
            break

    return chain

def find_arm_pairs_by_name_or_space(arm_obj, mask_str):
    """
    Находит пары верхних костей рук с учетом симметрии
    """
    arm = arm_obj.data
    bones = list(arm.bones)
    mask_tokens = [t.strip().lower() for t in mask_str.split(';') if t.strip()]

    candidates = []
    for b in bones:
        name_lower = b.name.lower()
        # Исключаем кости, которые явно не являются верхней частью руки
        if any(exclude in name_lower for exclude in ['forearm', 'lowerarm', 'hand', 'wrist', 'finger']):
            continue

        if any(tok in name_lower for tok in mask_tokens):
            candidates.append(b)

    if not candidates:
        return []

    # Разделяем на левые и правые
    left_bones = []
    right_bones = []

    for b in candidates:
        name_lower = b.name.lower()
        # Детекция стороны
        if (name_lower.endswith(('.l', '_l', '.left', ' l')) or
            'left' in name_lower or
            ('.l.' in name_lower) or
            ('_l_' in name_lower)):
            left_bones.append(b)
        elif (name_lower.endswith(('.r', '_r', '.right', ' r')) or
              'right' in name_lower or
              ('.r.' in name_lower) or
              ('_r_' in name_lower)):
            right_bones.append(b)

    # Если нет явного указания стороны, используем геометрию
    if not left_bones or not right_bones:
        for b in candidates:
            if b.head_local.x < -0.01:  # Отрицательный X обычно левая сторона
                left_bones.append(b)
            elif b.head_local.x > 0.01:  # Положительный X обычно правая сторона
                right_bones.append(b)

    # Находим наиболее подходящие пары
    pairs = []
    if left_bones and right_bones:
        # Берем самые верхние кости (максимальный Y и Z)
        left_bones.sort(key=lambda b: (b.head_local.y, b.head_local.z), reverse=True)
        right_bones.sort(key=lambda b: (b.head_local.y, b.head_local.z), reverse=True)

        # Формируем пары по высоте
        for left in left_bones[:2]:  # Берем до 2 лучших кандидатов
            for right in right_bones[:2]:
                # Проверяем симметричность по высоте
                if abs(left.head_local.y - right.head_local.y) < 0.5 and \
                   abs(left.head_local.z - right.head_local.z) < 0.5:
                    pairs.append((left.name, right.name))

    # Если не нашли пар, пробуем альтернативный метод
    if not pairs and len(candidates) >= 2:
        # Сортируем по X координате
        candidates.sort(key=lambda b: b.head_local.x)
        leftmost = candidates[0]
        rightmost = candidates[-1]

        if leftmost.head_local.x < 0 and rightmost.head_local.x > 0:
            pairs.append((leftmost.name, rightmost.name))

    return pairs

def determine_arm_rotation_axes(arm_rest_bone, settings, fw_world, lateral_world):
    """
    Определяет оптимальные оси вращения для руки на основе rest-позы
    """
    # Получаем локальную матрицу кости
    bone_matrix = arm_rest_bone.matrix_local.to_3x3()

    # Базовые оси в локальном пространстве кости
    local_x = Vector((1, 0, 0))
    local_y = Vector((0, 1, 0))
    local_z = Vector((0, 0, 1))

    # Преобразуем мировые оси в локальные
    world_to_local = bone_matrix.inverted()
    local_fw = (world_to_local @ fw_world).normalized()
    local_lateral = (world_to_local @ lateral_world).normalized()

    # Определяем, какая локальная ось наиболее близка к направлению кости
    bone_direction = (arm_rest_bone.tail_local - arm_rest_bone.head_local).normalized()

    # Находим, какая ось (X, Y, Z) наиболее выровнена с направлением кости
    axis_scores = {
        'X': abs(bone_direction.dot(local_x)),
        'Y': abs(bone_direction.dot(local_y)),
        'Z': abs(bone_direction.dot(local_z))
    }

    primary_axis_name = max(axis_scores, key=axis_scores.get)

    # Создаем mapping имени оси на вектор
    axis_vectors = {
        'X': local_x,
        'Y': local_y,
        'Z': local_z
    }

    primary_axis = axis_vectors[primary_axis_name]

    # Определяем ось для основного качания (обычно перпендикулярно primary_axis и local_fw)
    swing_axis = primary_axis.cross(local_fw)
    if swing_axis.length < 0.01:
        swing_axis = primary_axis.cross(local_lateral)
    if swing_axis.length < 0.01:
        swing_axis = local_z.cross(primary_axis)

    swing_axis.normalize()

    # Ось для бокового движения
    lateral_axis = swing_axis.cross(primary_axis).normalized()

    return {
        'primary': primary_axis,
        'swing': swing_axis,
        'lateral': lateral_axis,
        'bone_direction': bone_direction,
        'primary_axis_name': primary_axis_name
    }

def compute_arm_swing_angles(phase, settings, state, is_left_side):
    """
    Вычисляет углы вращения для руки на основе фазы и настроек
    """
    swing_amount = float(getattr(settings, "arm_swing_amount", 0.5))
    lateral_amount = float(getattr(settings, "arm_lateral_amount", 0.1))

    # Основное качание вперед-назад (синусоида)
    swing_angle = math.sin(phase) * swing_amount

    # Боковое движение (косинусоида, смещенная по фазе)
    lateral_angle = math.cos(phase * 0.8) * lateral_amount

    # Добавляем немного шума для естественности
    if getattr(settings, "noise_amount", 0.0) > 0:
        noise_scale = float(settings.noise_amount) * 0.1
        swing_angle += math.sin(phase * 3.7 + 1.3) * noise_scale
        lateral_angle += math.cos(phase * 4.2 + 0.7) * noise_scale

    # Корректируем в зависимости от стороны
    if not is_left_side:
        lateral_angle = -lateral_angle

    return {
        'swing': swing_angle,
        'lateral': lateral_angle,
        'combined': swing_angle + lateral_angle * 0.3
    }

def apply_arm_pose_and_keys(state, settings, arm_obj, phase_left, phase_right):
    """
    Применяет процедурную анимацию рук с поддержкой многосуставности
    """
    if not getattr(settings, "use_arm_animation", False):
        return

    if not state.get('left_arm_name') or not state.get('right_arm_name'):
        return

    f = state['current_frame']

    # --- КРИТИЧНОЕ ИСПРАВЛЕНИЕ: правильные фазы для противофазного движения ---
    # Левая рука должна идти вперед, когда ПРАВАЯ нога вперед
    # Правая рука должна идти вперед, когда ЛЕВАЯ нога вперед
    phase_offset = float(getattr(settings, "arm_phase_offset", math.pi))

    # Правильный расчет: левая рука с правой ногой, правая рука с левой ногой
    arm_phase_left = phase_right + phase_offset  # phase_right, а не phase_left!
    arm_phase_right = phase_left + phase_offset  # phase_left, а не phase_right!

    # Альтернативная логика (тоже правильная):
    # arm_phase_left = phase_left + math.pi  # Левая рука на пол-цикла позади левой ноги
    # arm_phase_right = phase_right + math.pi  # Правая рука на пол-цикла позади правой ноги

    # Получаем мировые оси
    fw_world = state.get('fw_world', Vector((0, 1, 0)))
    lateral_world = state.get('lateral_world_base', Vector((1, 0, 0)))

    # Параметры
    swing_amount = float(getattr(settings, "arm_swing_amount", 0.6))
    lateral_amount = float(getattr(settings, "arm_lateral_amount", 0.1))
    base_bend = float(getattr(settings, "arm_elbow_bend", 0.4))
    dynamic_bend = float(getattr(settings, "arm_dynamic_bend", 0.3))
    stiffness = float(getattr(settings, "arm_stiffness", 0.7))

    # Анимируем каждую руку
    for side in ['left', 'right']:
        is_left = (side == 'left')
        arm_name = state.get(f'{side}_arm_name')
        forearm_name = state.get(f'{side}_forearm_name')

        if not arm_name:
            continue

        # Выбираем фазу для этой руки
        phase = arm_phase_left if is_left else arm_phase_right

        # Получаем rest-кость
        arm_rest = state.get(f'{side}_arm_rest')
        if not arm_rest:
            continue

        # 1. ВРАЩЕНИЕ ПЛЕЧА

        # Основное качание вперед-назад
        swing_angle = math.sin(phase) * swing_amount

        # Боковое движение (меньшая амплитуда)
        lateral_angle = math.cos(phase * 1.2) * lateral_amount

        # Для правой руки иногда нужно инвертировать оси
        if not is_left:
            # Для симметрии можно инвертировать некоторые углы
            swing_angle = swing_angle  # или -swing_angle в зависимости от ориентации
            lateral_angle = -lateral_angle

        # Получаем pose-кость
        arm_pose = arm_obj.pose.bones.get(arm_name)
        if not arm_pose:
            continue

        # Определяем оси вращения
        # Для простоты используем те же оси, что и для ног
        inv_matrix = arm_rest.matrix_local.inverted().to_3x3()
        swing_axis_local = (inv_matrix @ lateral_world).normalized()
        lateral_axis_local = (inv_matrix @ fw_world).normalized()

        # Создаем кватернионы вращения
        swing_quat = Quaternion(swing_axis_local, swing_angle)
        lateral_quat = Quaternion(lateral_axis_local, lateral_angle)

        # Комбинируем вращения
        combined_quat = swing_quat @ lateral_quat

        # Применяем с учетом жесткости
        current_quat = arm_pose.rotation_quaternion.copy()
        target_quat = combined_quat

        if stiffness < 1.0:
            # Интерполяция для плавности
            final_quat = current_quat.slerp(target_quat, 1.0 - stiffness)
        else:
            final_quat = target_quat

        arm_pose.rotation_quaternion = final_quat
        arm_pose.keyframe_insert(data_path="rotation_quaternion", frame=f)

        # 2. ВРАЩЕНИЕ ЛОКТЯ (ПРЕДПЛЕЧЬЯ)

        if forearm_name:
            forearm_pose = arm_obj.pose.bones.get(forearm_name)
            if forearm_pose:
                # Базовый изгиб + динамический изгиб при движении вперед
                forward_factor = max(0, math.sin(phase))  # 0..1 когда рука вперед
                total_bend = base_bend + (forward_factor * dynamic_bend)

                # Ось сгиба локтя (обычно та же, что для качания)
                elbow_axis = swing_axis_local

                # Для правой руки иногда инвертируем
                if not is_left:
                    elbow_axis = -elbow_axis

                elbow_quat = Quaternion(elbow_axis, total_bend)
                forearm_pose.rotation_quaternion = elbow_quat
                forearm_pose.keyframe_insert(data_path="rotation_quaternion", frame=f)

    # Отладочная информация
    if getattr(settings, "debug_pw", False) and f % 30 == 0:
        sin_left = math.sin(arm_phase_left)
        sin_right = math.sin(arm_phase_right)
        print(f"[ARMS] Frame {f}: L phase={arm_phase_left:.2f} (sin={sin_left:.2f}), "
              f"R phase={arm_phase_right:.2f} (sin={sin_right:.2f})")
        print(f"[ARMS] Should be OPPOSITE: {sin_left * sin_right < 0}")


# --- Vehicle Utilities ---

# utils.py — ОКОНЧАТЕЛЬНАЯ РАБОЧАЯ ВЕРСИЯ ДЛЯ ТРАНСПОРТА



def init_vehicle_state(arm_obj, settings, frame_start, frame_end):
    """
    Инициализация состояния транспорта.
    Работает с 1, 2, 3, 4+ колёсами, включая моноколёса и дроны.
    """
    v_type = getattr(settings, 'vehicle_type', 'CAR')
    state = {
        'frame_start': int(frame_start),
        'frame_end': int(frame_end),
        'total_frames': max(1, int(frame_end) - int(frame_start) + 1),
        'fps': bpy.context.scene.render.fps or 24,
        'dt': 1.0 / (bpy.context.scene.render.fps or 24),
        'arm_obj': arm_obj,
        'vehicle_type': v_type,
        'current_frame': int(frame_start),
        'current_speed': 0.0,
        'current_steering': 0.0,
        'wheels': [],
        'axles': [],
        'suspension_states': {},
        'chassis_loc': arm_obj.location.copy(),
        'chassis_quat': arm_obj.rotation_quaternion.copy(),
        'prev_speed': 0.0,
    }

    # --- Пресеты ---
    presets = {
        'CAR':   {'num_axles': 2, 'steering_mode': 'FRONT', 'susp_stiff': 10.0, 'susp_damp': 0.5, 'roll_dir': 1},
        'BIKE':  {'num_axles': 1, 'steering_mode': 'TILT',  'susp_stiff': 15.0, 'susp_damp': 0.7, 'roll_dir': -1},
        'TANK':  {'num_axles': 2, 'steering_mode': 'TANK',  'susp_stiff': 8.0,  'susp_damp': 0.4, 'roll_dir': 1},
        'TRAIN': {'num_axles': 4, 'steering_mode': 'NONE',  'susp_stiff': 5.0,  'susp_damp': 0.3, 'roll_dir': 0},
        'MONO':  {'num_axles': 1, 'steering_mode': 'BALANCE','susp_stiff': 20.0, 'susp_damp': 0.8, 'roll_dir': 1, 'bob_amp': 0.05},
        'TRI':   {'num_axles': 2, 'steering_mode': 'FRONT', 'susp_stiff': 12.0, 'susp_damp': 0.6, 'roll_dir': -1},
        'DRONE': {'num_axles': 1, 'steering_mode': 'TILT',  'susp_stiff': 25.0, 'susp_damp': 0.9, 'roll_dir': -1, 'scale': 0.5},
        'UGV':   {'num_axles': 2, 'steering_mode': 'TANK',  'susp_stiff': 10.0, 'susp_damp': 0.5, 'roll_dir': 1},
        'QUAD':  {'num_axles': 2, 'steering_mode': 'ALL',   'susp_stiff': 8.0,  'susp_damp': 0.4, 'roll_dir': 1},
    }
    p = presets.get(v_type, presets['CAR'])

    # === Пресеты и параметры (безопасно) ===
    state.update({
        'num_axles': getattr(settings, 'vehicle_num_axles', p['num_axles']),
        'steering_mode': getattr(settings, 'steering_mode', p['steering_mode']),
        'suspension_stiffness': getattr(settings, 'suspension_stiffness', p['susp_stiff']),
        'suspension_damping': getattr(settings, 'suspension_damping', p['susp_damp']),
        'roll_dir': p['roll_dir'],
        'bob_amp': p.get('bob_amp', 0.0),
        'scale': p.get('scale', 1.0),
        'max_speed': getattr(settings, 'vehicle_max_speed', 5.0) * p.get('scale', 1.0),
        'accel_rate': (getattr(settings, 'vehicle_max_speed', 5.0) * p.get('scale', 1.0)) / max(getattr(settings, 'vehicle_accel_time', 3.0), 0.1),
        'brake_rate': (getattr(settings, 'vehicle_max_speed', 5.0) * p.get('scale', 1.0)) / max(getattr(settings, 'vehicle_brake_time', 2.0), 0.1),
        'steering_max_angle': math.radians(getattr(settings, 'steering_max_angle', 30)),
        'suspension_rest': getattr(settings, 'suspension_rest', 0.0),
        'suspension_travel': getattr(settings, 'suspension_travel', 0.1) * p.get('scale', 1.0),
        'roll_strength': getattr(settings, 'roll_strength', 0.1),
        'pitch_strength': getattr(settings, 'pitch_strength', 0.05),
        'noise_freq': getattr(settings, 'noise_freq', 5.0),
    })

    # === Направления — с правильными аргументами и защитой ===
    try:
        state['fw_world'] = forward_vector_world(arm_obj, settings, bone_name=None).normalized()
    except Exception as e:
        print(f"[VEHICLE] forward_vector_world failed: {e}, using fallback Y")
        state['fw_world'] = Vector((0, 1, 0))

    try:
        lateral = compute_lateral_orientation(
            arm_obj, settings, bone_name=None,
            forward_vec=state['fw_world'], up_vec=None, target_vec=None
        )
        state['lateral_world'] = lateral.normalized() if lateral.length > 0.01 else Vector((1, 0, 0))
    except Exception as e:
        print(f"[VEHICLE] compute_lateral_orientation failed: {e}, using fallback X")
        state['lateral_world'] = Vector((1, 0, 0))

    # --- Поиск колёс ---
    mask = getattr(settings, 'vehicle_wheel_mask', 'wheel;tire;rim;front;rear;left;right')
    wheel_pairs = find_wheel_pairs_by_name_or_space(arm_obj, mask)

    if not wheel_pairs:
        print("[VEHICLE] No wheels found — running with empty state")
        return state

    # --- Обработка осей (поддержка одиночных колёс) ---
    for axle_idx in range(state['num_axles']):
        if axle_idx < len(wheel_pairs):
            pair = wheel_pairs[axle_idx]
            left_name  = pair[0]
            right_name = pair[1] if len(pair) > 1 else None
        else:
            left_name, right_name = f"dummy_{axle_idx}", None

        left_bone  = arm_obj.data.bones.get(left_name)  if left_name  else None
        right_bone = arm_obj.data.bones.get(right_name) if right_name else None

        wheels_idx = []

        if left_bone:
            radius = get_wheel_radius(left_bone)
            idx = len(state['wheels'])
            state['wheels'].append({
                'name': left_name,
                'side': 'C' if right_bone is None else 'L',
                'axle': axle_idx,
                'radius': radius,
                'angle': 0.0,
                'steerable': (state['steering_mode'] not in ['TANK', 'NONE'] and
                             (axle_idx == 0 or state['steering_mode'] == 'ALL' or v_type == 'QUAD')),
                'driven': True
            })
            wheels_idx.append(idx)
            state['suspension_states'][left_name] = {'extension': state['suspension_rest'], 'velocity': 0.0}

        if right_bone:
            radius = get_wheel_radius(right_bone)
            idx = len(state['wheels'])
            state['wheels'].append({
                'name': right_name,
                'side': 'R',
                'axle': axle_idx,
                'radius': radius,
                'angle': 0.0,
                'steerable': state['wheels'][-1]['steerable'] if left_bone else False,
                'driven': True
            })
            wheels_idx.append(idx)
            state['suspension_states'][right_name] = {'extension': state['suspension_rest'], 'velocity': 0.0}

        if wheels_idx:
            track = abs(left_bone.head_local.x - right_bone.head_local.x) if left_bone and right_bone else 0.0
            state['axles'].append({'wheels': wheels_idx, 'track_width': track, 'is_front': axle_idx == 0})

    return state


def get_wheel_radius(bone):
    return (bone.tail_local - bone.head_local).length / 2.0


def update_vehicle_speed(state, throttle, brake, dt):
    target = state['max_speed'] * throttle
    rate = state['accel_rate'] if target > state['current_speed'] else state['brake_rate']
    if brake > 0:
        rate *= brake
        target = 0.0
    state['current_speed'] += (target - state['current_speed']) * rate * dt
    state['current_speed'] = max(-state['max_speed'], min(state['max_speed'], state['current_speed']))


def update_steering(state, steering_input, dt):
    state['current_steering'] += (steering_input - state['current_steering']) * 8.0 * dt
    state['current_steering'] = max(-1.0, min(1.0, state['current_steering']))


def apply_wheel_rotation(state, arm_obj, dt):
    base_speed = state['current_speed']

    # TANK: поворот на месте — разные скорости сторон
    if state['steering_mode'] == 'TANK' and abs(base_speed) < 0.01 and abs(state['current_steering']) > 0.1:
        left_speed  = -state['current_steering'] * 4.0
        right_speed =  state['current_steering'] * 4.0
    else:
        left_speed = right_speed = base_speed

    for wheel in state['wheels']:
        speed = left_speed if wheel['side'] in {'L', 'C'} else right_speed
        omega = speed / max(wheel['radius'], 0.01)
        wheel['angle'] += omega * dt

        pb = arm_obj.pose.bones[wheel['name']]
        quat = Quaternion((1, 0, 0), wheel['angle'])
        pb.rotation_quaternion = quat
        pb.keyframe_insert('rotation_quaternion', frame=state['current_frame'])


def apply_steering(state, arm_obj):
    if not state['axles']:
        return
    angle = state['current_steering'] * state['steering_max_angle']

    for axle in state['axles']:
        indices = axle['wheels']
        if not indices:
            continue

        # Одиночное колесо
        if len(indices) == 1:
            w = state['wheels'][indices[0]]
            if w.get('steerable', False):
                pb = arm_obj.pose.bones[w['name']]
                quat = Quaternion((0, 0, 1), angle)
                pb.rotation_quaternion = quat @ pb.rotation_quaternion
                pb.keyframe_insert('rotation_quaternion', frame=state['current_frame'])
            continue

        # Пара колёс — Ackermann
        left_idx, right_idx = indices
        left_wheel = state['wheels'][left_idx]
        if not left_wheel.get('steerable', False):
            continue

        if axle['track_width'] > 0.01 and abs(angle) > 0.01:
            tr = axle['track_width']
            r = tr / (2 * math.tan(abs(angle)))
            inner = math.atan(tr / (r - tr/2)) if r > tr/2 else angle
            outer = math.atan(tr / (r + tr/2)) if r > tr/2 else angle
            l_ang = -inner if angle < 0 else inner
            r_ang = -outer if angle < 0 else outer
        else:
            l_ang = r_ang = angle

        for idx, ang in [(left_idx, l_ang), (right_idx, r_ang)]:
            w = state['wheels'][idx]
            pb = arm_obj.pose.bones[w['name']]
            quat = Quaternion((0, 0, 1), ang)
            pb.rotation_quaternion = quat @ pb.rotation_quaternion
            pb.keyframe_insert('rotation_quaternion', frame=state['current_frame'])


def update_suspension(state, arm_obj, dt, phase=0.0):
    for wheel in state['wheels']:
        susp = state['suspension_states'][wheel['name']]
        force = -state['suspension_stiffness'] * (susp['extension'] - state['suspension_rest']) - state['suspension_damping'] * susp['velocity']
        susp['velocity'] += force * dt
        susp['extension'] += susp['velocity'] * dt
        susp['extension'] = max(-state['suspension_travel'], min(state['suspension_travel'], susp['extension']))

        noise = 0.05 * abs(state['current_speed'] / max(state['max_speed'], 0.1))
        susp['extension'] += math.sin(phase * state['noise_freq'] + wheel['axle']) * noise

        pb = arm_obj.pose.bones[wheel['name']]
        pb.location.z += susp['extension']
        pb.keyframe_insert('location', frame=state['current_frame'])


def apply_chassis_motion(state, arm_obj, dt):
    delta = state['fw_world'] * state['current_speed'] * dt
    state['chassis_loc'] += delta
    arm_obj.location = state['chassis_loc']
    arm_obj.keyframe_insert('location', frame=state['current_frame'])

    if abs(state['current_speed']) > 0.01:
        yaw_rate = state['current_steering'] * (state['steering_max_angle'] / 2)
        q = Quaternion(state['lateral_world'], yaw_rate * dt)
        state['chassis_quat'] = q @ state['chassis_quat']

    accel = (state['current_speed'] - state['prev_speed']) / dt
    pitch = accel * state['pitch_strength']
    roll  = (state['current_speed']**2 * abs(state['current_steering']) / 10.0) * state['roll_strength'] * state['roll_dir']
    roll_q  = Quaternion(state['fw_world'], roll * (1 if state['current_steering'] > 0 else -1))
    pitch_q = Quaternion(state['lateral_world'], pitch)

    arm_obj.rotation_quaternion = roll_q @ pitch_q @ state['chassis_quat']
    arm_obj.keyframe_insert('rotation_quaternion', frame=state['current_frame'])

    state['prev_speed'] = state['current_speed']


def find_wheel_pairs_by_name_or_space(arm_obj, mask_str):
    """
    Поддерживает: пары L/R, одиночные колёса, любое количество осей.
    """
    bones = arm_obj.data.bones
    tokens = [t.strip().lower() for t in mask_str.split(';') if t.strip()]
    candidates = [b for b in bones if any(tok in b.name.lower() for tok in tokens)]
    if not candidates:
        return []

    candidates.sort(key=lambda b: b.head_local.y, reverse=True)
    pairs = []
    used = set()

    for bone in candidates:
        if bone.name in used:
            continue
        pair = None
        for other in candidates:
            if other is bone or other.name in used:
                continue
            if ('l' in bone.name.lower() and 'r' in other.name.lower()) or \
               ('r' in bone.name.lower() and 'l' in other.name.lower()):
                pair = other
                break
        if pair:
            left  = bone if bone.head_local.x < pair.head_local.x else pair
            right = pair if left is bone else bone
            pairs.append((left.name, right.name))
            used.update([bone.name, pair.name])
        else:
            pairs.append((bone.name, None))
            used.add(bone.name)

    return pairs


# --- JUMP PHYSICS & UTILS ---

def calculate_jump_physics(height, gravity):
    """
    Рассчитывает физические параметры прыжка.
    Возвращает:
    - v0: Начальная вертикальная скорость (м/с)
    - t_up: Время полета вверх (с)
    - t_total: Полное время полета (с) при условии приземления на ту же высоту
    """
    if height < 0.001: return 0.0, 0.0, 0.0
    v0 = math.sqrt(2.0 * abs(gravity) * height)
    t_up = v0 / abs(gravity)
    return v0, t_up, t_up * 2.0


def detect_jump_morphology(arm_obj, settings):
    """
    Анализ скелета для процедурной адаптации (Spore-like).
    Определяет 'leg_length' для расчета глубины приседа.
    """
    morph = {'leg_length': 0.5}  # Default fallback

    # Пытаемся найти длину ноги через chain length
    left_name = getattr(settings, "left_leg_name", "")
    if not left_name:
        # Пытаемся найти пару автоматически
        pairs = find_leg_pairs_by_name_or_space(arm_obj, getattr(settings, "bone_name_mask", "leg"))
        if pairs: left_name = pairs[0][0]

    if left_name and left_name in arm_obj.data.bones:
        bone = arm_obj.data.bones[left_name]
        # Используем существующую функцию geometric_stride_length как прокси длины ноги
        # (обычно stride ~ 0.7 * leg_length * 2, но нам нужен порядок величины)
        chain_len = _compute_chain_length(bone)
        morph['leg_length'] = chain_len

    return morph



# --- УТИЛИТЫ ДЛЯ IK ТРЕКЕРОВ ---

def create_jump_trackers_with_anchored_behavior(arm_obj, state, settings, tracker_history):
    """
    Создает реальные объекты-пустышки (IK Targets) на основе рассчитанной истории.
    Использует режим интерполяции BEZIER для плавности или CONSTANT для фиксации.
    """
    if not getattr(settings, "use_ik", False) or not tracker_history:
        return

    coll_name = f"PW_JumpTrackers_{arm_obj.name}"
    if coll_name in bpy.data.collections:
        coll = bpy.data.collections[coll_name]
    else:
        coll = bpy.data.collections.new(coll_name)
        bpy.context.scene.collection.children.link(coll)

    # Создаем/находим трекеры
    for leg_name, trajectory in tracker_history.items():
        if not trajectory: continue

        # Ищем соответствующую кость стопы, чтобы знать имя
        foot_bone = find_target_bone(arm_obj, leg_name, 'foot', settings)
        tracker_name = f"PW_JumpTracker_{foot_bone}"

        if tracker_name in bpy.data.objects:
            obj = bpy.data.objects[tracker_name]
        else:
            obj = bpy.data.objects.new(tracker_name, None)
            obj.empty_display_type = 'SPHERE'
            obj.empty_display_size = 0.1
            coll.objects.link(obj)

        # Применяем анимацию
        for data in trajectory:
            frame = data['frame']
            obj.location = data['loc']
            obj.rotation_euler = (0, 0, 0)  # Пока без вращения стопы

            obj.keyframe_insert(data_path="location", frame=frame)

        # Настраиваем IK constraint на кости
        setup_ik_constraint(arm_obj, foot_bone, obj, settings)


def setup_ik_constraint(arm_obj, bone_name, target_obj, settings):
    """Навешивает IK на кость, если его нет"""
    pb = arm_obj.pose.bones.get(bone_name)
    if not pb: return

    # Удаляем конфликтующие IK (от ходьбы)
    for c in pb.constraints:
        if c.type == 'IK' and c.name != "PW_Jump_IK":
            c.mute = True

    const_name = "PW_Jump_IK"
    const = pb.constraints.get(const_name)
    if not const:
        const = pb.constraints.new('IK')
        const.name = const_name

    const.target = target_obj
    const.chain_count = int(getattr(settings, "ik_chain_count", 2))
    const.use_tail = True
    const.influence = 1.0


def get_current_foot_position(arm_obj, leg_name, settings):
    """
    Получает текущую мировую позицию стопы из pose bones.
    Использует ТЕКУЩУЮ позу, а не rest.
    """
    foot_bone_name = find_target_bone(arm_obj, leg_name, 'foot', settings)

    if not foot_bone_name:
        return arm_obj.matrix_world.translation.copy()

    # Ключевое: используем POSE bone, а не DATA bone
    pose_foot = arm_obj.pose.bones.get(foot_bone_name)

    if pose_foot:
        # Полная матрица трансформации из текущей позы
        world_matrix = arm_obj.matrix_world @ pose_foot.matrix
        return world_matrix.translation.copy()
    else:
        # Fallback: tail из data bone (rest)
        data_foot = arm_obj.data.bones.get(foot_bone_name)
        if data_foot:
            return arm_obj.matrix_world @ data_foot.tail_local
        else:
            return arm_obj.matrix_world.translation.copy()