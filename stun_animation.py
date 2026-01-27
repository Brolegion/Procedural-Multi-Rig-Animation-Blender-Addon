import bpy
import math
import random
from mathutils import Vector, Quaternion
from . import utils
from .noise_provider import get_default_noise_provider


def generate_stun_animation(arm_obj, settings):
    """
    Генерирует анимацию оглушения (Stun/Dizzy).
    Фазы: Impact -> Dizzy Loop -> Recovery
    """
    frame_start = int(settings.frame_start)
    fps = float(bpy.context.scene.render.fps)

    # Настройки времени
    dur_impact = int(0.3 * fps)  # Быстрый удар
    dur_dizzy = int(settings.stun_duration * fps)  # Основное качание
    dur_recover = int(0.8 * fps)  # Возврат

    total_frames = dur_impact + dur_dizzy + dur_recover
    settings.frame_end = frame_start + total_frames

    # Инициализация
    try:
        state = utils.init_walk_state(arm_obj, None, None, settings, frame_start, frame_start + 10, preserve_pose=True)
    except Exception as e:
        print(f"[PW ERROR] Stun Init failed: {e}")
        return False

    pelvis_start = state['rest_pelvis_world'].copy()
    fwd = state['fw_world'].normalized()
    right = state['lateral_world_base'].normalized()
    up = Vector((0, 0, 1))

    # IK трекеры (ноги стоят на месте)
    l_foot = utils.get_current_foot_position(arm_obj, state['left_name'], settings)
    r_foot = utils.get_current_foot_position(arm_obj, state['right_name'], settings)
    tracker_data = {state['left_name']: [], state['right_name']: []}

    print(f"[PW STUN] Duration: {total_frames} frames")

    for i in range(total_frames):
        frame = frame_start + i
        bpy.context.scene.frame_set(frame)
        state['current_frame'] = frame

        pelvis_offset = Vector((0, 0, 0))
        rot_offset = Quaternion((1, 0, 0, 0))

        severity = settings.stun_severity

        # --- ФАЗЫ ---
        if i < dur_impact:
            # 1. IMPACT (Удар)
            t = i / dur_impact
            # Резкий отход назад и вниз
            kick = math.sin(t * math.pi)  # 0 -> 1 -> 0

            pelvis_offset = -fwd * 0.2 * kick * severity
            pelvis_offset.z = -0.1 * kick * severity

            # Резкий наклон головы назад
            angle = math.radians(15) * kick * severity
            rot_offset = Quaternion(right, -angle)

        elif i < dur_impact + dur_dizzy:
            # 2. DIZZY (Головокружение)
            t_phase = (i - dur_impact) / fps  # Время в секундах

            # Круговое движение туловища (Lissajous)
            freq = 1.5
            sway_x = math.sin(t_phase * freq * 2 * math.pi) * 0.08 * severity
            sway_y = math.cos(t_phase * freq * 2 * math.pi) * 0.08 * severity

            # "Ватные ноги" - проседание
            sag = 0.1 * severity + (math.sin(t_phase * 3.0) * 0.02)

            pelvis_offset = Vector((sway_x, sway_y, -sag))

            # Вращение головы в противофазе телу (пытается стабилизироваться)
            q_pitch = Quaternion(right, sway_y * 1.0)
            q_roll = Quaternion(fwd, sway_x * 1.0)
            rot_offset = q_pitch @ q_roll

        else:
            # 3. RECOVERY (Восстановление)
            t = (i - (dur_impact + dur_dizzy)) / dur_recover
            # EaseOutQuad
            t_ease = 1.0 - (1.0 - t) * (1.0 - t)

            # Встряхивание головой (Shake it off)
            shake = math.sin(t * 25.0) * 0.05 * (1.0 - t_ease)

            # Возврат таза в 0
            # Здесь мы просто затухаем offset, так как процедурно считаем каждый кадр
            pelvis_offset = Vector((0, 0, 0))  # Можно интерполировать, но shake скроет это

            rot_offset = Quaternion(up, shake)

        # Применение
        target_pos = pelvis_start + pelvis_offset
        delta = target_pos - state['pelvis_world_current']
        utils.apply_center_motion(state, settings, arm_obj, delta)

        # Применяем вращение к тазу (Center Bone)
        pb_pelvis = arm_obj.pose.bones.get(state['center_name'])
        if pb_pelvis:
            pb_pelvis.rotation_quaternion = pb_pelvis.rotation_quaternion @ rot_offset
            pb_pelvis.keyframe_insert("rotation_quaternion", frame=frame)

        # Ноги фиксируем
        tracker_data[state['left_name']].append({'frame': frame, 'loc': l_foot})
        tracker_data[state['right_name']].append({'frame': frame, 'loc': r_foot})

    # IK
    if settings.use_ik:
        utils.create_jump_trackers_with_anchored_behavior(
            arm_obj, state, settings, tracker_data
        )

    return True