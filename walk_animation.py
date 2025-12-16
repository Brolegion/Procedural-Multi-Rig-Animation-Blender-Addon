import bpy
import math
import time
from mathutils import Vector, Matrix
from . import utils
from .translations import tr

def generate_cartoon_walk_for_pair(arm_obj, left_name_in, right_name_in, settings):
    frame_start = int(getattr(settings, "frame_start", 1))
    frame_end = int(getattr(settings, "frame_end", 120))
    preserve_pose = getattr(settings, "jump_preserve_pose", False)

    try:
        state = utils.init_walk_state(arm_obj, left_name_in, right_name_in, settings, frame_start, frame_end, preserve_pose)
    except Exception as e:
        print(f"[PW ERROR] generate_cartoon_walk_for_pair init error: {e}")
        return

    # Инициализация начальной позиции арматуры
    frequency = float(getattr(settings, "frequency", 1.0))
    total_cycles = frequency * (frame_end - frame_start) / 60.0
    stride_length = float(getattr(settings, "stride_length", 0.3778))
    total_distance = stride_length * total_cycles
    forward_axis = utils.forward_vector_world(arm_obj, settings)
    utils._dbg_write(f"[PW DEBUG] total_cycles={total_cycles}, stride_length={stride_length}, total_distance={total_distance}")

    # Основной цикл по кадрам
    for f in range(frame_start, frame_end + 1):
        bpy.context.scene.frame_set(f)
        bpy.context.view_layer.update()

        # Устанавливаем текущий кадр в state
        state['current_frame'] = f

        # Вычисление фаз
        phase_left, phase_right = utils.compute_phases(f, frame_start, state['total_frames'], settings)

        # Вычисление позиций стоп
        try:
            left_foot_world, left_contact, left_h_offset, left_lift = utils.precise_estimate_foot_world_pos(
                arm_obj, state['left_rest'], phase_left, settings, state['fw_world'], target_bone_type="foot"
            )
            right_foot_world, right_contact, right_h_offset, right_lift = utils.precise_estimate_foot_world_pos(
                arm_obj, state['right_rest'], phase_right, settings, state['fw_world'], target_bone_type="foot"
            )
        except Exception as e:
            utils._dbg_write(f"[PW ERROR] precise_estimate_foot_world_pos failed: {e}")
            left_foot_world, right_foot_world = Vector((0, 0, 0)), Vector((0, 0, 0))
            left_contact, right_contact = False, False
            left_lift, right_lift = 0.0, 0.0

        # Вычисление весов
        W_left, W_right, sp_left, sp_right = utils.compute_weights(phase_left, phase_right, state['p1'], state['p2'], settings)

        # Вертикальное движение центра масс
        utils.calculate_vertical_bob(state, settings, sp_left, sp_right, left_lift, right_lift)

        # Вычисление смещения центра масс
        delta_world, additional_delta_yaw = utils.compute_com_delta(state, settings, W_left, W_right, convert_forward_to_rotation=False)

        utils.compensate_lateral_if_cycle_start(state, settings, phase_left)

        # Применяем движение центра масс (исправленная функция!)
        utils.apply_center_motion(state, settings, arm_obj, delta_world, delta_yaw=additional_delta_yaw, convert_forward_to_rotation=False)

        # Пропуск ног, если только центр
        if getattr(settings, "center_only_compute", False):
            state['prev_W_left'] = W_left
            state['prev_W_right'] = W_right
            continue

        # Применение позы ног
        try:
            utils.apply_leg_pose_and_keys(state, settings, arm_obj, left_foot_world, right_foot_world, phase_left, phase_right)
        except Exception as e:
            utils._dbg_write(f"[PW ERROR] apply_leg_pose_and_keys failed: {e}")
        # после apply_leg_pose_and_keys добавить:

        # Apply arm animation
        try:
            if getattr(settings, "use_arm_animation", False):
                utils.apply_arm_pose_and_keys(state, settings, arm_obj, phase_left, phase_right)
        except Exception as e:
            utils._dbg_write(f"[PW ERROR] Arm animation failed at frame {f}: {e}")
        # Сохранение весов
        state['prev_W_left'] = W_left
        state['prev_W_right'] = W_right

        # Отладочные сообщения
        if getattr(settings, "debug_pw", True) and f < (frame_start + 4):
            utils._dbg_write(f"[PW DEBUG] f={f} fw={state['fw_world']} lat_sm={state['lateral_smoothed']:.4f} sm_delta={state['smoothed_delta']:.4f} arm_loc={arm_obj.location}")

    # Создание empties-трекеров траектории (только если use_ik=True)
    if getattr(settings, "use_ik", False):
        if state['ik_targets'] is None:
            state['ik_targets'] = {
                state['left_name']: {'target_bone_name': utils.find_target_bone(arm_obj, state['left_name'], 'foot', settings)},
                state['right_name']: {'target_bone_name': utils.find_target_bone(arm_obj, state['right_name'], 'foot', settings)}
            }
            if getattr(settings, "debug_pw", False):
                utils._dbg_write(f"[PW DEBUG] ik_targets was None, created fallback: {state['ik_targets']}")

        utils.create_foot_trackers(arm_obj, state, settings)
        utils.setup_ik_on_feet_with_trackers(arm_obj, state, settings)

    lang = settings.ui_language if hasattr(settings, "ui_language") else 'AUTO'
    print(utils.tr("Generated animation for {left}, {right}", lang).format(left=state['left_name'], right=state['right_name']))