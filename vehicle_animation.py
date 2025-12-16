import bpy
import math
from mathutils import Vector, Quaternion
from . import utils
from .translations import tr


def generate_vehicle_animation(arm_obj, settings):
    """
    Главная точка входа для всех колёсных транспортных средств.
    Поведение определяется исключительно settings.vehicle_animation_mode.
    Поддерживает физично-обоснованный TURN_IN_PLACE для моноколеса (MONO).
    """
    # --- Параметры секции и безопасность ---
    frame_start = int(getattr(settings, "frame_start", 1))
    frame_end = int(getattr(settings, "frame_end", 120))
    mode = getattr(settings, "vehicle_animation_mode", "DRIVE")

    try:
        state = utils.init_vehicle_state(arm_obj, settings, frame_start, frame_end)
    except Exception as e:
        print(f"[VEHICLE ERROR] Init failed: {e}")
        return

    total_frames = state["total_frames"]
    dt = state["dt"]

    # --- Параметры для TURN_IN_PLACE (пользовательские override через settings) ---
    if mode == "TURN_IN_PLACE":
        start_rot_z = arm_obj.rotation_euler.z
        # допускаем переопределение угла поворота (в градусах)
        target_angle_deg = getattr(settings, "turn_angle_deg", 360.0)
        target_angle_rad = math.radians(target_angle_deg)

        # параметры для моноколеса (настройки видимы и регулируемы)
        mono_R_default = getattr(settings, "mono_turn_radius", 0.02)  # метры (по умолчанию 2 см)
        mono_max_roll_deg = getattr(settings, "mono_max_roll_deg", 8.0)
        mono_max_pitch_deg = getattr(settings, "mono_max_pitch_deg", 4.0)
        mono_min_radius_factor = getattr(settings, "mono_min_radius_factor", 0.02)  # *wheel_radius
        mono_max_radius_factor = getattr(settings, "mono_max_radius_factor", 0.5)   # *wheel_radius

    # общие вспомогательные значения
    accumulated_progress = 0.0
    prev_yaw = arm_obj.rotation_euler.z

    # сохраняем стартовые трансформы (для точного возврата)
    saved_start_loc = arm_obj.location.copy()
    saved_start_rot_x = arm_obj.rotation_euler.x
    saved_start_rot_y = arm_obj.rotation_euler.y
    saved_start_rot_z = arm_obj.rotation_euler.z

    for f in range(frame_start, frame_end + 1):
        bpy.context.scene.frame_set(f)
        bpy.context.view_layer.update()
        state["current_frame"] = f

        # нормализованный прогресс 0..1
        frac = (f - frame_start) / max(1.0, total_frames)
        # вычисления для обычных режимов (как раньше)
        if mode == "DRIVE":
            throttle = 1.0
            brake = 0.0
            steering = math.sin(frac * math.pi * 2) * 0.4

        elif mode == "REVERSE":
            throttle = -0.9
            brake = 0.0
            steering = 0.0

        elif mode == "TURN_IN_PLACE":
            # общий TURN_IN_PLACE: задаём нулевые значения по умолчанию
            throttle = 0.0
            brake = 0.0
            steering = 0.0

            # Если это МОНО (один привод/одно колесо) — используем специальную реалистичную логику.
            is_mono = (state.get('vehicle_type') == 'MONO') or (state.get('num_axles', 1) == 1 and len(state.get('wheels', [])) == 1)

            if is_mono:
                # --- Параметры колес/радиуса и защита от нулей ---
                wheel_radius = None
                if state.get('wheels'):
                    # берем первый wheel как моноколесный
                    wheel_radius = state['wheels'][0].get('radius', None)
                if not wheel_radius or wheel_radius <= 0.0:
                    wheel_radius = getattr(settings, 'mono_wheel_radius', 0.3)

                # ограничиваем радиус поворотной петли относительно радиуса колеса
                R_turn = max(min(mono_R_default, wheel_radius * mono_max_radius_factor), wheel_radius * mono_min_radius_factor)

                # --- easing / envelope ---
                eased = utils.ease_in_out_cubic(frac)     # для плавного yaw (0..1)
                # envelope для смещения: плавно 0 -> peak -> 0, c нулевой скоростью на концах
                envelope = math.sin(math.pi * frac) ** 2  # sin^2(pi * frac) -> мягкие концы

                # вычисляем текущий yaw (плавный прогресс от начального угла)
                current_yaw = start_rot_z + (target_angle_rad * eased)

                # фаза для петли привязываем к текущему yaw — значит локальная петля вращается вместе с корпусом
                phase = current_yaw

                # локальная петля (эллиптическая/округлая)
                local_lat = R_turn * math.cos(phase) * envelope   # локальная lateral (x)
                local_fw  = R_turn * math.sin(phase) * envelope   # локальная forward (y)

                # преобразование в мировые координаты через fw_world / lateral_world
                fw = state.get('fw_world', Vector((0, 1, 0)))
                lat = state.get('lateral_world', Vector((1, 0, 0)))
                world_offset = (lat * local_lat) + (fw * local_fw)

                # Новая локальная мировая позиция (относительно сохранённой стартовой)
                new_loc = saved_start_loc + world_offset

                # наклоны: пропорциональны envelope, с аккуратной фазой для натуральности
                max_roll = math.radians(mono_max_roll_deg)
                max_pitch = math.radians(mono_max_pitch_deg)
                new_rot_x = saved_start_rot_x + (max_pitch * math.sin(phase + math.pi / 4) * envelope)
                new_rot_y = saved_start_rot_y + (max_roll  * math.sin(phase) * envelope)
                new_rot_z = current_yaw

                # Применяем трансформы и вставляем ключи (location + rotation_euler)
                arm_obj.location = new_loc
                arm_obj.keyframe_insert(data_path="location", frame=f)

                arm_obj.rotation_euler.x = new_rot_x
                arm_obj.rotation_euler.y = new_rot_y
                arm_obj.rotation_euler.z = new_rot_z
                arm_obj.keyframe_insert(data_path="rotation_euler", frame=f)

                # --- Вращение колеса: интегрируем по фактическому перемещению центра ---
                if 'mono_prev_loc' not in state:
                    # инициализация при первом кадре
                    state['mono_prev_loc'] = saved_start_loc.copy()
                    state['mono_prev_yaw'] = saved_start_rot_z

                prev_loc = state['mono_prev_loc']
                prev_yaw_local = state['mono_prev_yaw']

                # пройденное расстояние центра между кадрами
                delta_vec = new_loc - prev_loc
                dist = delta_vec.length

                # если центр практически не прошёл расстояния, можно учесть приближенную длину дуги
                delta_yaw = new_rot_z - prev_yaw_local
                arc_len = abs(R_turn * delta_yaw)  # длина дуги = R * Δθ
                # используем максимальное из них, чтобы колесо не "застряло" если R очень маленький
                dist_used = max(dist, arc_len)

                # обновляем state.current_speed для совместимости с остальным кодом
                if dt > 0.0:
                    state['current_speed'] = dist_used / dt
                else:
                    state['current_speed'] = 0.0

                # Интегрируем угловую позицию колёс вручную и ставим keyframes
                for wheel in state.get('wheels', []):
                    r = max(wheel.get('radius', wheel_radius), 0.001)
                    add_angle = dist_used / r
                    wheel['angle'] = wheel.get('angle', 0.0) + add_angle

                    try:
                        pb = arm_obj.pose.bones[wheel['name']]
                        quat = Quaternion((1, 0, 0), wheel['angle'])
                        pb.rotation_quaternion = quat
                        pb.keyframe_insert('rotation_quaternion', frame=state['current_frame'])
                    except Exception:
                        # если pose bone отсутствует — пропускаем, но не ломаем весь процесс
                        pass

                # обновляем prev для следующего кадра
                state['mono_prev_loc'] = new_loc.copy()
                state['mono_prev_yaw'] = new_rot_z

                # обновление подвески (поддерживаем прежнюю логику)
                utils.update_suspension(state, arm_obj, dt, phase=f)

                # Всё для MONO на этом кадре сделано — пропускаем стандартную логику
                continue

            else:
                # --- НЕ-MONO: стандартная логика TURN_IN_PLACE (как раньше) ---
                if state['steering_mode'] == 'TANK':
                    steering = 1.0 if frac < 0.5 else -1.0
                    state['current_speed'] = 0.001
                else:
                    steering = 1.0 if frac < 0.5 else -1.0

                base_progress = frac
                effective_progress = base_progress
                eased_progress = utils.ease_in_out_cubic(effective_progress)
                current_angle = start_rot_z + (target_angle_rad * eased_progress)
                delta_yaw = current_angle - prev_yaw

        elif mode == "DRIFT":
            throttle = 0.95
            brake = 0.0
            steering = 1.0 if frac < 0.6 else 0.0

        elif mode == "JUMP":
            throttle = 0.0
            brake = 0.0
            steering = 0.0

        elif mode == "IDLE":
            throttle = 0.0
            brake = 0.0
            steering = 0.0

        elif mode == "CIRCLE":
            throttle = 0.8
            brake = 0.0
            steering = 0.7

        else:
            throttle = 1.0
            brake = 0.0
            steering = 0.0

        # ---------- Применяем кадр (стандартная логика для НЕ-MONO) ----------
        utils.update_vehicle_speed(state, throttle, brake, dt)
        utils.update_steering(state, steering, dt)

        utils.apply_steering(state, arm_obj)
        utils.apply_wheel_rotation(state, arm_obj, dt)

        if mode == "JUMP":
            peak = frame_start + total_frames // 2
            impulse = -20.0 if f < peak else 25.0
            for wheel in state["wheels"]:
                susp = state["suspension_states"][wheel["name"]]
                susp["velocity"] += impulse * dt

        utils.update_suspension(state, arm_obj, dt, phase=f)

        # Для TURN_IN_PLACE: вращаем объект арматуры (для НЕ-MONO)
        if mode == "TURN_IN_PLACE":
            if not ((state.get('vehicle_type') == 'MONO') or (state.get('num_axles', 1) == 1 and len(state.get('wheels', [])) == 1)):
                try:
                    arm_obj.rotation_euler.z += delta_yaw
                    arm_obj.keyframe_insert(data_path="rotation_euler", frame=f)
                    prev_yaw = current_angle
                except Exception as e:
                    print(f"[VEHICLE TURN] Error applying rotation: {e}")

                try:
                    # минимальное движение шасси для естественного покачивания
                    utils.apply_chassis_motion(state, arm_obj, dt * 0.1)
                except Exception:
                    pass
        else:
            # Для остальных режимов - нормальное движение шасси
            utils.apply_chassis_motion(state, arm_obj, dt)

    # --- ФИНАЛЬНАЯ КОРРЕКТИРОВКА для MONO: точный возврат к стартовой позиции и корректный итоговый yaw ---
    if mode == "TURN_IN_PLACE" and ((state.get('vehicle_type') == 'MONO') or (state.get('num_axles', 1) == 1 and len(state.get('wheels', [])) == 1)):
        try:
            # возвращаем точную стартовую позицию (локейшн) и ротации X/Y в исходное состояние
            arm_obj.location = saved_start_loc
            arm_obj.rotation_euler.x = saved_start_rot_x
            arm_obj.rotation_euler.y = saved_start_rot_y
            # итоговый yaw = старт + целевой угол
            arm_obj.rotation_euler.z = saved_start_rot_z + target_angle_rad

            arm_obj.keyframe_insert(data_path="location", frame=frame_end)
            arm_obj.keyframe_insert(data_path="rotation_euler", frame=frame_end)
        except Exception as e:
            print(f"[VEHICLE MONO] Final correction failed: {e}")

    print(tr("Generated {mode} animation for {obj}",
             getattr(settings, "ui_language", "en"))
          .format(mode=mode, obj=arm_obj.name))



