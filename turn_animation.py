import bpy
import math
from mathutils import Vector, Quaternion
from .utils import (
    determine_spine_axis, parse_turn_limits, clamp,
    ease_in_out_cubic, forward_vector_world, find_spine_chain,
    save_bone_rotations, parse_turn_configs, apply_bone_rotation,
    apply_spine_chain_rotation, calculate_turn_segments, setup_turn_animation
)
from .translations import tr

# Основной файл
def generate_turn_animation(arm_obj, settings):
    """
    Генерация анимации поворота для цепочки костей позвоночника.
    """
    frame_start = int(settings.frame_start)
    frame_end = int(settings.frame_end)
    
    # Настройка анимации
    result = setup_turn_animation(arm_obj, settings, settings.spine_root_bone, settings.spine_end_bone)
    if result[0] is None:
        return
        
    spine_chain, turn_configs, current_rotations, rotation_axis_world, per_bone_limits = result
    
    # Вычисление сегментов анимации
    segments = calculate_turn_segments(frame_start, frame_end, turn_configs, settings)
    
    # Применение анимации для каждого сегмента
    for seg in segments:
        # Фаза 1: поворот к цели
        for f in range(seg['start_frame'], seg['end_forward'] + 1):
            t = float(f - seg['start_frame']) / max(1, (seg['end_forward'] - seg['start_frame']))
            eased = ease_in_out_cubic(t)
            current_angle_total = seg['target_angle'] * eased
            
            apply_spine_chain_rotation(
                arm_obj.pose, spine_chain, current_rotations, 
                current_angle_total, rotation_axis_world, per_bone_limits, f
            )
        
        # Фаза 2: возврат к нулю
        for f in range(seg['start_back'], seg['end_back'] + 1):
            t = float(f - seg['start_back']) / max(1, (seg['end_back'] - seg['start_back']))
            eased = ease_in_out_cubic(t)
            current_angle_total = seg['target_angle'] * (1.0 - eased)
            
            apply_spine_chain_rotation(
                arm_obj.pose, spine_chain, current_rotations, 
                current_angle_total, rotation_axis_world, per_bone_limits, f
            )

    lang = settings.ui_language if hasattr(settings, "ui_language") else 'AUTO'
    print(tr("Generated turn animation for spine chain from {root} to {end}", lang).format(
        root=settings.spine_root_bone, end=settings.spine_end_bone))