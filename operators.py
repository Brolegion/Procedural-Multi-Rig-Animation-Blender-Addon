# operators.py
import bpy
import os
from mathutils import Vector
from .translations import tr
from .properties import apply_creature_preset_callback
from .utils import choose_primary_pair, choose_center_bone
from .walk_animation import generate_cartoon_walk_for_pair
from .turn_animation import generate_turn_animation
from .fullbodyswing_animation import generate_full_body_swing
from .vehicle_animation import generate_vehicle_animation
from .jump_animation import generate_armature_jump
from .damage_animation import generate_damage_animation
from .dodge_animation import generate_dodge_animation
from .idle_animation import generate_complex_idle
from .painc_animation import generate_panic_animation
from .sneak_animation import generate_sneak_animation
from .crawl_animation import generate_crawl_animation  # если есть
from .rage_animation import generate_rage_animation    # если есть
from .stun_animation import generate_stun_animation
from .death_animation import generate_death_animation
from .fall_animation import generate_fall_animation





# imports for simple unit animations
# Ensure simple_unit_anim.py exists and exposes these functions
from .simple_unit_anim import (
    generate_idle as simple_generate_idle,
    generate_move as simple_generate_move,
    generate_turn as simple_generate_turn,
    generate_jump as simple_generate_jump,
    generate_damage as simple_generate_damage,
    generate_dodge as simple_generate_dodge,
    generate_fly as simple_generate_fly,        # ПРАВИЛЬНО
    generate_roll as simple_generate_roll,      # ПРАВИЛЬНО
    generate_shoot as simple_generate_shoot,    # ПРАВИЛЬНО
    generate_panic as simple_generate_panic,    # ПРАВИЛЬНО
    generate_surprise as simple_generate_surprise, # ПРАВИЛЬНО
    generate_death as simple_generate_death,
    generate_stun as simple_generate_stun
)

# ==================== ОБЩИЕ ФУНКЦИИ (используются и для ходьбы, и для поворотов) ====================

def _settings_to_dict(settings):
    """Собирает все поля настроек в словарь для экспорта."""
    keys = [
        "frame_start", "frame_end", "frequency",
        "step_height", "stride_angle", "floatiness",
        "bone_name_mask", "left_leg_name", "right_leg_name",
        "forward_axis", "center_enabled", "center_apply_mode",
        "center_target_bone", "center_stride_scale", "center_preserve_height",
        "center_bob_amount", "center_only_compute", "backward_bias",
        "push_strength", "push_profile", "push_aggressiveness", "com_inertia", "com_vertical_mode",
        "animation_type", "creature_preset", "use_orbital", "use_invert", "use_lag", "noise_amount",
        "ui_language", "turn_angles", "turn_speed", "spine_root_bone", "spine_end_bone",
        "single_phase"
    ]
    out = {}
    for k in keys:
        out[k] = getattr(settings, k, "")
    return out

def _dict_to_settings(d, settings):
    """Применяет словарь настроек к объекту settings (robust type conversion)."""
    for k, v in d.items():
        if not hasattr(settings, k):
            continue
        try:
            current = getattr(settings, k)
            if isinstance(current, bool):
                if isinstance(v, str):
                    vv = v.strip().lower()
                    val = vv in ('1', 'true', 'yes', 'on')
                else:
                    val = bool(v)
            elif isinstance(current, int):
                val = int(float(v))
            elif isinstance(current, float):
                val = float(v)
            else:
                val = v
            setattr(settings, k, val)
        except Exception:
            try:
                setattr(settings, k, v)
            except Exception:
                pass


# operators.py (добавьте или замените секцию операторов списков)

class PW_OT_BoneListAdd(bpy.types.Operator):
    """Add selected bone to a specific list (Damage/Dodge)"""
    bl_idname = "pw.bone_list_add"
    bl_label = "Add Bone"
    bl_options = {'REGISTER', 'UNDO'}
    
    target_list: bpy.props.StringProperty() # 'damage_primary', 'dodge_secondary', etc.

    def execute(self, context):
        settings = context.scene.pw_settings
        
        # Mapping: target_list -> (collection_prop, active_index_prop_name)
        mapping = {
            'damage_primary':   (settings.damage_bones_primary,   'damage_active_primary'),
            'damage_secondary': (settings.damage_bones_secondary, 'damage_active_secondary'),
            'dodge_primary':    (settings.dodge_bones_primary,    'dodge_active_primary'),
            'dodge_secondary':  (settings.dodge_bones_secondary,  'dodge_active_secondary'),
            'panic_primary': (settings.panic_bones_primary, 'dodge_active_primary'),
            'panic_secondary': (settings.panic_bones_secondary, 'dodge_active_secondary'),
            'rage_primary': (settings.rage_bones_primary, 'rage_active_primary'),
            'rage_secondary': (settings.rage_bones_secondary, 'rage_active_secondary'),
        }
        
        if self.target_list not in mapping:
            self.report({'ERROR'}, f"Unknown list type: {self.target_list}")
            return {'CANCELLED'}
            
        collection, index_prop = mapping[self.target_list]
        
        # Получаем имя активной кости
        bone_name = context.active_pose_bone.name if context.active_pose_bone else "Bone"
        
        # Добавляем
        item = collection.add()
        item.bone_name = bone_name
        item.weight = 1.0
        
        # Обновляем индекс (setattr нужен, т.к. имя свойства динамическое)
        setattr(settings, index_prop, len(collection) - 1)
        
        return {'FINISHED'}

class PW_OT_BoneListRemove(bpy.types.Operator):
    """Remove active item from a specific list"""
    bl_idname = "pw.bone_list_remove"
    bl_label = "Remove Bone"
    bl_options = {'REGISTER', 'UNDO'}
    
    target_list: bpy.props.StringProperty() 

    def execute(self, context):
        settings = context.scene.pw_settings
        mapping = {
            'damage_primary':   (settings.damage_bones_primary,   'damage_active_primary'),
            'damage_secondary': (settings.damage_bones_secondary, 'damage_active_secondary'),
            'dodge_primary':    (settings.dodge_bones_primary,    'dodge_active_primary'),
            'dodge_secondary':  (settings.dodge_bones_secondary,  'dodge_active_secondary'),
        }
        
        if self.target_list not in mapping: return {'CANCELLED'}
        collection, index_prop = mapping[self.target_list]
        
        index = getattr(settings, index_prop)
        
        if 0 <= index < len(collection):
            collection.remove(index)
            # Безопасный сдвиг индекса
            new_idx = max(0, min(index, len(collection) - 1))
            setattr(settings, index_prop, new_idx)
            
        return {'FINISHED'}

class PW_OT_BoneListMove(bpy.types.Operator):
    """Move item up/down in a specific list"""
    bl_idname = "pw.bone_list_move"
    bl_label = "Move Bone"
    bl_options = {'REGISTER', 'UNDO'}
    
    target_list: bpy.props.StringProperty()
    direction: bpy.props.StringProperty(default="UP") # UP / DOWN

    def execute(self, context):
        settings = context.scene.pw_settings
        mapping = {
            'damage_primary':   (settings.damage_bones_primary,   'damage_active_primary'),
            'damage_secondary': (settings.damage_bones_secondary, 'damage_active_secondary'),
            'dodge_primary':    (settings.dodge_bones_primary,    'dodge_active_primary'),
            'dodge_secondary':  (settings.dodge_bones_secondary,  'dodge_active_secondary'),
        }
        
        if self.target_list not in mapping: return {'CANCELLED'}
        collection, index_prop = mapping[self.target_list]
        index = getattr(settings, index_prop)
        
        if self.direction == "UP":
            if index > 0:
                collection.move(index, index - 1)
                setattr(settings, index_prop, index - 1)
        elif self.direction == "DOWN":
            if index < len(collection) - 1:
                collection.move(index, index + 1)
                setattr(settings, index_prop, index + 1)
                
        return {'FINISHED'}




class PW_OT_PanicAutoFill(bpy.types.Operator):
    """Auto-detect bones for Panic (Spine -> Primary, Head/Arms -> Secondary)"""
    bl_idname = "pw.panic_auto_fill"
    bl_label = "Auto-Fill"
    bl_description = "Automatically find bones for panic animation"

    def execute(self, context):
        settings = context.scene.pw_settings
        arm = context.active_object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Select an armature first")
            return {'CANCELLED'}

        # 1. Очистка
        settings.panic_bones_primary.clear()
        settings.panic_bones_secondary.clear()

        # 2. Поиск позвоночника (Primary)
        from . import utils

        spine_root = getattr(settings, "spine_root_bone", "spine")
        spine_end = getattr(settings, "spine_end_bone", "head")

        # Пытаемся найти цепочку
        chain = utils.find_spine_chain(arm, spine_root, spine_end)

        # Fallback: просто ищем по именам, если цепочка пустая
        if not chain:
            for b in arm.data.bones:
                nm = b.name.lower()
                if ('spine' in nm or 'torso' in nm or 'chest' in nm or 'pelvis' in nm) and 'head' not in nm:
                    chain.append(b.name)

        # Добавляем в Primary (кроме головы)
        for name in chain:
            if 'head' not in name.lower() and 'neck' not in name.lower():
                item = settings.panic_bones_primary.add()
                item.bone_name = name
                item.weight = 1.0

        # 3. Поиск головы (Secondary)
        head_bone = utils.find_target_bone(arm, None, 'head', settings)
        if head_bone:
            item = settings.panic_bones_secondary.add()
            item.bone_name = head_bone
            item.weight = 0.7  # Голова сильно участвует в панике

        # 4. Руки (через маску)
        arm_mask = "arm;upperarm;shoulder;hand"
        arm_pairs = utils.find_arm_pairs_by_name_or_space(arm, arm_mask)

        if arm_pairs:
            for l, r in arm_pairs:
                # Левая рука
                i1 = settings.panic_bones_secondary.add()
                i1.bone_name = l
                i1.weight = 0.8
                # Правая рука
                i2 = settings.panic_bones_secondary.add()
                i2.bone_name = r
                i2.weight = 0.8

        # 5. Также добавляем шею, если есть
        for b in arm.data.bones:
            nm = b.name.lower()
            if 'neck' in nm and b.name not in [item.bone_name for item in settings.panic_bones_secondary]:
                item = settings.panic_bones_secondary.add()
                item.bone_name = b.name
                item.weight = 0.6

        self.report({'INFO'},
                    f"Auto-filled Panic: {len(settings.panic_bones_primary)} Prim, {len(settings.panic_bones_secondary)} Sec")
        return {'FINISHED'}
class PW_OT_DodgeAutoFill(bpy.types.Operator):
    """Auto-detect bones for Dodge (Spine -> Primary, Arms/Head -> Secondary)"""
    bl_idname = "pw.dodge_auto_fill"
    bl_label = "Auto-Fill"
    bl_description = "Automatically find spine and limbs for dodge"

    def execute(self, context):
        settings = context.scene.pw_settings
        arm = context.active_object
        if not arm or arm.type != 'ARMATURE': return {'CANCELLED'}
        
        # 1. Очистка
        settings.dodge_bones_primary.clear()
        settings.dodge_bones_secondary.clear()
        
        # 2. Поиск позвоночника (Primary)
        # Импортируем локально, чтобы не было циклических зависимостей
        from . import utils 
        
        spine_root = getattr(settings, "spine_root_bone", "spine")
        spine_end = getattr(settings, "spine_end_bone", "head")
        
        # Пытаемся найти цепочку
        chain = utils.find_spine_chain(arm, spine_root, spine_end)
        
        # Fallback: просто ищем по именам, если цепочка пустая
        if not chain:
            for b in arm.data.bones:
                nm = b.name.lower()
                if ('spine' in nm or 'torso' in nm or 'chest' in nm) and 'head' not in nm:
                    chain.append(b.name)
        
        # Добавляем в Primary (кроме головы)
        for name in chain:
            if 'head' not in name.lower() and 'neck' not in name.lower():
                item = settings.dodge_bones_primary.add()
                item.bone_name = name
                item.weight = 1.0

        # 3. Поиск головы и рук (Secondary)
        head_bone = utils.find_target_bone(arm, None, 'head', settings)
        if head_bone:
            item = settings.dodge_bones_secondary.add()
            item.bone_name = head_bone
            item.weight = 0.6 # Голова стабилизируется, но не на 100%

        # Руки (через маску)
        arm_mask = "arm;upperarm;shoulder"
        arm_pairs = utils.find_arm_pairs_by_name_or_space(arm, arm_mask)
        
        if arm_pairs:
            for l, r in arm_pairs:
                # Левая рука
                i1 = settings.dodge_bones_secondary.add()
                i1.bone_name = l
                i1.weight = 1.0 
                # Правая рука
                i2 = settings.dodge_bones_secondary.add()
                i2.bone_name = r
                i2.weight = 1.0

        self.report({'INFO'}, f"Auto-filled Dodge: {len(settings.dodge_bones_primary)} Prim, {len(settings.dodge_bones_secondary)} Sec")
        return {'FINISHED'}

class PW_OT_RageAutoFill(bpy.types.Operator):
    """Auto-detect bones for Rage animation (Spine/Arms/Head)"""
    bl_idname = "pw.rage_auto_fill"
    bl_label = "Auto-Fill"
    bl_description = "Automatically find bones for rage animation"

    def execute(self, context):
        settings = context.scene.pw_settings
        arm = context.active_object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "Select an armature first")
            return {'CANCELLED'}

        # 1. Очистка
        settings.rage_bones_primary.clear()
        settings.rage_bones_secondary.clear()

        # 2. Поиск позвоночника (Primary)
        from . import utils

        spine_root = getattr(settings, "spine_root_bone", "spine")
        spine_end = getattr(settings, "spine_end_bone", "head")

        chain = utils.find_spine_chain(arm, spine_root, spine_end)

        if not chain:
            for b in arm.data.bones:
                nm = b.name.lower()
                if ('spine' in nm or 'torso' in nm or 'chest' in nm or 'pelvis' in nm) and 'head' not in nm:
                    chain.append(b.name)

        # Добавляем в Primary
        for name in chain:
            if 'head' not in name.lower() and 'neck' not in name.lower():
                item = settings.rage_bones_primary.add()
                item.bone_name = name
                item.weight = 1.0

        # 3. Голова (Secondary)
        head_bone = utils.find_target_bone(arm, None, 'head', settings)
        if head_bone:
            item = settings.rage_bones_secondary.add()
            item.bone_name = head_bone
            item.weight = 0.8

        # 4. Руки (Secondary)
        arm_mask = "arm;upperarm;shoulder;hand"
        arm_pairs = utils.find_arm_pairs_by_name_or_space(arm, arm_mask)

        if arm_pairs:
            for l, r in arm_pairs:
                # Левая рука
                i1 = settings.rage_bones_secondary.add()
                i1.bone_name = l
                i1.weight = 1.0
                # Правая рука
                i2 = settings.rage_bones_secondary.add()
                i2.bone_name = r
                i2.weight = 1.0

        self.report({'INFO'},
                    f"Auto-filled Rage: {len(settings.rage_bones_primary)} Prim, {len(settings.rage_bones_secondary)} Sec")
        return {'FINISHED'}
class PW_OT_PresetExport(bpy.types.Operator):
    """Экспортирует настройки в файл пресета."""
    bl_idname = "pw.export_preset"
    bl_label = "Export Preset"
    bl_description = "Export current settings to a preset file"
    filepath: bpy.props.StringProperty(subtype="FILE_PATH")
    filename_ext = ".txt"

    def execute(self, context):
        settings = context.scene.pw_settings
        d = _settings_to_dict(settings)
        try:
            with open(self.filepath, 'w', encoding='utf-8') as f:
                for k, v in d.items():
                    f.write(f"{k}={v}\n")
            self.report({'INFO'}, f"Preset exported to {self.filepath}")
            return {'FINISHED'}
        except Exception as e:
            self.report({'ERROR'}, f"Export failed: {str(e)}")
            return {'CANCELLED'}

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

class PW_OT_PresetImport(bpy.types.Operator):
    """Импортирует настройки из файла пресета."""
    bl_idname = "pw.import_preset"
    bl_label = "Import Preset"
    bl_description = "Import settings from a preset file"
    filepath: bpy.props.StringProperty(subtype="FILE_PATH")

    def execute(self, context):
        settings = context.scene.pw_settings
        if not os.path.exists(self.filepath):
            self.report({'ERROR'}, f"File not found: {self.filepath}")
            return {'CANCELLED'}
        try:
            with open(self.filepath, 'r', encoding='utf-8') as f:
                d = {}
                for ln in f:
                    ln = ln.strip()
                    if not ln or ln.startswith('#'):
                        continue
                    if '=' in ln:
                        k, v = ln.split('=', 1)
                        d[k.strip()] = v.strip()
            _dict_to_settings(d, settings)
            # заставляем UI перерисоваться, чтобы новые значения сразу отображались
            try:
                if context and context.area:
                    context.area.tag_redraw()
            except Exception:
                pass
            self.report({'INFO'}, f"Preset imported from {self.filepath}")
            return {'FINISHED'}
        except Exception as e:
            self.report({'ERROR'}, f"Import failed: {str(e)}")
            return {'CANCELLED'}

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}


class PW_OT_ApplyPreset(bpy.types.Operator):
    """Применить выбранный пресет"""
    bl_idname = "pw.apply_preset"
    bl_label = "Apply Preset"
    bl_description = "Apply the selected preset"

    preset_type: bpy.props.StringProperty(default='CUSTOM')  # Добавляем свойство

    def execute(self, context):
        settings = context.scene.pw_settings

        # Если задан конкретный пресет через preset_type, используем его
        if self.preset_type != 'CUSTOM':
            settings.creature_preset = self.preset_type

        # Если это пресет урона, автоматически устанавливаем тип анимации
        if self.preset_type.startswith('DAMAGE_'):
            settings.animation_type = 'DAMAGE'

        # Применяем пресет
        apply_creature_preset_callback(settings, context)
        self.report({'INFO'}, f"Preset {settings.creature_preset} applied")
        return {'FINISHED'}

# ==================== SIMPLE UNIT OPERATOR WRAPPERS ====================
# Эти операторы просто вызывают функции из simple_unit_anim и служат для UI / быстрого доступа.

class PW_OT_GenerateSimpleIdle(bpy.types.Operator):
    """Generate simple idle animation (squash/stretch etc)"""
    bl_idname = "pw.generate_simple_idle"
    bl_label = "Generate Simple Idle"
    bl_description = "Generate idle animation for simple single-bone units"

    def execute(self, context):
        settings = context.scene.pw_settings
        arm_obj = context.active_object

        if not arm_obj:
            self.report({'WARNING'}, "No active object")
            return {'CANCELLED'}

        try:
            # Функция simple_generate_idle сама получит target через get_animation_target
            # (который теперь в utils.py), поэтому отдельно его получать не нужно.
            simple_generate_idle(arm_obj, settings)

            self.report({'INFO'}, "Simple idle generated")
            return {'FINISHED'}

        except Exception as e:
            self.report({'ERROR'}, f"Simple idle failed: {e}")
            import traceback
            traceback.print_exc()
            return {'CANCELLED'}


class PW_OT_GenerateSimpleIdle(bpy.types.Operator):
    """Generate simple idle animation"""
    bl_idname = "pw.generate_simple_idle"
    bl_label = "Generate Simple Idle"
    bl_description = "Generate idle animation for simple single-bone units"

    def execute(self, context):
        settings = context.scene.pw_settings
        obj = context.active_object
        if not obj:
            self.report({'WARNING'}, "No active object")
            return {'CANCELLED'}
        try:
            simple_generate_idle(obj, settings)          # ← только obj
            self.report({'INFO'}, "Simple idle generated")
            return {'FINISHED'}
        except Exception as e:
            self.report({'ERROR'}, f"Simple idle failed: {e}")
            import traceback; traceback.print_exc()
            return {'CANCELLED'}


class PW_OT_GenerateSimpleMove(bpy.types.Operator):
    """Generate simple move/slide/crawl animation"""
    bl_idname = "pw.generate_simple_move"
    bl_label = "Generate Simple Move"
    bl_description = "Generate move/slide animation for simple units"

    def execute(self, context):
        settings = context.scene.pw_settings
        obj = context.active_object
        if not obj:
            self.report({'WARNING'}, "No active object")
            return {'CANCELLED'}
        try:
            simple_generate_move(obj, settings)
            self.report({'INFO'}, "Simple move generated")
            return {'FINISHED'}
        except Exception as e:
            self.report({'ERROR'}, f"Simple move failed: {e}")
            import traceback; traceback.print_exc()
            return {'CANCELLED'}


# Alias: Slide -> Move
class PW_OT_GenerateSimpleSlide(PW_OT_GenerateSimpleMove):
    bl_idname = "pw.generate_simple_slide"
    bl_label = "Generate Simple Slide"


class PW_OT_GenerateSimpleTurn(bpy.types.Operator):
    """Generate simple turn for single-bone units"""
    bl_idname = "pw.generate_simple_turn"
    bl_label = "Generate Simple Turn"
    bl_description = "Generate simple turn animation"

    def execute(self, context):
        settings = context.scene.pw_settings
        obj = context.active_object
        if not obj:
            self.report({'WARNING'}, "No active object")
            return {'CANCELLED'}
        try:
            simple_generate_turn(obj, settings, angle_deg=settings.turn_angle)
            self.report({'INFO'}, "Simple turn generated")
            return {'FINISHED'}
        except Exception as e:
            self.report({'ERROR'}, f"Simple turn failed: {e}")
            import traceback; traceback.print_exc()
            return {'CANCELLED'}


class PW_OT_GenerateSimpleJump(bpy.types.Operator):
    """Generate simple jump for single-bone units"""
    bl_idname = "pw.generate_simple_jump"
    bl_label = "Generate Simple Jump"
    bl_description = "Generate simple jump animation"

    height: bpy.props.FloatProperty(default=1.0)

    def execute(self, context):
        settings = context.scene.pw_settings
        obj = context.active_object
        if not obj:
            self.report({'WARNING'}, "No active object")
            return {'CANCELLED'}
        try:
            simple_generate_jump(obj, settings, height=self.height)
            self.report({'INFO'}, "Simple jump generated")
            return {'FINISHED'}
        except Exception as e:
            self.report({'ERROR'}, f"Simple jump failed: {e}")
            import traceback; traceback.print_exc()
            return {'CANCELLED'}


class PW_OT_GenerateSimpleDamage(bpy.types.Operator):
    """Generate simple damage / recoil"""
    bl_idname = "pw.generate_simple_damage"
    bl_label = "Generate Simple Damage"
    bl_description = "Generate simple damage recoil/shake"

    intensity: bpy.props.FloatProperty(default=1.0)

    def execute(self, context):
        settings = context.scene.pw_settings
        obj = context.active_object
        if not obj:
            self.report({'WARNING'}, "No active object")
            return {'CANCELLED'}
        try:
            simple_generate_damage(obj, settings, intensity=self.intensity)
            self.report({'INFO'}, "Simple damage generated")
            return {'FINISHED'}
        except Exception as e:
            self.report({'ERROR'}, f"Simple damage failed: {e}")
            import traceback; traceback.print_exc()
            return {'CANCELLED'}


class PW_OT_GenerateSimpleDodge(bpy.types.Operator):
    """Generate simple dodge / evade animation"""
    bl_idname = "pw.generate_simple_dodge"
    bl_label = "Generate Simple Dodge"
    bl_description = "Generate dodge animation for simple units"

    def execute(self, context):
        settings = context.scene.pw_settings
        obj = context.active_object

        if not obj:
            self.report({'WARNING'}, "No active object")
            return {'CANCELLED'}

        try:
            # Собираем параметры для передачи в функцию
            kwargs = {
                'dodge_style': settings.dodge_style,
                'lateral_dist': settings.dodge_lateral_dist,
                'dodge_side': settings.dodge_side,
                'dodge_quick_frac': settings.dodge_quick_frac,
            }

            # Добавляем параметры в зависимости от стиля

            if settings.dodge_style == 'HOP':
                kwargs['hop_height'] = getattr(settings, "dodge_hop_height", 0.3)
            elif settings.dodge_style == 'SLIDE':
                kwargs['slide_tilt'] = getattr(settings, "dodge_slide_tilt", 25.0)
            elif settings.dodge_style == 'MATRIX':
                kwargs['matrix_lean'] = getattr(settings, "dodge_matrix_lean", 35.0)

            success = simple_generate_dodge(obj, settings, **kwargs)

            if success:
                self.report({'INFO'}, f"Simple dodge ({settings.dodge_style}) generated")
                return {'FINISHED'}
            else:
                self.report({'ERROR'}, "Failed to generate dodge animation")
                return {'CANCELLED'}

        except Exception as e:
            self.report({'ERROR'}, f"Simple dodge failed: {e}")
            import traceback
            traceback.print_exc()
            return {'CANCELLED'}


class PW_OT_GenerateSimpleSurprise(bpy.types.Operator):
    """Generate surprise/jump animation for simple units"""
    bl_idname = "pw.generate_simple_surprise"
    bl_label = "Generate Simple Surprise"
    bl_description = "Generate surprise reaction animation"

    def execute(self, context):
        settings = context.scene.pw_settings
        obj = context.active_object
        if not obj:
            self.report({'WARNING'}, "No active object")
            return {'CANCELLED'}
        try:
            simple_generate_surprise(obj, settings)          # ← было target → теперь obj
            self.report({'INFO'}, "Simple surprise generated")
            return {'FINISHED'}
        except Exception as e:
            self.report({'ERROR'}, f"Simple surprise failed: {e}")
            import traceback; traceback.print_exc()
            return {'CANCELLED'}


class PW_OT_GenerateSimpleDeath(bpy.types.Operator):
    """Generate death animation for simple units (5 styles available)"""
    bl_idname = "pw.simple_generate_death"
    bl_label = "Generate Simple Death"
    bl_description = "Generate death animation with selected style"

    @classmethod
    def poll(cls, context):
        return context.active_object is not None

    def execute(self, context):
        settings = context.scene.pw_settings
        obj = context.active_object

        if not obj:
            self.report({'WARNING'}, "No active object selected")
            return {'CANCELLED'}

        try:
            kwargs = {
                'death_style': settings.death_style,
            }

            # Добавляем параметры в зависимости от стиля
            if settings.death_style == 'COLLAPSE':
                kwargs.update({
                    'collapse_power': settings.death_collapse_power,
                    'dissolve_speed': settings.death_dissolve_speed,
                    'collapse_shake': settings.death_collapse_shake,
                })
            elif settings.death_style == 'EXPLOSION':
                kwargs.update({
                    'explosion_power': settings.death_explosion_power,
                    'spin_speed': settings.death_spin_speed,
                    'explosion_shake': settings.death_explosion_shake,
                })
            elif settings.death_style == 'FALL':
                kwargs.update({
                    'fall_height': settings.death_fall_height,
                    'fall_bounce': settings.death_fall_bounce,
                    'fall_side': settings.death_fall_side,
                    'fall_rotation': settings.death_fall_rotation,
                })
            elif settings.death_style == 'SIDE_TUMBLE':
                kwargs.update({
                    'side_angle': settings.death_side_angle,
                    'side_wobble': settings.death_side_wobble,
                    'side_slide': settings.death_side_slide,
                })
            elif settings.death_style == 'KNOCKBACK':
                kwargs.update({
                    'knockback_distance': settings.death_knockback_distance,
                    'knockback_height': settings.death_knockback_height,
                    'knockback_spin': settings.death_knockback_spin,
                })

            success = simple_generate_death(obj, settings, **kwargs)

            if success:
                self.report({'INFO'}, f"Death animation ({settings.death_style}) generated successfully!")
            else:
                self.report({'ERROR'}, "Failed to generate death animation")

            return {'FINISHED'}

        except Exception as e:
            self.report({'ERROR'}, f"Death animation failed: {str(e)}")
            import traceback
            traceback.print_exc()
            return {'CANCELLED'}

class PW_OT_GenerateSimpleStun(bpy.types.Operator):
    """Generate stun animation for simple units"""
    bl_idname = "pw.simple_generate_stun"
    bl_label = "Generate Simple Stun"
    bl_description = "Generate stun animation"

    def execute(self, context):
        settings = context.scene.pw_settings
        obj = context.active_object
        if not obj:
            self.report({'WARNING'}, "No active object")
            return {'CANCELLED'}
        try:
            simple_generate_stun(obj, settings)          # ← было target → теперь obj
            self.report({'INFO'}, "Simple stun generated")
            return {'FINISHED'}
        except Exception as e:
            self.report({'ERROR'}, f"Simple stun failed: {e}")
            import traceback; traceback.print_exc()
            return {'CANCELLED'}


class PW_OT_GenerateSimplePanic(bpy.types.Operator):
    """Generate panic/vibration animation for simple units"""
    bl_idname = "pw.generate_simple_panic"
    bl_label = "Generate Simple Panic"
    bl_description = "Generate panic vibration animation"

    def execute(self, context):
        settings = context.scene.pw_settings
        obj = context.active_object
        if not obj:
            self.report({'WARNING'}, "No active object")
            return {'CANCELLED'}
        try:
            simple_generate_panic(obj, settings)             # ← было target → теперь obj
            self.report({'INFO'}, "Simple panic generated")
            return {'FINISHED'}
        except Exception as e:
            self.report({'ERROR'}, f"Simple panic failed: {e}")
            import traceback; traceback.print_exc()
            return {'CANCELLED'}


class PW_OT_GenerateSimpleShoot(bpy.types.Operator):
    """Generate shooting recoil animation for simple units"""
    bl_idname = "pw.generate_simple_shoot"
    bl_label = "Generate Simple Shoot"
    bl_description = "Generate shooting recoil animation"

    def execute(self, context):
        settings = context.scene.pw_settings
        obj = context.active_object
        if not obj:
            self.report({'WARNING'}, "No active object")
            return {'CANCELLED'}
        try:
            simple_generate_shoot(obj, settings)             # ← было target → теперь obj
            self.report({'INFO'}, "Simple shoot generated")
            return {'FINISHED'}
        except Exception as e:
            self.report({'ERROR'}, f"Simple shoot failed: {e}")
            import traceback; traceback.print_exc()
            return {'CANCELLED'}


class PW_OT_GenerateSimpleRoll(bpy.types.Operator):
    """Generate rolling/tumbling animation for simple units"""
    bl_idname = "pw.generate_simple_roll"
    bl_label = "Generate Simple Roll"
    bl_description = "Generate rolling animation"

    def execute(self, context):
        settings = context.scene.pw_settings
        obj = context.active_object
        if not obj:
            self.report({'WARNING'}, "No active object")
            return {'CANCELLED'}
        try:
            simple_generate_roll(obj, settings)              # ← было target → теперь obj
            self.report({'INFO'}, "Simple roll generated")
            return {'FINISHED'}
        except Exception as e:
            self.report({'ERROR'}, f"Simple roll failed: {e}")
            import traceback; traceback.print_exc()
            return {'CANCELLED'}


class PW_OT_GenerateSimpleFly(bpy.types.Operator):
    """Generate flying/hovering animation for simple units"""
    bl_idname = "pw.generate_simple_fly"
    bl_label = "Generate Simple Fly"
    bl_description = "Generate flying animation"

    def execute(self, context):
        settings = context.scene.pw_settings
        obj = context.active_object
        if not obj:
            self.report({'WARNING'}, "No active object")
            return {'CANCELLED'}
        try:
            simple_generate_fly(obj, settings)               # ← было target → теперь obj
            self.report({'INFO'}, "Simple fly generated")
            return {'FINISHED'}
        except Exception as e:
            self.report({'ERROR'}, f"Simple fly failed: {e}")
            import traceback; traceback.print_exc()
            return {'CANCELLED'}


# Новый оператор (для отдельной кнопки, если нужно)
class PW_OT_GenerateVehicle(bpy.types.Operator):
    """Generate procedural vehicle animation"""
    bl_idname = "pw.generate_vehicle"
    bl_label = "Generate Vehicle Anim"
    bl_description = "Generate animation for wheeled vehicles"

    def execute(self, context):
        settings = context.scene.pw_settings
        arm_obj = context.active_object
        if not arm_obj or arm_obj.type != 'ARMATURE':
            self.report({'WARNING'}, "Select an armature")
            return {'CANCELLED'}
        try:
            generate_vehicle_animation(arm_obj, settings)
            self.report({'INFO'}, "Vehicle animation generated")
            return {'FINISHED'}
        except Exception as e:
            self.report({'ERROR'}, f"Failed: {str(e)}")
            return {'CANCELLED'}

# === ОПЕРАТОР ДЛЯ ВСЕХ MECHANICAL АНИМАЦИЙ ===
class PW_OT_GenerateMechanical(bpy.types.Operator):
    bl_idname = "pw.generate_mechanical"
    bl_label = "Generate Mechanical Animation"
    mode: bpy.props.StringProperty(default="VEHICLE")  # из кнопки

    def execute(self, context):
        s = context.scene.pw_settings
        obj = context.active_object

        if s.mech_unit_type in {'VEHICLE', 'DRONE'}:
            from .vehicle_animation import generate_vehicle_animation
            generate_vehicle_animation(obj, s)  # внутри читает vehicle_animation_mode
        elif s.mech_unit_type == 'HOVER':
            simple_generate_fly(obj, s)
        # и т.д.

        return {'FINISHED'}



# ==================== ОПЕРАТОРЫ ДЛЯ ГЕНЕРАЦИИ АНИМАЦИИ ХОДЬБЫ ====================

class PW_OT_Generate(bpy.types.Operator):
    """Генерирует процедурную анимацию и применяет центр масс (COM)."""
    bl_idname = "pw.generate_animation"
    bl_label = "Generate Animation"
    bl_description = "Generate procedural animation and compute/apply COM"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        settings = context.scene.pw_settings
        arm_obj = context.active_object
        lang = settings.ui_language if hasattr(settings, "ui_language") else 'AUTO'
        
        if not arm_obj or arm_obj.type != 'ARMATURE':
            self.report({'WARNING'}, tr("Select an armature object", lang))
            return {'CANCELLED'}

        # Применяем пресет перед генерацией анимации
        if settings.creature_preset != 'CUSTOM':
            apply_creature_preset_callback(settings, context)

        auto_left, auto_right = choose_primary_pair(arm_obj, settings.bone_name_mask)
        if settings.left_leg_name and settings.right_leg_name:
            left_name = settings.left_leg_name
            right_name = settings.right_leg_name
        else:
            left_name = auto_left
            right_name = auto_right

        if not settings.center_target_bone:
            settings.center_target_bone = choose_center_bone(arm_obj) or ""

        try:
            # Prior behaviour (complex living rigs)
            if settings.animation_type == 'WALK':
                generate_cartoon_walk_for_pair(arm_obj, left_name, right_name, settings)
            elif settings.animation_type == 'IDLE':
                generate_complex_idle(arm_obj, settings)
            elif settings.animation_type == 'TURN' and getattr(settings, "unit_category", "LIVING") != 'SIMPLE':
                generate_turn_animation(arm_obj, settings)
            elif settings.animation_type == 'FULL_BODY_SWING':
                generate_full_body_swing(arm_obj, left_name, right_name, settings)
            # В методе execute класса PW_OT_Generate добавьте:
            elif settings.animation_type == 'DODGE':
                # Для сложных арматур используем функцию уклонения
                success = generate_dodge_animation(arm_obj, settings)
            elif settings.animation_type == 'DAMAGE':
                generate_damage_animation(arm_obj, settings)
            elif settings.animation_type == 'PANIC':
                success = generate_panic_animation(arm_obj, settings)
            elif settings.animation_type == 'JUMP':
                # Для сложных арматур используем новую функцию
                generate_armature_jump(arm_obj, settings)
            elif settings.animation_type == 'SNEAK':
                success = generate_sneak_animation(arm_obj, settings)
            elif settings.animation_type == 'CRAWL':
                success = generate_crawl_animation(arm_obj, settings)
            elif settings.animation_type == 'RAGE':
                success = generate_rage_animation(arm_obj, settings)
            elif settings.animation_type == 'STUN':
                success = generate_stun_animation(arm_obj, settings)
            elif settings.animation_type == 'DEATH':
                generate_death_animation(arm_obj, settings)
            elif settings.animation_type == 'FALL':
                success = generate_fall_animation(arm_obj, settings)
            else:
                # Handle SIMPLE units and other simple animation types
                if getattr(settings, "unit_category", "LIVING") == 'SIMPLE':
                    atype = settings.animation_type.upper() if getattr(settings, "animation_type", None) else "IDLE"
                    if atype == 'IDLE':
                        simple_generate_idle(arm_obj, settings)
                    elif atype in ('DRONE', 'SLIDE', 'MOVE', 'WALK'):
                        # slide/move — same generator for simple units
                        simple_generate_move(arm_obj, settings)
                    elif atype == 'TURN':
                        simple_generate_turn(arm_obj, settings)
                    elif atype == 'JUMP':
                        # JUMP may use settings.jump_height if present; fallback to 1.0
                        h = float(getattr(settings, "jump_height", getattr(settings, "simple_jump_height", 1.0)))
                        simple_generate_jump(arm_obj, settings, height=h)
                    elif atype == 'ATTACK' or atype == 'DAMAGE':
                        simple_generate_damage(arm_obj, settings, intensity=1.0)
                    elif atype == 'DODGE' or atype == 'EVADE':
                        simple_generate_dodge(arm_obj, settings)
                    else:
                        # fallback to move for unknown simple types
                        simple_generate_move(arm_obj, settings)
                elif settings.unit_category == 'MECHANICAL':
                    atype = settings.mech_animation_type
                    if atype == 'VEHICLE' or atype == 'DRONE':  # DRONE как variant vehicle
                        generate_vehicle_animation(arm_obj, settings)
                    elif atype == 'FLY':
                        simple_generate_fly(arm_obj, settings)  # Reuse from simple
                    elif atype == 'IDLE':
                        simple_generate_idle(arm_obj, settings)
                    elif atype == 'TURN':
                        simple_generate_turn(arm_obj, settings)
                    elif atype == 'JUMP':
                        simple_generate_jump(arm_obj, settings)
                    # ... аналогично для других (DAMAGE, DODGE, etc. — reuse simple funcs)
                    else:
                        # Fallback to MOVE
                        simple_generate_move(arm_obj, settings)
                    self.report({'INFO'}, f"{atype} mech animation generated")
                else:
                    # non-simple fallback to walk generator
                    generate_cartoon_walk_for_pair(arm_obj, left_name, right_name, settings)
        except Exception as e:
            self.report({'ERROR'}, f"Animation generation failed: {str(e)}")
            import traceback
            traceback.print_exc()
            return {'CANCELLED'}

        self.report({'INFO'}, tr("Generated animation for {left}, {right}", lang).format(
            left=left_name if left_name else settings.spine_root_bone,
            right=right_name if right_name else settings.spine_end_bone
        ))
        return {'FINISHED'}

# ==================== ОПЕРАТОРЫ ДЛЯ УПРАВЛЕНИЯ ПОВОРОТОВ ====================

class PW_OT_AddTurnConfig(bpy.types.Operator):
    """Добавляет новую конфигурацию поворота."""
    bl_idname = "pw.add_turn_config"
    bl_label = "Add Turn Config"
    bl_description = "Add a new turn configuration"
    index: bpy.props.IntProperty()

    def execute(self, context):
        settings = context.scene.pw_settings
        if not settings:
            return {'CANCELLED'}
        current_configs = settings.turn_angles.split(';') if settings.turn_angles else []
        new_config = f"{settings.turn_angle},{settings.turn_speed}"
        current_configs.insert(self.index, new_config)
        settings.turn_angles = ';'.join(current_configs)
        return {'FINISHED'}

class PW_OT_SetNegativeAngle(bpy.types.Operator):
    """Устанавливает отрицательный угол поворота"""
    bl_idname = "pw.set_negative_angle"
    bl_label = "Set Negative Angle"
    bl_description = "Set a negative angle value"
    
    angle: bpy.props.FloatProperty(default=-90.0)
    
    def execute(self, context):
        settings = context.scene.pw_settings
        settings.turn_angle = self.angle
        return {'FINISHED'}

# ==================== REGISTRATION ====================

def register():
    bpy.utils.register_class(PW_OT_PresetExport)
    bpy.utils.register_class(PW_OT_PresetImport)
    bpy.utils.register_class(PW_OT_ApplyPreset)

    # Simple unit operators
    bpy.utils.register_class(PW_OT_GenerateSimpleIdle)
    bpy.utils.register_class(PW_OT_GenerateSimpleMove)
    bpy.utils.register_class(PW_OT_GenerateSimpleSlide)
    bpy.utils.register_class(PW_OT_GenerateSimpleTurn)
    bpy.utils.register_class(PW_OT_GenerateSimpleJump)
    bpy.utils.register_class(PW_OT_GenerateSimpleDamage)
    bpy.utils.register_class(PW_OT_GenerateSimpleDodge)
    
    # New simple unit operators
    bpy.utils.register_class(PW_OT_GenerateSimpleSurprise)
    bpy.utils.register_class(PW_OT_GenerateSimplePanic)
    bpy.utils.register_class(PW_OT_GenerateSimpleShoot)
    bpy.utils.register_class(PW_OT_GenerateSimpleRoll)
    bpy.utils.register_class(PW_OT_GenerateSimpleFly)
    bpy.utils.register_class(PW_OT_GenerateSimpleDeath)
    bpy.utils.register_class(PW_OT_GenerateSimpleStun)

    bpy.utils.register_class(PW_OT_Generate)
    bpy.utils.register_class(PW_OT_AddTurnConfig)
    bpy.utils.register_class(PW_OT_SetNegativeAngle)

    bpy.utils.register_class(PW_OT_GenerateMechanical)
    
    bpy.utils.register_class(PW_OT_BoneListAdd)
    bpy.utils.register_class(PW_OT_BoneListRemove)
    bpy.utils.register_class(PW_OT_BoneListMove)
    bpy.utils.register_class(PW_OT_DodgeAutoFill)
    bpy.utils.register_class(PW_OT_PanicAutoFill)
    bpy.utils.register_class(PW_OT_RageAutoFill)

def unregister():
    bpy.utils.unregister_class(PW_OT_SetNegativeAngle)
    bpy.utils.unregister_class(PW_OT_AddTurnConfig)
    bpy.utils.unregister_class(PW_OT_Generate)
    
    # New simple unit operators (reverse order)
    bpy.utils.unregister_class(PW_OT_GenerateSimpleFly)
    bpy.utils.unregister_class(PW_OT_GenerateSimpleRoll)
    bpy.utils.unregister_class(PW_OT_GenerateSimpleShoot)
    bpy.utils.unregister_class(PW_OT_GenerateSimplePanic)
    bpy.utils.unregister_class(PW_OT_GenerateSimpleSurprise)
    
    # Original simple unit operators
    bpy.utils.unregister_class(PW_OT_GenerateSimpleDodge)
    bpy.utils.unregister_class(PW_OT_GenerateSimpleDamage)
    bpy.utils.unregister_class(PW_OT_GenerateSimpleJump)
    bpy.utils.unregister_class(PW_OT_GenerateSimpleTurn)
    bpy.utils.unregister_class(PW_OT_GenerateSimpleSlide)
    bpy.utils.unregister_class(PW_OT_GenerateSimpleMove)
    bpy.utils.unregister_class(PW_OT_GenerateSimpleIdle)
    bpy.utils.unregister_class(PW_OT_GenerateSimpleDeath)
    bpy.utils.unregister_class(PW_OT_GenerateSimpleStun)

    bpy.utils.unregister_class(PW_OT_ApplyPreset)
    bpy.utils.unregister_class(PW_OT_PresetImport)
    bpy.utils.unregister_class(PW_OT_PresetExport)

    bpy.utils.unregister_class(PW_OT_GenerateMechanical)
    
    bpy.utils.unregister_class(PW_OT_BoneListAdd)
    bpy.utils.unregister_class(PW_OT_BoneListRemove)
    bpy.utils.unregister_class(PW_OT_BoneListMove)
    bpy.utils.unregister_class(PW_OT_DodgeAutoFill)
    bpy.utils.unregister_class(PW_OT_PanicAutoFill)
    bpy.utils.unregister_class(PW_OT_RageAutoFill)
