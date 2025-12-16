bl_info = {
    "name": "Procedural Walk — Brolegion",
    "author": "Brolegion",
    "version": (0, 9, 15),
    "blender": (4, 0, 0),
    "location": "View3D > Sidebar > Procedural Anim",
    "description": "Procedural animations with COM/root-motion, forward axis, manual bone selection, presets import/export, UI grouping, and turn animation.",
    "category": "Animation",
}

import bpy
from .properties import PW_Settings, register as register_properties, unregister as unregister_properties
from .ui import PW_PT_Panel, register as register_ui, unregister as unregister_ui
from .operators import (
    PW_OT_PresetExport,
    PW_OT_PresetImport,
    PW_OT_Generate,
    PW_OT_AddTurnConfig,
    PW_OT_ApplyPreset,
    PW_OT_SetNegativeAngle,
    register as register_operators,
    unregister as unregister_operators,
)
from . import translations
from . import utils
from . import walk_animation
from . import turn_animation
from . import fullbodyswing_animation
from . import simple_unit_anim
from . import jump_animation

def register():
    register_properties()
    register_ui()
    register_operators()
    translations.register()
    print("Procedural Walk addon registered successfully")

def unregister():
    unregister_operators()
    unregister_ui()
    unregister_properties()
    translations.unregister()
    print("Procedural Walk addon unregistered")

if __name__ == "__main__":
    register()
