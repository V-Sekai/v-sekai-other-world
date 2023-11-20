#########################################
#######       Rig On The Fly      #######
####### Copyright © 2021 Dypsloom #######
#######    https://dypsloom.com/  #######
#########################################

import bpy
from ..core import ikLimb, fkLimb

FKIKSWITCH_ID = '_ROTF_IKFKSWITCH'


class IKLimbOperator(bpy.types.Operator):
    bl_idname = "rotf.ik_limb"
    bl_label = "IK Limb"
    bl_description = "Changes selected controllers and their two parents to work in IK. Unexpected results if the limb chain does not have a clear bending angle in bind pose and/or middle controller has non 0 Y rotation"
    bl_options = {'REGISTER', 'UNDO', 'INTERNAL'}

    def execute(self, context):
        result = ikLimb.IKLimb()
        if result != None:
            self.report(*result) # * unpacks list into a tuple
            return {'CANCELLED'}
        return {'FINISHED'}

class FKLimbOperator(bpy.types.Operator):
    bl_idname = "rotf.fk_limb"
    bl_label = "FK Limb"
    bl_description = "Changes selected IK handle controller back to working in FK"
    bl_options = {'REGISTER', 'UNDO', 'INTERNAL'}

    def execute(self, context):
        fkLimb.FKLimb()
        return {'FINISHED'}