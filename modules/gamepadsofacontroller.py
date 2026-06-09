from sofagamepadcontroller import GamepadSofaController, GamepadCallbacks
import numpy as np

def addGamepadControl(rootnode, targetMO, emio):

    # Scales to adjust the sensitivity of the target movement and 
    # gripper control based on the gamepad input
    TARGET_SCALE = 2.0
    GRIPPER_SCALE = 1.5

    targetScale = TARGET_SCALE
    gripperScale = GRIPPER_SCALE
    invert = False

    def openGripper(value: int):
        emio.centerpart.effector.Distance.DistanceMapping.restLengths[0] += value * gripperScale # the factor is here to adjust the speed of the gripper opening/closing
        emio.centerpart.effector.Distance.DistanceMapping.restLengths[0] = min(max(emio.centerpart.effector.Distance.DistanceMapping.restLengths[0], 15), 50) # limits of the gripper opening

    def closeGripper(value: int):
        emio.centerpart.effector.Distance.DistanceMapping.restLengths[0] -= value * gripperScale # the factor is here to adjust the speed of the gripper opening/closing
        emio.centerpart.effector.Distance.DistanceMapping.restLengths[0] = min(max(emio.centerpart.effector.Distance.DistanceMapping.restLengths[0], 15), 50) # limits of the gripper opening

    def moveTargetXZ(horizontal: int, vertical: int):
        target_pos = targetMO.position.value[0][0:3]
        target_pos = target_pos + targetScale * np.array([horizontal, 0, -vertical * [1, -1][invert]])
        targetMO.position.value = [target_pos.tolist() + [0., 0., 0., 1.]]
    
    def moveTargetY(_: int, vertical: int):
        target_pos = targetMO.position.value[0][0:3]
        target_pos = target_pos + targetScale * np.array([0, vertical * [1, -1][invert], 0])
        targetMO.position.value = [target_pos.tolist() + [0., 0., 0., 1.]]

    def precisionMode():
        nonlocal targetScale
        nonlocal gripperScale
        targetScale = 0.1 * TARGET_SCALE if targetScale >= TARGET_SCALE else TARGET_SCALE
        gripperScale = 0.3 * GRIPPER_SCALE if gripperScale >= GRIPPER_SCALE else GRIPPER_SCALE

    def invertVertical():
        nonlocal invert
        invert = not invert

    callbacks = GamepadCallbacks()
    callbacks.sticks.left.moved = moveTargetXZ
    callbacks.sticks.right.moved = moveTargetY
    callbacks.triggers.right.z = openGripper if emio.centerpart.Effector.getChild("Distance") is not None else None # If there is no centerpart, we don't need to control the gripper, so we set the callback to None
    callbacks.triggers.left.z = closeGripper if emio.centerpart.Effector.getChild("Distance") is not None else None
    callbacks.buttons.south.released = precisionMode
    callbacks.buttons.east.released = invertVertical

    rootnode.addObject(GamepadSofaController(callbacks=callbacks))