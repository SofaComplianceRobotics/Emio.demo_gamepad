def createScene(rootnode):

    from emio.utils.header import addHeader, addSolvers
    from emio.parts.controllers.assemblycontroller import AssemblyController
    from emio.parts.gripper import Gripper
    from emio import Emio
    import Sofa

    settings, modelling, simulation = addHeader(rootnode, inverse=True)
    rootnode.VisualStyle.displayFlags = ["showVisual", "showInteractionForceFields"]

    # Units are: s, mm, kg
    rootnode.dt = 0.03
    rootnode.gravity = [0., -9810., 0.]

    # Add Emio to the scene
    emio = Emio(name="Emio",
                legsName=["longleg"],
                legsModel=["beam"],
                legsPositionOnMotor=["counterclockwisedown", "clockwisedown", "counterclockwisedown", "clockwisedown"],
                centerPartName="whitepart",
                centerPartType="deformable",
                centerPartClass=Gripper,
                extended=True)
    if not emio.isValid():
        Sofa.msg_error(simulation, "Emio is not valid, could not add it to the scene graph.")
        return

    simulation.addChild(emio)
    addSolvers(emio, rayleighMass=0, rayleighStiffness=0)

    emio.attachCenterPartToLegs()
    assemblycontroller = AssemblyController(emio)
    emio.addObject(assemblycontroller)

    # Add effector
    emio.effector.addObject("MechanicalObject", template="Rigid3", position=[0, 0, 0, 0, 0, 0, 1] * 4)
    emio.effector.addObject("RigidMapping", rigidIndexPerPoint=[0, 1, 2, 3])

    # Target
    effectorTarget = modelling.addChild('Target')
    effectorTarget.addObject('EulerImplicitSolver', firstOrder=True)
    effectorTarget.addObject('CGLinearSolver', iterations=50, tolerance=1e-10, threshold=1e-10)
    effectorTarget.addObject('MechanicalObject', template='Rigid3',
                             position=[0, -170, 0, 0, 0, 0, 1],
                             showObject=True, showObjectScale=20)

    
    # Add inverse components and GUI
    import Sofa.ImGui as MyGui
    emio.addInverseComponentAndGUI(effectorTarget.getMechanicalState().position.linkpath, barycentric=True)
    MyGui.MoveWindow.setTCPLimits(-100, 100,
                                emio.motors[0].JointActuator.minAngle.value,
                                emio.motors[0].JointActuator.maxAngle.value)
    MyGui.MoveWindow.addAccessory("Gripper's opening (mm)", emio.centerpart.effector.Distance.DistanceMapping.restLengths, 8, 70)
    MyGui.ProgramWindow.addGripper(emio.centerpart.effector.Distance.DistanceMapping.restLengths, 8, 70)

    # Components for the connection to the real robot 
    emio.addConnectionComponents()

    # Gamepad controller
    from modules import gamepadsofacontroller
    gamepadsofacontroller.addGamepadControl(rootnode, effectorTarget.getMechanicalState(), emio)