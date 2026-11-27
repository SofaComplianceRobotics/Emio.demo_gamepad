def createScene(rootnode):

    from emio.utils.header import addHeader, addSolvers
    from emio.parts.controllers.assemblycontroller import AssemblyController
    from emio.parts.gripper import Gripper
    from emio import Emio
    from math import pi
    import Sofa

    settings, modelling, simulation = addHeader(rootnode, inverse=True)
    rootnode.VisualStyle.displayFlags = ["showVisual", "showInteractionForceFields"]

    # Units are: s, mm, kg
    rootnode.dt = 0.03
    rootnode.gravity = [0., -9810., 0.]
    rootnode.ConstraintSolver.energyWeight.value = 0.1

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
    addSolvers(emio, rayleighMass=0, rayleighStiffness=0.01)

    emio.attachCenterPartToLegs()
    assemblycontroller = AssemblyController(emio)
    emio.addObject(assemblycontroller)

    # Add effector
    emio.effector.addObject("MechanicalObject", template="Rigid3", position=[0, 0, 0, 0, 0, 0, 1] * 4)
    emio.effector.addObject("RigidMapping", rigidIndexPerPoint=[0, 1, 2, 3])

    # Target
    target = modelling.addChild('Target')
    initialPosition = [0, -210, 0, 0, 0, 0, 1]
    target.addObject('EulerImplicitSolver', firstOrder=True)
    target.addObject('CGLinearSolver', iterations=50, tolerance=1e-10, threshold=1e-10)
    target.addObject('MechanicalObject', template='Rigid3',
                                        position=initialPosition,
                                        showObject=True, showObjectScale=20)
    limit=60
    target.addData(name="min", value=-limit, type="float")
    target.addData(name="max", value=limit, type="float")
    target.addData(name="minDistance", value=10, type="float")
    target.addData(name="maxDistance", value=40, type="float")

    # Add inverse components 
    emio.addInverseComponentAndGUI(target.getMechanicalState().position.linkpath, barycentric=True, withGUI=False)
    effectorMaxSpeed = emio.CenterPart.LegsAttach.Effector.EffectorCoord.maxSpeed
    effectorMaxSpeed.value = 100
    for i in range(4):
        emio.motors[i].JointActuator.maxAngle.value = 0.5 * pi
        emio.motors[i].JointActuator.minAngle.value = -0.5 * pi

   # Add GUI elements
    import Sofa.ImGui as MyGui
    import Sofa.SofaConstraintSolver
    import Sofa.SoftRobotsInverse
    import Sofa.SoftRobots
    MyGui.setInverseProblemSolver(solver=rootnode.ConstraintSolver)
    MyGui.addTCP(label="TCP", 
                 constraint=emio.effector.EffectorCoord, 
                 min=target.min.value, max=target.max.value)
    MyGui.addAccessoryFeature(accessoryLabel="Gripper", 
                              featureLabel="Opening (mm)", 
                              data=emio.centerpart.effector.Distance.DistanceMapping.restLengths, 
                              min=target.minDistance.value, 
                              max=target.maxDistance.value)
    MyGui.MyRobotWindow.Settings.addData(label="TCP max speed", data=effectorMaxSpeed, min=0, max=100)
    MyGui.MyRobotWindow.Settings.addData(label="Opening optimization weight", data=emio.centerpart.effector.Distance.PositionEffector.weight, min=0, max=1)

    # Components for the connection to the real robot 
    emio.addConnectionComponents()

    # Gamepad controller
    from modules import gamepadsofacontroller
    gamepadsofacontroller.addGamepadControl(rootnode, target, emio, initialPosition)