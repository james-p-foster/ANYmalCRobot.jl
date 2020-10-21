using ANYmalCRobot
using MeshCat
using MeshCatMechanisms
using RigidBodyDynamics

anymal = ANYmalCRobot.mechanism()
vis = Visualizer()
mvis = MechanismVisualizer(
    anymal,
    URDFVisuals(ANYmalCRobot.urdfpath(), package_path=[ANYmalCRobot.packagepath()]),
    vis)

anymal_state = MechanismState(anymal)
ANYmalCRobot.set_nominal!(anymal_state)
set_configuration!(mvis, configuration(anymal_state))

ANYmalCRobot.visualize_frames(anymal, mvis)