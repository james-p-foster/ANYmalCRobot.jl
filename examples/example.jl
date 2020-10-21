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

ANYmalCRobot.visualize_frames(anymal, mvis)