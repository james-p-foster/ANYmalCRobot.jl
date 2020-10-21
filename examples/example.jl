using ANYmalCRobot
using MeshCat
using MeshCatMechanisms
using RigidBodyDynamics

# Initialize anymal mechanism and MeshCat visualizer
anymal = ANYmalCRobot.mechanism()
vis = Visualizer()
mvis = MechanismVisualizer(
    anymal,
    URDFVisuals(ANYmalCRobot.urdfpath(), package_path=[ANYmalCRobot.packagepath()]),
    vis)

# set_nominal!() is a helper function to put the robot into a default config
anymal_state = MechanismState(anymal)
ANYmalCRobot.set_nominal!(anymal_state)
set_configuration!(mvis, configuration(anymal_state))

# Helper functions to visualise joint frames and contacts
# Try running visualize_frames() with optional kwarg "show_all_frames = true" to
# show all frames, by default only revolute are shown!
ANYmalCRobot.visualize_frames(anymal, mvis)
ANYmalCRobot.visualize_contacts(anymal, mvis)