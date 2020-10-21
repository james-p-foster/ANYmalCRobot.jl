module ANYmalCRobot

using RigidBodyDynamics
using RigidBodyDynamics.Contact
using MeshCat
using MeshCatMechanisms

packagepath() = joinpath(@__DIR__, "..")
urdfpath() = joinpath(packagepath(), "anymal_c_simple_description/urdf/anymal.urdf")

function default_contact_model()
    SoftContactModel(hunt_crossley_hertz(k = 500e3), ViscoelasticCoulombModel(0.8, 20e3, 100.))
end

function mechanism(::Type{T} = Float64;
    floating = true,
    contact_model = default_contact_model(),
    remove_fixed_tree_joints = false, add_flat_ground = false) where {T}
    mechanism = RigidBodyDynamics.parse_urdf(urdfpath(); 
        scalar_type = T, 
        floating = floating, 
        remove_fixed_tree_joints = remove_fixed_tree_joints)

    if contact_model != nothing
        for leg in (:LF, :LH, :RF, :RH)
            foot = findbody(mechanism, "$(string(leg))_FOOT")
            frame = default_frame(foot)

            # bottom of foot
            add_contact_point!(foot, ContactPoint(Point3D(frame, 0, 0, -0.0075),
            contact_model))
        end
    end

    if add_flat_ground
        frame = root_frame(mechanism)
        ground = HalfSpace3D(Point3D(frame, 0., 0., 0.), FreeVector3D(frame, 0., 0., 1.))
        add_environment_primitive!(mechanism, ground)
    end

    remove_fixed_tree_joints && remove_fixed_tree_joints!(mechanism)

    return mechanism
end

function set_nominal!(anymal_state::MechanismState)
    mechanism = anymal_state.mechanism
    zero!(anymal_state)

    # Give the legs a natural configuration
    hip_roll = 0.3
    hip_pitch = 0.8
    knee_pitch = 1.2
    for leg in (:LF, :LH, :RF, :RH)
        roll = findjoint(mechanism, "$(string(leg))_HAA")
        pitch = findjoint(mechanism, "$(string(leg))_HFE")
        knee = findjoint(mechanism, "$(string(leg))_KFE")
        set_configuration!(anymal_state, roll, 
            first(string(leg)) == 'R' ? hip_roll : -hip_roll)
        set_configuration!(anymal_state, pitch, 
            last(string(leg)) == 'F' ? hip_pitch : -hip_pitch)
        set_configuration!(anymal_state, knee, 
            last(string(leg)) == 'H' ? knee_pitch : -knee_pitch)
    end

    # Lift the floating base joint off the ground
    floating_joint = first(out_joints(root_body(mechanism), mechanism))
    set_configuration!(anymal_state, floating_joint, [1; 0; 0; 0; 0; 0; 0.52])
    return anymal_state
end

function visualize_frames(mechanism::Mechanism, mvis::MechanismVisualizer;
    show_all_frames = false)
    mechanism_joints = joints(mechanism)
    if !(show_all_frames)
        filter!(x -> string(x.joint_type) != "Fixed joint" &&
            string(x.joint_type) != "Quaternion floating joint", 
            mechanism_joints)
    end
    for joint in mechanism_joints
        body = successor(joint, mechanism)
        body_frame = default_frame(body)
        setelement!(mvis, body_frame, 0.2, "$(body.name)_frame")
    end
end

function __init__()
    if !isfile(urdfpath())
        error("Could not find $(urdfpath()). Please run `import Pkg; Pkg.build(\"ANYmalCRobot\")`.")
    end
end

end # module