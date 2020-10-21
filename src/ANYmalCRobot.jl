module ANYmalCRobot

using RigidBodyDynamics
using RigidBodyDynamics.Contact

packagepath() = joinpath(@__DIR__, "..")
urdfpath() = joinpath(packagepath(), "anymal_c_simple_description/urdf/anymal.urdf")

function default_contact_model()
    SoftContactModel(hunt_crossley_hertz(k = 500e3), ViscoelasticCoulombModel(0.8, 20e3, 100.))
end

function mechanism(::Type{T} = Float64;
    floating = true,
    contactmodel = default_contact_model(),
    remove_fixed_tree_joints = false, add_flat_ground = false) where {T}
    mechanism = RigidBodyDynamics.parse_urdf(urdfpath(); 
        scalar_type = T, 
        floating = floating, 
        remove_fixed_tree_joints = remove_fixed_tree_joints)

    if add_flat_ground
        frame = root_frame(mechanism)
        ground = HalfSpace3D(Point3D(frame, 0., 0., 0.), FreeVector3D(frame, 0., 0., 1.))
        add_environment_primitive!(mechanism, ground)
    end

    remove_fixed_tree_joints && remove_fixed_tree_joints!(mechanism)

    return mechanism
end

function __init__()
    if !isfile(urdfpath())
        error("Could not find $(urdfpath()). Please run `import Pkg; Pkg.build(\"ANYmalCRobot\")`.")
    end
end

end # module