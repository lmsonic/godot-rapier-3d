extends RigidBody3D

func _physics_process(delta: float) -> void:
	var t = PhysicsServer3D.body_get_state(get_rid(),PhysicsServer3D.BODY_STATE_TRANSFORM)
	print(t)
