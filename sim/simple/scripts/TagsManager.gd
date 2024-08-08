extends Node3D

func _ready():
	var path = "res://tags/"
	var i = 0
	for file in DirAccess.get_files_at(path):
		if not file.ends_with(".png"):
			continue

		# Get aruco id from image name
		var regex = RegEx.new()
		regex.compile("aruco_(?<id>[0-9]+)")
		var result = regex.search(file)
		if not result:
			return
		var aruco_id = result.get_string("id")

		var mesh_instance = MeshInstance3D.new()
		var material = StandardMaterial3D.new()
		material.albedo_texture = load(path + file)
		material.emission_enabled = true
		material.emission_texture = load(path + file)
		mesh_instance.material_override = material
		mesh_instance.mesh = create_plane_mesh(Vector2(0.22, 0.22))
		mesh_instance.position = Vector3(1*i, 0, 0)
		mesh_instance.set_name("aruco_%d" % i)
		mesh_instance.set_meta("id", int(aruco_id))
		mesh_instance.set_meta("size_m", 0.2)
		i += 1
		add_child(mesh_instance)
		print(mesh_instance)

func create_plane_mesh(size: Vector2) -> PlaneMesh:
	var plane_mesh = QuadMesh.new()
	plane_mesh.size = size
	return plane_mesh
