extends Camera3D

const screenshot_path = "res://screenshots/"

func _ready():
	await take_screenshots()
	get_tree().quit()

func calculate_camera_matrix():
	var viewport_size = get_viewport().get_visible_rect().size
	var fov_rad = deg_to_rad(get_fov())
	var focal_length_x = viewport_size.x / (2.0 * tan(fov_rad / 2.0))
	var focal_length_y = viewport_size.y / (2.0 * tan(fov_rad / 2.0))

	var center_x = viewport_size.x / 2.0
	var center_y = viewport_size.y / 2.0
	
	var camera_matrix = PackedFloat32Array([
		focal_length_x, 0.0, center_x,
		0.0, focal_length_y, center_y,
		0.0, 0.0, 1.0
	])
	return camera_matrix

func take_screenshot(step: int):
		var image = get_viewport().get_texture().get_image()
		image.flip_y() # Images are flipped by default in Godot, so we need to flip them back
		var file_name = "world_%d" % step
		var file_path = screenshot_path + file_name
		image.save_png(file_path + ".png")
		generate_json(file_path + ".json")

func take_screenshots():
	var dir = DirAccess.open("res://")
	if not dir.dir_exists(screenshot_path):
		dir.make_dir_recursive(screenshot_path)

	for step in range(4):
		await get_tree().create_timer(0.5).timeout 
		take_screenshot(step)
		position.x += 1

func generate_json(path: String):
	var world_json = {
		"camera": {
			"calibration_matrix": [[0,0,0],[0,0,0],[0,0,0]],
			"position": from_vector3_to_json(position),
			"rotation": from_vector3_to_json(rotation),
		},
		"tags": [],
	}
	
	var calibration_matrix = calculate_camera_matrix()
	for i in range(3):
		world_json.camera.calibration_matrix[i] = [calibration_matrix[i*3 + 0], calibration_matrix[i*3 + 1], calibration_matrix[i*3 + 2]]
	
	var tag_json = {
		"id": 0,
		"position": [0, 0, 0],
		"rotation": [0, 0, 0],
		"size_m": 0,
	}

	for root_kids in get_node("/root").get_children(true):
		for tag in root_kids.get_children(true):
			if not tag.name.to_lower().begins_with("aruco_"):
				continue
			var sprite: Sprite3D = tag
			print(sprite, sprite.name, sprite.get_meta("id"), sprite.position, sprite.rotation)
			tag_json.id = sprite.get_meta("id")
			tag_json.position = from_vector3_to_json(sprite.position)
			tag_json.rotation = from_vector3_to_json(sprite.rotation)
			tag_json.size_m = sprite.get_meta("size_m")
			world_json["tags"].push_front(tag_json.duplicate())

	var save_file = FileAccess.open(path, FileAccess.WRITE)
	save_file.store_line(JSON.stringify(world_json))

func from_vector3_to_json(from: Vector3):
	return {
		"x": from.x,
		"y": from.y,
		"z": from.z,
	}
