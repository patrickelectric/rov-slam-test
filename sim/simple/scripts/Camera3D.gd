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
		generate_json(file_path)

func take_screenshots():
	var dir = DirAccess.open("res://")
	if not dir.dir_exists(screenshot_path):
		dir.make_dir_recursive(screenshot_path)

	for step in range(4):
		await get_tree().create_timer(0.5).timeout 
		take_screenshot(step)
		position.x += 1

func generate_json(path: String):
	var viewport_size = get_viewport().get_visible_rect().size
	var source = {
		"id": path.get_file(),
		"resolution": {
			"width": viewport_size.x,
			"height": viewport_size.y
		},
		"calibration_matrix": [[],[],[]]
	}
	var calibration_matrix = calculate_camera_matrix()
	for i in range(3):
		source.calibration_matrix[i] = [calibration_matrix[i*3 + 0], calibration_matrix[i*3 + 1], calibration_matrix[i*3 + 2]]

	var save_file = FileAccess.open(path + "_camera.json", FileAccess.WRITE)
	save_file.store_line(JSON.stringify(source))

	save_file = FileAccess.open(path + "_source.json", FileAccess.WRITE)
	save_file.store_line(JSON.stringify({
		"position": from_vector3_to_json(position),
		"rotation": from_vector3_to_json(rotation),
	}))
	
	var tags = {
		"tags": []
	}
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
			tags["tags"].push_front(tag_json.duplicate())

	save_file = FileAccess.open(path + "_tags.json", FileAccess.WRITE)
	save_file.store_line(JSON.stringify(tags))

func from_vector3_to_json(from: Vector3):
	return {
		"x": from.x,
		"y": from.y,
		"z": from.z,
	}

# Sensitivity settings for mouse and keyboard inputs
var move_speed: float = 5.0

var rot_x = 0
var rot_y = 0

var LOOKAROUND_SPEED = -0.001

func _input(event):
	if event is InputEventMouseMotion:
		# modify accumulated mouse rotation
		rot_x += event.relative.x * LOOKAROUND_SPEED
		rot_y += event.relative.y * LOOKAROUND_SPEED
		transform.basis = Basis() # reset rotation
		rotate_object_local(Vector3(0, 1, 0), rot_x) # first rotate in Y
		rotate_object_local(Vector3(1, 0, 0), rot_y) # then rotate in X

func _process(delta: float) -> void:
	var positionLabel: Label = get_node("positionLabel")
	var rotation_deg = [rad_to_deg(rotation.x), rad_to_deg(rotation.y), rad_to_deg(rotation.z)]
	var text = "Coord: %.1f %.1f %.1f (%.1f, %.1f, %.1f)" % [position.x, position.y, position.z, rotation_deg[0], rotation_deg[1], rotation_deg[2]]
	positionLabel.set_text(text)
	
	
	# Handle keyboard input for camera movement
	var direction = Vector3.ZERO

	if Input.is_action_pressed("ui_up"):
		direction -= transform.basis.z
	if Input.is_action_pressed("ui_down"):
		direction += transform.basis.z
	if Input.is_action_pressed("ui_left"):
		direction -= transform.basis.x
	if Input.is_action_pressed("ui_right"):
		direction += transform.basis.x
	if Input.is_action_pressed("up"):
		direction += transform.basis.y
	if Input.is_action_pressed("down"):
		direction -= transform.basis.y

	# Normalize the direction vector
	direction = direction.normalized()

	# Move the camera
	position += direction * move_speed * delta

func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseMotion:
		Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)
