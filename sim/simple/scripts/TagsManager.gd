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

		var sprite = Sprite3D.new()
		sprite.set_draw_flag(SpriteBase3D.DrawFlags.FLAG_DOUBLE_SIDED, false)
		sprite.texture = load(path + file)
		var size = sprite.texture.get_width()
		var size_m = 0.2
		sprite.pixel_size = 0.2 / size
		sprite.position = Vector3(1*i, 0, 0)
		sprite.set_name("aruco_%d" % i)
		sprite.set_meta("id", int(aruco_id))
		sprite.set_meta("size_m", size_m)
		i += 1
		add_child(sprite)
