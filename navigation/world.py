import json
from pydantic import BaseModel
import rerun as rr
from utils.vector import Vec3
from typing import Dict


class TagData(BaseModel):
    id: int
    size_m: float
    position: Vec3
    rotation: Vec3


class World:
    def __init__(self, configuration_file: str) -> None:
        try:
            with open(configuration_file, "r") as file:
                config = json.load(file)

                if "tags" not in config:
                    self.tags = [ TagData(**tag) for tag in config ]
                else:
                    self.tags = [ TagData(**tag) for tag in config["tags"] ]
        except (FileNotFoundError, json.JSONDecodeError) as e:
            raise ValueError(f"Error reading world configuration file: {e}")

        self.absolute_tags: Dict[int, TagData] = {
            tag.id: tag for tag in self.tags if tag.position is not None and tag.rotation is not None
        }

        self.tags_size = {tag.id: tag.size_m for tag in self.tags}

        # Initialize known tags (code remains the same)
        for tag in self.tags:
            rr.log(
                f"world/tag_{tag.id}/image",
                rr.Pinhole(
                    focal_length=300,
                    width=tag.size_m * 1000,
                    height=tag.size_m * 1000,
                ),
            )
            rr.log(
                f"world/tag_{tag.id}",
                rr.Transform3D(
                    translation=[tag.position.x, tag.position.y, tag.position.z]
                ),
            )

    def get_abs_tag(self, tag_id: int) -> TagData | None:
        return self.absolute_tags.get(tag_id)
