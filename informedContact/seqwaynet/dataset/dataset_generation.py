import robotic as ry
import numpy as np
from seqwaynet.dataset.utils import utils
import pathlib
import json
import pickle


class DatasetGenerator:
    def __init__(self, dataset_path):
        self.dataset_path = dataset_path
        self.resources_path = f"{pathlib.Path(__file__).parent.resolve()}/resources"

    @staticmethod
    def get_waypoints_and_contactpoints(config, number_of_waypoints):
        list_of_waypoints = []
        list_of_contactpoints = []

        for u in number_of_waypoints:
            tmp = []
            for v in range(u - 1):
                tmp.append(config.getFrame(f"waypoint_up_{u}_{v}").getPosition())
            for v in range(u - 1):
                tmp.append(config.getFrame(f"waypoint_{u}_{v}").getPosition())
            for v in range(u - 1):
                tmp.append(config.getFrame(f"waypoint_down_{u}_{v}").getPosition())
            waypoints = np.array(tmp[1:])
            contactpoints = np.array(tmp[:-1])
            list_of_waypoints.append(waypoints)
            list_of_contactpoints.append(contactpoints)

        return list_of_waypoints, list_of_contactpoints

    def generate(
        self,
        task,
        count,
        number_of_waypoints=[12, 16, 20],
        generation_types=["image", "config"],
    ):
        if task == "bolt":
            pathlib.Path(f"{self.dataset_path}/bolt").mkdir(parents=True, exist_ok=True)

            # common info
            info = {}

            # camera pose (assumed to be common for all scenes)
            if "image" in generation_types:
                config = ry.Config()
                config.addFile(f"{self.resources_path}/scenes/bolt.g")
                for i in range(1, 4):
                    frame = config.getFrame(f"camera_{i}")
                    info[f"camera{i}_position"] = frame.getPosition()
                    info[f"camera{i}_rotation"] = frame.getRotationMatrix()

            with open(f"{self.dataset_path}/bolt/info.pkl", "wb") as file:
                pickle.dump(info, file)

            # individual data generation
            for i in range(count):
                self.generate_random_bolt(i, number_of_waypoints, generation_types)
            print(f"Generated {count} random bolts")

    def generate_random_bolt(self, index, number_of_waypoints, generation_types):
        directory_path = self.dataset_path + "/bolt/"
        directory_name = "bolt_" + str(index)

        rng = np.random.default_rng()
        config = ry.Config()

        config.addFile(f"{self.resources_path}/scenes/bolt.g")

        # Randomize the box
        box = config.getFrame("box")
        box_x = 0.0 + rng.random() * -0.35  # random x
        box_y = 0.2 + rng.random() * 0.15  # random y
        box_length = 0.85 + rng.random() * 0.21  # random length
        box_radius = 0.03 + rng.random() * 0.01  # random radius
        box_color = rng.random(size=3)  # random color
        box.setRelativePosition([box_x, box_y, 0.20])
        box.setShape(type=ry.ST.ssBox, size=[box_length, 0.04, 0.05, box_radius])
        box.setColor(box_color)

        # Randomize the head body.
        head_body = config.getFrame("head_body")
        head_body_x = -0.04 + rng.random() * -0.05  # random x
        head_body_radius = 0.01 + rng.random() * 0.01  # random radius
        head_body_length = 0.10 + rng.random() * 0.10  # random length
        head_body_y = head_body_length / 2 + box_radius  # random y
        head_body_color = rng.random(size=3)  # random color
        head_body.setRelativePosition([head_body_x, head_body_y, 0.0])
        head_body.setShape(
            type=ry.ST.ssBox, size=[0.02, head_body_length, 0.01, head_body_radius]
        )
        head_body.setColor(head_body_color)

        # Randomize the head tip.
        head_tip = config.getFrame("head_tip")
        head_tip_radius = 0.02 + rng.random() * 0.01  # random radius
        head_tip_y = head_body_length / 2 + head_tip_radius  # random y
        head_tip_color = rng.random(size=3)  # random color
        head_tip.setRelativePosition([0.0, head_tip_y, 0])
        head_tip.setShape(type=ry.ST.ssBox, size=[0.02, 0.02, 0.01, head_tip_radius])
        head_tip.setColor(head_tip_color)

        # Randomize the locks.
        lock_list = config.getFrameNames()[-17:]
        lock_width = 0.10 + rng.random() * 0.15  # random width
        lock_color = rng.random(size=3)  # random color
        for lock in lock_list:
            config.getFrame(lock).setColor(lock_color)
        config.getFrame("lock2_top_r").setShape(
            type=ry.ST.box, size=[lock_width, 0.05, 0.03, 0.5]
        )
        config.getFrame("lock2_side_r").setShape(
            type=ry.ST.box, size=[lock_width, 0.03, 0.14, 0.5]
        )

        # Generate waypoints and contactpoints labels
        for each in number_of_waypoints:
            tmp_w = None

            # Feasible end point
            end_point = (
                (config.getFrame("lock3_top").getPosition()[0] - 0.1)
                - (config.getFrame("lock2_top_r").getPosition()[0] + lock_width / 2)
            ) / 2
            end_point = (
                (config.getFrame("lock2_top_r").getPosition()[0] + lock_width / 2)
                + end_point
                - config.getFrame("box").getPosition()[0]
            )

            tmp_c = config.addFrame(f"contact_start", "head_tip")

            waypoint_spacing = np.linspace(head_body_x, end_point, num=each)

            # Waypoints and contactpoints for climbing the curve
            radius_up_distance = tmp_c.getPosition()
            curve_up_spacing = np.linspace(0, 90, num=each)
            for i in range(each):
                tmp_w = config.addFrame(f"waypoint_up_{each}_{i}", "box")
                # tmp_w.setShape(type=ry.ST.marker, size=[0.05])
                tmp_w.setPosition(radius_up_distance)
                tmp_w.setRelativePosition(
                    utils.rotate_points_around_x(
                        np.array(tmp_w.getRelativePosition()),
                        np.radians(curve_up_spacing[i]),
                    )
                    + np.random.normal(0, 0.00001, (3,))
                )

            # Waypoints and contactpoints for direct path
            linear_distance = tmp_w.getPosition()
            for j in range(each - 1):
                tmp_w = config.addFrame(f"waypoint_{each}_{j}", "box")
                # tmp_w.setShape(type=ry.ST.marker, size=[0.05])
                tmp_w.setRelativePosition(
                    np.array([waypoint_spacing[j + 1], 0, 0])
                    + np.random.normal(0, 0.00001, (3,))
                )
                tmp_w.setPosition(
                    [tmp_w.getPosition()[0], linear_distance[1], linear_distance[2]]
                )

            # Waypoints and contactpoints for descending the curve
            radius_down_distance = tmp_w.getPosition()
            curve_down_spacing = np.linspace(360, 270, num=each)
            for k in range(each - 1):
                tmp_w = config.addFrame(f"waypoint_down_{each}_{k}", "box")
                # tmp_w.setShape(type=ry.ST.marker, size=[0.05])
                tmp_w.setPosition(radius_down_distance)
                tmp_w.setRelativePosition(
                    utils.rotate_points_around_x(
                        np.array(tmp_w.getRelativePosition()),
                        np.radians(curve_down_spacing[k + 1]),
                    )
                    + np.random.normal(0, 0.00001, (3,))
                )

        # Applying random rotation to the setup
        box_quat_alpha = -np.pi / 6 + rng.random() * 10 * np.pi / 24  # random alpha
        box_quat_w = np.cos(box_quat_alpha / 2)  # random w
        box_quat_z = np.sin(box_quat_alpha / 2)  # random z
        box.setQuaternion([box_quat_w, 0, 0, box_quat_z])

        # Label list varying number of waypoints and contactpoints
        list_of_waypoints, list_of_contactpoints = self.get_waypoints_and_contactpoints(
            config, number_of_waypoints
        )

        # Changing the parents of the lock parts from box to None.Necessary for the manipulate the box without the locks
        # First parent relation was necessary to determine relative position of the locks
        for lock in lock_list:
            if lock not in ("lock_part_1", "lock_part_2", "lock_part_3", "lock_part_4"):
                old = config.getFrame(lock)
                new = config.addFrame(f"new_{lock}", "lock_part")
                new.setPosition(old.getPosition())
                new.setQuaternion(old.getQuaternion())
                new.setShape(type=ry.ST.box, size=old.getSize())
                new.setContact(-1)
                new.setColor(lock_color)
                config.delFrame(lock)
            else:
                config.delFrame(lock)

        if "image" in generation_types:
            # First camera angle
            cam = ry.CameraView(config)
            cam.setCamera("camera_1")
            rgb_cam1, depth_cam1 = cam.computeImageAndDepth(config)
            pcl_cam1 = ry.depthImage2PointCloud(depth_cam1, cam.getFxycxy())

            # Second camera angle
            cam = ry.CameraView(config)
            cam.setCamera("camera_2")
            rgb_cam2, depth_cam2 = cam.computeImageAndDepth(config)
            pcl_cam2 = ry.depthImage2PointCloud(depth_cam2, cam.getFxycxy())

            # Third camera angle
            cam.setCamera("camera_3")
            rgb_cam3, depth_cam3 = cam.computeImageAndDepth(config)
            pcl_cam3 = ry.depthImage2PointCloud(depth_cam3, cam.getFxycxy())

            # Create an indexed directory for the dataset, and add config, images and pointclouds
            utils.create_directory(directory_path, directory_name)
            utils.save_as_pickle(
                list_of_waypoints,
                directory_path + directory_name + f"/{index}_waypoints.pkl",
            )
            utils.save_as_pickle(
                list_of_contactpoints,
                directory_path + directory_name + f"/{index}_contactpoints.pkl",
            )
            utils.save_config_to_file(
                config, directory_path + directory_name + f"/{index}_bolt.g"
            )

            utils.save_images_to_file(
                rgb_cam1,
                depth_cam1,
                pcl_cam1,
                directory_path + directory_name + f"/{index}_angle1",
            )
            utils.save_images_to_file(
                rgb_cam2,
                depth_cam2,
                pcl_cam2,
                directory_path + directory_name + f"/{index}_angle2",
            )
            utils.save_images_to_file(
                rgb_cam3,
                depth_cam3,
                pcl_cam3,
                directory_path + directory_name + f"/{index}_angle3",
            )

        if "config" in generation_types:
            frames = config.getFrameNames()
            objs = []
            for frame_name in frames:
                obj = config.getFrame(frame_name)
                obj_info = obj.info()

                if (
                    "shape" in obj_info.keys()
                    and obj_info["shape"] != "marker"
                    and "panda" not in obj_info["name"]
                    and "table" not in obj_info["name"]
                ):
                    # print(obj_info)
                    objs.append(
                        {
                            "name": frame_name,
                            "pos": obj.getPosition(),
                            "quat": obj.getQuaternion(),
                            "size": obj.getSize(),
                            "shape": obj_info["shape"],
                        }
                    )

            utils.save_as_pickle(
                objs, directory_path + directory_name + f"/{index}_config.pkl"
            )

        # config.view(pause=True)
