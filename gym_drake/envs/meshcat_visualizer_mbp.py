# -*- coding: utf8 -*-

'''
Usage:

Start the MeshcatVisualizer with the command `meshcat-server` in another
terminal. The visualization will be available at the web url provided by that
server in real time when the correspondin environment's render() method is
called.
'''

import numpy as np
from pydrake.all import (
    Context,
    DispatchLoadMessage,
    DrakeMockLcm,
    PoseBundle,
    Quaternion,
    RigidTransform,
    RotationMatrix,
)
from drake import lcmt_viewer_load_robot
from utils import Rgba2Hex
import meshcat
import meshcat.transformations as tf

class MeshcatVisualizerMBP():
    def __init__(self, scene_graph, prefix="SceneGraph", zmq_url="tcp://127.0.0.1:6000"):
        self._zmq_url = zmq_url
        self._scene_graph = scene_graph
        self._scene_graph_output = scene_graph.AllocateOutput()
        self.prefix = prefix
        self.vis = meshcat.Visualizer(zmq_url=zmq_url)
        self.vis[self.prefix].delete()

    def load(self):
        """
        Loads `meshcat` visualization elements.
        @pre The `scene_graph` used to construct this object must be part of a
        fully constructed diagram (e.g. via `DiagramBuilder.Build()`).
        """
        # Intercept load message via mock LCM.
        mock_lcm = DrakeMockLcm()
        DispatchLoadMessage(self._scene_graph, mock_lcm)
        load_robot_msg = lcmt_viewer_load_robot.decode(
            mock_lcm.get_last_published_message("DRAKE_VIEWER_LOAD_ROBOT"))
        # Translate elements to `meshcat`.
        for i in range(load_robot_msg.num_links):
            link = load_robot_msg.link[i]
            [source_name, frame_name] = link.name.split("::")

            for j in range(link.num_geom):
                geom = link.geom[j]
                element_local_tf = RigidTransform(
                    RotationMatrix(Quaternion(geom.quaternion)),
                    geom.position).GetAsMatrix4()

                if geom.type == geom.BOX:
                    assert geom.num_float_data == 3
                    meshcat_geom = meshcat.geometry.Box(geom.float_data)
                elif geom.type == geom.SPHERE:
                    assert geom.num_float_data == 1
                    meshcat_geom = meshcat.geometry.Sphere(geom.float_data[0])
                elif geom.type == geom.CYLINDER:
                    assert geom.num_float_data == 2
                    meshcat_geom = meshcat.geometry.Cylinder(
                        geom.float_data[1],
                        geom.float_data[0])
                    # In Drake, cylinders are along +z
                    # In meshcat, cylinders are along +y
                    # Rotate to fix this misalignment
                    extra_rotation = tf.rotation_matrix(
                        np.pi/2., [1, 0, 0])
                    element_local_tf[0:3, 0:3] = \
                        element_local_tf[0:3, 0:3].dot(
                            extra_rotation[0:3, 0:3])
                elif geom.type == geom.MESH:
                    meshcat_geom = \
                        meshcat.geometry.ObjMeshGeometry.from_file(
                            geom.string_data[0:-3] + "obj")
                else:
                    print "UNSUPPORTED GEOMETRY TYPE ", \
                        geom.type, " IGNORED"
                    continue

                self.vis[self.prefix][source_name][str(link.robot_num)][
                    frame_name][str(j)]\
                    .set_object(meshcat_geom,
                                meshcat.geometry.MeshLambertMaterial(
                                    color=Rgba2Hex(geom.color)))
                self.vis[self.prefix][source_name][str(link.robot_num)][
                    frame_name][str(j)].set_transform(element_local_tf)

    def draw(self, context):
        assert isinstance(context, Context)
        self._scene_graph.CalcOutput(context, self._scene_graph_output)
        pose_bundle = self._scene_graph_output.get_data(0).get_value()
        # pose_bundle = self._scene_graph.get_output_port(0).EvalAbstract(context).get_value()

        for frame_i in range(pose_bundle.get_num_poses()):
            # SceneGraph currently sets the name in PoseBundle as
            #    "get_source_name::frame_name".
            [source_name, frame_name] = pose_bundle.get_name(frame_i)\
                .split("::")
            model_id = pose_bundle.get_model_instance_id(frame_i)
            self.vis[self.prefix][source_name][str(model_id)][frame_name]\
                .set_transform(pose_bundle.get_pose(frame_i).matrix())
