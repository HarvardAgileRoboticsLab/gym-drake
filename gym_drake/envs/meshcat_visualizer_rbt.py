# -*- coding: utf8 -*-

'''
Usage:

Start the MeshCat visualizer with the command `meshcat-server` in another
terminal. The visualization will be available at the web url provided by that
server in real time when the corresponding environment's render() method is
called.
'''

import numpy as np
from pydrake.all import (
    RigidBodyTree,
    Shape,
)
from utils import Rgba2Hex
import meshcat
import meshcat.transformations as tf


class MeshcatVisualizerRBT(object):
    def __init__(self,
                 rbtree,
                 draw_timestep=0.033333,
                 prefix="RBViz",
                 zmq_url="tcp://127.0.0.1:6000",
                 draw_collision=False):
        self.timestep = draw_timestep
        self.rbtree = rbtree
        self.draw_collision = draw_collision

        # Set up meshcat
        self.prefix = prefix
        self.vis = meshcat.Visualizer(zmq_url=zmq_url)
        self.vis[self.prefix].delete()

        # Publish the tree geometry to get the visualizer started
        self.PublishAllGeometry()

    def PublishAllGeometry(self):
        n_bodies = self.rbtree.get_num_bodies()-1
        all_meshcat_geometry = {}
        for body_i in range(n_bodies):

            body = self.rbtree.get_body(body_i+1)
            # TODO(gizatt) Replace these body-unique indices
            # with more readable body.get_model_name() or other
            # model index information when an appropriate
            # function gets bound in pydrake.
            body_name = body.get_name() + ("(%d)" % body_i)

            if self.draw_collision:
                draw_elements = [self.rbtree.FindCollisionElement(k)
                                 for k in body.get_collision_element_ids()]
            else:
                draw_elements = body.get_visual_elements()

            for element_i, element in enumerate(draw_elements):
                element_local_tf = element.getLocalTransform()
                if element.hasGeometry():
                    geom = element.getGeometry()

                    geom_type = geom.getShape()
                    if geom_type == Shape.SPHERE:
                        meshcat_geom = meshcat.geometry.Sphere(geom.radius)
                    elif geom_type == Shape.BOX:
                        meshcat_geom = meshcat.geometry.Box(geom.size)
                    elif geom_type == Shape.CYLINDER:
                        meshcat_geom = meshcat.geometry.Cylinder(
                            geom.length, geom.radius)
                        # In Drake, cylinders are along +z
                        # In meshcat, cylinders are along +y
                        # Rotate to fix this misalignment
                        extra_rotation = tf.rotation_matrix(
                            np.pi/2., [1, 0, 0])
                        element_local_tf[0:3, 0:3] = \
                            element_local_tf[0:3, 0:3].dot(
                                extra_rotation[0:3, 0:3])
                    elif geom_type == Shape.MESH:
                        meshcat_geom = \
                            meshcat.geometry.ObjMeshGeometry.from_file(
                                geom.resolved_filename[0:-3] + "obj")
                        # respect mesh scale
                        element_local_tf[0:3, 0:3] *= geom.scale
                    else:
                        print "UNSUPPORTED GEOMETRY TYPE ",\
                              geom.getShape(), " IGNORED"
                        continue

                    rgba = [1., 0.7, 0., 1.]
                    if not self.draw_collision:
                        rgba = element.getMaterial()
                    self.vis[self.prefix][body_name][str(element_i)]\
                        .set_object(meshcat_geom,
                                    meshcat.geometry.MeshLambertMaterial(
                                        color=Rgba2Hex(rgba)))
                    self.vis[self.prefix][body_name][str(element_i)].\
                        set_transform(element_local_tf)

    def draw(self, state):
        ''' Evaluates the robot state and draws it.
        '''

        positions = state[0:self.rbtree.get_num_positions()]
        kinsol = self.rbtree.doKinematics(positions)

        body_fill_index = 0
        for body_i in range(self.rbtree.get_num_bodies()-1):
            tf = self.rbtree.relativeTransform(kinsol, 0, body_i+1)
            body = self.rbtree.get_body(body_i+1)
            # Don't try to update the transform of geometry
            # that doesn't exist.
            if ((self.draw_collision and
                    len(body.get_collision_element_ids()) > 0)
                or
                (not self.draw_collision and
                    len(body.get_visual_elements()) > 0)):
                body_name = body.get_name() + ("(%d)" % body_i)
                self.vis[self.prefix][body_name].set_transform(tf)
