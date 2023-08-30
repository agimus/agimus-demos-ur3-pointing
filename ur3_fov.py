from __future__ import print_function
import pinocchio, hpp.rostools, hppfcl, numpy as np

def _add_fov_to_gui(gui, name, filename_or_pts, group=None, color=None):
    if isinstance(filename_or_pts, str):
        gui.addMesh(name, filename_or_pts)
        if color is not None: gui.setColor(name, [0.1, 0.1, 0.9, 0.2])
    else:
        assert color is not None
        gui.addCurve(name, filename_or_pts, color)
        gui.setCurveMode(name, "TRIANGLE_FAN")
    if group is not None: gui.addToGroup(name, group)
    gui.setBoolProperty(name, "Selectable", False)

_tetahedron_tris = hppfcl.StdVec_Triangle()
_tetahedron_tris.append(hppfcl.Triangle(0, 1, 2))
_tetahedron_tris.append(hppfcl.Triangle(0, 2, 3))
_tetahedron_tris.append(hppfcl.Triangle(0, 3, 4))
_tetahedron_tris.append(hppfcl.Triangle(0, 4, 1))

class Feature:
    def __init__(self, n, s):
        self.name = n
        self.size = s
class Features:
    """
    Set of features with visibility specifications

      - n_visibility_thr: minimal number of features visible in the set to be
        able to perform an accurate localization,
      - depth_margin: increasing ratio of the pyramidal fields of view in the
        depth direction. If C is the top of the pyramid and P a vertex, P is
        moved away from C in such a way that the distance PC is multiplied by
        1 + depth_margin.
      - size_margin increasing ratio of the pyramid base.
    """
    def __init__(self, features, n_visibility_thr, depth_margin, size_margin):
        self.features = features
        self.n_visibility_thr = n_visibility_thr
        self.depth_margin = depth_margin
        self.size_margin = size_margin
    @property
    def names(self):
        return [ t.name for t in self.features ]
    @property
    def sizes(self):
        return [ t.size for t in self.features ]

class RobotFOV:
    def __init__(self,
            urdfString=None,
            urdfFilename=None,
            fov = np.radians((69.4, 52)),
            geoms = [],
            optical_frame = "camera_color_optical_frame",
            group_camera_link = "robot/ur3/camera_link",
            camera_link = "camera_link",
            modelConfig = None
            ):
        self.optical_frame = optical_frame
        self.group_camera_link = group_camera_link
        self.camera_link = camera_link
        self.theta_x = fov[0]/2
        self.theta_y = fov[1]/2

        if modelConfig is None:
            self.modelConfig = lambda q : q
        else:
            self.modelConfig = modelConfig

        if isinstance(fov, str):
            fov_fcl = hppfcl.MeshLoader().load(hpp.rostools.retrieve_resource(fov))
        else:
            # build FOV tetahedron
            dx, dy = np.tan(fov/2)
            pts = hppfcl.StdVec_Vec3f()
            FOV_DEPTH = 1.5
            self.fov = [(0,0,0),
                    ( FOV_DEPTH*dx,  FOV_DEPTH*dy, FOV_DEPTH),
                    (-FOV_DEPTH*dx,  FOV_DEPTH*dy, FOV_DEPTH),
                    (-FOV_DEPTH*dx, -FOV_DEPTH*dy, FOV_DEPTH),
                    ( FOV_DEPTH*dx, -FOV_DEPTH*dy, FOV_DEPTH),]
            pts.extend([ np.array(e) for e in self.fov ])
            self.fov.append(self.fov[1])
            fov_fcl = hppfcl.BVHModelOBBRSS()
            fov_fcl.beginModel(4, 5)
            fov_fcl.addSubModel(pts, _tetahedron_tris)
            fov_fcl.endModel()
            self.fov_fcl = fov_fcl

        self.cos_angle_thr = np.cos(np.radians(70))
        if urdfFilename is None:
            assert urdfString is not None
            # Pinocchio does not allow to build a GeometryModel from a XML string.
            urdfFilename = '/tmp/tmp.urdf'
            with open(urdfFilename, 'w') as f:
                f.write(urdfString)
        self.model, self.gmodel = pinocchio.buildModelsFromUrdf(
                hpp.rostools.retrieve_resource(urdfFilename),
                root_joint = None,
                geometry_types=pinocchio.GeometryType.COLLISION)

        id_parent_frame = self.model.getFrameId(self.optical_frame)
        parent_frame = self.model.frames[id_parent_frame]
        go = pinocchio.GeometryObject("field_of_view", id_parent_frame, parent_frame.parent,
                fov_fcl, parent_frame.placement)

        self.gid_field_of_view = self.gmodel.addGeometryObject(go, self.model)
        for n in geoms:
            assert self.gmodel.existGeometryName(n)
            self.gmodel.addCollisionPair(pinocchio.CollisionPair(self.gmodel.getGeometryId(n), self.gid_field_of_view))

        self.data = pinocchio.Data(self.model)
        self.gdata = pinocchio.GeometryData(self.gmodel)

    def reduceModel(self, jointsToRemove, config, len_prefix=0):
        jointIds = [ self.model.getJointId(jn[len_prefix:]) for jn in jointsToRemove ]
        self.model, self.gmodel = pinocchio.buildReducedModel(self.model, self.gmodel, jointIds, np.array(config))
        self.data = pinocchio.Data(self.model)
        self.gdata = pinocchio.GeometryData(self.gmodel)

    def updateGeometryPlacements(self, q):
        pinocchio.framesForwardKinematics(self.model, self.data, np.array(q))
        pinocchio.updateGeometryPlacements(self.model, self.data, self.gmodel, self.gdata)

    def featureToTetahedronPts(self, oMt, size, margin = 0.002, size_margin = 0):
        """ It assumes that updateGeometryPlacements has been called """
        if not isinstance(oMt, pinocchio.SE3):
            oMt = pinocchio.XYZQUATToSE3(oMt)

        pts = hppfcl.StdVec_Vec3f()
        idc = self.model.getFrameId(self.optical_frame)

        C = self.data.oMf[idc].translation

        pts.append(C)
        increasedSize = (1 + size_margin)*size
        for pt in [
                np.array(( increasedSize / 2,  increasedSize / 2, 0)),
                np.array((-increasedSize / 2,  increasedSize / 2, 0)),
                np.array((-increasedSize / 2, -increasedSize / 2, 0)),
                np.array(( increasedSize / 2, -increasedSize / 2, 0)), ]:
            P = oMt * pt
            u = (C-P)
            u /= np.linalg.norm(u)
            pts.append(P + margin * u)
        return pts

    def featureVisible(self, robot, feature_name, oMt, size, margin, size_margin, verbose=False):
        """ It assumes that updateGeometryPlacements has been called """
        idc = self.model.getFrameId(self.optical_frame)
        camera = self.model.frames[idc]
        oMc = self.data.oMf[idc] # pose of the camera
        if not isinstance(oMt, pinocchio.SE3):
            oMt = pinocchio.XYZQUATToSE3(oMt) # pose of the feature

        ## Check if the feature is in the field of view
        # Axis of the camera frame
        z_c = oMc.rotation[:,2] # View axis (z-axis of the camera frame)
        x_c = oMc.rotation[:,0]
        y_c = oMc.rotation[:,1]
        
        C = oMc.translation # camera
        X = oMt.translation # feature
        CX = X - C
        CXy = CX - np.dot(CX, x_c) * x_c
        CXx = CX - np.dot(CX, y_c) * y_c
        cos_x = np.dot(z_c, CXx) / ( np.linalg.norm(z_c) * np.linalg.norm(CXx) )
        cos_y = np.dot(z_c, CXy) / ( np.linalg.norm(z_c) * np.linalg.norm(CXy) )

        # If the cos along x and y camera axis are smaller than the threshold
        # given by the camera field-of-view, then the feature are not in the FoV
        if cos_x < np.cos(self.theta_x) or cos_y < np.cos(self.theta_y):
            # Feature not in the field of view
            return False

        ## Checking the angle with which we see the feature
        # For april tags, if the angle is too steep, it is not readable
        # oMc.rotation[:,2] view axis
        # oMt.rotation[:,2] normal of the feature, on the feature side.
        cos_theta = - np.dot(oMc.rotation[:,2], oMt.rotation[:,2])
        if cos_theta < self.cos_angle_thr:
            return False

        ## Check if the view to the feature is free or clogged
        pts = self.featureToTetahedronPts(oMt, size, margin, size_margin)

        tetahedron = hppfcl.BVHModelOBBRSS()
        tetahedron.beginModel(4, 5)
        tetahedron.addSubModel(pts, _tetahedron_tris)
        tetahedron.endModel()

        request = hppfcl.CollisionRequest()
        Id = hppfcl.Transform3f()
        # Check if the feature is blocked from the view by an obstacle
        for go, oMg in zip(self.gmodel.geometryObjects, self.gdata.oMg):
            # Don't check for collision with the camera, except with the field_of_view
            if go.parentJoint == camera.parent and go.name != "field_of_view": continue
            result = hppfcl.CollisionResult()
            hppfcl.collide(go.geometry, hppfcl.Transform3f(oMg), tetahedron, Id, request, result)
            if result.isCollision():
                if go.name != "field_of_view":
                    if verbose:
                        print("Feature ", feature_name, " in collision with ", go.name)
                    return False
        return True

    def robotClogsFieldOfView(self):
        """whether the camera is clogged by the selected robot bodies. It assumes that updateGeometryPlacements has been called """
        return pinocchio.computeCollisions(self.gmodel, self.gdata, True)

    def clogged (self, q, robot, featuress, verbose=False):
        def _print(*args):
            if verbose:
                print(*args)
        # should see at least n_visibility_thr feature per featuress
        self.updateGeometryPlacements(self.modelConfig(q))
        for features in featuress:
            nvisible = 0
            for oMt, feature in zip(robot.hppcorba.robot.getJointsPosition(q, features.names), features.features):
                if self.featureVisible(robot, feature.name, oMt, feature.size, features.depth_margin,
                                   features.size_margin, verbose=verbose):
                    nvisible+=1
            if nvisible < features.n_visibility_thr:
                _print("Not enough features visible among ", features)
                return True
        return False

    def visible (self, q, robot, featuress, verbose=False):
        """Returns the number of visible features for the given configuration"""
        def _print(*args):
            if verbose:
                print(*args)
        # should see at least n_visibility_thr feature per featuress
        self.updateGeometryPlacements(self.modelConfig(q))
        nvisibles = []
        for features in featuress:
            nvisible = 0
            for oMt, feature in zip(robot.hppcorba.robot.getJointsPosition(q, features.names), features.features):
                if self.featureVisible(robot, feature.name, oMt, feature.size, features.depth_margin,
                                   features.size_margin, verbose=verbose):
                    nvisible+=1
            nvisibles.append(nvisible)
        return nvisibles

    def addModel(self, name, mesh, frame_name, fMm):
        if not isinstance(fMm, pinocchio.SE3):
            fMm = pinocchio.XYZQUATToSE3(fMm)
        id_parent_frame = self.model.getFrameId(frame_name)
        parent_frame = self.model.frames[id_parent_frame]

        go = pinocchio.GeometryObject(name, id_parent_frame, parent_frame.parent,
                hppfcl.MeshLoader().load(hpp.rostools.retrieve_resource(mesh)),
                parent_frame.placement * fMm)

        self.gmodel.addGeometryObject(go, self.model)
        # self.gmodel.addCollisionPair(pinocchio.CollisionPair(self.gid_field_of_view, self.gmodel.ngeoms-1))
        self.gdata = pinocchio.GeometryData(self.gmodel)

    def appendUrdfModel(self, urdfFilename, frame_name, fMm, prefix = None):
        print("appendUrdfModel")
        if not isinstance(fMm, pinocchio.SE3):
            fMm = pinocchio.XYZQUATToSE3(fMm)
        id_parent_frame = self.model.getFrameId(frame_name)

        model, gmodel = pinocchio.buildModelsFromUrdf(
                hpp.rostools.retrieve_resource(urdfFilename),
                geometry_types=pinocchio.GeometryType.COLLISION)

        if prefix is not None:
            for i in range(1, len(model.names)):
                model.names[i] = prefix + model.names[i]
            for f in model.frames:
                f.name = prefix + f.name
            for go in gmodel.geometryObjects:
                go.name = prefix + go.name

        id_parent_joint = self.model.frames[id_parent_frame].parent
        id_new_joint = self.model.addJoint(id_parent_joint, pinocchio.JointModelFreeFlyer(), pinocchio.SE3(), "part_joint")
        nmodel, ngmodel = pinocchio.appendModel(self.model, model, self.gmodel, gmodel, id_new_joint, fMm)

        self.model, self.gmodel = nmodel, ngmodel
        self.data = pinocchio.Data(self.model)
        self.gdata = pinocchio.GeometryData(self.gmodel)

    def loadInGui(self, v):
        gui = v.client.gui
        _add_fov_to_gui(gui, "field_of_view", self.fov,
                color = [0.1, 0.1, 0.9, 0.2],
                group = self.group_camera_link)
        idl = self.model.getFrameId(self.camera_link)
        idc = self.model.getFrameId(self.optical_frame)
        assert self.model.frames[idl].parent == self.model.frames[idc].parent
        gui.applyConfiguration("field_of_view", pinocchio.SE3ToXYZQUATtuple(
            self.model.frames[idl].placement.inverse() * self.model.frames[idc].placement))
        gui.refresh()

class RobotFOVGuiCallback:
    def __init__(self, robot, robot_fov, featuress, modelConfig=None):
        self.robot = robot
        self.fov = robot_fov
        self.namess = [ [ "fov_" + t.name for t in features.features ] for features in featuress ]
        self.featuress = featuress
        self.initialized = False
        if modelConfig is None:
            modelConfig = lambda q : q
        else:
            self.modelConfig = modelConfig

    def initialize(self, viewer):
        self.initialized = True
        gui = viewer.client.gui
        for names in self.namess:
            for name in names:
                _add_fov_to_gui(gui, name, [ (0,0,0), ] * 6,
                        color = [ 0.1, 0.9, 0.1, 0.2 ],
                        group = "gepetto-gui")

    def show(self, viewer):
        if not self.initialized: self.initialize(viewer)
        gui = viewer.client.gui
        for names in self.namess:
            for name in names:
                gui.setVisibility(name, "ON")
    def hide(self, viewer):
        if not self.initialized: self.initialize(viewer)
        gui = viewer.client.gui
        for names in self.namess:
            for name in names:
                gui.setVisibility(name, "OFF")

    def __call__(self, viewer, q):
        if not self.initialized:
            self.initialize(viewer)
        gui = viewer.client.gui

        self.fov.updateGeometryPlacements(self.modelConfig(q))
        for names, features in zip(self.namess, self.featuress):
            oMts = self.robot.hppcorba.robot.getJointsPosition(q, [ t.name for t in features.features ])
            for name, oMt, feature in zip (names, oMts, features.features):
                pts = [ pt.tolist() for pt in self.fov.featureToTetahedronPts(oMt, feature.size, features.depth_margin, features.size_margin) ]
                gui.setCurvePoints(name, pts + pts[1:2])
                if self.fov.featureVisible(self.robot, feature.name, oMt, feature.size, features.depth_margin,
                                       features.size_margin):
                    gui.setColor(name, [ 0.1, 0.9, 0.1, 0.2 ])
                else:
                    gui.setColor(name, [ 0.9, 0.1, 0.1, 0.2 ])

if __name__ == "__main__":
    urdfString = process_xacro\
      ("package://agimus_demos/urdf/ur3_robot_sim.urdf.xacro",
       "transmission_hw_interface:=hardware_interface/PositionJointInterface")
    robot_fov = RobotFOV(urdfString=urdfString)
