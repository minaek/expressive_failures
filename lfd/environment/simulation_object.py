from __future__ import division

import trajoptpy
import sim_util
import numpy as np

from lfd.environment.sim_util import make_table_xml

class SimulationObject(object):
    add_after = False # if True, this object needs to be added after the BulletEnvironment has been created
    def __init__(self, names, dynamic=True):
        self.names = names
        self.dynamic = dynamic
        self.sim = None

    def add_to_env(self, sim):
        raise NotImplementedError
    
    def remove_from_env(self):
        raise NotImplementedError
    
    def get_bullet_objects(self):
        if self.sim is None:
            raise RuntimeError("get_bullet_objects should only be called when the object is in an environment")
        bt_objs = []
        for name in self.names:
            body = self.sim.env.GetKinBody(name)
            bt_obj = self.sim.bt_env.GetObjectFromKinBody(body)
            bt_objs.append(bt_obj)
        return bt_objs
    
    def get_state(self):
        return np.asarray([bt_obj.GetTransform() for bt_obj in self.get_bullet_objects()])
    
    def set_state(self, tfs):
        for (bt_obj, tf) in zip(self.get_bullet_objects(), tfs):
            bt_obj.SetTransform(tf)
    
    def _get_constructor_info(self):
        args = [self.names]
        kwargs = {"dynamic":self.dynamic}
        return (type(self).__name__, type(self).__module__), args, kwargs

class XmlSimulationObject(SimulationObject):
    def __init__(self, xml, dynamic=False):
        super(XmlSimulationObject, self).__init__(None, dynamic=dynamic) # TODO: None names
        self.xml = xml

    def add_to_env(self, sim):
        self.sim = sim
        pre_names = [body.GetName() for body in self.sim.env.GetBodies()]
        if '<' in self.xml: # TODO: fix this hack
            self.sim.env.LoadData(self.xml)
        else:
            self.sim.env.Load(self.xml)
        post_names = [body.GetName() for body in self.sim.env.GetBodies()]
        self.names = [name for name in post_names if name not in pre_names]
    
    def remove_from_env(self):
        for bt_obj in self.get_bullet_objects():
            self.sim.env.Remove(bt_obj.GetKinBody())
        self.sim = None
    
    def _get_constructor_info(self):
        args = [self.xml]
        kwargs = {"dynamic":self.dynamic}
        return (type(self).__name__, type(self).__module__), args, kwargs
    
    def __repr__(self):
        return "XmlSimulationObject(%s, dynamic=%r)" % (self.xml, self.dynamic)

class BoxSimulationObject(XmlSimulationObject):
    def __init__(self, name, translation, extents, dynamic=True):
        xml = """
        <Environment>
          <KinBody name="%s">
            <Body type="%s" name="%s_link">
              <Translation>%f %f %f</Translation>
              <Geom type="box">
                <extents>%f %f %f</extents>
              </Geom>
            </Body>
          </KinBody>
        </Environment>
        """ % (name, 'dynamic' if dynamic else 'static', name, translation[0], translation[1], translation[2], extents[0], extents[1], extents[2])
        super(BoxSimulationObject, self).__init__(xml, dynamic=dynamic)
        self.name = name
        self.translation = translation
        self.extents = extents
    
    def _get_constructor_info(self):
        args = [self.name, self.translation, self.extents]
        kwargs = {"dynamic":self.dynamic}
        return (type(self).__name__, type(self).__module__), args, kwargs
    
    def __repr__(self):
        return "BoxSimulationObject(%s, %s, %s, dynamic=%r)" % (self.name, self.translation, self.extents, self.dynamic)

class CylinderSimulationObject(XmlSimulationObject):
    def __init__(self, name, translation, radius, height, dynamic=True):
        xml = """
        <Environment>
          <KinBody name="%s">
            <Body type="%s" name="%s_link">
              <Translation>%f %f %f</Translation>
              <Geom type="cylinder">
                <rotationaxis>1 0 0 90</rotationaxis>
                <radius>%f</radius>
                <height>%f</height>
              </Geom>
            </Body>
          </KinBody>
        </Environment>
        """ % (name, 'dynamic' if dynamic else 'static', name, translation[0], translation[1], translation[2], radius, height)
        super(CylinderSimulationObject, self).__init__(xml, dynamic=dynamic)
        self.name = name
        self.translation = translation
        self.radius = radius
        self.height = height
    
    def _get_constructor_info(self):
        args = [self.name, self.translation, self.radius, self.height]
        kwargs = {"dynamic":self.dynamic}
        return (type(self).__name__, type(self).__module__), args, kwargs
    
    def __repr__(self):
        return "CylinderSimulationObject(%s, %s, %s, %s, dynamic=%r)" % (self.name, self.translation, self.radius, self.height, self.dynamic)

class TableSimulationObject(XmlSimulationObject):
    def __init__(self, name, translation, extents, dynamic=True):
        xml = sim_util.make_table_xml(translation, extents)
        super(TableSimulationObject, self).__init__(xml, dynamic=dynamic)
        self.name = name
        self.translation = translation
        self.extents = extents

    def _get_constructor_info(self):
        args = [self.name, self.translation, self.extents]
        kwargs = {"dynamic":self.dynamic}
        return (type(self).__name__, type(self).__module__), args, kwargs
    
    def __repr__(self):
        return "TableSimulationObject(%s, %s, %s, dynamic=%r)" % (self.name, self.translation, self.extents, self.dynamic)

