import pytest
from klampt import *
from klampt.model.types import knownTypes,make,objectToTypes,transfer
from klampt.model.subrobot import SubRobotModel
import os

KLAMPT_EXAMPLES_DIR = os.path.expanduser('~/Klampt-examples')
KLAMPT_EXAMPLES_DATA_DIR = os.path.expanduser('~/Klampt-examples/data')

def test_make():
    for type in knownTypes():
        if type not in ['RigidObjectModel','RobotModel','TerrainModel']:
            obj = make(type)
            assert type in objectToTypes(obj) or type in ['Point','Vector','LinearPath']

def test_transfer():
    world = WorldModel()
    world.readFile(os.path.join(KLAMPT_EXAMPLES_DATA_DIR,"robots/kinova_with_robotiq_85.urdf"))
    world.readFile(os.path.join(KLAMPT_EXAMPLES_DATA_DIR,"robots/robotiq_85.rob"))
    r1 = world.robot(0)
    r2 = world.robot(1)
    q0 = r1.getConfig()
    q0[9] = 1
    r1.setConfig(q0)
    q = transfer(r1.getConfig(),r1,r1)
    assert len(q)==r1.numLinks()
    q = transfer(r1.getConfig(),r1,r2,{'gripper:*':'*'})
    assert len(q)==r2.numLinks()
    l2 = transfer(r1.link(9),r1,r2,{'gripper:*':'*'})
    assert l2.index >= 0
    assert l2.robot().index == r2.index
    i2 = transfer([0,1,2,3,4,5],r1,r2,{'gripper:*':'*'})
    assert all(v == -1 for v in i2)
    s1 = SubRobotModel(r1,list(range(9,r1.numLinks())))
    s2 = transfer(s1,r1,r2,{'gripper:*':'*'})
    assert len(s2._links) > 0
    assert all(v >= 0 for v in s2._links)

    q = transfer(r2.getConfig(),r2,r1,{'*':'gripper:*'})
    assert len(q)==r1.numLinks()
    l1 = transfer(r2.link(3),r2,r1,{'*':'gripper:*'})
    assert l1.index >= 0
    s2 = SubRobotModel(r2,[0,1,2,3,4])
    s1 = transfer(s2,r2,r1,{'*':'gripper:*'})
    assert len(s1._links)==len(s2._links)
    