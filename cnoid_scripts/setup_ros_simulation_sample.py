### for development version
from cnoid.Base import RootItem
from cnoid.Base import ItemTreeView

from cnoid.BodyPlugin import AISTSimulatorItem
from cnoid.BodyPlugin import BodyItem
from cnoid.BodyPlugin import WorldItem

from cnoid.ROSPlugin import WorldROSItem
from cnoid.ROSPlugin import BodyROSItem
from cnoid.ROSPlugin import ROSControlItem

import cnoid.Util

## addWorld
ri = RootItem.instance
wd = ri.findItem('World')
if wd is None:
    wd = WorldItem()
    wd.setName('World')
    ri.addChildItem(wd)
    ItemTreeView.instance.checkItem(wd)

## add Robot
bi = BodyItem()
bi.load(cnoid.Util.shareDirectory + '/model/SR1/SR1.body')
bi.body.updateLinkTree()
bi.body.initializePosition()
bi.body.calcForwardKinematics()
bi.storeInitialState()
wd.insertChildItem(bi, wd.childItem)
ItemTreeView.instance.checkItem(bi)

## add BodyROSItem
bri = BodyROSItem()
bri.setName('BodyROSItem')
# bri.nameSpace =
# bri.controlTime =
# bri.jointStateUpdateRate =
bi.addChildItem(bri)
ItemTreeView.instance.checkItem(bri)

## add
rci = ROSControlItem()
rci.setName('ROSControlItem')
## rci.nameSpace = ''
bi.addChildItem(rci)
ItemTreeView.instance.checkItem(rci)

## add floor
env_fl = BodyItem()
env_fl.load(cnoid.Util.shareDirectory + '/model/misc/floor.body')
env_fl.body.updateLinkTree()
env_fl.body.initializePosition()
env_fl.body.calcForwardKinematics()
env_fl.storeInitialState()
wd.insertChildItem(env_fl, wd.childItem)
ItemTreeView.instance.checkItem(env_fl)

## addSim
sim = wd.findItem('AISTSimulator')
if sim is None:
    sim = AISTSimulatorItem()
    sim.setName('AISTSimulator')
    wd.addChildItem(sim)## ? insert
    ItemTreeView.instance.checkItem(sim)

## GLVISION

## add ROSWorld
rw = wd.findItem('WorldROS')
if rw is None:
    rw = WorldROSItem()
    rw.setName('WorldROS')
    rw.maxClockPublishingRate = 100
    wd.addChildItem(rw)## ? insert
    ItemTreeView.instance.checkItem(rw)
