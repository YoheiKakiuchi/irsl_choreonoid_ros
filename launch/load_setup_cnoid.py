from irsl_choreonoid_ros.setup_cnoid import SetupCnoid
import os
from distutils.util import strtobool

fname='cnoid_settings.yaml'
if 'SETUP_CNOID_FILE' in os.environ:
    tmp=os.environ['SETUP_CNOID_FILE']
    if len(tmp) > 0:
        fname=tmp

cnoid=SetupCnoid.setCnoidFromYaml(fname)

if 'SETUP_CNOID_RUN_SIM' in os.environ:
    tmp=os.environ['SETUP_CNOID_RUN_SIM']
    if len(tmp) > 0:
        if bool(strtobool(tmp)):
            cnoid.startSimulator()
