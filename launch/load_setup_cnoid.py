from irsl_choreonoid_ros.setup_cnoid import SetupCnoid
import os
fname='cnoid_settings.yaml'
if 'SETUP_CNOID_FILE' in os.environ:
    tmp=os.environ['SETUP_CNOID_FILE']
    if len(tmp) > 0:
        fname=tmp
SetupCnoid.setCnoidFromYaml(fname)
