import numpy as np
import os
import pinocchio
from pinocchio.visualize import MeshcatVisualizer
import time

def load_model():
    package_dir = "/home/olagh48652/spot_unh_py/spot_arm_description"
    model_dir = "/home/olagh48652/spot_unh_py" 
    urdf_filename = os.path.join(package_dir, "urdf", "standalone_arm.urdf")
    print("Loading URDF file: ", urdf_filename)
    model, collision_model, visual_model = pinocchio.buildModelsFromUrdf(urdf_filename, package_dirs=model_dir)
    
    # Check the model loaded
    if model:
        print("Model loaded successfully")
    else:
        print("Failed to load model")

    return model, collision_model, visual_model

model, collision_model, visual_model = load_model()

collision_model.addAllCollisionPairs()

# Create data required by the algorithms
data = model.createData()
collision_data = collision_model.createData()

# Sample a random configuration
q = pinocchio.randomConfiguration(model)
print("Joint configuration: %s" % q.T)

# Perform the forward kinematics over the kinematic tree
pinocchio.forwardKinematics(model, data, q)

IDX_TOOL = model.getFrameId('arm_f1x')

pinocchio.framesForwardKinematics(model, data, q)

oMtool = data.oMf[IDX_TOOL]
# SE3 objet
# Tool placement:   R =
# -0.665022  0.437127  0.605529
#  0.168602 -0.701997  0.691934
#  0.727543  0.562245  0.393143
#  p = 0.00513197   0.186422   0.314292


print("Tool placement:",oMtool)

# Print out the placement of each joint of the kinematic tree
for name, oMi in zip(model.names, data.oMi):
    print("{:<24} : {: .2f} {: .2f} {: .2f}".format(name, *oMi.translation.T.flat))

# Initialize visualization with MeshCat.
viz = MeshcatVisualizer(model, collision_model, visual_model, data=data)
viz.initViewer(open=True)
viz.loadViewerModel()
viz.displayCollisions(True)
viz.displayVisuals(True)
viz.displayFrames(True)
DT = 1e-3
for t in range(1000):
    qnext = pinocchio.randomConfiguration(model)
    viz.display(qnext)
    time.sleep(DT/10)
