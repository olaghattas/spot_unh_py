import numpy as np
import os
import pinocchio



def load_model():
    """
    Gets the example Panda models.

    Returns
    -------
        tuple[`pinocchio.Model`]
            A 3-tuple containing the model, collision geometry model, and visual geometry model.
    """

    package_dir = "/home/olagh/spot_unh_py/spot_arm_description"  
    urdf_filename = os.path.join(package_dir, "urdf", "standalone_arm.urdf")
    model = pinocchio.buildModelsFromUrdf(urdf_filename)
    print(model)
    return model

load_model()